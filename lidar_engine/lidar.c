#define _GNU_SOURCE
#include "lidar.h"
#include "types.h"
#include "config.h"
#include "mathlut.h"
#include "rt_utils.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/socket.h>
#include <arpa/inet.h>

int lidar_open_uart(void) {
    int uart = open(UART_PORT, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (uart < 0) return -1;
    struct termios opts;
    tcgetattr(uart, &opts);
    cfsetspeed(&opts, LIDAR_BAUD);
    opts.c_cflag = (opts.c_cflag & ~CSIZE) | CS8;
    opts.c_cflag &= ~(PARENB | PARODD | CSTOPB);
    opts.c_lflag = opts.c_oflag = 0;
    tcsetattr(uart, TCSANOW, &opts);
    return uart;
}

// HSV-ish depth ramp, red (near) -> violet (far).
void color_from_depth(float z, uint8_t* r, uint8_t* g, uint8_t* b) {
    float v = (z + 1000.0f) / 2000.0f;
    if (v < 0) v = 0;
    if (v > 1) v = 1;
    float h = v * 5.0f + 1.0f;
    int i = (int)h;
    float f = h - i;
    uint8_t q = (1 - f) * 255, t = f * 255;
    switch (i) {
        case 0: *r=255; *g=0;   *b=0;   break;
        case 1: *r=255; *g=t;   *b=0;   break;
        case 2: *r=q;   *g=255; *b=0;   break;
        case 3: *r=0;   *g=255; *b=t;   break;
        case 4: *r=0;   *g=q;   *b=255; break;
        default:*r=t;   *g=0;   *b=255; break;
    }
}

// Transform a single LiDAR return into world XYZ for the active axis.
static inline void transform_point(ScanAxis axis, float dist,
                                   float th_sin, float th_cos,
                                   float s_sin, float s_cos,
                                   struct Point* pt) {
    if (axis == AXIS_PITCH) {
        float lf = dist * th_cos;
        float lu = LIDAR_PITCH_OFFSET;
        pt->x = dist * th_sin;
        pt->y = lf * s_cos - lu * s_sin;
        pt->z = -(lf * s_sin + lu * s_cos);
    } else { // AXIS_ROLL
        float ls = dist * th_sin;
        float lu = LIDAR_ROLL_OFFSET;
        pt->x = ls * s_cos - lu * s_sin;
        pt->y = dist * th_cos;
        pt->z = -(ls * s_sin + lu * s_cos);
    }
}

static void send_udp(int sock, uint8_t* udp, int pkt_idx, int pts_cnt) {
    *(uint32_t*)&udp[4] = pts_cnt;
    pthread_mutex_lock(&G.net_mtx);
    if (atomic_load(&G.client_set))
        sendto(sock, udp, pkt_idx, 0, (struct sockaddr*)&G.client, sizeof(G.client));
    pthread_mutex_unlock(&G.net_mtx);
}

void lidar_capture_angle(int uart, int sock, ScanAxis axis, float actual_angle) {
    tcflush(uart, TCIFLUSH);

    uint8_t pkt_buf[4096];
    uint8_t udp[1400];
    int pkt_len = 0, pkt_idx = 8, pts_cnt = 0;
    memcpy(udp, "PTS:", 4);

    float s_sin = fast_sin(actual_angle);
    float s_cos = fast_cos(actual_angle);
    float last_dist = -1;

    double t_cap = time_now();
    while ((time_now() - t_cap) < SCAN_CAPTURE_TIME) {
        spin_us(500);
        int n = read(uart, pkt_buf + pkt_len, sizeof(pkt_buf) - pkt_len);
        if (n > 0) pkt_len += n;

        int i = 0;
        while (i < pkt_len - 10) {
            if (pkt_buf[i] == 0xAA && pkt_buf[i+1] == 0x55) {
                int lsn = pkt_buf[i+3];
                if (i + 10 + 2*lsn > pkt_len) break;   // wait for full packet

                uint16_t fsa = (pkt_buf[i+5]<<8) | pkt_buf[i+4];
                uint16_t lsa = (pkt_buf[i+7]<<8) | pkt_buf[i+6];

                for (int j = 0; j < lsn; j++) {
                    uint16_t raw = (pkt_buf[i+10+2*j+1]<<8) | pkt_buf[i+10+2*j];
                    float dist = (float)(raw >> 2);
                    if (dist < SCAN_MIN_DIST || dist > SCAN_MAX_DIST) continue;

                    if (last_dist > 0 && fabsf(dist - last_dist) > FLYING_PIXEL_THRESH) {
                        last_dist = dist;          // flying-pixel reject
                        continue;
                    }
                    last_dist = dist;

                    float lidar_ang = (fsa>>1)/64.0f +
                        ((lsa>fsa ? lsa-fsa : lsa+46080-fsa)>>1)/64.0f * (j/(float)lsn);

                    struct Point* pt = (struct Point*)&udp[pkt_idx];
                    transform_point(axis, dist, fast_sin(lidar_ang), fast_cos(lidar_ang),
                                    s_sin, s_cos, pt);
                    color_from_depth(pt->z, &pt->r, &pt->g, &pt->b);

                    pkt_idx += sizeof(struct Point);
                    pts_cnt++;
                    if (pkt_idx + sizeof(struct Point) > sizeof(udp)) {
                        send_udp(sock, udp, pkt_idx, pts_cnt);
                        pkt_idx = 8; pts_cnt = 0;
                    }
                }
                i += 10 + 2*lsn;
            } else i++;
        }
        if (i > 0) { memmove(pkt_buf, pkt_buf + i, pkt_len - i); pkt_len -= i; }
    }

    if (pts_cnt > 0) send_udp(sock, udp, pkt_idx, pts_cnt);

    char msg[64];
    snprintf(msg, sizeof(msg), "STATUS:%s:%.2f",
             axis == AXIS_PITCH ? "PITCH" : "ROLL", actual_angle);
    pthread_mutex_lock(&G.net_mtx);
    if (atomic_load(&G.client_set))
        sendto(sock, msg, strlen(msg), 0, (struct sockaddr*)&G.client, sizeof(G.client));
    pthread_mutex_unlock(&G.net_mtx);
}
