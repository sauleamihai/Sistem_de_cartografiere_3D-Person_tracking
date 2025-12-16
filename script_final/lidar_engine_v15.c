#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/i2c-dev.h>
#include <linux/watchdog.h>
#include <termios.h>
#include <stdatomic.h>
#include <lgpio.h>

// ================= CONFIGURATION =================
#define MOTOR_CHIP 4
#define R_STEP 23
#define R_DIR  24
#define P_STEP 25
#define P_DIR  16

#define FAN1_PWM 22
#define FAN1_IN1 17
#define FAN1_IN2 27
#define FAN2_PWM 26
#define FAN2_IN3 5
#define FAN2_IN4 6

#define I2C_BUS_0 "/dev/i2c-0"
#define I2C_BUS_1 "/dev/i2c-1"
#define UART_PORT "/dev/ttyAMA0"
#define UDP_PORT_CMD 5005
#define UDP_PORT_DATA 5006

// === CALIBRARE GEOMETRICA (IMPORTANT) ===

#define LIDAR_PITCH_OFFSET 55.0f  

#define LIDAR_ROLL_OFFSET 90.0f   

// Optimization Constants
#define LUT_RES 0.25f          
#define LUT_SIZE 1500 

// Mechanics & Sensors
#define STEPS_PER_REV 200
#define MICROSTEPS 32
#define STEPS_PER_DEG ((STEPS_PER_REV * MICROSTEPS) / 360.0)
#define MAX_SPEED 800.0
#define ACCELERATION 400.0
#define AS5600_ADDR 0x36
#define MPU6050_ADDR 0x68
#define HTU21D_ADDR 0x40
#define BMP280_ADDR 0x76

// Scan Settings
#define PITCH_MIN -45
#define PITCH_MAX 45
#define ROLL_MIN -30
#define ROLL_MAX 30
#define TEMP_MIN 30.0
#define TEMP_MAX 50.0
#define TEMP_CRITICAL 60.0
#define SCAN_MIN_DIST 150
#define SCAN_MAX_DIST 7000
#define FLYING_PIXEL_THRESH 50 
#define SCAN_STEP 0.25f 

// ================= GLOBAL STATE =================
struct Motor {
    atomic_int pos;
    atomic_int target;
    double speed;
    double last_time;
};

struct {
    struct Motor roll;
    struct Motor pitch;
    atomic_bool running;
    atomic_bool emergency_stop;
    atomic_bool scanning;
    atomic_int scan_mode; 
    
    // Telemetry
    double temp1, temp2;
    int fan1_speed, fan2_speed;
    
    // Calibration
    uint16_t enc_roll_zero;
    uint16_t enc_pitch_zero;
    
    // Net
    struct sockaddr_in client_addr;
    int client_set;
    
    // Locks
    pthread_mutex_t i2c0_lock;
    pthread_mutex_t i2c1_lock;
    pthread_mutex_t net_lock;
    
    // LUT
    float sin_lut[LUT_SIZE];
    float cos_lut[LUT_SIZE];
} state;

struct __attribute__((packed)) Point {
    float x, y, z;
    uint8_t r, g, b;
};

// ================= UTILITIES =================
void init_math_lut() {
    //printf("Generating LUTs (0.25 deg)...\n");
    for (int i = 0; i < LUT_SIZE; i++) {
        double rad = (i * LUT_RES) * (M_PI / 180.0);
        state.sin_lut[i] = sin(rad);
        state.cos_lut[i] = cos(rad);
    }
}

static inline float fast_sin(float deg) {
    while (deg < 0) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;
    int idx = (int)(deg / LUT_RES);
    if (idx >= LUT_SIZE) idx = LUT_SIZE - 1;
    return state.sin_lut[idx];
}

static inline float fast_cos(float deg) {
    while (deg < 0) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;
    int idx = (int)(deg / LUT_RES);
    if (idx >= LUT_SIZE) idx = LUT_SIZE - 1;
    return state.cos_lut[idx];
}

static inline double get_time_s() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

void busy_wait_us(long us) {
    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);
    long ns = us * 1000;
    while (1) {
        clock_gettime(CLOCK_MONOTONIC, &now);
        long elapsed = (now.tv_sec - start.tv_sec) * 1000000000L + (now.tv_nsec - start.tv_nsec);
        if (elapsed >= ns) break;
        __asm__ __volatile__("nop");
    }
}

void spin_wait_s(double sec) {
    double end = get_time_s() + sec;
    while (get_time_s() < end) __asm__ __volatile__("nop");
}

void pin_core(int core, int prio, const char* name) {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(core, &set);
    pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
    if (prio > 0) {
        struct sched_param p = {.sched_priority = prio};
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &p);
    }
    printf("%s pinned to Core %d\n", name, core);
}

// ================= COLOR MAPPING =================
void map_color_depth(float z, uint8_t* r, uint8_t* g, uint8_t* b) {
    float val = (z + 1000.0f) / 2000.0f;
    if (val < 0) { val = 0; }
    if (val > 1) { val = 1; }
    
    float h = val * 5.0f + 1.0f;
    int i = (int)h;
    float f = h - i;
    uint8_t q = (1 - f) * 255;
    uint8_t t = f * 255;
    switch(i) {
        case 0: *r=255; *g=0; *b=0; break;
        case 1: *r=255; *g=t; *b=0; break;
        case 2: *r=q; *g=255; *b=0; break;
        case 3: *r=0; *g=255; *b=t; break;
        case 4: *r=0; *g=q; *b=255; break;
        case 5: *r=t; *g=0; *b=255; break;
        default: *r=255; *g=0; *b=255; break;
    }
}

// ================= HARDWARE DRIVERS =================
int i2c_open(const char* bus, int addr) {
    int fd = open(bus, O_RDWR);
    if (fd < 0) return -1;
    ioctl(fd, I2C_SLAVE, addr);
    return fd;
}
void i2c_write_byte(int fd, uint8_t reg, uint8_t val, pthread_mutex_t* lock) {
    pthread_mutex_lock(lock); uint8_t buf[2] = {reg, val}; write(fd, buf, 2); pthread_mutex_unlock(lock);
}
uint16_t i2c_read_word(int fd, uint8_t reg, pthread_mutex_t* lock) {
    pthread_mutex_lock(lock); write(fd, &reg, 1); uint8_t buf[2]; read(fd, buf, 2); pthread_mutex_unlock(lock);
    return (buf[0] << 8) | buf[1];
}

uint16_t as5600_raw(int fd, pthread_mutex_t* lock) { return i2c_read_word(fd, 0x0C, lock); }

double htu21d_read(int fd) {
    pthread_mutex_lock(&state.i2c0_lock); uint8_t cmd = 0xF3; write(fd, &cmd, 1); usleep(55000);
    uint8_t buf[3]; int n = read(fd, buf, 3); pthread_mutex_unlock(&state.i2c0_lock);
    if (n != 3) { return NAN; }
    return -46.85 + (175.72 * (((buf[0]<<8)|buf[1]) & 0xFFFC) / 65536.0);
}
double bmp280_read(int fd) {
    pthread_mutex_lock(&state.i2c0_lock); uint8_t reg = 0xFA; write(fd, &reg, 1);
    uint8_t buf[3]; int n = read(fd, buf, 3); pthread_mutex_unlock(&state.i2c0_lock);
    if (n != 3) { return NAN; }
    return (double)((buf[0]<<12)|(buf[1]<<4)|(buf[2]>>4)) / 16384.0;
}

// ================= MOTOR LOGIC =================
void motor_move_to(struct Motor* m, double angle) {
    atomic_store(&m->target, (int)(angle * STEPS_PER_DEG));
}

// ================= SCANNER THREAD =================
void run_scan_pass(struct Motor* m, double start, double end, const char* axis_name, 
                  int uart, int sock) {
    double step = (end > start) ? SCAN_STEP : -SCAN_STEP;
    uint8_t packet_buf[4096];
    int packet_len = 0;
    uint8_t udp[1400];
    float last_dist = -1.0f;
    
    for (double angle = start; (step > 0 ? angle <= end : angle >= end) && atomic_load(&state.scanning); angle += step) {
        if (atomic_load(&state.emergency_stop)) break;
        
        atomic_store(&m->target, (int)(angle * STEPS_PER_DEG));
        
        double t0 = get_time_s();
        while (atomic_load(&m->pos) != atomic_load(&m->target) && (get_time_s() - t0) < 5.0) 
            __asm__ __volatile__("nop");
        spin_wait_s(0.03); 
        
        tcflush(uart, TCIFLUSH);
        double t_cap = get_time_s();
        int pkt_idx = 8, pts_cnt = 0;
        memcpy(udp, "PTS:", 4);
        packet_len = 0;
        
        float s_sin = fast_sin((float)angle);
        float s_cos = fast_cos((float)angle);
        
        while ((get_time_s() - t_cap) < 0.25) { 
            busy_wait_us(500); 
            
            int n = read(uart, packet_buf + packet_len, 4096 - packet_len);
            if (n > 0) packet_len += n;
            
            int i = 0;
            while (i < packet_len - 10) {
                if (packet_buf[i] == 0xAA && packet_buf[i+1] == 0x55) {
                    int lsn = packet_buf[i+3];
                    if (i + 10 + 2*lsn > packet_len) break;
                    
                    uint16_t fsa = (packet_buf[i+5]<<8)|packet_buf[i+4];
                    uint16_t lsa = (packet_buf[i+7]<<8)|packet_buf[i+6];
                    
                    for (int j = 0; j < lsn; j++) {
                        uint16_t raw_dist = ((packet_buf[i+10+2*j+1]<<8)|packet_buf[i+10+2*j]);
                        float dist = (float)(raw_dist >> 2);
                        
                        if (dist < SCAN_MIN_DIST || dist > SCAN_MAX_DIST) continue;
                        if (last_dist > 0 && fabsf(dist - last_dist) > FLYING_PIXEL_THRESH) {
                            last_dist = dist; continue;
                        }
                        last_dist = dist;
                        
                        float lidar_ang = (fsa>>1)/64.0f + ((lsa>fsa ? lsa-fsa : lsa+46080-fsa)>>1)/64.0f * (j/(float)lsn);
                        float th_sin = fast_sin(lidar_ang);
                        float th_cos = fast_cos(lidar_ang);
                        
                        struct Point* pt = (struct Point*)&udp[pkt_idx];
                        
                        // == DUAL OFFSET & STRAIGHT WALLS ===
                        
                        if (m == &state.roll) {
                            
                            float local_side = dist * th_sin;   
                            float local_up   = LIDAR_ROLL_OFFSET;
                            
                            pt->x = local_side * s_cos - local_up * s_sin;
                            
                            pt->y = dist * th_cos;
                        
                            pt->z = - (local_side * s_sin + local_up * s_cos);
                            
                        } else {       
                            float local_fwd = dist * th_cos;    
                            float local_up  = LIDAR_PITCH_OFFSET;
                            
                            pt->x = dist * th_sin; 
                            pt->y = local_fwd * s_cos - local_up * s_sin;
                            pt->z = - (local_fwd * s_sin + local_up * s_cos);
                        }
                        map_color_depth(pt->z, &pt->r, &pt->g, &pt->b);
                        
                        pkt_idx += sizeof(struct Point);
                        pts_cnt++;
                        
                        if (pkt_idx + sizeof(struct Point) > sizeof(udp)) {
                            *(uint32_t*)&udp[4] = pts_cnt;
                            pthread_mutex_lock(&state.net_lock);
                            if (state.client_set) sendto(sock, udp, pkt_idx, 0, (struct sockaddr*)&state.client_addr, sizeof(state.client_addr));
                            pthread_mutex_unlock(&state.net_lock);
                            pkt_idx = 8; pts_cnt = 0;
                        }
                    }
                    i += 10 + 2*lsn;
                } else i++;
            }
            if (i > 0) {
                memmove(packet_buf, packet_buf + i, packet_len - i);
                packet_len -= i;
            }
        }
        
        if (pts_cnt > 0) {
            *(uint32_t*)&udp[4] = pts_cnt;
            pthread_mutex_lock(&state.net_lock);
            if (state.client_set) sendto(sock, udp, pkt_idx, 0, (struct sockaddr*)&state.client_addr, sizeof(state.client_addr));
            pthread_mutex_unlock(&state.net_lock);
        }
        
        char msg[64]; snprintf(msg, 64, "STATUS:%s:%.2f", axis_name, angle);
        pthread_mutex_lock(&state.net_lock);
        if (state.client_set) sendto(sock, msg, strlen(msg), 0, (struct sockaddr*)&state.client_addr, sizeof(state.client_addr));
        pthread_mutex_unlock(&state.net_lock);
    }
}
void* thread_scanner(void* arg) {
    (void)arg;
    pin_core(2, 90, "Scanner");
    int uart = open(UART_PORT, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (uart < 0) return NULL;
    struct termios opts; tcgetattr(uart, &opts); cfsetspeed(&opts, B115200); 
    opts.c_cflag = (opts.c_cflag & ~CSIZE) | CS8; opts.c_cflag &= ~(PARENB | PARODD | CSTOPB); 
    opts.c_lflag = opts.c_oflag = 0; tcsetattr(uart, TCSANOW, &opts);
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    printf("🔭 Scanner Thread Ready (V7.8 - Dual Offset & Straight Walls)\n");
    while (atomic_load(&state.running)) {
        if (!atomic_load(&state.scanning)) { usleep(100000); continue; }
        int mode = atomic_load(&state.scan_mode);
        run_scan_pass(&state.pitch, PITCH_MIN, PITCH_MAX, "ROLL", uart, sock);
        if (mode == 1 && atomic_load(&state.scanning)) {
            atomic_store(&state.pitch.target, 0);
            double t0 = get_time_s();
            while (atomic_load(&state.pitch.pos) != 0 && (get_time_s()-t0 < 5.0)) __asm__ __volatile__("nop");
            run_scan_pass(&state.roll, ROLL_MIN, ROLL_MAX, "PITCH", uart, sock);
            atomic_store(&state.roll.target, 0);
        }
        atomic_store(&state.scanning, 0);
        atomic_store(&state.pitch.target, 0);
        char* done = "STATUS:DONE";
        pthread_mutex_lock(&state.net_lock);
        if (state.client_set) sendto(sock, done, strlen(done), 0, (struct sockaddr*)&state.client_addr, sizeof(state.client_addr));
        pthread_mutex_unlock(&state.net_lock);
    }
    close(uart); close(sock); return NULL;
}
void* thread_temp(void* arg) {
    (void)arg;
    pin_core(1, 0, "Temperature");
    int h = lgGpiochipOpen(MOTOR_CHIP);
    if (h < 0) return NULL;
    lgGpioClaimOutput(h, 0, FAN1_PWM, 0); lgGpioClaimOutput(h, 0, FAN1_IN1, 0); lgGpioClaimOutput(h, 0, FAN1_IN2, 0);
    lgGpioClaimOutput(h, 0, FAN2_PWM, 0); lgGpioClaimOutput(h, 0, FAN2_IN3, 0); lgGpioClaimOutput(h, 0, FAN2_IN4, 0);
    lgGpioWrite(h, FAN1_IN1, 1); lgGpioWrite(h, FAN1_IN2, 0);
    lgGpioWrite(h, FAN2_IN3, 1); lgGpioWrite(h, FAN2_IN4, 0);
    int htu = i2c_open(I2C_BUS_0, HTU21D_ADDR);
    int bmp = i2c_open(I2C_BUS_0, BMP280_ADDR);
    if (htu >= 0) { pthread_mutex_lock(&state.i2c0_lock); uint8_t r=0xFE; write(htu, &r, 1); pthread_mutex_unlock(&state.i2c0_lock); usleep(15000); }
    if (bmp >= 0) { i2c_write_byte(bmp, 0xF4, 0x27, &state.i2c0_lock); usleep(10000); }
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    double last_telem = 0;
    while (atomic_load(&state.running)) {
        double t1 = (htu >= 0) ? htu21d_read(htu) : NAN;
        double t2 = (bmp >= 0) ? bmp280_read(bmp) : NAN;
        if (isnan(t1)) { t1 = TEMP_MIN; }
        if (isnan(t2)) { t2 = TEMP_MIN; }
        state.temp1 = t1; state.temp2 = t2;
        double max_t = (t1 > t2) ? t1 : t2;
        if (max_t >= TEMP_CRITICAL) {
            atomic_store(&state.emergency_stop, 1);
            atomic_store(&state.scanning, 0);
            lgTxPwm(h, FAN1_PWM, 1000, 100, 0, 0); lgTxPwm(h, FAN2_PWM, 1000, 100, 0, 0);
            printf("CRITICAL TEMP: %.1f°C\n", max_t);
            pthread_mutex_lock(&state.net_lock);
            if (state.client_set) {
                char msg[64]; snprintf(msg, 64, "CRITICAL_TEMP:%.1f", max_t);
                sendto(sock, msg, strlen(msg), 0, (struct sockaddr*)&state.client_addr, sizeof(state.client_addr));
            }
            pthread_mutex_unlock(&state.net_lock);
            break;
        }
        int fan = 0;
        if (max_t > TEMP_MIN) {
            fan = (int)(((max_t - TEMP_MIN) / (TEMP_MAX - TEMP_MIN)) * 100);
            if (fan > 100) fan = 100;
        }
        lgTxPwm(h, FAN1_PWM, 1000, (float)fan, 0, 0);
        lgTxPwm(h, FAN2_PWM, 1000, (float)fan, 0, 0);
        state.fan1_speed = state.fan2_speed = fan;
        if (get_time_s() - last_telem >= 5.0) {
            char msg[64]; snprintf(msg, 64, "TEMP:%.1f,%.1f,%d,%d", t1, t2, fan, fan);
            pthread_mutex_lock(&state.net_lock);
            if (state.client_set) sendto(sock, msg, strlen(msg), 0, (struct sockaddr*)&state.client_addr, sizeof(state.client_addr));
            pthread_mutex_unlock(&state.net_lock);
            last_telem = get_time_s();
        }
        sleep(2);
    }
    close(sock); lgGpiochipClose(h); if(htu>=0) close(htu); if(bmp>=0) close(bmp); return NULL;
}
void* thread_network(void* arg) {
    (void)arg;
    pin_core(0, 0, "Network");
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in srv = {.sin_family = AF_INET, .sin_addr.s_addr = INADDR_ANY, .sin_port = htons(UDP_PORT_CMD)};
    bind(sock, (struct sockaddr*)&srv, sizeof(srv));
    char buf[1024]; struct sockaddr_in cli; socklen_t len = sizeof(cli);
    while (atomic_load(&state.running)) {
        int n = recvfrom(sock, buf, sizeof(buf)-1, 0, (struct sockaddr*)&cli, &len);
        if (n > 0) {
            buf[n] = 0;
            if (strstr(buf, "PING")) {
                sendto(sock, "PONG", 4, 0, (struct sockaddr*)&cli, len);
                continue;
            }
            pthread_mutex_lock(&state.net_lock);
            state.client_addr = cli; state.client_addr.sin_port = htons(UDP_PORT_DATA); state.client_set = 1;
            pthread_mutex_unlock(&state.net_lock);
            printf("📨 CMD: %s\n", buf);
            if (strstr(buf, "SCAN")) {
                int m = strstr(buf, "GRID") ? 1 : 0;
                atomic_store(&state.scan_mode, m); atomic_store(&state.scanning, 1);
                sendto(sock, "STATUS:STARTED", 14, 0, (struct sockaddr*)&cli, len);
            } else if (strstr(buf, "STOP")) atomic_store(&state.scanning, 0);
        }
    }
    close(sock); return NULL;
}
void* thread_watchdog(void* arg) {
    (void)arg;
    pin_core(0, 50, "Watchdog");
    int wd_fd = open("/dev/watchdog", O_WRONLY);
    if (wd_fd < 0) { printf("No Watchdog\n"); return NULL; }
    int t=15; ioctl(wd_fd, WDIOC_SETTIMEOUT, &t);
    while (atomic_load(&state.running)) { write(wd_fd, "V", 1); sleep(5); }
    write(wd_fd, "V", 1); close(wd_fd); return NULL;
}
int main() {
    memset(&state, 0, sizeof(state));
    atomic_init(&state.running, 1);
    pthread_mutex_init(&state.i2c0_lock, NULL);
    pthread_mutex_init(&state.i2c1_lock, NULL);
    pthread_mutex_init(&state.net_lock, NULL);
    init_math_lut();
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) perror("mlockall");
    printf("Syncing Encoders...\n");
    int enc_r = i2c_open(I2C_BUS_0, AS5600_ADDR);
    if(enc_r>=0) { 
        state.enc_roll_zero = as5600_raw(enc_r, &state.i2c0_lock);
        motor_move_to(&state.roll, 0); 
        close(enc_r); 
    }
    int enc_p = i2c_open(I2C_BUS_1, AS5600_ADDR);
    if(enc_p>=0) { 
        state.enc_pitch_zero = as5600_raw(enc_p, &state.i2c1_lock);
        motor_move_to(&state.pitch, 0); 
        close(enc_p); 
    }
    state.roll.last_time = state.pitch.last_time = get_time_s();
    pthread_t t1, t2, t3, t4;
    pthread_create(&t1, NULL, thread_scanner, NULL);
    pthread_create(&t2, NULL, thread_temp, NULL);
    pthread_create(&t3, NULL, thread_network, NULL);
    pthread_create(&t4, NULL, thread_watchdog, NULL);
    usleep(500000);
    pin_core(3, 99, "Motor Control");
    int h = lgGpiochipOpen(MOTOR_CHIP);
    lgGpioClaimOutput(h, 0, R_STEP, 0); lgGpioClaimOutput(h, 0, R_DIR, 0);
    lgGpioClaimOutput(h, 0, P_STEP, 0); lgGpioClaimOutput(h, 0, P_DIR, 0);
    struct Motor* mr = &state.roll;
    struct Motor* mp = &state.pitch;
    while (atomic_load(&state.running)) {
        if (atomic_load(&state.emergency_stop)) break;
        double now = get_time_s();
        if (atomic_load(&mr->pos) != atomic_load(&mr->target)) {
            double interval = (mr->speed == 0) ? 0.001 : 1.0 / fabs(mr->speed);
            if ((now - mr->last_time) >= interval) {
                int dist = atomic_load(&mr->target) - atomic_load(&mr->pos);
                double dt = now - mr->last_time;
                if(dt>0.1) dt=0.1;
                double stop = (mr->speed*mr->speed)/800.0;
                if(abs(dist)<=stop) mr->speed = (mr->speed>0)? fmax(10, mr->speed-400*dt) : fmin(-10, mr->speed+400*dt);
                else mr->speed = (dist>0)? fmin(800, mr->speed+400*dt) : fmax(-800, mr->speed-400*dt);
                lgGpioWrite(h, R_DIR, (mr->speed > 0));
                lgGpioWrite(h, R_STEP, 1);
                busy_wait_us(2);
                lgGpioWrite(h, R_STEP, 0);
                atomic_store(&mr->pos, atomic_load(&mr->pos) + ((mr->speed > 0) ? 1 : -1));
                mr->last_time = now;
            }
        }
        if (atomic_load(&mp->pos) != atomic_load(&mp->target)) {
            double interval = (mp->speed == 0) ? 0.001 : 1.0 / fabs(mp->speed);
            if ((now - mp->last_time) >= interval) {
                int dist = atomic_load(&mp->target) - atomic_load(&mp->pos);
                double dt = now - mp->last_time;
                if(dt>0.1) dt=0.1;
                double stop = (mp->speed*mp->speed)/800.0;
                if(abs(dist)<=stop) mp->speed = (mp->speed>0)? fmax(10, mp->speed-400*dt) : fmin(-10, mp->speed+400*dt);
                else mp->speed = (dist>0)? fmin(800, mp->speed+400*dt) : fmax(-800, mp->speed-400*dt);
                lgGpioWrite(h, P_DIR, (mp->speed > 0));
                lgGpioWrite(h, P_STEP, 1);
                busy_wait_us(2);
                lgGpioWrite(h, P_STEP, 0);
                atomic_store(&mp->pos, atomic_load(&mp->pos) + ((mp->speed > 0) ? 1 : -1));
                mp->last_time = now;
            }
        }
    }
    atomic_store(&state.running, 0);
    pthread_join(t1, NULL); pthread_join(t2, NULL); pthread_join(t3, NULL); pthread_join(t4, NULL);
    lgGpiochipClose(h);
    return 0;
}
