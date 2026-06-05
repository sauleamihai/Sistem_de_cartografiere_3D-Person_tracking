#define _GNU_SOURCE
#include "network.h"
#include "types.h"
#include "config.h"
#include "motor.h"
#include "encoder.h"
#include "rt_utils.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>

// UDP command server. Parses simple text commands and sets atomic flags;
// the scanner/motor threads act on those flags. Replies on the sender's port,
// and records the client (with the data port) for the streaming threads.
void* thread_net(void* arg) {
    (void)arg;
    pin_thread(CORE_NONRT, PRIO_NONRT, "Network");

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in srv = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = INADDR_ANY,
        .sin_port = htons(UDP_PORT_CMD)
    };
    bind(sock, (struct sockaddr*)&srv, sizeof(srv));

    char buf[256];
    struct sockaddr_in cli;
    socklen_t len = sizeof(cli);

    while (atomic_load(&G.running)) {
        int n = recvfrom(sock, buf, sizeof(buf)-1, 0, (struct sockaddr*)&cli, &len);
        if (n <= 0) continue;
        buf[n] = 0;

        if (strstr(buf, "PING")) {
            sendto(sock, "PONG", 4, 0, (struct sockaddr*)&cli, len);
            continue;
        }

        pthread_mutex_lock(&G.net_mtx);
        G.client = cli;
        G.client.sin_port = htons(UDP_PORT_DATA);
        atomic_store(&G.client_set, 1);
        pthread_mutex_unlock(&G.net_mtx);

        printf("[NET] %s\n", buf);

        if (strstr(buf, "SCAN")) {
            atomic_store(&G.scan_mode, strstr(buf, "GRID") ? 1 : 0);
            atomic_store(&G.scanning, 1);
            sendto(sock, "OK:STARTED", 10, 0, (struct sockaddr*)&cli, len);
        } else if (strstr(buf, "STOP")) {
            atomic_store(&G.scanning, 0);
            sendto(sock, "OK:STOPPED", 10, 0, (struct sockaddr*)&cli, len);
        } else if (strstr(buf, "HOME")) {
            motor_set_target(&G.roll, 0);
            motor_set_target(&G.pitch, 0);
            sendto(sock, "OK:HOMING", 9, 0, (struct sockaddr*)&cli, len);
        } else if (strstr(buf, "STATUS")) {
            char resp[128];
            snprintf(resp, sizeof(resp), "R:%.1f,P:%.1f,T:%.1f",
                     encoder_get(&G.enc_roll), encoder_get(&G.enc_pitch), G.temp1);
            sendto(sock, resp, strlen(resp), 0, (struct sockaddr*)&cli, len);
        }
    }
    close(sock);
    return NULL;
}
