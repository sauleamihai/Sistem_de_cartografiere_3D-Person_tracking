#define _GNU_SOURCE
#include "watchdog.h"
#include "types.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/watchdog.h>

// Hardware watchdog: if this thread stops kicking (process hung/killed
// uncleanly), the SoC watchdog reboots the board. On clean shutdown we write
// the magic 'V' before close() to DISARM it, so `systemctl stop` does not
// trigger a reboot.
void* thread_watchdog(void* arg) {
    (void)arg;
    int fd = open("/dev/watchdog", O_WRONLY);
    if (fd < 0) return NULL;

    int timeout = 15;
    ioctl(fd, WDIOC_SETTIMEOUT, &timeout);

    while (atomic_load(&G.running)) {
        (void)!write(fd, "V", 1);   // kick
        sleep(5);
    }
    (void)!write(fd, "V", 1);       // magic close -> disarm
    close(fd);
    return NULL;
}
