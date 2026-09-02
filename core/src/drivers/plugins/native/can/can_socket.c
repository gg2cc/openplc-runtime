/**
 * @file can_socket.c
 * @brief Implementation of SocketCAN raw socket operations
 */

#include "can_socket.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef __linux__
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

int can_socket_open(const char *ifname, plugin_logger_t *logger)
{
    if (!ifname || !ifname[0])
        return -1;

    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0)
    {
        plugin_logger_error(logger, "Failed to create CAN socket: %s", strerror(errno));
        return -1;
    }

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);

    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0)
    {
        plugin_logger_error(logger, "Failed to find CAN interface %s: %s", ifname, strerror(errno));
        close(s);
        return -1;
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        plugin_logger_error(logger, "Failed to bind CAN socket to %s: %s", ifname, strerror(errno));
        close(s);
        return -1;
    }

    /* Set a 500ms read timeout so receive thread can periodically check exit flag */
    struct timeval tv;
    tv.tv_sec  = 0;
    tv.tv_usec = 500000;
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(tv));

    plugin_logger_info(logger, "SocketCAN socket bound successfully to %s (fd=%d)", ifname, s);
    return s;
}

int can_socket_read(int fd, uint32_t *can_id, bool *eff, bool *rtr, uint8_t *dlc, uint8_t *payload)
{
    if (fd < 0 || !can_id || !payload)
        return -1;

    struct can_frame frame;
    ssize_t nbytes = read(fd, &frame, sizeof(struct can_frame));
    if (nbytes < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
        {
            /* The receive timeout lets the thread check its stop flag periodically. */
            return 1;
        }
        return -1;
    }

    if (nbytes < (ssize_t)sizeof(struct can_frame))
    {
        return -1;
    }

    *eff = (frame.can_id & CAN_EFF_FLAG) != 0;
    *rtr = (frame.can_id & CAN_RTR_FLAG) != 0;

    if (*eff)
    {
        *can_id = frame.can_id & CAN_EFF_MASK;
    }
    else
    {
        *can_id = frame.can_id & CAN_SFF_MASK;
    }

    *dlc = frame.can_dlc > 8 ? 8 : frame.can_dlc;
    memcpy(payload, frame.data, *dlc);
    return 0;
}

int can_socket_write(int fd, uint32_t can_id, bool eff, bool rtr, uint8_t dlc,
                     const uint8_t *payload)
{
    if (fd < 0 || !payload)
        return -1;

    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));

    frame.can_id = can_id;
    if (eff)
        frame.can_id |= CAN_EFF_FLAG;
    if (rtr)
        frame.can_id |= CAN_RTR_FLAG;

    frame.can_dlc = dlc > 8 ? 8 : dlc;
    memcpy(frame.data, payload, frame.can_dlc);

    ssize_t nbytes = write(fd, &frame, sizeof(struct can_frame));
    if (nbytes < 0)
    {
        return -1;
    }
    return 0;
}

void can_socket_close(int fd)
{
    if (fd >= 0)
    {
        close(fd);
    }
}

#else

int can_socket_open(const char *ifname, plugin_logger_t *logger)
{
    (void)ifname;
    plugin_logger_warn(logger, "SocketCAN is only supported on Linux hosts");
    return -1;
}

int can_socket_read(int fd, uint32_t *can_id, bool *eff, bool *rtr, uint8_t *dlc, uint8_t *payload)
{
    (void)fd;
    (void)can_id;
    (void)eff;
    (void)rtr;
    (void)dlc;
    (void)payload;
    return -1;
}

int can_socket_write(int fd, uint32_t can_id, bool eff, bool rtr, uint8_t dlc,
                     const uint8_t *payload)
{
    (void)fd;
    (void)can_id;
    (void)eff;
    (void)rtr;
    (void)dlc;
    (void)payload;
    return -1;
}

void can_socket_close(int fd)
{
    (void)fd;
}

#endif
