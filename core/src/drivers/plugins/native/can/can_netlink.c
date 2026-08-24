/**
 * @file can_netlink.c
 * @brief Netlink helper implementation for configuring CAN hardware parameters
 */

#include "can_netlink.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#ifdef __linux__
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <linux/can/netlink.h>

#ifndef CAN_CTRLMODE_3SAMPLES
#ifdef CAN_CTRLMODE_3_SAMPLES
#define CAN_CTRLMODE_3SAMPLES CAN_CTRLMODE_3_SAMPLES
#endif
#endif

#define NL_BUF_SIZE 4096

struct nl_req {
    struct nlmsghdr nh;
    struct ifinfomsg ifi;
    char buf[NL_BUF_SIZE];
};

static struct rtattr *add_attr(struct nlmsghdr *n, int maxlen, int type, const void *data, int alen)
{
    int len = RTA_LENGTH(alen);
    if (NLMSG_ALIGN(n->nlmsg_len) + RTA_ALIGN(len) > (unsigned int)maxlen) {
        return NULL;
    }
    struct rtattr *rta = (struct rtattr *)(((char *)n) + NLMSG_ALIGN(n->nlmsg_len));
    rta->rta_type = type;
    rta->rta_len = len;
    if (alen > 0 && data) {
        memcpy(RTA_DATA(rta), data, alen);
    }
    n->nlmsg_len = NLMSG_ALIGN(n->nlmsg_len) + RTA_ALIGN(len);
    return rta;
}

static struct rtattr *nest_attr(struct nlmsghdr *n, int maxlen, int type)
{
    return add_attr(n, maxlen, type, NULL, 0);
}

static void end_nest_attr(struct nlmsghdr *n, struct rtattr *nest)
{
    nest->rta_len = (char *)n + NLMSG_ALIGN(n->nlmsg_len) - (char *)nest;
}

static int send_nl_req(int fd, struct nl_req *req, plugin_logger_t *logger)
{
    struct sockaddr_nl nladdr;
    memset(&nladdr, 0, sizeof(nladdr));
    nladdr.nl_family = AF_NETLINK;

    struct iovec iov = {
        .iov_base = req,
        .iov_len = req->nh.nlmsg_len
    };

    struct msghdr msg = {
        .msg_name = &nladdr,
        .msg_namelen = sizeof(nladdr),
        .msg_iov = &iov,
        .msg_iovlen = 1
    };

    if (sendmsg(fd, &msg, 0) < 0) {
        plugin_logger_error(logger, "Netlink sendmsg failed: %s", strerror(errno));
        return -1;
    }

    char reply_buf[1024];
    struct iovec reply_iov = {
        .iov_base = reply_buf,
        .iov_len = sizeof(reply_buf)
    };
    struct msghdr reply_msg = {
        .msg_name = &nladdr,
        .msg_namelen = sizeof(nladdr),
        .msg_iov = &reply_iov,
        .msg_iovlen = 1
    };

    int status = recvmsg(fd, &reply_msg, 0);
    if (status < 0) {
        plugin_logger_error(logger, "Netlink recvmsg failed: %s", strerror(errno));
        return -1;
    }

    struct nlmsghdr *h = (struct nlmsghdr *)reply_buf;
    if (NLMSG_OK(h, (unsigned int)status)) {
        if (h->nlmsg_type == NLMSG_ERROR) {
            struct nlmsgerr *err = (struct nlmsgerr *)NLMSG_DATA(h);
            if (err->error != 0) {
                plugin_logger_warn(logger, "Netlink error response: %s (code %d)",
                                   strerror(-err->error), err->error);
                return err->error;
            }
        }
    }
    return 0;
}

int can_netlink_down(const char *ifname, plugin_logger_t *logger)
{
    int nl_fd = socket(AF_NETLINK, SOCK_RAW, NETLINK_ROUTE);
    if (nl_fd < 0) {
        plugin_logger_error(logger, "Failed to open Netlink socket: %s", strerror(errno));
        return -1;
    }

    unsigned int ifindex = if_nametoindex(ifname);
    if (ifindex == 0) {
        plugin_logger_warn(logger, "CAN interface %s does not exist", ifname);
        close(nl_fd);
        return -1;
    }

    struct nl_req req;
    memset(&req, 0, sizeof(req));

    req.nh.nlmsg_len = NLMSG_LENGTH(sizeof(struct ifinfomsg));
    req.nh.nlmsg_type = RTM_NEWLINK;
    req.nh.nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK;
    req.nh.nlmsg_seq = 1;

    req.ifi.ifi_family = AF_UNSPEC;
    req.ifi.ifi_index = ifindex;
    req.ifi.ifi_flags = 0;
    req.ifi.ifi_change = IFF_UP;

    int ret = send_nl_req(nl_fd, &req, logger);
    close(nl_fd);
    return ret;
}

int can_netlink_configure_and_up(const can_hardware_config_t *hw, plugin_logger_t *logger)
{
    if (!hw || !hw->interface[0]) return -1;

    /* Step 1: Bring down the interface first if auto_bringup is enabled */
    if (hw->auto_bringup) {
        can_netlink_down(hw->interface, logger);
    }

    int nl_fd = socket(AF_NETLINK, SOCK_RAW, NETLINK_ROUTE);
    if (nl_fd < 0) {
        plugin_logger_error(logger, "Failed to open Netlink socket: %s", strerror(errno));
        return -1;
    }

    unsigned int ifindex = if_nametoindex(hw->interface);
    if (ifindex == 0) {
        plugin_logger_warn(logger, "CAN interface %s not found on host", hw->interface);
        close(nl_fd);
        return -1;
    }

    /* Step 2: Configure bitrate, SJW, sample_point, restart_ms */
    struct nl_req req;
    memset(&req, 0, sizeof(req));

    req.nh.nlmsg_len = NLMSG_LENGTH(sizeof(struct ifinfomsg));
    req.nh.nlmsg_type = RTM_NEWLINK;
    req.nh.nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK;
    req.nh.nlmsg_seq = 2;

    req.ifi.ifi_family = AF_UNSPEC;
    req.ifi.ifi_index = ifindex;

    /* IFLA_LINKINFO nest */
    struct rtattr *linkinfo = nest_attr(&req.nh, sizeof(req), IFLA_LINKINFO);
    if (linkinfo) {
        add_attr(&req.nh, sizeof(req), IFLA_INFO_KIND, "can", strlen("can"));
        struct rtattr *infodata = nest_attr(&req.nh, sizeof(req), IFLA_INFO_DATA);
        if (infodata) {
            /* Bit timing */
            struct can_bittiming bt;
            memset(&bt, 0, sizeof(bt));
            bt.bitrate = hw->bitrate;
            bt.sjw = hw->sjw;
            if (hw->sample_point > 0.0 && hw->sample_point < 1.0) {
                bt.sample_point = (uint32_t)(hw->sample_point * 1000.0);
            }
            add_attr(&req.nh, sizeof(req), IFLA_CAN_BITTIMING, &bt, sizeof(bt));

            /* Bus-Off Auto Restart */
            if (hw->restart_ms > 0) {
                uint32_t restart_ms = hw->restart_ms;
                add_attr(&req.nh, sizeof(req), IFLA_CAN_RESTART_MS, &restart_ms, sizeof(restart_ms));
            }

            /* Control Mode Flags */
            struct can_ctrlmode cm;
            memset(&cm, 0, sizeof(cm));
            if (hw->loopback) {
                cm.mask |= CAN_CTRLMODE_LOOPBACK;
                cm.flags |= CAN_CTRLMODE_LOOPBACK;
            }
            if (hw->listen_only) {
                cm.mask |= CAN_CTRLMODE_LISTENONLY;
                cm.flags |= CAN_CTRLMODE_LISTENONLY;
            }
            if (hw->triple_sampling) {
                cm.mask |= CAN_CTRLMODE_3SAMPLES;
                cm.flags |= CAN_CTRLMODE_3SAMPLES;
            }
            if (cm.mask != 0) {
                add_attr(&req.nh, sizeof(req), IFLA_CAN_CTRLMODE, &cm, sizeof(cm));
            }

            end_nest_attr(&req.nh, infodata);
        }
        end_nest_attr(&req.nh, linkinfo);
    }

    int ret = send_nl_req(nl_fd, &req, logger);
    if (ret != 0) {
        plugin_logger_warn(logger, "Failed to apply CAN bitrate/timing on %s via Netlink (may require root or virtual interface)", hw->interface);
    } else {
        plugin_logger_info(logger, "Successfully applied Netlink CAN timing to %s", hw->interface);
    }

    /* Step 3: Bring link UP if auto_bringup is true */
    if (hw->auto_bringup) {
        memset(&req, 0, sizeof(req));
        req.nh.nlmsg_len = NLMSG_LENGTH(sizeof(struct ifinfomsg));
        req.nh.nlmsg_type = RTM_NEWLINK;
        req.nh.nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK;
        req.nh.nlmsg_seq = 3;

        req.ifi.ifi_family = AF_UNSPEC;
        req.ifi.ifi_index = ifindex;
        req.ifi.ifi_flags = IFF_UP;
        req.ifi.ifi_change = IFF_UP;

        if (send_nl_req(nl_fd, &req, logger) == 0) {
            plugin_logger_info(logger, "Brought CAN interface %s UP", hw->interface);
        } else {
            plugin_logger_warn(logger, "Failed to bring CAN interface %s UP via Netlink", hw->interface);
        }
    }

    close(nl_fd);
    return 0;
}

#else

int can_netlink_down(const char *ifname, plugin_logger_t *logger)
{
    (void)ifname;
    (void)logger;
    return 0;
}

int can_netlink_configure_and_up(const can_hardware_config_t *hw, plugin_logger_t *logger)
{
    (void)hw;
    (void)logger;
    return 0;
}

#endif
