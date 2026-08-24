/**
 * @file can_netlink.h
 * @brief Netlink helper interface for configuring CAN hardware bitrate, SJW, sample point, and
 * restart-ms
 */

#ifndef CAN_NETLINK_H
#define CAN_NETLINK_H

#include "can_config.h"
#include "plugin_logger.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Configure CAN interface parameters (bitrate, sjw, sample_point, restart_ms, etc.) and
     * bring up link
     *
     * @param hw Pointer to CAN hardware configuration
     * @param logger Pointer to initialized plugin logger
     * @return 0 on success, negative on failure
     */
    int can_netlink_configure_and_up(const can_hardware_config_t *hw, plugin_logger_t *logger);

    /**
     * @brief Bring down CAN interface
     *
     * @param ifname Interface name (e.g. "can0")
     * @param logger Pointer to initialized plugin logger
     * @return 0 on success, negative on failure
     */
    int can_netlink_down(const char *ifname, plugin_logger_t *logger);

#ifdef __cplusplus
}
#endif

#endif /* CAN_NETLINK_H */
