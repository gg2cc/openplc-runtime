/**
 * @file can_socket.h
 * @brief SocketCAN wrapper header for reading and writing CAN raw frames
 */

#ifndef CAN_SOCKET_H
#define CAN_SOCKET_H

#include <stdbool.h>
#include <stdint.h>
#include "plugin_logger.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Open and bind a SocketCAN raw socket to a named CAN interface
 *
 * @param ifname CAN interface name (e.g. "can0")
 * @param logger Pointer to initialized plugin logger
 * @return File descriptor >= 0 on success, negative value on error
 */
int can_socket_open(const char *ifname, plugin_logger_t *logger);

/**
 * @brief Read a single CAN frame from SocketCAN
 *
 * @param fd Open SocketCAN file descriptor
 * @param can_id Output CAN ID
 * @param eff Output flag indicating if Extended Frame Format (29-bit)
 * @param rtr Output flag indicating if Remote Transmission Request
 * @param dlc Output Data Length Code (0..8)
 * @param payload Buffer (at least 8 bytes) to receive CAN payload
 * @return 0 on success, negative value on timeout or error
 */
int can_socket_read(int fd, uint32_t *can_id, bool *eff, bool *rtr, uint8_t *dlc, uint8_t *payload);

/**
 * @brief Write a single CAN frame to SocketCAN
 *
 * @param fd Open SocketCAN file descriptor
 * @param can_id CAN ID to transmit
 * @param eff True if Extended Frame Format (29-bit)
 * @param rtr True if Remote Transmission Request
 * @param dlc Data Length Code (0..8)
 * @param payload Buffer containing CAN payload
 * @return 0 on success, negative value on error
 */
int can_socket_write(int fd, uint32_t can_id, bool eff, bool rtr, uint8_t dlc, const uint8_t *payload);

/**
 * @brief Close SocketCAN file descriptor
 *
 * @param fd SocketCAN file descriptor
 */
void can_socket_close(int fd);

#ifdef __cplusplus
}
#endif

#endif /* CAN_SOCKET_H */
