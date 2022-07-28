/*
 * Copyright (C) 2022, HENSOLDT Cyber GmbH
 *
 * @defgroup FCAdapter component
 * @{
 *
 * @file
 *
 * @brief FCAdapter interface
 *
 */
#pragma once

#include <stdint.h>

#include "if_FCAdapter.h"
#include "mavlink/minimal/mavlink.h"

#include "OS_Socket.h"

/**
 * @brief Initialize FCAdapter
 *
 * Starts listening to the specified socket adress via UDP.
 *
 * @param[in] rpc Handle of the CAmkES FCAdapter interface.
 * @param[in] addr Socket address to bind to.
 * @param[in] mavChannel MAVLink channel to read messages from.
 *
 * @return An error code
 * @retval OS_SUCCESS                   Operation was successful.
 * @retval OS_ERROR_ABORTED             If the Network Stack has
 *                                      experienced a fatal error.
 * @retval OS_ERROR_NETWORK_UNREACHABLE If the network is unreachable.
 * @retval OS_ERROR_INSUFFICIENT_SPACE  If no free sockets could be
 *                                      found.
 * @retval OS_ERROR_IO                  If the specified address can
 *                                      not be found.
 */
OS_Error_t
FCAdapter_init(
    const if_FCAdapter_t* rpc,
    const OS_Socket_Addr_t* addr,
    const uint8_t mavChannel);

/**
 * @brief Read one MAVLink message.
 *
 * @param[in] rpc Handle of the CAmkES FCAdapter interface.
 * @param[out] message Received MAVLink message
 *
 * @return An error code
 * @retval OS_SUCCESS                     Operation was successful.
 * @retval OS_ERROR_ABORTED               If the Network Stack has
 *                                        experienced a fatal error.
 * @retval OS_ERROR_NOT_INITIALIZED       If the function was called before
 *                                        the Network Stack was fully
 *                                        initialized.
 * @retval OS_ERROR_CONNECTION_CLOSED     If no further communication is
 *                                        possible on the socket.
 * @retval OS_ERROR_NETWORK_CONN_SHUTDOWN If the connection got shut down.
 */
OS_Error_t
FCAdapter_recvMessage(
    const if_FCAdapter_t* rpc,
    mavlink_message_t* message);

/**
 * @brief Send one MAVLink message.
 *
 * @param[in] rpc Handle of the CAmkES FCAdapter interface.
 * @param[out] message Received MAVLink message
 *
 * @return An error code
 * @retval OS_SUCCESS                          Operation was successful.
 * @retval OS_ERROR_ABORTED                    If the Network Stack has
 *                                             experienced a fatal error.
 * @retval OS_ERROR_NOT_INITIALIZED            If the function was called before
 *                                             the Network Stack was fully
 *                                             initialized.
 * @retval OS_ERROR_INSUFFICIENT_SPACE         If there is not enough space.
 * @retval OS_ERROR_CONNECTION_CLOSED          If no further communication is
 *                                             possible on the socket.
 * @retval OS_ERROR_NETWORK_ADDR_NOT_AVAILABLE If the address is not available.
 * @retval OS_ERROR_NETWORK_HOST_UNREACHABLE   If the host is not unreachable.
 */
OS_Error_t
FCAdapter_sendMessage(
    const if_FCAdapter_t* rpc,
    const mavlink_message_t* message);

/** @} */
