/*
 * Copyright (C) 2022, HENSOLDT Cyber GmbH
 */

#pragma once

#include <stdint.h>

#include "mavlink/minimal/mavlink.h"

#include "OS_Error.h"
#include "OS_Socket.h"

typedef struct
{
    OS_Error_t (*init)(const OS_Socket_Addr_t* addr, uint8_t mavChannel);
    OS_Error_t (*recvMessage)(mavlink_message_t* message);
    OS_Error_t (*sendMessage)(const mavlink_message_t* message);
} if_FCAdapter_t;

#define IF_FCADAPTER_ASSIGN(_prefix_)            \
  {                                              \
    .init = _prefix_##_rpc_init,                 \
    .recvMessage = _prefix_##_rpc_recvMessage, \
    .sendMessage = _prefix_##_rpc_sendMessage, \
  }
