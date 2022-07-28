/*
 * Copyright (C) 2022, HENSOLDT Cyber GmbH
 */

#include "FCAdapter_client.h"
#include "lib_macros/Check.h"

OS_Error_t
FCAdapter_init(
    const if_FCAdapter_t* rpc,
    const OS_Socket_Addr_t* addr,
    const uint8_t mavChannel)
{
    CHECK_PTR_NOT_NULL(rpc);
    CHECK_PTR_NOT_NULL(rpc->init);
    return rpc->init(addr, mavChannel);
}

OS_Error_t
FCAdapter_recvMessage(
    const if_FCAdapter_t* rpc,
    mavlink_message_t* message)
{
    CHECK_PTR_NOT_NULL(rpc);
    CHECK_PTR_NOT_NULL(rpc->recvMessage);
    return rpc->recvMessage(message);
}

OS_Error_t
FCAdapter_sendMessage(
    const if_FCAdapter_t* rpc,
    const mavlink_message_t* const message)
{
    CHECK_PTR_NOT_NULL(rpc);
    CHECK_PTR_NOT_NULL(rpc->sendMessage);
    return rpc->sendMessage(message);
}
