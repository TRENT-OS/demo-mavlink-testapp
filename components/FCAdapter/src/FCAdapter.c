/*
 * Copyright (C) 2022, HENSOLDT Cyber GmbH
 */

#include <camkes.h>

#include "mavlink/common/mavlink.h"
#include "lib_debug/Debug.h"

#include "OS_Error.h"
#include "OS_Socket.h"

static const if_OS_Socket_t networkStackCtx = IF_OS_SOCKET_ASSIGN(networkStack);

typedef struct
{
    OS_Socket_Handle_t hSocket;
    OS_Socket_Addr_t dstAddr;
    uint8_t mavChannel;
    mavlink_status_t mavStatus;
    // The index of the next byte to decode.
    size_t bufIdx;
    // The number of bytes inside the buffer.
    size_t bufLen;
    uint8_t buf[4096];
} FCAdapter_t;

static FCAdapter_t ctx = {0};

OS_Error_t
fcAdapter_rpc_init(
    const OS_Socket_Addr_t* const addr,
    const uint8_t mavChannel)
{
    OS_Error_t err;

    do
    {
        err = OS_Socket_create(&networkStackCtx, &ctx.hSocket, OS_AF_INET,
                               OS_SOCK_DGRAM);
    }
    while (err == OS_ERROR_NOT_INITIALIZED);
    if (err != OS_SUCCESS)
    {
        Debug_LOG_ERROR("`OS_Socket_create` failed. Error code: %d", err);
        return err;
    }

    err = OS_Socket_bind(ctx.hSocket, addr);
    if (err != OS_SUCCESS)
    {
        Debug_LOG_ERROR("`OS_Socket_bind` failed. Error code: %d", err);
        return err;
    }

    return OS_SUCCESS;
}

OS_Error_t
fcAdapter_rpc_recvMessage(
    mavlink_message_t* const message)
{
    OS_Error_t err;

    for (;;)
    {
        if (ctx.bufIdx < ctx.bufLen)
        {
            // Enough bytes in buffer
            bool good = mavlink_parse_char(
                            ctx.mavChannel,
                            ctx.buf[ctx.bufIdx],
                            message,
                            &ctx.mavStatus);
            ++ctx.bufIdx;
            if (good)
            {
                return OS_SUCCESS;
            }
        }
        else
        {
            // No bytes remaining in buffer
            seL4_Yield();
            do
            {
                err = OS_Socket_recvfrom(
                          ctx.hSocket,
                          &ctx.buf,
                          sizeof(ctx.buf),
                          &ctx.bufLen,
                          &ctx.dstAddr);
            }
            while (err == OS_ERROR_TRY_AGAIN);
            if (err != OS_SUCCESS)
            {
                Debug_LOG_ERROR("`OS_Socket_recvfrom` failed. Error code: %d", err);
                return err;
            }
        }
    }
}

OS_Error_t
fcAdapter_rpc_sendMessage(
    const mavlink_message_t* const message)
{
    OS_Error_t err;

    // Maximum mavlink message length.
    uint8_t buf[279];
    const size_t bufLen = mavlink_msg_to_send_buffer(buf, message);
    size_t bufIdx = 0;
    while (bufIdx < bufLen)
    {
        size_t written;
        err = OS_Socket_sendto(
                  ctx.hSocket,
                  &buf[bufIdx],
                  bufLen - bufIdx,
                  &written,
                  &ctx.dstAddr);
        if (err == OS_ERROR_TRY_AGAIN)
        {
            continue;
        }
        if (err != OS_SUCCESS)
        {
            Debug_LOG_ERROR("`OS_Socket_write` failed. Error code: %d", err);
            return err;
        }

        bufIdx += written;
    }

    return OS_SUCCESS;
}

int run(
    void)
{
    Debug_LOG_INFO("[FCAdapter '%s'] starting", get_instance_name());

    return 0;
}
