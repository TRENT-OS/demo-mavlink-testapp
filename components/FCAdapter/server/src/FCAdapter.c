/*
 * Copyright (C) 2022, HENSOLDT Cyber GmbH
 */

#include <camkes.h>

#include "mavlink/minimal/mavlink.h"
#include "lib_debug/Debug.h"

#include "OS_Error.h"
#include "OS_Socket.h"

#define GET_CLIENT(cli, cid) \
    do { \
        if ((cli = getClient(cid)) == NULL) \
        { \
            Debug_LOG_ERROR("Could not get state for client with clientId %u, " \
                            "the badge number is most likely not properly " \
                            "configured", cid); \
            return OS_ERROR_NOT_FOUND; \
        } \
    } while(0)

// In its current implementation, the component supports a maximum of 4 clients
// to be connected to it.
#define MAX_CLIENTS_NUM 4

// The FCAdapter_CLIENT_ASSIGN_BADGES() macro will start assigning badge
// numbers with the minimum value below. Adjusting the value below will also
// require an adaptation in the main CAmkES file of this component for the
// above mentioned macro.
#define MIN_BADGE_ID 101

typedef struct
{
    bool inUse;
    unsigned int clientId;
    OS_Socket_Handle_t hSocket;
    OS_Socket_Addr_t dstAddr;
    OS_Error_t error;
    uint8_t mavChannel;
    mavlink_status_t mavStatus;
    // The index of the next byte to decode.
    size_t bufIdx;
    // The number of bytes inside the buffer.
    size_t bufLen;
    uint8_t buf[4096];
} FCAdapter_Client_t;

static bool initializationComplete = false;

static FCAdapter_Client_t clients[MAX_CLIENTS_NUM] = {0};

static const if_OS_Socket_t networkStackCtx = IF_OS_SOCKET_ASSIGN(networkStack);

seL4_Word fcAdapter_rpc_get_sender_id(
    void);

static int
get_client_id(
    void)
{
    return fcAdapter_rpc_get_sender_id();
}

static FCAdapter_Client_t*
getClient(
    const int clientId)
{
    for (int i = 0; i < MAX_CLIENTS_NUM; i++)
    {
        if ((clients[i].inUse) &&
            (clients[i].clientId == clientId))
        {
            return &clients[i];
        }
    }

    return NULL;
}

// static FCAdapter_Client_t*
// getClientBySocketHandle(
//     int socketHandle)
// {
//     for (int i = 0; i < MAX_CLIENTS_NUM; i++)
//     {
//         if ((clients[i].inUse) &&
//             (clients[i].hSocket.handleID == socketHandle))
//         {
//             return &clients[i];
//         }
//     }

//     return NULL;
// }

static OS_Error_t
initializeClients(
    void)
{
    const unsigned int numberConnectedClients = fcAdapter_rpc_num_badges();

    if (MAX_CLIENTS_NUM < numberConnectedClients)
    {
        Debug_LOG_ERROR(
            "[FCAdapter '%s'] is configured for %d clients, but %d clients are"
            " connected",
            get_instance_name(),
            MAX_CLIENTS_NUM,
            numberConnectedClients);
        return OS_ERROR_OUT_OF_BOUNDS;
    }

    for (int i = 0; i < numberConnectedClients; i++)
    {
        const seL4_Word clientId = fcAdapter_rpc_enumerate_badge(i);

        Debug_LOG_DEBUG(
            "[FCAdapter '%s'] clientId (%d): %d, Min: %d, Max: %d",
            get_instance_name(),
            i,
            clientId,
            MIN_BADGE_ID,
            MIN_BADGE_ID + numberConnectedClients - 1);

        if ((clientId < MIN_BADGE_ID) ||
            (clientId >= MIN_BADGE_ID +
             numberConnectedClients))
        {
            Debug_LOG_ERROR(
                "[FCAdapter '%s'] Badge Id is out of bounds: %d, Min: %d,"
                " Max: %d",
                get_instance_name(),
                (int)clientId,
                MIN_BADGE_ID,
                MIN_BADGE_ID + numberConnectedClients - 1);
            return OS_ERROR_OUT_OF_BOUNDS;
        }

        clients[i].inUse = true;
        clients[i].clientId = MIN_BADGE_ID + i;
    }

    return OS_SUCCESS;
}

OS_Error_t
fcAdapter_rpc_init(
    const OS_Socket_Addr_t* const addr,
    const uint8_t mavChannel)
{
    OS_Error_t err;
    FCAdapter_Client_t* client;
    GET_CLIENT(client, get_client_id());

    do
    {
        err = OS_Socket_create(&networkStackCtx, &client->hSocket, OS_AF_INET,
                               OS_SOCK_DGRAM);
    }
    while (err == OS_ERROR_NOT_INITIALIZED);
    if (err != OS_SUCCESS)
    {
        Debug_LOG_ERROR("`OS_Socket_create` failed. Error code: %d", err);
        return err;
    }

    err = OS_Socket_bind(client->hSocket, addr);
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
    FCAdapter_Client_t* client;
    GET_CLIENT(client, get_client_id());

    for (;;)
    {
        if (client->bufIdx < client->bufLen)
        {
            // Enough bytes in buffer
            bool good = mavlink_parse_char(
                            client->mavChannel,
                            client->buf[client->bufIdx],
                            message,
                            &client->mavStatus);
            ++client->bufIdx;
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
                          client->hSocket,
                          &client->buf,
                          sizeof(client->buf),
                          &client->bufLen,
                          &client->dstAddr);
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
    FCAdapter_Client_t* client;
    GET_CLIENT(client, get_client_id());

    // Maximum mavlink message length.
    uint8_t buf[279];
    const size_t bufLen = mavlink_msg_to_send_buffer(buf, message);
    size_t bufIdx = 0;
    while (bufIdx < bufLen)
    {
        size_t written;
        err = OS_Socket_sendto(
                  client->hSocket,
                  &buf[bufIdx],
                  bufLen - bufIdx,
                  &written,
                  &client->dstAddr);
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

void
post_init()
{
    OS_Error_t err = initializeClients();
    if (err == OS_SUCCESS)
    {
        initializationComplete = true;
    }
}

int run(
    void)
{
    Debug_LOG_INFO("[FCAdapter '%s'] starting", get_instance_name());
    if (!initializationComplete)
    {
        Debug_LOG_ERROR("FCAdapter initialization failed, stopping");
        return -1;
    }

    return 0;
}
