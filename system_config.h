#ifndef SYSTEM_CONFIG_H_
#define SYSTEM_CONFIG_H_

// Debug
#if !defined(NDEBUG)
#define Debug_Config_STANDARD_ASSERT
#define Debug_Config_ASSERT_SELF_PTR
#else
#define Debug_Config_DISABLE_ASSERT
#define Debug_Config_NO_ASSERT_SELF_PTR
#endif

#define Debug_Config_LOG_LEVEL Debug_LOG_LEVEL_TRACE
#define Debug_Config_INCLUDE_LEVEL_IN_MSG
#define Debug_Config_LOG_WITH_FILE_LINE

// Memory
#define Memory_Config_USE_STDLIB_ALLOC

//-----------------------------------------------------------------------------
// ChanMUX
//-----------------------------------------------------------------------------

#define CHANMUX_CHANNEL_NIC_CTRL      4
#define CHANMUX_CHANNEL_NIC_DATA      5
#define CHANMUX_CHANNEL_NVM           6

//-----------------------------------------------------------------------------
// ChanMUX clients
//-----------------------------------------------------------------------------

#define CHANMUX_ID_NIC        101

// NIC driver
#define NIC_DRIVER_RINGBUFFER_NUMBER_ELEMENTS 16
#define NIC_DRIVER_RINGBUFFER_SIZE                                             \
  (NIC_DRIVER_RINGBUFFER_NUMBER_ELEMENTS * 4096)

// Network stack
#define OS_NETWORK_MAXIMUM_SOCKET_NO 16
#define ETH_ADDR "10.0.0.11"
#define ETH_GATEWAY_ADDR "10.0.0.1"
#define ETH_SUBNET_MASK "255.255.255.0"
#define MAVLINK_LISTEN_PORT 14540

#define MAVLINK_CHANNEL MAVLINK_COMM_0
#define MAVLINK_THIS_SYSTEM_ID 255
#define MAVLINK_THIS_COMPONENT_ID 190

#endif // SYSTEM_CONFIG_H_
