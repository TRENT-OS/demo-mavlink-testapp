cmake_minimum_required(VERSION 3.7.2)


find_package("os-sdk" REQUIRED)
os_sdk_set_defaults()
os_sdk_setup(CONFIG_FILE "system_config.h" CONFIG_PROJECT "system_config")

project(demo_exercise C)

# Network stack
#include("plat/${PLATFORM}/plat_nic.cmake")
CAmkESAddCPPInclude("plat/${PLATFORM}")

NetworkStack_PicoTcp_DeclareCAmkESComponent(
  NetworkStack_PicoTcp
  C_FLAGS
    -DNetworkStack_PicoTcp_USE_HARDCODED_IPADDR
    -DDEV_ADDR=ETH_ADDR
    -DGATEWAY_ADDR=ETH_GATEWAY_ADDR
    -DSUBNET_MASK=ETH_SUBNET_MASK
)

#ChanMux Nic
NIC_ChanMux_DeclareCAmkESComponent(
    NwDriver
    CHANMUX_CHANNEL_NIC_CTRL
    CHANMUX_CHANNEL_NIC_DATA
)

ChanMux_UART_DeclareCAmkESComponents(
    ChanMux_UART
    components/ChanMux/ChanMux_config.c
    system_config
)


# Time server
TimeServer_DeclareCAmkESComponent(
  TimeServer
)

DeclareCAmkESComponent(
  TestApp
  INCLUDES
    include/mavlink
    include/mavlink/common
  SOURCES
    components/TestApp/src/TestApp.c
  C_FLAGS
    -Wall
    -Werror
  LIBS
    system_config
    lib_debug
    os_core_api
    os_filesystem
    os_socket_client
)

os_sdk_create_CAmkES_system("demo_exercise.camkes")
