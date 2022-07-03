cmake_minimum_required(VERSION 3.7.2)

os_set_config_file(
  "system_config.h"
  CONFIG_PROJECT_NAME "system_config"
)

project(demo_exercise C)

# Network stack
include("plat/${PLATFORM}/plat_nic.cmake")

NetworkStack_PicoTcp_DeclareCAmkESComponent(
  NetworkStack_PicoTcp
  C_FLAGS
    -DNetworkStack_PicoTcp_USE_HARDCODED_IPADDR
    -DDEV_ADDR=ETH_ADDR
    -DGATEWAY_ADDR=ETH_GATEWAY_ADDR
    -DSUBNET_MASK=ETH_SUBNET_MASK
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

DeclareAndCreateCAmkESSystem(demo_exercise.camkes)
GenerateSimulateScript()
