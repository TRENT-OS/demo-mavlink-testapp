#
# Copyright (C) 2022-2024, HENSOLDT Cyber GmbH
# 
# SPDX-License-Identifier: GPL-2.0-or-later
#
# For commercial licensing, contact: info.cyber@hensoldt.net
#

cmake_minimum_required(VERSION 3.7.2)

project(demo_exercise C)

#-------------------------------------------------------------------------------
# use the SDK
find_package(os-sdk REQUIRED)
os_sdk_set_defaults()
os_sdk_setup(CONFIG_FILE "system_config.h" CONFIG_PROJECT "system_config")
set(LibUtilsDefaultZfLogLevel 2 CACHE STRING "")

# Kernel settings
set(KernelArmHypervisorSupport ON CACHE BOOL "" FORCE)
set(KernelArmVtimerUpdateVOffset OFF CACHE BOOL "" FORCE)
set(KernelArmDisableWFIWFETraps ON CACHE BOOL "" FORCE)
set(LibUSB OFF CACHE BOOL "" FORCE)
set(VmInitRdFile ON CACHE BOOL "" FORCE)
#set(VmVirtioConsole ON CACHE BOOL "" FORCE)
set(CapDLLoaderMaxObjects 8192 CACHE STRING "" FORCE)
set(VmVirtioNetVirtqueue ON CACHE BOOL "" FORCE)
set(VmPCISupport ON CACHE BOOL "" FORCE)

include("plat/${PLATFORM}/plat.cmake")

# os_sdk_setup()
os_sdk_import_camkes_vm()
os_sdk_import_from_global_components(
    "components/FileServer"
    "components/VirtQueue"
)

CAmkESAddCPPInclude("plat/${PLATFORM}")


#-------------------------------------------------------------------------------
# config
AddCamkesCPPFlag(cpp_flags CONFIG_VARS VmEmmc2NoDMA)

if(NOT KernelPlatformQEMUArmVirt)
    AddCamkesCPPFlag(cpp_flags CONFIG_VARS VmVUSB)
endif()

if(KernelPlatformTK1)
    AddCamkesCPPFlag(cpp_flags CONFIG_VARS Tk1DeviceFwd Tk1Insecure)
endif()


#-------------------------------------------------------------------------------
# VM Images

list(
    APPEND CMAKE_MODULE_PATH
    "${CMAKE_CURRENT_LIST_DIR}/camkes-vm-images"
    "${CMAKE_CURRENT_LIST_DIR}/camkes-vm-linux"
)
find_package(camkes-vm-images REQUIRED)
find_package(camkes-vm-linux REQUIRED)
include("${CAMKES_VM_LINUX_HELPERS_PATH}")

# images for the selected platform
set(VM_IMG_LINUX "${CAMKES_VM_IMAGES_DIR}/${PLATFORM}/linux")
set(VM_IMG_ROOTFS "${CAMKES_VM_IMAGES_DIR}/${PLATFORM}/rootfs.cpio.gz")
# set(VM_IMG_DTB "${CAMKES_VM_IMAGES_DIR}/${PLATFORM}/linux-dtb")


# patch root-fs
AddFileToOverlayDir(
    "interfaces"
    "${CMAKE_CURRENT_SOURCE_DIR}/overlay_files/network_interfaces/interfaces"
    "etc/network"
    overlay
)
AddFileToOverlayDir(
    "S90ping_test"
    "${CMAKE_CURRENT_SOURCE_DIR}/overlay_files/init_scripts/ping_test"
    "etc/init.d"
    overlay
)
AddFileToOverlayDir(
    "repeater"
    "${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/udp-repeater/bin/repeater"
    "opt"
    overlay
)
AddFileToOverlayDir(
    "rules.json"
    "${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/udp-repeater/conf/rules.json"
    "opt"
    overlay
)

AddOverlayDirToRootfs(
    overlay
    "${CAMKES_VM_IMAGES_DIR}/${PLATFORM}/rootfs.cpio.gz"
    "buildroot"
    "rootfs_install"
    VM_IMG_ROOTFS_PATCHED
    rootfs_target
    GZIP
)
# AddToFileServer("linux-initrd" "${VM_IMG_ROOTFS_PATCHED}")

# UpdateDtbFromInitrd(
#     "${CAMKES_VM_IMAGES_DIR}/${PLATFORM}/linux-dtb"
#     "${VM_IMG_ROOTFS_PATCHED}"
#     "0x4d700000"
#     target_dtb_gen
#     DTB_FOR_VM
# )
#AddToFileServer("linux-dtb" "${DTB_FOR_VM}" DEPENDS target_dtb_gen)

DefineCAmkESVMFileServer(
    INSTANCE "fileServer"
    FILES
        "linux:${VM_IMG_LINUX}"
        "linux-initrd:${VM_IMG_ROOTFS_PATCHED}"
)


# DefineCAmkESVMFileServer(
#     INSTANCE "fileServer"
#     FILES
#         "linux:${VM_IMG_LINUX}"
#         "linux-initrd:${VM_IMG_ROOTFS_PATCHED}"
#         #        "linux-dtb:${DTB_FOR_VM}"
#     DEPENDS
#         rootfs_target
#         target_dtb_gen
# )


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
    # ignore MAVLink errors according to https://mavlink.io/en/mavgen_c/#build-warnings
    -Wno-address-of-packed-member
    -Wno-cast-align
  LIBS
    system_config
    lib_debug
    os_core_api
    os_filesystem
    os_socket_client
)

DeclareCAmkESComponent(
    PingClient
    INCLUDES
        # no include paths needed
    SOURCES
        components/PingClient/ping_client.c
    C_FLAGS
        -Wall
        -Werror
    LIBS
        virtqueue
        sel4_autoconf
)


#-------------------------------------------------------------------------------
# Make repeater

add_custom_command(OUTPUT ${CMAKE_CURRENT_LIST_DIR}/3rdParty/udp-repeater/bin/repeater
                   COMMAND make clean && make
                   WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/3rdParty/udp-repeater/src)

# ExtendCAmkESComponentInstance(
#     FileServer
#     fileServer
#     LIBS
#         fileserver_cpio
# )


# CAmkESAddCPPInclude("${SDK_SEL4_CAMKES_DIR}/libs/sel4_projects_libs/libsel4vm/include/sel4vm")

#-------------------------------------------------------------------------------

os_sdk_create_CAmkES_system("demo_exercise.camkes")
