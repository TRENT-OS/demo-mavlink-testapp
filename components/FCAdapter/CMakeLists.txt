#
# FCAdapter
#
# Copyright (C) 2022, HENSOLDT Cyber GmbH
#

cmake_minimum_required(VERSION 3.17)

function(FCAdapter_DeclareCAmkESComponent name)

    DeclareCAmkESComponent(
        ${name}
        INCLUDES
        SOURCES
            ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/server/src/FCAdapter.c
        C_FLAGS
            -Wall
            -Werror
        LIBS
            lib_debug
            lib_macros
            lib_mavlink
            os_core_api
            os_socket_client
    )

endfunction()

#-------------------------------------------------------------------------------
# Create the corresponding client project, which every client of this FCAdapter
# component instance must include.
project(FCAdapter_client C)

add_library(${PROJECT_NAME} INTERFACE)

target_sources(${PROJECT_NAME}
    INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/client/src/FCAdapter_client.c
)

target_include_directories(${PROJECT_NAME}
    INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/client/include
)

