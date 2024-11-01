/*
 * Copyright (C) 2022-2024, HENSOLDT Cyber GmbH
 * 
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * For commercial licensing, contact: info.cyber@hensoldt.net
 */

#include <stdlib.h>
#include <string.h>

#include "OS_Error.h"
#include "OS_Socket.h"
#include "common/mavlink.h"
#include "interfaces/if_OS_Socket.h"
#include "lib_debug/Debug.h"
#include "mavlink_msg_actuator_output_status.h"
#include "mavlink_msg_command_long.h"
#include "mavlink_msg_param_set.h"
#include "mavlink_msg_param_value.h"
#include "minimal/mavlink_msg_heartbeat.h"
#include "network/OS_NetworkTypes.h"
#include "network/OS_SocketTypes.h"
#include "system_config.h"
#include <camkes.h>

static const if_OS_Socket_t networkStackCtx = IF_OS_SOCKET_ASSIGN(networkStack);

typedef struct
{
	void (*wait_for_ready)(void);
} if_vm0_t;

static const if_vm0_t vm0 = {.wait_for_ready = pingready_wait,};

/**
 * Waits until the network stack is initialized.
 */
static OS_Error_t waitForNetworkStackInit(const if_OS_Socket_t *const ctx) {
  for (;;) {
    OS_NetworkStack_State_t state = OS_Socket_getStatus(ctx);
    if (state == RUNNING) {
      return OS_SUCCESS;
    } else if (state == FATAL_ERROR) {
      Debug_LOG_ERROR("A fatal error occured in the Network Stack component.");
      return OS_ERROR_ABORTED;
    }
    seL4_Yield();
  }
}

/**
 * Waits until a connection could successfully be established.
 */
static OS_Error_t waitForIncomingConnection(const if_OS_Socket_t *const ctx,
                                            const int serverHandleId) {
  OS_Error_t ret;
  for (;;) {
    ret = OS_Socket_wait(ctx);
    if (ret != OS_SUCCESS) {
      Debug_LOG_ERROR(
          "{Server Handle: %d} `OS_Socket_wait` failed. Error code: %d",
          serverHandleId, ret);
      return ret;
    }

    OS_Socket_Evt_t event;
    int numberOfEvents;
    ret =
        OS_Socket_getPendingEvents(ctx, &event, sizeof(event), &numberOfEvents);
    if (ret != OS_SUCCESS) {
      Debug_LOG_ERROR("{Server Handle: %d} `OS_Socket_getPendingEvents` "
                      "failed. Error code: %d",
                      serverHandleId, ret);
      return ret;
    }
    if (numberOfEvents == 0) {
      Debug_LOG_TRACE("{Server Handle: %d} `OS_Socket_getPendingEvents` did "
                      "not return any pending events.",
                      serverHandleId);
      continue;
    }
    if (numberOfEvents != 1) {
      Debug_LOG_ERROR(
          "{Server Handle: %d} `OS_Socket_getPendingEvents` returned with an "
          "unexpected number of events (actual: %d, expected 1).",
          serverHandleId, numberOfEvents);
      return OS_ERROR_INVALID_STATE;
    }
    if (event.socketHandle != serverHandleId) {
      Debug_LOG_ERROR("{Server Handle: %d} Received unexpected handle (actual: "
                      "%d, expected: %d)",
                      serverHandleId, event.socketHandle, serverHandleId);
      return OS_ERROR_INVALID_HANDLE;
    }
    if (event.eventMask & OS_SOCK_EV_ERROR) {
      Debug_LOG_ERROR(
          "{Server Handle: %d} Connection failed to establish. Error code: %d",
          serverHandleId, event.currentError);
      return event.currentError;
    }
    if (event.eventMask & OS_SOCK_EV_FIN) {
      Debug_LOG_ERROR(
          "{Server Handle: %d} Connection has unexpectedly finished.",
          serverHandleId);
      return OS_ERROR_NETWORK_CONN_REFUSED;
    }
    if (event.eventMask & OS_SOCK_EV_CLOSE) {
      Debug_LOG_ERROR("{Server Handle: %d} Connection was unexpectedly closed.",
                      serverHandleId);
      return OS_ERROR_CONNECTION_CLOSED;
    }
    if (event.eventMask & OS_SOCK_EV_CONN_ACPT) {
      Debug_LOG_DEBUG("{Server Handle: %d} Connection established.",
                      serverHandleId);
      return OS_SUCCESS;
    }
  }
}

/**
 * Binds the server and begins listening.
 */
static OS_Error_t initializeServer(const if_OS_Socket_t *const ctx,
                                   OS_Socket_Handle_t *const server) {
  OS_Error_t ret;
  ret = OS_Socket_create(ctx, server, OS_AF_INET, OS_SOCK_STREAM);
  if (ret != OS_SUCCESS) {
    Debug_LOG_ERROR("`OS_Socket_create` failed. Error code: %d", ret);
    return ret;
  }

  const OS_Socket_Addr_t dstAddr = {
      .addr = OS_INADDR_ANY_STR,
      .port = MAVLINK_LISTEN_PORT,
  };
  ret = OS_Socket_bind(*server, &dstAddr);
  if (ret != OS_SUCCESS) {
    Debug_LOG_ERROR("`OS_Socket_bind` failed. Error code: %d", ret);
    OS_Socket_close(*server);
    return ret;
  }

  ret = OS_Socket_listen(*server, 1);
  if (ret != OS_SUCCESS) {
    Debug_LOG_ERROR("`OS_Socket_listen` failed. Error code: %d", ret);
    OS_Socket_close(*server);
    return ret;
  }
  return OS_SUCCESS;
}

/**
 * Writes a MAVLINK message to a socket.
 */
static OS_Error_t sendMessage(const OS_Socket_Handle_t socket,
                              const mavlink_message_t *const message) {
  OS_Error_t ret;
  // Maximum mavlink message size.
  uint8_t buf[279];
  uint8_t *bufPtr = (uint8_t *)&buf;
  size_t bufLen = mavlink_msg_to_send_buffer(bufPtr, message);

  size_t totalWritten = 0;
  while (totalWritten < bufLen) {
    size_t written = 0;
    ret = OS_Socket_write(socket, bufPtr, bufLen, &written);
    totalWritten += written;
    bufPtr += written;

    switch (ret) {
    case OS_SUCCESS: {
      Debug_LOG_DEBUG(
          "{Socket Handle: %d} `OS_Socket_write` wrote %zu bytes of data.",
          socket.handleID, written);

      break;
    }
    case OS_ERROR_TRY_AGAIN: {
      Debug_LOG_DEBUG("{Socket Handle: %d} `OS_Socket_write` returned try again", socket.handleID);
      continue;
    }
    default: {
      Debug_LOG_ERROR(
          "{Socket Handle: %d} `OS_Socket_write` failed. Error code: %d",
          socket.handleID, ret);
      return ret;
    }
    }
  }

  return OS_SUCCESS;
}

typedef enum {
  PILOT_STATE_ARM,
  PILOT_STATE_ARM_DONE,
  PILOT_STATE_QUERY_ALTITUDE,
  PILOT_STATE_QUERY_ALTITUDE_DONE,
  PILOT_STATE_TAKEOFF,
  PILOT_STATE_TAKEOFF_DONE,
  PILOT_STATE_HOVERING,
  PILOT_STATE_FLYING,
  PILOT_STATE_FLYING_DONE,
} PilotState;

/**
 * Communicates with the flight controller via MAVLINK.
 */
static OS_Error_t handleMavLinkConnection(const OS_Socket_Handle_t socket,
                                          mavlink_channel_t channel) {
  // NOTE: This here is a mess that only demonstrates that communication
  // somewhat works. Actual software that uses MAVLink would work out an
  // actual architecture that does things correctly and ergonomically.

  static const uint32_t DEFAULT_DEBOUNCE = 50;

  OS_Error_t ret;

  size_t receiveBufferLength;
  uint8_t receiveBuffer[OS_DATAPORT_DEFAULT_SIZE];

  PilotState state = PILOT_STATE_ARM;
  uint32_t debounce = 0;
  uint8_t target_component = 0;
  uint8_t target_system = 0;
  float highest_point[3] = {NAN, NAN, NAN}; // NED relative to drone.
  float altitude = NAN;
  bool mode_set = false;

  for (;;) {
    ret = OS_Socket_read(socket, receiveBuffer, sizeof(receiveBuffer),
                         &receiveBufferLength);
    switch (ret) {
    case OS_SUCCESS: {
      Debug_LOG_DEBUG(
          "{Socket Handle: %d} `OS_Socket_read` read %zu bytes of data.",
          socket.handleID, receiveBufferLength);
      break;
    }
    case OS_ERROR_TRY_AGAIN: {
      continue;
    }
    case OS_ERROR_CONNECTION_CLOSED:
    case OS_ERROR_NETWORK_CONN_SHUTDOWN: {
      Debug_LOG_INFO(
          "{Socket Handle: %d} `OS_Socket_read` reported connection closed.",
          socket.handleID);
      return ret;
    }
    default: {
      Debug_LOG_ERROR(
          "{Socket Handle: %d} `OS_Socket_read` failed. Error code: %d",
          socket.handleID, ret);
      return ret;
    }
    }

    mavlink_status_t status_in;
    mavlink_message_t msg_in;
    for (size_t byteIndex = 0; byteIndex < receiveBufferLength; ++byteIndex) {
      if (mavlink_parse_char(channel, receiveBuffer[byteIndex], &msg_in,
                             &status_in)) {
        Debug_LOG_DEBUG("{Socket Handle: %d} Received MAVLINK message (ID: %d, "
                        "sequence: %d) from (component: %d, system: %d)",
                        socket.handleID, msg_in.msgid, msg_in.seq,
                        msg_in.compid, msg_in.sysid);

        switch (msg_in.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
          mavlink_heartbeat_t heartbeat;
          mavlink_msg_heartbeat_decode(&msg_in, &heartbeat);
          mode_set =
              heartbeat.base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED &&
              heartbeat.custom_mode == 6; // PX4_CUSTOM_MAIN_MODE_OFFBOARD
          break;
        }
        case MAVLINK_MSG_ID_ATTITUDE:
        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: {
          if (debounce > 0) {
            debounce--;
            break;
          }
	  static int tmp = 0;
	  if(tmp <= 0)
	  {
          	pingreq_emit();
          	vm0.wait_for_ready();
		tmp = 3;
	  }
	  tmp--;
          switch (state) {
          case PILOT_STATE_ARM: {
            target_system = msg_in.sysid;
            target_component = msg_in.compid;

            mavlink_command_long_t cmd_out = {0};
            cmd_out.target_system = target_system;
            cmd_out.target_component = target_component;
            cmd_out.command = MAV_CMD_COMPONENT_ARM_DISARM;
            cmd_out.param1 = 1.0; // Arm = true
            cmd_out.param2 = 0.0; // Force Arm = false
            // cmd_out.param2 = 21196.0; // Force Arm = true

            mavlink_message_t msg_out;
            mavlink_msg_command_long_encode(MAVLINK_THIS_SYSTEM_ID,
                                            MAVLINK_THIS_COMPONENT_ID, &msg_out,
                                            &cmd_out);
            ret = sendMessage(socket, &msg_out);
            if (ret != OS_SUCCESS) {
              Debug_LOG_ERROR(
                  "{Socket Handle: %d} `sendMessage` failed. Error code: %d",
                  socket.handleID, ret);
              return ret;
            }
            state = PILOT_STATE_ARM_DONE;
            Debug_LOG_INFO("Set state to `PILOT_STATE_ARM_DONE`");
            break;
          }
          case PILOT_STATE_QUERY_ALTITUDE: {
            mavlink_command_long_t cmd_out = {0};
            cmd_out.target_system = target_system;
            cmd_out.target_component = target_component;
            cmd_out.command = MAV_CMD_REQUEST_MESSAGE;
            cmd_out.param1 = MAVLINK_MSG_ID_ALTITUDE;

            mavlink_message_t msg_out;
            mavlink_msg_command_long_encode(MAVLINK_THIS_SYSTEM_ID,
                                            MAVLINK_THIS_COMPONENT_ID, &msg_out,
                                            &cmd_out);
            ret = sendMessage(socket, &msg_out);
            if (ret != OS_SUCCESS) {
              Debug_LOG_ERROR(
                  "{Socket Handle: %d} `sendMessage` failed. Error code: %d",
                  socket.handleID, ret);
              return ret;
            }
            state = PILOT_STATE_QUERY_ALTITUDE_DONE;
            Debug_LOG_INFO("Set state to `PILOT_STATE_QUERY_ALTITUDE_DONE`");
            break;
          }
          case PILOT_STATE_TAKEOFF: {
            mavlink_command_long_t cmd_out = {0};
            cmd_out.target_system = target_system;
            cmd_out.target_component = target_component;
            cmd_out.command = MAV_CMD_NAV_TAKEOFF;
            cmd_out.param1 = 0.0;            // Pitch (rad)
            cmd_out.param4 = 0.0;            // Yaw (rad)
            cmd_out.param5 = NAN;            // Latitude (m)
            cmd_out.param6 = NAN;            // Longitude (m)
            cmd_out.param7 = altitude + 3.0; // Altitude (m)

            mavlink_message_t msg_out;
            mavlink_msg_command_long_encode(MAVLINK_THIS_SYSTEM_ID,
                                            MAVLINK_THIS_COMPONENT_ID, &msg_out,
                                            &cmd_out);
            ret = sendMessage(socket, &msg_out);
            if (ret != OS_SUCCESS) {
              Debug_LOG_ERROR(
                  "{Socket Handle: %d} `sendMessage` failed. Error code: %d",
                  socket.handleID, ret);
              return ret;
            }
            state = PILOT_STATE_TAKEOFF_DONE;
            Debug_LOG_INFO("Set state to `PILOT_STATE_TAKEOFF_DONE`");
            break;
          }
          case PILOT_STATE_FLYING: {
            {
              mavlink_set_position_target_local_ned_t setpoint = {0};
              setpoint.target_component = target_component;
              setpoint.target_system = target_system;
              setpoint.coordinate_frame = MAV_FRAME_LOCAL_NED;
              // Flag to specify that velocity, acceleration and yaw are not
              // set.
              setpoint.type_mask = 0b0000110111111000;
              setpoint.x = highest_point[0];
              setpoint.y = highest_point[1];
              setpoint.z = highest_point[2];

              mavlink_message_t msg_out;
              mavlink_msg_set_position_target_local_ned_encode(
                  MAVLINK_THIS_SYSTEM_ID, MAVLINK_THIS_COMPONENT_ID, &msg_out,
                  &setpoint);
              ret = sendMessage(socket, &msg_out);
              if (ret != OS_SUCCESS) {
                Debug_LOG_ERROR(
                    "{Socket Handle: %d} `sendMessage` failed. Error code: %d",
                    socket.handleID, ret);
                return ret;
              }
              Debug_LOG_DEBUG("Sent setpoint message.");
            }
            {
              // Need to send manual control messages so that PX4 doesn't assume
              // that we disconnected.
              mavlink_manual_control_t ctrl = {0};
              ctrl.target = target_system;
              ctrl.x = INT16_MAX;
              ctrl.y = INT16_MAX;
              ctrl.z = INT16_MAX;
              ctrl.r = INT16_MAX;

              mavlink_message_t msg_out;
              mavlink_msg_manual_control_encode(MAVLINK_THIS_SYSTEM_ID,
                                                MAVLINK_THIS_COMPONENT_ID,
                                                &msg_out, &ctrl);
              ret = sendMessage(socket, &msg_out);
              if (ret != OS_SUCCESS) {
                Debug_LOG_ERROR(
                    "{Socket Handle: %d} `sendMessage` failed. Error code: %d",
                    socket.handleID, ret);
                return ret;
              }
              Debug_LOG_DEBUG("Sent manual control message.");
            }

            if (!mode_set) {
              mavlink_command_long_t cmd_out = {0};
              cmd_out.target_system = target_system;
              cmd_out.target_component = target_component;
              cmd_out.command = MAV_CMD_DO_SET_MODE;
              cmd_out.param1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED; // Base Mode
              cmd_out.param2 =
                  6.0; // Custom Mode = PX4_CUSTOM_MAIN_MODE_OFFBOARD
              cmd_out.param3 = 0.0; // Submode

              mavlink_message_t msg_out;
              mavlink_msg_command_long_encode(MAVLINK_THIS_SYSTEM_ID,
                                              MAVLINK_THIS_COMPONENT_ID,
                                              &msg_out, &cmd_out);
              ret = sendMessage(socket, &msg_out);
              if (ret != OS_SUCCESS) {
                Debug_LOG_ERROR(
                    "{Socket Handle: %d} `sendMessage` failed. Error code: %d",
                    socket.handleID, ret);
                return ret;
              }
              Debug_LOG_DEBUG("Sent message to set mode to offboard.");
            }

            break;
          }
          default: {
            break;
          }
          }
          break;
        }
        case MAVLINK_MSG_ID_COMMAND_ACK: {
          mavlink_command_ack_t ack;
          mavlink_msg_command_ack_decode(&msg_in, &ack);
          switch (ack.result) {
          case MAV_RESULT_ACCEPTED: {
            switch (ack.command) {
            case MAV_CMD_COMPONENT_ARM_DISARM: {
              state = PILOT_STATE_QUERY_ALTITUDE;
              Debug_LOG_INFO("Set state to `PILOT_STATE_QUERY_ALTITUDE`");
              break;
            }
            case MAV_CMD_NAV_TAKEOFF: {
              state = PILOT_STATE_HOVERING;
              Debug_LOG_INFO("Set state to `PILOT_STATE_HOVERING`");
              break;
            }
            default: {
              break;
            }
            }
            break;
          }
          case MAV_RESULT_TEMPORARILY_REJECTED: {
            switch (ack.command) {
            case MAV_CMD_COMPONENT_ARM_DISARM: {
              state = PILOT_STATE_ARM;
              debounce = DEFAULT_DEBOUNCE;
              Debug_LOG_INFO("Set state to `PILOT_STATE_ARM` to try again "
                             "after rejection.");
              break;
            }
            case MAV_CMD_NAV_TAKEOFF: {
              state = PILOT_STATE_TAKEOFF;
              debounce = DEFAULT_DEBOUNCE;
              Debug_LOG_INFO("Set state to `PILOT_STATE_TAKEOFF` to try again "
                             "after rejection.");
              break;
            }
            default: {
              break;
            }
            }
            break;
          }
          case MAV_RESULT_IN_PROGRESS: {
            Debug_LOG_DEBUG("Command %d in progress.", ack.command);
            break;
          }
          case MAV_RESULT_FAILED: {
            Debug_LOG_ERROR("Command %d failed.", ack.command);
            break;
          }
          default: {
            Debug_LOG_ERROR(
                "Flight controller yielded result %d for command %d.",
                ack.result, ack.command);
            break;
          }
          }
          break;
        }
        case MAVLINK_MSG_ID_ALTITUDE: {
          mavlink_altitude_t resp_altitude;
          mavlink_msg_altitude_decode(&msg_in, &resp_altitude);
          altitude = resp_altitude.altitude_local;
          Debug_LOG_DEBUG("Altitude: %f", (double)altitude);
          if (state == PILOT_STATE_QUERY_ALTITUDE_DONE) {
            state = PILOT_STATE_TAKEOFF;
            Debug_LOG_INFO("Set state to `PILOT_STATE_TAKEOFF`");
          }
          break;
        }
        case MAVLINK_MSG_ID_PARAM_SET: {
          mavlink_param_set_t param_set;
          mavlink_msg_param_set_decode(&msg_in, &param_set);

          if (!memcmp(param_set.param_id, "HIGHEST_POINT_X", 15)) {
            highest_point[0] = param_set.param_value;
            Debug_LOG_DEBUG("Set parameter `HIGHEST_POINT_X` to %.2f",
                            param_set.param_value);
          }
          if (!memcmp(param_set.param_id, "HIGHEST_POINT_Y", 15)) {
            highest_point[1] = param_set.param_value;
            Debug_LOG_DEBUG("Set parameter `HIGHEST_POINT_Y` to %.2f",
                            param_set.param_value);
          }
          if (!memcmp(param_set.param_id, "HIGHEST_POINT_Z", 15)) {
            highest_point[2] = param_set.param_value;
            Debug_LOG_DEBUG("Set parameter `HIGHEST_POINT_Y` to %.2f",
                            param_set.param_value);
            if (state == PILOT_STATE_HOVERING) {
              state = PILOT_STATE_FLYING;
              Debug_LOG_INFO("Set state to `PILOT_STATE_FLYING`");
            }
          }

          break;
        }
        default: {
          break;
        }
        }
      }
    }
  }

  return OS_SUCCESS;
}

int run() {
  OS_Error_t ret;
  Debug_LOG_INFO("Starting TestApp.");

  Debug_LOG_INFO("wait for vm0");
  vm0.wait_for_ready();
  Debug_LOG_INFO("vm0 ready");
  ret = waitForNetworkStackInit(&networkStackCtx);
  if (ret != OS_SUCCESS) {
    Debug_LOG_ERROR("`waitForNetworkStackInit` failed. Error code: %d", ret);
    return -1;
  }
  Debug_LOG_DEBUG("Network stack initialized.");

  Debug_LOG_INFO("Initializing server.");
  OS_Socket_Handle_t server;
  ret = initializeServer(&networkStackCtx, &server);
  if (ret != OS_SUCCESS) {
    Debug_LOG_ERROR("`listenForSocket` failed. Error code: %d", ret);
    return -1;
  }
  Debug_LOG_DEBUG("Finished initializing server.");

  do {
    Debug_LOG_INFO("Waiting for an incoming connection.");
    ret = waitForIncomingConnection(&networkStackCtx, server.handleID);
    if (ret != OS_SUCCESS) {
      Debug_LOG_ERROR("`waitForIncomingConnection` failed. Error code: %d",
                      ret);
      OS_Socket_close(server);
      return -1;
    }

    OS_Socket_Handle_t socket;
    OS_Socket_Addr_t srcAddr = {0};
    ret = OS_Socket_accept(server, &socket, &srcAddr);
    if (ret != OS_SUCCESS) {
      continue;
    }

    Debug_LOG_INFO("Now communicating with the flight controller via MAVLINK.");
    ret = handleMavLinkConnection(socket, MAVLINK_CHANNEL);
    if (ret != OS_SUCCESS) {
      Debug_LOG_ERROR("`handleMavLinkConnection` failed. Error code: %d", ret);
      continue;
    }
    Debug_LOG_INFO("Communication with flight controller has stopped.");
  } while (ret == OS_ERROR_TRY_AGAIN);

  OS_Socket_close(server);

  if (ret != OS_SUCCESS) {
    Debug_LOG_ERROR("`OS_Socket_accept` failed. Error code: %d", ret);
    return -1;
  }

  return 0;
}
