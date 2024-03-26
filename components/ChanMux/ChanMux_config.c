/*
 *  Channel MUX
 *
 * Copyright (C) 2020-2024, HENSOLDT Cyber GmbH
 * 
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * For commercial licensing, contact: info.cyber@hensoldt.net
 */

#include "system_config.h"
#include "ChanMux/ChanMux.h"
#include "ChanMuxNic.h"
#include <camkes.h>


//------------------------------------------------------------------------------
static unsigned int
resolveChannel(
    unsigned int  sender_id,
    unsigned int  chanNum_local)
{
    // ToDo: this function is supposed to map a "local" channel number passed
    //       by a sender to a "global" channel number used by the Proxy. Goal
    //       is, that only ChanMUX is aware of the global channel number that
    //       the proxy needs. All components should just use local channel
    //       numbers.
    //       Unfortunately, the NIC protocol needs to be changed first, because
    //       is still uses global channel numbers in the NIC_OPEN command. This
    //       is a legacy from the time there the control channel was shared for
    //       multiple NICs, we do not plan to use this any longer.
    //       For now this function does not implement any mapping, but it does
    //       some access control at least. Component can only use their channel
    //       numbers. We do not look into the protocol, thus rough NIC drivers
    //       may still use anything in the NIC_OPEN command

    switch (sender_id)
    {
    //----------------------------------
    case CHANMUX_ID_NIC:
        switch (chanNum_local)
        {
        //----------------------------------
        case CHANMUX_CHANNEL_NIC_CTRL: // ToDo: use local channel number
            return CHANMUX_CHANNEL_NIC_CTRL;
        //----------------------------------
        case CHANMUX_CHANNEL_NIC_DATA: // ToDo: use local channel number
            return CHANMUX_CHANNEL_NIC_DATA;
        //----------------------------------
        default:
            break;
        }
        break;
    //----------------------------------
 //   case CHANMUX_ID_STORAGE:
 //       // TODO: check that chanNum_local is 0, reject anything else
 //       return CHANMUX_CHANNEL_NVM;
    //----------------------------------
    default:
        break;
    }

    return INVALID_CHANNEL;
}


//------------------------------------------------------------------------------
static struct
{
    uint8_t ctrl[128];
    // FIFO is big enough to store 1 minute of network "background" traffic.
    // Value found by manual testing, may differ in less noisy networks
    uint8_t data[1024 * PAGE_SIZE];
} nic_fifo[1];

static struct
{
    ChanMux_Channel_t ctrl;
    ChanMux_Channel_t data;
} nic_channel[1];


//------------------------------------------------------------------------------
//static uint8_t nvm_fifo[1][PAGE_SIZE];

//static ChanMux_Channel_t nvm_channel[1];


//------------------------------------------------------------------------------
static const ChanMux_ChannelCtx_t channelCtx[] =
{

    CHANNELS_CTX_NIC_CTRL_DATA(
        CHANMUX_CHANNEL_NIC_CTRL,
        CHANMUX_CHANNEL_NIC_DATA,
        0,
        nwDriver_ctrl_portRead,
        nwDriver_ctrl_portWrite,
        nwDriver_data_portRead,
        nwDriver_data_portWrite,
        nwDriver_ctrl_eventHasData_emit,
        nwDriver_data_eventHasData_emit),

//    CHANMUX_CHANNEL_CTX(
//        CHANMUX_CHANNEL_NVM,
//        &nvm_channel[0],
//        nvm_fifo[0], // must be the buffer and not a pointer
//        CHANMUX_DATAPORT_ASSIGN(chanMuxStorage_chan_portRead,
//                                chanMuxStorage_chan_portWrite),
//        chanMuxStorage_chan_eventHasData_emit),
};


//------------------------------------------------------------------------------
// this is used by the ChanMux component
const ChanMux_Config_t cfgChanMux =
{
    .resolveChannel = &resolveChannel,
    .numChannels    = ARRAY_SIZE(channelCtx),
    .channelCtx     = channelCtx,
};
