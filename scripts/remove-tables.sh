#!/bin/bash
#
# Copyright (C) 2022-2024, HENSOLDT Cyber GmbH
# 
# SPDX-License-Identifier: GPL-2.0-or-later
#
# For commercial licensing, contact: info.cyber@hensoldt.net
#

sudo ip link set tap0 down
sudo ip link set br0 down
sudo ip link delete tap0
sudo ip link delete br0
