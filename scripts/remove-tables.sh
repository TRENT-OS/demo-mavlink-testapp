#!/bin/bash

sudo ip link set tap0 down
sudo ip link set br0 down
sudo ip link delete tap0
sudo ip link delete br0
