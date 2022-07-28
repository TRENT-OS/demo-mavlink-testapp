#!/bin/bash


echo "Download AirSim"
wget --verbose https://github.com/microsoft/AirSim/releases/download/v1.7.0-linux/Blocks.zip
echo "Unzip Airsim and configure"
unzip Blocks.zip
mkdir -p ~/Documents/AirSim && cp ../airsim-settings.json ~/Documents/AirSim/settings.json
echo "Download PX4"
git clone --recursive -j $(nproc --all) https://github.com/PX4/PX4-Autopilot.git
echo "Install Python dependencies for PX4"
pip3 install --user pyserial empy toml numpy pandas jinja2 pyyaml pyros-genmsg packaging
echo "Download mavp2p"
wget https://github.com/aler9/mavp2p/releases/download/v0.6.5/mavp2p_v0.6.5_linux_amd64.tar.gz
echo "Extract mavp2p"
tar -xvf mavp2p_v0.6.5_linux_amd64.tar.gz
echo "Download airsim-agent"
git clone ssh://git@bitbucket.cc.ebs.corp:7999/~paka101/airsim-agent.git
echo "Install dependencies for airsim-agent"
cd airsim-agent && pip install --user airsim==1.5.0 pymavlink
cd ..

