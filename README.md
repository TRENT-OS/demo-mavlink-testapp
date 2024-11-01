An TRENTOS-based application to showcase MAVLINK usage.

# Setup
1. Set up TRENTOS and visit `$TRENTOS_SDK_ROOT/sdk/demos`.
  ```
  $ cd "$TRENTOS_SDK_ROOT/sdk/demos"
  ```
2. Clone this repository:
  ```sh
  $ git clone ssh://git@bitbucket.hensoldt-cyber.systems:7999/~paukai01/trentos-mavlink-testapp.git
  ```
3. Download and install the AirSim Blocks environment as described [here](https://microsoft.github.io/AirSim/use_precompiled/).
  A Docker environment can be found [here](https://microsoft.github.io/AirSim/docker_ubuntu/).
4. Copy the `$TESTAPP_ROOT/airsim-settings.json` to the correct folder renamed as `settings.json`. See [AirSim Settings](https://microsoft.github.io/AirSim/settings/) for more information.
  ```sh
  $ cd "$TESTAPP_ROOT"
  $ mkdir -p ~/Documents/AirSim && cp airsim-settings.json ~/Documents/AirSim/settings.json
  ```
5. Download [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot):
  ```sh
  $ git clone --recursive https://github.com/PX4/PX4-Autopilot.git
  ```
6. Install the Python packages as described [here](https://docs.px4.io/master/en/dev_setup/building_px4.html#failed-to-import-python-packages).
7. Install the latest version of [mavp2p](https://github.com/aler9/mavp2p/releases).
8. Install [airsim-agent](https://bitbucket.cc.ebs.corp/users/paka101/repos/airsim-agent/browse).

# Compiling and Running
Running in the order this document follows is recommended.
Each section corresponds to one shell, there will be in total four parallel shells running.

## mavp2p
The `172.17.0.1` address refers to the TRENTOS Docker container. You may need to change that one.
```sh
$ mavp2p udps:127.0.0.1:14550 tcpc:172.17.0.1:14540
```

## Microsoft AirSim
```sh
$ cd "$AIRSIM_ROOT"
$ ./Blocks.sh
```
On some systems with outdated hardware, you may need to run the following command instead.
Be aware that this is discouraged and deprecated.
```sh
$ cd "$AIRSIM_ROOT"
$ ./Blocks.sh -windowed -opengl4
```

## Airsim-Agent
```
$ python3 "$AIRSIM_PROXY_ROOT"
```

## MAVLink-TestApp
```sh
$ cd "$TRENTOS_SDK_ROOT/.."
$ sdk/scripts/open_trentos_build_env.sh sdk/build-system.sh sdk/demos/trentos-mavlink-testapp sabre build-sabre-Debug-trentos-mavlink-testapp -DCMAKE_BUILD_TYPE=Debug
$ sdk/scripts/open_trentos_test_env.sh -d '-p 14540:14540' -d '--name=trentos' sdk/demos/trentos-mavlink-testapp/run_qemu.sh build-sabre-Debug-trentos-mavlink-testapp/images/os_image.elf
```

## PX4-Autopilot
```sh
$ cd "$PX4_ROOT"
$ make px4_sitl none_iris
```
