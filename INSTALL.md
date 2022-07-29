# Setup
## Script
`scritps/download_external_software_required.sh` downloads the required packages and extract them to the location the run script expects.
Additional installation steps required for PX4

### PX4
The installation of the needed dependencies is described [here](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html). On Ubuntu you need to run:
```sh
bash PX4-Autopilot/Tools/setup/ubuntu.sh
pip3 install --user pyserial empy toml numpy pandas jinja2 pyyaml pyros-genmsg packaging
```

## Manual

1. We assume that this gitmodule is cloned into seos_test/src/demos/demo_mavlink_testapp
2. Download AirSim Blocks from [github](https://github.com/microsoft/AirSim/releases/download/v1.7.0-linux/Blocks.zip) and unziping Blocks.zip into script folder
3. Clone PX4 autopilot from [github](https://github.com/PX4/PX4-Autopilot.git) into the script folder
4. Execute the two commands from the previos section PX4
5. Download mavp2p from [github](https://github.com/aler9/mavp2p/releases/download/v0.6.5/mavp2p_v0.6.5_linux_amd64.tar.gz) and extract the tar.gz into script folder
6. Download the airsim-agent from [bitbucket](https://bitbucket.cc.ebs.corp/users/paka101/repos/airsim-agent)
7. Install python requirements by running pip3 install --user airsim==1.5.0 pymavlink
8. Build the proxy app by compiling the trentos sdk.

# Compiling and Running

## Script
A script that starts all needed programs can be found at scripts/setup-flight-demo.
It will spawn a tmux session, in which all needed programs are run.

If you interrupt execution, you can connect to the tmux session by running 'tmux attach'

tmux commands:
Controll comand (C[) is ctrl-B
To switch between splits inside the window C[ and then arrows.
To switch windows C[, then n for next or p for previous window (can be seen in the botom of the window)
Open a new split horizontally C[, then "
Open a new split vertically C[, then %


## Manual
Running in the order this document follows is recommended.
Each section corresponds to one shell, there will be in total seven parallel shells running.

### mavp2p
The `172.17.0.1` address refers to the TRENTOS Docker container. You may need to change that one.
```sh
$ mavp2p udps:127.0.0.1:14550 tcpc:172.17.0.1:14540
```

### Microsoft AirSim
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

### Airsim-Agent
```
$ python3 "$AIRSIM_PROXY_ROOT"
```

### PX4-Autopilot
```sh
$ cd "$PX4_ROOT"
$ PX4_SIM_SPEED_FACTOR=X make px4_sitl none_iris
```

On systems without dedicated gpu the sim speed factor needs to be set, on those with dedicated gpu it is not needed.

### MAVLink-TestApp
```sh
$ cd "$SEOS_TEST/.."
$ seos_tests/seos_sandbox/scripts/open_trentos_build_env.sh seos_tests/seos_sandbox/build-system.sh seos_tests/src/demos/demo_mavlink_testapp $TARGET_PLATFORM build-$TARGET_PLATFORM-Debug-demo_mavlink_testapp -DCMAKE_BUILD_TYPE=Debug
$ cd build-$TARGET_PLATFORM-Debug-demo_mavlink_testapp && ../qemu/build/aarch64-softmmu/qemu-system-aarch64 -machine virt,virtualization=on,highmem=off,secure=off -cpu cortex-a53 -nographic  -m size=3072M -serial mon:stdio -serial tcp:localhost:4444,server -serial telnet:localhost:4445,server -kernel images/capdl-loader-image-arm-qemu-arm-virt
```

### Proxy_App
```sh
$ OS-SDK/build/proxy/proxy_app -c TCP:4444 -t 1
```

### Linux vm telnet
```sh
$ telnet localhost 4445
```

