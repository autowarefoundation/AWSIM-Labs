# Quick Start Demo

Below you can find instructions on how to setup the self-driving demo of AWSIM simulation controlled by Autoware.
The instruction assumes using the Ubuntu OS.

### Demo configuration

The simulation provided in the AWSIM demo is configured as follows:

| AWSIM Demo Settings |                                            |
|:--------------------|:-------------------------------------------|
| Vehicle             | Lexus RX 450h                              |
| Environment         | Japan Tokyo Nishishinjuku                  |
| Sensors             | GNSS<br>IMU<br>3 x VLP16<br>Traffic Light Camera |
| Traffic             | Randomized traffic                         |
| ROS2                | humble                                     |

### PC specs

Please make sure that your machine meets the following requirements in order to run the simulation correctly:

| Required PC Specs         |                    |
|:--------------------------|:-------------------|
| OS                        | Ubuntu 22.04       |
| CPU                       | 6c12t or higher    |
| GPU                       | RTX 2080 or higher |
| Nvidia Driver (Ubuntu 22) | >=545              |


### DDS configuration

In order to run AWSIM Labs with the best performance and without hogging the network, please follow the steps below.

Add the following lines to `~/.bashrc` file:

``` bash
if [ ! -e /tmp/cycloneDDS_configured ]; then
	sudo sysctl -w net.core.rmem_max=2147483647
	sudo sysctl -w net.ipv4.ipfrag_time=3
    sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728     # (128 MB)
	sudo ip link set lo multicast on
	touch /tmp/cycloneDDS_configured
fi
```

Every time you restart this machine, and open a new terminal, the above commands will be executed.

Until you restart the machine, they will not be executed again.

#### CycloneDDS configuration

Save the following as `cyclonedds.xml` in your home directory `~`:
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain Id="any">
        <General>
            <Interfaces>
                <NetworkInterface name="lo" priority="default" multicast="default" />
            </Interfaces>
            <AllowMulticast>default</AllowMulticast>
            <MaxMessageSize>65500B</MaxMessageSize>
        </General>
        <Internal>
            <SocketReceiveBufferSize min="10MB"/>
            <Watermarks>
                <WhcHigh>500kB</WhcHigh>
            </Watermarks>
        </Internal>
    </Domain>
</CycloneDDS>

```

Make sure the following lines are added to the `~/.bashrc` file:

``` bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/home/your_username/cyclonedds.xml
```

Replace `your_username` with your actual username.

!!! note
    You should use the absolute path to the `cyclonedds.xml` file.

!!! warning
    A system restart is required for these changes to work.

!!! warning
    **DO NOT** set `export ROS_LOCALHOST_ONLY=1`. CycloneDDS configuration will be enough.  

## Start the demo

### Running the AWSIM demo

To run the simulator, please follow the steps below.

1. Install Nvidia GPU driver (Skip if already installed).

    1. Add Nvidia driver to apt repository
    ```
    sudo add-apt-repository ppa:graphics-drivers/ppa
    sudo apt update
    ```
    2. Install the recommended version of the driver.
    ```
    sudo ubuntu-drivers autoinstall

    # or install a specific version (following was tested)
    sudo apt install nvidia-driver-550
    ```
    3. Reboot your machine to make the installed driver detected by the system.
    ```
    sudo reboot
    ```
    4. Open terminal and check if `nvidia-smi` command is available and outputs summary similar to the one presented below.
    ```
    $ nvidia-smi
    +-----------------------------------------------------------------------------------------+
    | NVIDIA-SMI 550.54.15              Driver Version: 550.54.15      CUDA Version: 12.4     |
    |-----------------------------------------+------------------------+----------------------+
    | GPU  Name                 Persistence-M | Bus-Id          Disp.A | Volatile Uncorr. ECC |
    | Fan  Temp   Perf          Pwr:Usage/Cap |           Memory-Usage | GPU-Util  Compute M. |
    |                                         |                        |               MIG M. |
    |=========================================+========================+======================|
    |   0  NVIDIA GeForce RTX 3080        Off |   00000000:2D:00.0  On |                  N/A |
    | 30%   40C    P8             35W /  320W |    5299MiB /  10240MiB |      7%      Default |
    |                                         |                        |                  N/A |
    +-----------------------------------------+------------------------+----------------------+
    ...
    ```

2. Install Vulkan Graphics Library (Skip if already installed).

    1. Update the environment.
    ```
    sudo apt update
    ```
    2. Install the library.
    ```
    sudo apt install libvulkan1
    ```

3. Download and Run AWSIM Demo binary.

    1. Download the latest release from:

        [AWSIM Labs GitHub Releases Page](https://github.com/autowarefoundation/AWSIM-Labs/releases){.md-button .md-button--primary}

    2. Unzip the downloaded file.

    3. Make the file executable.

        Right click the `awsim_labs.x86_64` file and check the `Execute` checkbox

        ![](Image_1.png)

        or execute the command below.

        ```
        chmod +x <path to AWSIM folder>/awsim_labs.x86_64
        ```

    4. Launch `awsim_labs.x86_64`.
        ```
        ./<path to AWSIM folder>/awsim_labs.x86_64
        ```

        It may take some time for the application to start the so please wait until image similar to the one presented below is visible in your application window.

        ![](Image_0.png)

### Launching Autoware

In order to configure and run the Autoware software with the AWSIM demo, please:

1. Download `map files (pcd, osm)` and unzip them.
    [Download Map files (pcd, osm)](https://github.com/tier4/AWSIM/releases/download/v1.1.0/nishishinjuku_autoware_map.zip){.md-button .md-button--primary}

2. Clone [Autoware](https://github.com/autowarefoundation/autoware) and move to the directory.
```
git clone https://github.com/autowarefoundation/autoware.git
cd autoware
```
3. Switch branch to `main`.
```
git checkout main
```
4. Configure the environment. (Skip if Autoware environment has been configured before)
```
./setup-dev-env.sh
```
5. Create the `src` directory and clone external dependent repositories into it.
```
mkdir src
vcs import src < autoware.repos
```
6. Install dependent ROS packages.
```
source /opt/ros/humble/setup.bash
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```
7. Build the workspace.
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1
```
8. Launch Autoware.
```
source install/setup.bash
ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=sample_vehicle sensor_model:=awsim_labs_sensor_kit map_path:=<absolute path of map folder>

# Use the absolute path for the map folder, don't use the ~ operator.

# Example:
ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=sample_vehicle sensor_model:=awsim_labs_sensor_kit map_path:=/home/your_username/autoware_map/nishishinjuku_autoware_map
```
![](Image_2.png)

## Let's run the self driving simulation

1. Launch AWSIM and Autoware according to the steps described earlier in this document.
![](Image_top.png)

2. The Autoware will automatically set its pose estimation as presented below.
![](Image_Initial.png)

3. Set the navigation goal for the vehicle.
![](Image_goal_0.png)
![](Image_goal_1.png)

4. Optionally, you can define an intermediate point through which the vehicle will travel on its way to the destination.
![](Image_checkpoint_0.png)
The generated path can be seen on the image below.
![](Image_path.png)

5. Enable self-driving.

To make the vehicle start navigating please engage its operation using the command below.

```
cd autoware
source install/setup.bash
ros2 topic pub /autoware/engage autoware_vehicle_msgs/msg/Engage '{engage: True}' -1
```

![](Image_running.png)

The self-driving simulation demo has been successfully launched!

## Troubleshooting

In case of any problems with running the sample AWSIM binary with Autoware, start with checking our [Troubleshooting page](../../DeveloperGuide/TroubleShooting/index.md) with the most common problems.

## Appendix
- [AWSIM ROS2 topic list](../../Components/ROS2/ROS2TopicList/index.md)
