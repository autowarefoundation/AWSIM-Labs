# Troubleshooting Guide for AWSIM Labs and Autoware

This document outlines common issues encountered when working with AWSIM and Autoware, along with suggested solutions.

---

## 1. General Plugin and Runtime Issues

### Massive Output of Plugin Errors

**Solution:**  
Re-clone the AWSIM repository using:

```bash
git clone <awsim_repository_url>
```

### RuntimeError: error not set

**Error Message:**  
`RuntimeError: error not set, at C:\ci\ws\src\ros2\rcl\rcl\src\rcl\node.c:262`

**Solution:**  
Ensure ROS2 environment variables and configuration files are properly set:

- Environment variables
- `cyclonedds_config.xml`

---

## 2. ROS2 Communication Issues

### `ros2 topic list` Does Not Display

**Possible Causes & Solutions:**

- Your machine's `ROS_DOMAIN_ID` differs — ensure it matches across systems.
- ROS2 is not sourced — run:
  ```bash
  source /opt/ros/<distro>/setup.bash
  ```

### ROS2 Topic List Not Displayed Between Windows (AWSIM) and Ubuntu (Autoware)

**Solution:**  
Allow communication through the Windows Firewall.

---

## 3. Simulation and Runtime Behavior

### Self-Driving Stops in the Middle of the Road

**Solution:**  
Check the accuracy of your map data:

- PointCloud
- VectorMap
- 3D FBX models

### Poor Network Performance When Connecting AWSIM and Autoware

**Solution:**  
Make ROS localhost-only. Add the following to your `.bashrc`:

```bash
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if [ ! -e /tmp/cycloneDDS_configured ]; then
  sudo sysctl -w net.core.rmem_max=2147483647
  sudo ip link set lo multicast on
  touch /tmp/cycloneDDS_configured
fi
```

### Lidar (Colored PointCloud) Not Showing in RViz

**Solution:**  
Reduce processing load (only for `awsim-stable` branch of Autoware):

```bash
cd <path_to_your_autoware_folder>
wget "https://drive.google.com/uc?export=download&id=11mkwfg-OaXIp3Z5c3R58Pob3butKwE1Z" -O patch.sh
bash patch.sh && rm patch.sh
```

---

## 4. Crashes and Errors

### Segmentation Fault When Starting AWSIM Binary

**Solution:**

- Verify Nvidia drivers and Vulkan API are properly installed.
- Disable `Graphic Jobs` in Unity Player Settings to prevent segmentation faults.
  See
  related [Unity Forum post](https://discussions.unity.com/t/segmentation-fault-core-dumped-in-standalone-app-but-not-in-editor/868646).

### Unity Editor Crashes

**Solution:**  
Check the appropriate log files:

**Editor Logs**

- **Windows:** `C:\Users\username\AppData\Local\Unity\Editor\Editor.log`
- **Linux:** `~/.config/unity3d/.Editor.log`

**Player Logs**

- **Windows:** `C:\Users\username\AppData\LocalLow\CompanyName\ProductName\output_log.txt`
- **Linux:** `~/.config/unity3d/CompanyName/ProductName/Player.log`

More info: [Unity Documentation - Log Files](https://docs.unity3d.com/2021.1/Documentation/Manual/LogFiles.html)

### Unity Editor Error when Playing

**Error:**
```log
RuntimeError: failed to get symbol 'rmw_init_options_init' due to Environment variable 'AMENT_PREFIX_PATH' is not set or empty, at ./src/functions.cpp:171, at ./src/rcl/init_options.c:75
ROS2.Utils.CheckReturnEnum (System.Int32 ret) (at <2034886caef046439cfa6ebcd9ed8cd7>:0)
ROS2.Ros2cs.Init () (at <2034886caef046439cfa6ebcd9ed8cd7>:0)
ROS2.ROS2ForUnity..ctor () (at Assets/Ros2ForUnity/Scripts/ROS2ForUnity.cs:326)
ROS2.ROS2UnityCore..ctor () (at Assets/Ros2ForUnity/Scripts/ROS2UnityCore.cs:55)
AWSIM.SimulatorROS2Node.Initialize () (at Assets/AWSIM/Scripts/ROS/SimulatorROS2Node.cs:37)
```

**Solution:**

Required environment variables:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/add_your_username/cyclonedds.xml
```

- If you are launching unityhub **directly from a launcher**,
  - Make sure the CycloneDDS related 2 environment variables are set in the:
    - `~/.profile` file.
    - if `~/.bash_profile` or `~/.bash_login` exists, they will be executed instead of `~/.profile`.
- If you are launching unityhub **from a terminal**,
  - Make sure the CycloneDDS related 2 environment variables are set in the `~/.bashrc` file.

More information can be found at [DDS settings for ROS 2 and Autoware](https://autowarefoundation.github.io/autoware-documentation/main/installation/additional-settings-for-developers/network-configuration/dds-settings/).

---

## 5. Setup and Compatibility Errors

### Initial Pose Does Not Match Automatically

**Solution:**  
Set the initial pose manually.
![Initial Pose](Image_Initial_0.png)  
![Initial Pose](Image_Initial_1.png)

### Unity Safe Mode / libssl Error

**Error Message:**  
`No usable version of libssl was found`

**Solution:**

1. Download:
   ```bash
   wget http://security.ubuntu.com/ubuntu/pool/main/o/openssl1.0/libssl1.0.0_1.0.2n-1ubuntu5.11_amd64.deb
   ```
2. Install:
   ```bash
   sudo dpkg -i libssl1.0.0_1.0.2n-1ubuntu5.11_amd64.deb
   ```

### Plugin Dependency Error in Windows Unity Editor

**Error Message:**  
`Plugins: Failed to load 'Assets/RGLUnityPlugin/Plugins/Windows/x86_64/RobotecGPULidar.dll'...`

**Solution:**  
Install [Microsoft Visual C++ Redistributable (X64)](https://learn.microsoft.com/en-us/cpp/windows/latest-supported-vc-redist?view=msvc-170#visual-studio-2015-2017-2019-and-2022)

---

## 6. System Compatibility and Driver Issues

### Unity Simulation Freezes on Windows

**Solution:**  
Update or install the latest Network Interface Card (NIC) drivers from the hardware vendor (not Microsoft).

### Unity Project Crashes While Loading

**Solution:**  
Install the recommended Nvidia driver version 545.
