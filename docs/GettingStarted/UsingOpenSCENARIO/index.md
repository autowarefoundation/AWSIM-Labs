# Using OpenSCENARIO
!!! Warning

    Running AWSIM with scenario_simulator_v2 is still a prototype, so stable running is not guaranteed.

Below you can find instructions on how to set up the OpenSCENARIO execution using [scenario_simulator_v2](https://github.com/tier4/scenario_simulator_v2) with AWSIM as a simulator
The instruction assumes using the Ubuntu OS.

## Prerequisites
- Follow [Setup Unity Project tutorial](../SetupUnityProject/index.md)

## Build Autoware with `scenario_simulator_v2`

In order to configure the Autoware software with the AWSIM Labs demo, please:

1. Clone the [Autoware](https://github.com/autowarefoundation/autoware.git) and move to the directory.
   ```
   git clone git@github.com:autowarefoundation/autoware.git
   cd autoware
   ```
2. Check out to the `awsim-labs-stable` branch _(This is a temporary branch, and it will be used in the main branch later on.)_
   ```
   git checkout main
   ```
3. Configure the environment. _(Skip if Autoware environment has been configured before)_
   ```
   ./setup-dev-env.sh
   ```
4. Create the `src` directory and clone external dependent repositories into it.
   ```
   mkdir src
   vcs import src < autoware.repos
   vcs import src < simulator.repos
   ```
5. Download `shinjuku_map.zip`
[archive](https://github.com/tier4/AWSIM/releases/download/v1.2.0/shinjuku_map.zip){.md-button .md-button--primary}

6. Unzip it to `src/simulator` directory
   ```
   unzip <Download directory>/shinjuku_map.zip -d src/simulator
   ```
7. Install dependent ROS packages.
   ```
   source /opt/ros/humble/setup.bash
   rosdep update
   rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
   ```
8. Build the workspace.
   ```
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-w"
   ```

## Running the demo

1. Download and run the latest version of AWSIM Labs.  
[Releases](https://github.com/autowarefoundation/AWSIM-Labs/releases){.md-button .md-button--primary}  
2. In the `Loader` menu, from the top left, select the `AutowareSimulationScenarioSimulator` and click the `Load` button.
3. Launch `scenario_test_runner`.
   ```
   source install/setup.bash
   ros2 launch scenario_test_runner scenario_test_runner.launch.py                        \
   architecture_type:=awf/universe/20240605  record:=false                                \
   scenario:='$(find-pkg-share scenario_test_runner)/scenario/sample_awsim.yaml'          \
   sensor_model:=awsim_labs_sensor_kit  vehicle_model:=awsim_labs_vehicle                 \
   launch_simple_sensor_simulator:=false autoware_launch_file:="e2e_simulator.launch.xml" \
   initialize_duration:=260 port:=8080
   ```

## Troubleshooting

In case of problems, make sure that the [regular demo](https://autowarefoundation.github.io/AWSIM-Labs/main/GettingStarted/QuickStartDemo/) work well with the Autoware built above. Follow the [troubleshooting page](https://autowarefoundation.github.io/AWSIM-Labs/main/DeveloperGuide/TroubleShooting/) there if necessary.

## Appendix
- [AWSIM ROS2 topic list](../../Components/ROS2/ROS2TopicList/index.md)
