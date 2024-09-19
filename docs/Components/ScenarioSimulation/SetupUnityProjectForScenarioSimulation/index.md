# Running AWSIM Labs from Unity Editor with `scenario_simulator_v2`

Below you can find instructions on how to set up the scenario execution using `scenario_simulator_v2` with AWSIM Labs run from Unity Editor as a simulator
The instruction assumes using the Ubuntu OS.

## Prerequisites
1. Build Autoware by following [Build Autoware with `scenario_simulator_v2`](https://autowarefoundation.github.io/AWSIM-Labs/main/GettingStarted/UsingOpenSCENARIO) section from the scenario simulator and AWSIM Labs quick start guide.

2. Follow [Setup Unity Project tutorial](../../../GettingStarted/SetupUnityProject/index.md)

## Running the demo

1. Open `LoaderScene.unity` scene placed under `Assets/AWSIM/Scenes/Composition` directory.
2. Run the simulation by clicking `Play` button placed at the top section of Editor.
3. In the play mode expand the dropdown on the top left, select the `AutowareSimulationScenarioSimulator` and click the `Load` button.
4. Launch `scenario_test_runner`.
   ```
   source install/setup.bash
   ros2 launch scenario_test_runner scenario_test_runner.launch.py                        \
   architecture_type:=awf/universe/20240605  record:=false                                \
   scenario:='$(find-pkg-share scenario_test_runner)/scenario/sample_awsim.yaml'          \
   sensor_model:=awsim_labs_sensor_kit  vehicle_model:=awsim_labs_vehicle                 \
   launch_simple_sensor_simulator:=false autoware_launch_file:="e2e_simulator.launch.xml" \
   initialize_duration:=260 port:=8080
   ```
## Other sample scenarios

### Conventional traffic lights demo

This scenario controls traffic signals in the scene based on OpenSCENARIO. It can be used to verify whether traffic light recognition pipeline works well in Autoware.

```
ros2 launch scenario_test_runner scenario_test_runner.launch.py                                           \
architecture_type:=awf/universe/20240605  record:=false                                                   \
scenario:='$(find-pkg-share scenario_test_runner)/scenario/sample_awsim_conventional_traffic_lights.yaml' \
sensor_model:=awsim_labs_sensor_kit  vehicle_model:=awsim_labs_vehicle                                    \
launch_simple_sensor_simulator:=false autoware_launch_file:="e2e_simulator.launch.xml"                    \
initialize_duration:=260 port:=8080
```

### V2I traffic lights demo

This scenario publishes V2I traffic signals information based on OpenSCENARIO. It can be used to verify Autoware responds to V2I traffic lights information correctly.

```
ros2 launch scenario_test_runner scenario_test_runner.launch.py                                  \
architecture_type:=awf/universe/20240605  record:=false                                          \
scenario:='$(find-pkg-share scenario_test_runner)/scenario/sample_awsim_v2i_traffic_lights.yaml' \
sensor_model:=awsim_labs_sensor_kit  vehicle_model:=awsim_labs_vehicle                           \
launch_simple_sensor_simulator:=false autoware_launch_file:="e2e_simulator.launch.xml"           \
initialize_duration:=260 port:=8080
```
