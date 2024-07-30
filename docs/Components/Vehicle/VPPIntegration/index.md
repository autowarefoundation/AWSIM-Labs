# VPP Integration with the Ego Vehicle

VPP is a package that provides realistic vehicle simulation. Here you can find a guide on how to integrate VPP with the
Ego Vehicle.

!!! tip "Example Vehicle"
    It is recommended to take a look at the default Lexus vehicle prefab as example to see how the components are set up.

## Initial setup

To be able to use VPP with the ego vehicle, please attach these components to the vehicle prefab:
From the VPP package:

Added to prefab root:

- `Vehicle Controller`
- `Standard Input`
- `Camera Target` (Optional, you can use your own implementation)
- `Telemetry Window` (Optional, used to display telemetry data)
- `Visual Effects` (Optional, used to visualize the steering wheel)
- `VP Vehicle Toolkit`

!!! warning "Wheel Colliders"
    VPP uses its own implementation of the `Wheel Colliders`! For the wheel colliders, please add the `Wheel Colliders`
    provided by the VPP package. They share the same name with the default Unity components.

After adding those scripts continue with adding the following components to the vehicle prefab:

Added to prefab root:

- `AutowareVPPAdapter.cs`: Used for applying the control inputs to the vehicle.
- `Ros2ToVPPInput.cs`: Used for receiving the control inputs from the Autoware.
- `VPPVehicleSignalHandler.cs`: Used for handling the signals from the vehicle.
- `VehiclePedalMapLoader.cs`: Used for loading the pedal maps for the vehicle.

To be able to report the vehicle state to the Autoware, you'll need this script in the URDF.
_This script is added in the `VehicleStatusSensor` prefab by default._

Added to URDF:

- `VPPToRos2Publisher.cs`: Used for publishing the vehicle state to the Autoware.

## Setting up the components

After doing the initial setup, we will need to configure the components for your vehicle.

Set up the `Vehicle Controller` and the `Wheel Colliders` with the correct values for your vehicle. Using values from
the real vehicles will result in a more realistic simulation.

You can refer to [VPP documentation](https://vehiclephysics.com/components/component-guide/) for more detailed
information about the VPP components.

As for the other components we've added to the vehicle, we have to give their necessary references.

!!! warning "Camera Controller"
    If you've added the `Camera Target` component, you'll need a separate gameObject with a `Camera Controller`. Don't
    forget to assign vehicle Transform reference in this component. This is used by default in the `AutowareSimulation`
    scene.

### AutowareVPPAdapter.cs

| Variable                       | Description                                                                                                                                 |
|--------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------|
| `Front Wheels`                 | Used for handling the steering input. Give front wheel colliders of the vehicle.                                                            |
| `Simulate Steering`            | Simulate the steering instead of assigning wheel angles directly.                                                                           |
| `Steer Wheel Input`            | Change applied to steering wheel when simulating steering. Applied every in fixed update as an input to the VPP steering. _(0-100)_         |
| `Do Use Pedal Maps`            | Whether to use the pedal maps for the vehicle or not. If set to false the vehicle will use the PID Controller for the longitudinal control. |
| `Emergency Brake Percent`      | The amount of brake pedal percent applied when the emergency is triggered. _(0-1)_                                                          |
| `Update Position Offset Y`     | The height offset value for the vehicle position when initializing position of the vehicle from the Rviz. _(In meters)_                     |
| `Update Position Ray Origin Y` | The height of the raycast origin for the vehicle position when initializing position of the vehicle from the Rviz. _(In meters)_            |

### Ros2ToVPPInput.cs

| Variable  | Description                                                                                             |
|-----------|---------------------------------------------------------------------------------------------------------|
| `Adapter` | Reference to the `AutowareVPPAdapter` script. _(Drag the object that has the corresponding component.)_ |
| `Topics`  | Topics used in the Ros2 communication. You can change the topics according to your case.                |

- `QoS Settings`: Quality of Service settings for the Ros2 communication. Currently, it is as follows:

| Variable             | Value           |
|----------------------|-----------------|
| `Reliability Policy` | Reliable        |
| `Durability Policy`  | Transient Local |
| `History Policy`     | Keep Last       |
| `Depth`              | 1               |

- `Position QoS Input`: QoS settings for the position topic:

| Variable             | Value     |
|----------------------|-----------|
| `Reliability Policy` | Reliable  |
| `Durability Policy`  | Volatile  |
| `History Policy`     | Keep Last |
| `Depth`              | 1         |

### VehicleSignalHandler.cs

| Variable     | Description                                                                                             |
|--------------|---------------------------------------------------------------------------------------------------------|
| `Adapter`    | Reference to the `AutowareVPPAdapter` script. _(Drag the object that has the corresponding component.)_ |
| `Vp Vehicle` | Reference to the `Vehicle Controller` script. _(Drag the object that has the corresponding component.)_ |

Rest of the signal related settings are same as `VehicleVisualEffect.cs`. These two scripts are basically same.

### VehiclePedalMapLoader.cs

| Variable        | Description                                                                                                   |
|-----------------|---------------------------------------------------------------------------------------------------------------|
| `Accel Map Csv` | The file reference of the acceleration pedal map. _(Drag the corresponding csv file from the Project Window)_ |
| `Brake Map Csv` | The file reference of the brake pedal map. _(Drag the corresponding csv file from the Project Window)_        |

!!! warning "Pedal Maps"
    These pedal maps are loaded in runtime. The default location for the pedal maps is `Assets/Resources/Pedal Maps/...`.
    You can find the Lexus's pedal maps in the same location.

### VPPToRos2Publisher.cs

Same as `VehicleStatusSensor.cs` but the `Adapter` reference is the `AutowareVPPAdapter` script.

## Setting up the child objects

For the VPP to work correctly, you need to set up the child objects of the vehicle prefab.

### Setting up reference for Ackermann Steering:

1. Create an empty game object named `"Ackermann"` and set as the direct child of the prefab. Then assign reference in
   the `Vehicle Controller`.
2. Move position of the created game object to the middle of the rear axle and set its height to the bottom of the rear
   wheels. Make sure the rotations are `(0,0,0)`

Assign the reference in the `Vehicle Controller`. For details, you can refer to the VPP.

VPP Reference: [Ackermann](https://vehiclephysics.com/blocks/steering/)

### Setting up the reference for Dynamics:

1. Create an empty game object named `"Dynamics"` and set as the direct child of the prefab. Add the following
   components to this object:
    - `Rolling Friction`
    - `Anti-roll Bar` (x2)

2. Create an empty game object named `"Aero"` and set as the child of the "Dynamics". Add the following
   components to this object:
    - `Aerodynamic Surface`

Assign the references in the `Vehicle Controller`. For configuring these components, you can refer to the VPP.

VPP Reference: [Dynamics](https://vehiclephysics.com/components/vehicle-dynamics/)
