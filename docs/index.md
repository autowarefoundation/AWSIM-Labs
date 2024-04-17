# Welcome to AWSIM Labs

![](assets/images/E2ESim.png){: style="height:300px"}

![](assets/images/autoware-foundation.png){: style="height:90px"}
![](assets/images/awsim-labs-logo.png){: style="height:90px"}

[AWSIM Labs](https://github.com/autowarefoundation/AWSIM) is currently being developed under the [Autoware Labs](https://github.com/orgs/autowarefoundation/discussions/4550) initiative. Main purpose of this fork is to provide faster implementation of features needed by the users of the AWSIM while also ensuring a high-performance simulation environment for the [Autoware](https://github.com/autowarefoundation/autoware).

This is a fork of [TIER IV's AWSIM](https://github.com/tier4/AWSIM).

### Features

- Many predefined components included (Vehicle dynamic models, Sensor models, Environment configuration, ROS2 communication, etc)
- Support for Ubuntu 22.04 and windows10/11
- ROS2 native communication (humble)
- [Open sourced](https://autowarefoundation.com/autowarefoundation/AWSIM)
- Made with [Unity](https://unity.com/)


### Feature differences from the main AWSIM

| AWSIM                                      | AWSIM Labs                       |
|--------------------------------------------|----------------------------------|
| Using HDRP                                 | Using URP                        |
| Using Unity 2021.1.7f1                     | Using Unity LTS 2022.3.21f1      |
| Limited interaction with simulation and UI | Interactable simulation and UI   |
| Uses more resources                        | Uses less resources              |
| -                                          | Multiple scene and vehicle setup |

## Try the simulation demo yourself!
We don't have a release yet. Please build it from the source.

[Download AWSIM Demo for Ubuntu](https://github.com/autowarefoundation/AWSIM/releases/download/v1.2.0/AWSIM_v1.2.0.zip){.md-button .md-button--primary}

To test the AWSIM Labs demo with Autoware please refer to the [Quick start demo](./GettingStarted/QuickStartDemo/index.md) section.
