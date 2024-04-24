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


### Feature differences from the TIER IV/AWSIM

| Features                                  | AWSIM 1.2.1      | AWSIM Labs 1.0.0-beta |
|-------------------------------------------|------------------|-----------------------|
| Rendering Pipeline                        | HDRP             | URP                   |
| Unity Version                             | Unity 2021.1.7f1 | Unity LTS 2022.3.21f1 |
| Resource usage                            | Heavy            | Light                 |
| Can reset vehicle position on runtime     | ❌                | ✅                     |
| Multiple scene and vehicle setup          | ❌                | ✅                     |
| Multi-lidars are enabled by default       | ❌                | ✅                     |
| CI for build                              | ❌                | ✅                     |
| CI for documentation generation within PR | ❌                | ✅                     |

## Try the simulation demo yourself!
We don't have a release yet. Please build it from the source.

[Download AWSIM Demo for Ubuntu](https://github.com/autowarefoundation/AWSIM/releases/download/v1.2.0/AWSIM_v1.2.0.zip){.md-button .md-button--primary}

To test the AWSIM Labs demo with Autoware please refer to the [Quick start demo](./GettingStarted/QuickStartDemo/index.md) section.
