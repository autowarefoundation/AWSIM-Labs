# Setup Unity Project

!!! info

    It is advised to checkout the [Quick Start Demo](../QuickStartDemo) tutorial before reading this section.

This page is a tutorial for setting up a AWSIM Unity project.

## Environment preparation

### System setup

=== "Ubuntu 22"
    1. Make sure your machine meets the [required hardware specifications](../QuickStartDemo/#pc-specs).
        - *NOTE: PC requirements may vary depending on simulation contents which may change as the simulator develops*
    2. Prepare a desktop PC with Ubuntu 22.04 installed.
    2. Install [Nvidia drivers and Vulkan Graphics API](../QuickStartDemo/#running-the-awsim-simulation-demo).
    3. Install [git](https://git-scm.com/).
    4. Follow [the DDS configuration guide](../QuickStartDemo/index.md#dds-configuration).

=== "Windows"
    1. Make sure your machine meets the [required hardware specifications](../QuickStartDemo/#pc-specs).
        - *NOTE: PC requirements may vary depending on simulation contents which may change as the simulator develops*
    2. Prepare a desktop PC with Windows 10 or 11 (64 bit) installed.
    3. Install [git](https://git-scm.com/).
    4. Install [Microsoft Visual C++ Redistributable packages for Visual Studio 2015, 2017, 2019, and 2022](https://learn.microsoft.com/en-us/cpp/windows/latest-supported-vc-redist?view=msvc-170#visual-studio-2015-2017-2019-and-2022) (X64 Architecture)

### ROS 2

AWSIM comes with a *standalone* flavor of [`Ros2ForUnity`](../../Components/ROS2/ROS2ForUnity/index.md). This means that, to avoid internal conflicts between different ROS 2 versions, you shouldn't run the Editor or AWSIM binary with ROS 2 sourced.

!!! warning

    Do not run the AWSIM, Unity Hub, or the Editor with ROS 2 sourced.

=== "Ubuntu 22"
    - Make sure that the terminal which you are using to run Unity Hub, Editor, or AWSIM doesn't have ROS 2 sourced.
    - It is common to have ROS 2 sourced automatically with `~/.bashrc` or `~/.profile`. Make sure it is not obscuring your working environment:
        - Running Unity Hub from the Ubuntu GUI menu takes the environment configuration from `~/.profile`.
        - Running Unity Hub from the terminal uses the current terminal configuration from `~/.profile` and `~/.bashrc`.
        - Running Unity Editor from the UnityHub inherits the environment setup from the Unity Hub.

=== "Windows"
    - Make sure your Windows environment variables are ROS 2 free.

### Unity installation

!!! info

    AWSIM's Unity version is currently **2022.3.36f1**

Follow the steps below to install Unity on your machine:

#### Install UnityHub:
1. Install UnityHub to manage Unity projects. Please go to [Unity download page](https://unity3d.com/get-unity/download) and download latest `UnityHub.AppImage`.
2. Install Unity 2022.3.36f1 via UnityHub:
    - Open new terminal, navigate to directory where `UnityHub.AppImage` is download and execute the following command:
```
./UnityHub.AppImage
```
#### Install Unity Editor:
1. Click `Installs` from the left menu and then click the `Install Editor` button from the top right.
2. From the new window click the `Official releases` tab and check for the current version of Unity.
   - If you can't find the version, click to `Archive` and search for the version in the Unity archives page. It will direct you to [Unity Archive](https://unity3d.com/get-unity/download/archive).
3. After finding the version, click the download/install button to start the installation process.
   - At this point, your Unity installation process should have started.

       === "Ubuntu 22"
       - *NOTE: If the installation process has not started after clicking the download/install button, please copy the hyperlink (by right clicking the button and selecting `Copy link address`) and add it as an argument for Unity Hub app. An example command:
       ```
       ./UnityHub.AppImage unityhub://2022.3.21f1/d91830b65d9b
       ```

4. After successful installation the version will be available in the `Installs` tab in Unity Hub.


### Open AWSIM project

To open the Unity AWSIM project in Unity Editor:

=== "Using Unity Hub"
    1. Make sure you have the AWSIM repository cloned and ROS 2 is not sourced.
        ```
        git clone git@github.com:autowarefoundation/AWSIM.git
        ```

    2. Launch UnityHub.
        ```
        ./UnityHub.AppImage
        ```

        !!! info

            If you are launching the Unity Hub from the Ubuntu applications menu (without the terminal), make sure that system optimizations are set. To be sure, run the terminal at least once before running the Unity Hub. This will apply the OS settings.

    3. Open the project in UnityHub
        - Click the `Open` button from the `Projects` tab,
        - Navigate the directory where the AWSIM repository was cloned to and select the directory,
        - The project should be added to `Projects` tab in Unity Hub. To launch the project in Unity Editor simply click the `AWSIM` project.
        - The project is now ready to use.
        - The project can be opened by double clicking on the project name or selecting it from the list then clicking the `Open` button on the top right.


=== "Using Terminal"

    1. Enter the AWSIM directory (make sure ROS 2 is not sourced).
        ```
        cd AWSIM
        ```

    2. If your Unity Editor is in default location, run the project using the editor command.
        ```
        ~/Unity/Hub/Editor/[your_editor_version]/Editor/Unity -projectPath .
        ```

        !!! info
            Please specify your Unity Editor version in the path.
            If your Unity Editor is installed in different location, please adjust the path accordingly.

!!! warning

    If you get the safe mode dialog when starting UnityEditor, you may need to install openssl.

    1. download libssl  
    `$ wget http://security.ubuntu.com/ubuntu/pool/main/o/openssl1.0/libssl1.0.0_1.0.2n-1ubuntu5.13_amd64.deb`
    2. install  
    `sudo dpkg -i libssl1.0.0_1.0.2n-1ubuntu5.13_amd64.deb`

### Import external packages

To properly run and use AWSIM project in Unity it is required to download map package which is not included in the repository.

1. Download and import `Nishishinjuku_URP_v0.2.0.unitypackage`

    [Download Map Package](https://autoware-files.s3.us-west-2.amazonaws.com/awsim-labs/Nishishinjuku_URP_v0.2.0.unitypackage){.md-button .md-button--primary}

2. In Unity Editor, from the menu bar at the top, select `Assets -> Import Package -> Custom Package...` and navigate the `Nishishinjuku_URP.unitypackage` file you've downloaded and open.
3. Click `Import` button in the popup window to import the package.
4. `Nishishinjuku` package should be successfully imported under `Assets/AWSIM/Externals/`directory. You can access the directory from the `Project` window in Unity Editor.

!!! info

    The Externals directory is added to the `.gitignore` because the map has a large file size and should not be directly uploaded to the repository.

## Import Vehicle Physics Pro Community Edition Asset

Import Vehicle Physics Pro CE by following these instructions: [VPP CE Setup](../../DeveloperGuide/EditorSetup/VPPCommunityEdition/index.md)

## Import Graphy Asset

Import Graphy by following these instructions: [Graphy Asset Setup](../../DeveloperGuide/EditorSetup/Graphy/index.md)

## Run the demo in Editor

The following steps describe how to run the demo in Unity Editor:

1. Open the `AutowareSimulation.unity` scene placed under `Assets/AWSIM/Scenes/Main` directory
2. Run the simulation by clicking `Play` button placed at the top section of Editor.
3. Now you should see the simulation running in the Editor.

If you encounter any issues while running the simulation, please refer to the [Troubleshooting](../../DeveloperGuide/TroubleShooting) section or open an issue in the [AWSIM Labs GitHub](https://github.com/autowarefoundation/AWSIM-Labs/issues).
