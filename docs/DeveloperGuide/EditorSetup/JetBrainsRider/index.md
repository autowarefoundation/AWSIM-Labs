# JetBrains Rider setup with Unity

## Install JetBrains Rider:

Follow the steps in:

- https://www.jetbrains.com/help/rider/Installation_guide.html#snap

```bash
sudo snap install rider --classic
```

## Install .NET SDK:

Follow the steps in:
- https://learn.microsoft.com/en-us/dotnet/core/install/linux-ubuntu-install?tabs=dotnet9&pivots=os-linux-ubuntu-2204#ubuntu-2204

```bash
sudo add-apt-repository ppa:dotnet/backports
sudo apt-get update && \
  sudo apt-get install -y dotnet-sdk-9.0
```

## Connect Rider to Unity Editor:

Follow the steps in:

- https://www.jetbrains.com/help/rider/Unity.html#8f092cb7_9

```bash
1) Open an existing Unity project in the Unity Editor.

2) Select Edit > Preferences (Unity > Settings on macOS) and open the External Tools page.

3) In the External Script Editor, select a "Rider" installation.

4) In the Preferences window, click "Regenerate project files" under the External Tools section.

5) While still in the Unity Editor, right-click anywhere in the Project view and select Open C# Project.

6) Rider will start automatically and open the solution related to this Unity project. Once the solution is loaded, Rider and the Unity Editor become connected. The Unity icon on the toolbar shows the current connection status:
```
