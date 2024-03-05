# JetBrains Rider setup with Unity

## Install JetBrains Rider:

Follow the steps in:

- https://www.jetbrains.com/help/rider/Installation_guide.html#snap

```bash
sudo snap install rider --classic
```

## Install .NET SDK:

Follow the steps in:

- https://learn.microsoft.com/en-us/dotnet/core/install/linux-ubuntu#register-the-microsoft-package-repository

```bash
# Get Ubuntu version
declare repo_version=$(if command -v lsb_release &> /dev/null; then lsb_release -r -s; else grep -oP '(?<=^VERSION_ID=).+' /etc/os-release | tr -d '"'; fi)

# Download Microsoft signing key and repository
wget https://packages.microsoft.com/config/ubuntu/$repo_version/packages-microsoft-prod.deb -O packages-microsoft-prod.deb

# Install Microsoft signing key and repository
sudo dpkg -i packages-microsoft-prod.deb

# Clean up
rm packages-microsoft-prod.deb

# Update packages
sudo apt update
sudo apt install dotnet-sdk-8.0
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
