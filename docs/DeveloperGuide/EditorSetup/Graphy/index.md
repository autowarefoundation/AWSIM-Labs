# Graphy Asset Setup

## Add Graphy from Asset Store

1) Go to Unity Asset Store and add Graphy to personal assets.

Graphy Asset Store link:

- https://assetstore.unity.com/packages/tools/gui/graphy-ultimate-fps-counter-stats-monitor-debugger-105778

## Add Graphy to the Unity Editor

1) Open up the Unity Editor.
  - Open up a temporary new scene by File -> New Scene -> Empty(Built-in) -> Create
  - This is due to a bug with Unity crashing on certain Linux configurations.
  - Once the package is imported, you can open up the desired scene.
2) Go to the `Window` menu and select `Package Manager`.
3) Make sure the `My Assets` tab is selected from the top left of the Package Manager window.
4) Find & select the Graphy from the list and click `Download` or `Import` from the bottom left of the Package Manager
   window.
5) There will be a popup window showing contents of the package. Click `Import` to add Graphy to the project.

After the import is complete, you should be able to see Graphy prefab in the `Hierarchy` window of
the `AutowareSimulation` scene. If it's missing you can add it to scene by following steps below.

## Integrating Graphy into custom scenes

Graphy is pre-integrated within the `AutowareSimulation` scene. To incorporate Graphy into your own custom scenes, please adhere to the following steps:

1) Go to the Assets folder in the Project window.
2) Open `Graphy > Prefab` folder.
3) Drag the `Graphy` prefab into the scene.
4) You can customize your Graphy by selecting the Graphy prefab in the scene and changing the settings in the inspector
   window.

## Useful links:

Unity Package manager:

- https://docs.unity3d.com/Manual/upm-ui.html

Graphy Github page:

- https://github.com/Tayx94/graphy

Graphy Documentation:

- https://github.com/Tayx94/graphy/blob/master/Readme!%20-%20Graphy%20-%20Documentation.pdf
