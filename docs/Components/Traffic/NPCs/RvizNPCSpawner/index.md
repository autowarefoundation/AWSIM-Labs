## Introduction

This feature allows you to spawn NPCs (pedestrians, cars, buses) in the Unity scene (AWSIM) from Autoware (RVIZ). It includes bulk deletion, vehicle velocity specification, and ensures NPC vehicles move straight, only colliding with the ground.

## Setup

You can change the configuration of the NPCs in the `RVIZNPCSpawner` component in AWSIM. It is attached to the Environment game object in the AutowareSimulation scene. You can also attach the `RVIZNPCSpawner` to any other game object and set all the required values in your custom scene.

1. Attach the `RVIZNPCSpawner` component to one of the game objects in the scene. By default, it's attached to `Environment`.

2. In AWSIM, you can add more cars (smaller vehicles) or buses (large vehicles) in the `Npc Car prefabs` / `Npc Bus prefabs` lists respectively.

3. Then, select the `parent objects` under which the Pedestrians or Vehicles will be spawned.

4. Finally, adjust the `Despawn time` to destroy the NPCs after a delay.

  ![alt text](rvizspawner.png)

## Spawning NPCs from Autoware

1. Open up Autoware.

2. Click the `2D Dummy Car`,`2D Dummy Bus` or `2D Dummy Pedestrian` button in the toolbar.  

  ![alt text](image-1.png)

3. Set the pose of the dummy object by clicking and dragging on the map.

  ![alt text](spawnPedestrian.gif)

4. Set the velocity of the object in `Tool Properties` -> `2D Dummy Car/Pedestrian panel`.

  ![alt text](toolsprop.png)

5. NPCs will not be spawned if the you try to place them outside the map.
