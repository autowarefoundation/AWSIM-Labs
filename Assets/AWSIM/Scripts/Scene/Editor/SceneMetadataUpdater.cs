using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using System.IO;
using System.Collections.Generic;

namespace AWSIM.Scripts.Scene.Editor
{
    [InitializeOnLoad]
    public static class SceneMetadataUpdater
    {
        static SceneMetadataUpdater()
        {
            // Automatically update the database
            UpdateAllBuildScenes();
            // Subscribe to the scene save event
            EditorSceneManager.sceneSaved += OnSceneSaved;
        }

        private static void OnSceneSaved(UnityEngine.SceneManagement.Scene scene)
        {
            // Update the Scene Metadata Database when a scene is saved
            UpdateSceneMetadata(scene);
        }

        private static void UpdateSceneMetadata(UnityEngine.SceneManagement.Scene scene)
        {
            SceneMetadataDatabase database = GetSceneMetadataDatabase();
            if (database == null) return;

            // Get the scene name
            string sceneName = Path.GetFileNameWithoutExtension(scene.path);

            // Check if the scene is already in the database
            SceneInfo existingSceneInfo = null;
            foreach (var sceneInfo in database.sceneInfos)
            {
                if (sceneInfo.sceneName == sceneName)
                {
                    existingSceneInfo = sceneInfo;
                    break;
                }
            }

            // Check if the scene info already exists
            if (existingSceneInfo == null)
            {
                SceneInfo newSceneInfo = new SceneInfo
                {
                    sceneName = sceneName,
                    isSs2Scene = false
                };

                var sceneList = new List<SceneInfo>(database.sceneInfos) { newSceneInfo };
                database.sceneInfos = sceneList.ToArray();
            }
            else
            {
                // Debug.Log($"{sceneName} is already in the database.");
            }

            // Save changes to the asset
            EditorUtility.SetDirty(database);
            AssetDatabase.SaveAssets();

            Debug.Log($"Scene metadata updated for scene: {sceneName}");
        }

        // Populate scenes from the build list
        private static void UpdateAllBuildScenes()
        {
            var database = GetSceneMetadataDatabase();
            if (database == null) return;

            var sceneInfos = new List<SceneInfo>();

            // Get all scenes from the build settings
            foreach (var buildScene in EditorBuildSettings.scenes)
            {
                if (buildScene.enabled)
                {
                    var scenePath = buildScene.path;
                    var sceneName = Path.GetFileNameWithoutExtension(scenePath);

                    // Check if the scene already exists in the database
                    SceneInfo existingSceneInfo = null;
                    foreach (var sceneInfo in database.sceneInfos)
                    {
                        if (sceneInfo.sceneName == sceneName)
                        {
                            existingSceneInfo = sceneInfo;
                            break;
                        }
                    }

                    if (existingSceneInfo == null)
                    {
                        // If the scene is not found, create a new entry with default values
                        SceneInfo newSceneInfo = new SceneInfo
                        {
                            sceneName = sceneName,
                            isSs2Scene = false
                        };

                        sceneInfos.Add(newSceneInfo);
                    }
                    else
                    {
                        // Retain the old values
                        sceneInfos.Add(existingSceneInfo);
                    }
                }
            }

            // Update the SceneMetadataDatabase with the new scene list
            database.sceneInfos = sceneInfos.ToArray();

            EditorUtility.SetDirty(database);
            AssetDatabase.SaveAssets();

            Debug.Log("Scene metadata updated for all scenes in the build settings.");
        }

        // Helper method to get the SceneMetadataDatabase
        private static SceneMetadataDatabase GetSceneMetadataDatabase()
        {
            // Find the SceneMetadataDatabase asset
            string[] guids = AssetDatabase.FindAssets("t:SceneMetadataDatabase");
            if (guids.Length == 0)
            {
                Debug.LogError("SceneMetadataDatabase asset not found.");
                return null;
            }

            string path = AssetDatabase.GUIDToAssetPath(guids[0]);
            var database = AssetDatabase.LoadAssetAtPath<SceneMetadataDatabase>(path);

            if (database == null)
            {
                Debug.LogError("Failed to load SceneMetadataDatabase.");
                return null;
            }

            return database;
        }
    }
}
