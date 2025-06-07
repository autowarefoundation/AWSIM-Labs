using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityEngine.UI;
using UnityEngine.SceneManagement;
using System.Linq;


namespace AWSIM.Loader
{
    /// <summary>
    /// Manager for configuring the Ego.
    /// </summary>
    public class MapManager : MonoBehaviour, IConfigurableManager
    {
        [Tooltip("Scene names that can't be used and aren't shown in a GUI selector.")]
        public List<string> forbiddenSceneNames = new List<string>() { "AWSIMSimulation" };

        [HideInInspector]
        public string spawnedMapName;

        [HideInInspector]
        public string loaderSceneName = "LoaderScene";

        public MapConfiguration mapConfiguration { private set; get; }

        /// <summary>
        /// Map configuration.
        /// </summary>
        public MapConfiguration MapConfiguration => mapConfiguration;

        [Tooltip("Map dropdown selector.")]
        public Dropdown mapUISelecor;

        /// <summary>
        /// Log callback.
        /// </summary>
        public Action<LogLevel, string> Log { get; set; }

        private EgosManager _egosManager;

        public void Start()
        {
            if (!forbiddenSceneNames.Contains(SceneManager.GetActiveScene().name))
            {
                forbiddenSceneNames.Add(SceneManager.GetActiveScene().name);
            }
            _egosManager = GameObject.FindObjectOfType<EgosManager>();
        }

        /// <summary>
        /// Additively loads a map specified in a configuration.
        /// </summary>
        public AsyncOperation LoadMap()
        {
            Log(LogLevel.LOG_INFO, $"Loading scene {mapConfiguration.mapName}");
            spawnedMapName = mapConfiguration.mapName;
            return SceneManager.LoadSceneAsync(mapConfiguration.mapName, LoadSceneMode.Additive);
        }

        /// <summary>
        /// Set up the UI for map configuration
        /// </summary>
        public void LoadUI()
        {
            mapUISelecor.options.Clear();
            for (int i = 0; i < SceneManager.sceneCountInBuildSettings; ++i)
            {
                bool sceneNameValid = true;
                var sceneName = SceneUtility.GetScenePathByBuildIndex(i).Split('/').Last().Replace(".unity", "");
                foreach (var scene in forbiddenSceneNames)
                {
                    if (scene == sceneName)
                    {
                        sceneNameValid = false;
                        break;
                    }
                }
                if (sceneNameValid)
                {
                    mapUISelecor.options.Add(
                        new Dropdown.OptionData(
                            sceneName
                        )
                    );
                }
            }

            // Set the listener for future value changes.
            mapUISelecor.onValueChanged.AddListener(delegate { UpdateFields(mapUISelecor.options[mapUISelecor.value]); });

            // Check if any maps were actually found and added to the dropdown.
            if (mapUISelecor.options.Count > 0)
            {
                mapUISelecor.value = 0;
                mapUISelecor.RefreshShownValue();

                // Manually call UpdateFields with the initial, default value (at index 0).
                // This ensures the ego spawn position is set on start, not just on change.
                UpdateFields(mapUISelecor.options[0]);
            }
        }

        /// <summary>
        /// Load and validate config.
        /// </summary>
        public bool LoadConfig(AWSIMConfiguration config)
        {
            this.mapConfiguration = config.mapConfiguration;

            // Validate config
            if (SceneUtility.GetBuildIndexByScenePath(mapConfiguration.mapName) < 0)
            {
                Log(LogLevel.LOG_ERROR, $"Map '{mapConfiguration.mapName}' not found.");
                return false;
            }

            foreach (var scene in forbiddenSceneNames)
            {
                if (scene == mapConfiguration.mapName)
                {
                    Log(LogLevel.LOG_ERROR, $"Scene name '{mapConfiguration.mapName}' is reserved and cannot be used.");
                    return false;
                }
            }
            return true;
        }

        // Sin (other part is in EgosManager.cs)
        private void UpdateFields(Dropdown.OptionData data)
        {
            double mapPosX = 0;
            double mapPosY = 0;
            double mapPosZ = 0;
            double mapRotX = 0;
            double mapRotY = 0;
            double mapRotZ = 0;

            switch (data.text)
            {
                case "Shinjuku" or "ShinjukuNight":
                    mapPosX = 81380.72;
                    mapPosY = 49918.78;
                    mapPosZ = 41.57;
                    mapRotX = 0;
                    mapRotY = 0;
                    mapRotZ = 35;
                    break;
                case "Parking Area":
                    mapPosX = 81580.52;
                    mapPosY = 50083.58;
                    mapPosZ = 33.985;
                    mapRotX = 0;
                    mapRotY = 0;
                    mapRotZ = 100;
                    break;
            }
            _egosManager.UpdateMapDefaultSpawn(mapPosX, mapPosY, mapPosZ, mapRotX, mapRotY, mapRotZ);
        }
    }
}
