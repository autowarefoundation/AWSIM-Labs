using UnityEngine;
using UnityEngine.SceneManagement;
using AWSIM.Scripts.UI.Toggle;
using AWSIM.Scripts.UI;
using AWSIM.TrafficSimulation;

namespace AWSIM.Loader
{
    static class SimConfiguration
    {
        public static void Configure(EgosManager egoManager, MapManager mapManager, SimulationManager simulationManager)
        {
            //////////////////////
            // Levels load order:
            // LoaderScene -> MapScene -> SimulationScene
            //
            // Code in this section activates after all "Start()" and "Awake()" from map and simulation scenes.

            //////////////////////
            // Configuration resources

            // Get our scene handles
            Scene simulationScene = SceneManager.GetSceneByName(simulationManager.simulationSceneName);
            Scene mapScene = SceneManager.GetSceneByName(mapManager.spawnedMapName);

            // We set the map scene to active to get its environment parameters (skybox and ambient lighting)
            SceneManager.SetActiveScene(mapScene);

            // Get configurations
            var simulationConfiguration = simulationManager.simulationConfiguration;
            var mapConfiguration = mapManager.mapConfiguration;
            var egoConfiguration = egoManager.egoConfiguration;

            //////////////////////
            // Ego Spawn

            // We want our ego to spawn under the Simulation scene root object
            var ego = egoManager.SpawnEgo(egoManager.gameObject.transform);

            //////////////////////
            // Core initialization

            // Set camera GUI
            FollowCamera followCamera = GameObject.FindObjectOfType<FollowCamera>();
            GameObject.FindObjectOfType<MainCameraViewUI>().SetFollowCamera(followCamera);

            // Set Ego position manager
            Scripts.UI.EgoVehiclePositionManager positionManager = GameObject.FindObjectOfType<Scripts.UI.EgoVehiclePositionManager>();
            positionManager.EgoTransform = ego.transform;
            positionManager.Activate();

            // Set scene time scale
            DemoUI demoUi = GameObject.FindObjectOfType<DemoUI>();
            demoUi.SetTimeScale(simulationConfiguration.timeScale);
            demoUi.TimeScaleSlider.value = simulationConfiguration.timeScale;

            TrafficControlManager trafficControlManager = GameObject.FindObjectOfType<TrafficControlManager>();
            trafficControlManager.TrafficManager = GameObject.FindObjectOfType<TrafficManager>();
            trafficControlManager.Activate();

            UIKeyboardControlToggle uiKeyboardControlToggle = GameObject.FindObjectOfType<UIKeyboardControlToggle>();
            uiKeyboardControlToggle.Activate();

            UITrafficControlVisibilityToggle uiTrafficControlVisibilityToggle = GameObject.FindObjectOfType<UITrafficControlVisibilityToggle>();
            uiTrafficControlVisibilityToggle.Activate();

            UITrafficControlPlayToggle uiTrafficControlPlayToggle = GameObject.FindObjectOfType<UITrafficControlPlayToggle>();
            uiTrafficControlPlayToggle.Activate();

            UITrafficVehicleDensity uiTrafficVehicleDensity = GameObject.FindObjectOfType<UITrafficVehicleDensity>();
            uiTrafficVehicleDensity.Activate();

            BirdEyeView birdEyeView = GameObject.FindObjectOfType<BirdEyeView>();
            birdEyeView.Activate();

            GraphicsSettings graphicsSettings = GameObject.FindObjectOfType<GraphicsSettings>();
            graphicsSettings.Activate();

            UISensorInteractionPanel uiSensorInteractionPanel = GameObject.FindObjectOfType<UISensorInteractionPanel>();
            uiSensorInteractionPanel.Activate();

            UIMainCameraToggle uiMainCameraToggle = GameObject.FindObjectOfType<UIMainCameraToggle>();
            uiMainCameraToggle.Activate();

            VehicleDashboard vehicleDashboard = GameObject.FindObjectOfType<VehicleDashboard>();
            vehicleDashboard.Activate();

            // Set traffic on/off
            var trafficSims = GameObject.FindObjectsOfType<TrafficSimulation.TrafficManager>();
            foreach (var trafficSim in trafficSims)
            {
                trafficSim.gameObject.SetActive(simulationConfiguration.useTraffic);
            }

            // Turn shadows for directional light
            if (mapConfiguration.useShadows)
            {
                var lights = GameObject.FindObjectsOfType<Light>();
                foreach (Light light in lights)
                {
                    if (light.type == LightType.Directional)
                    {
                        light.shadows = LightShadows.Hard;
                    }
                }
            }
        }
    }
}
