using System;
using UnityEngine;
using UnityEngine.UI;

namespace AWSIM.Scripts.UI
{
    public class UIBridge : MonoBehaviour
    {
        private EgoVehiclePositionManager egoVehiclePositionManager;
        private TrafficControlManager trafficControlManager;
        [SerializeField] InputField trafficSeedInputField;

        private void Awake()
        {
            egoVehiclePositionManager = GetComponent<EgoVehiclePositionManager>();
            trafficControlManager = GetComponent<TrafficControlManager>();
        }

        public void ResetEgoToSpawnPoint()
        {
            egoVehiclePositionManager.ResetEgoToSpawnPoint();
        }

        public void TrafficManagerPauseResumeToggle()
        {
            trafficControlManager.TrafficManagerPauseResumeToggle();
        }

        public void TrafficManagerEnable()
        {
            trafficControlManager.TrafficManagerEnable();
        }

        public void TrafficManagerDisable()
        {
            trafficControlManager.TrafficManagerDisable();
        }

        public void TrafficManagerReset()
        {
            trafficControlManager.TrafficManagerReset();
        }

        public void TrafficMangerSetSeed()
        {
            if (!trafficSeedInputField)
            {
                Debug.LogWarning("TrafficSeedInputField reference is missing. Can't set seed without it!");
                return;
            }

            var seed = Convert.ToInt32(trafficSeedInputField.text);
            trafficControlManager.TrafficMangerSetSeed(seed);
        }
    }
}