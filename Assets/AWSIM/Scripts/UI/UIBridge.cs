using System;
using UnityEngine;
using UnityEngine.UI;

namespace AWSIM.Scripts.UI
{
    public class UIBridge : MonoBehaviour
    {
        [SerializeField] private InputField trafficSeedInputField;

        private EgoVehiclePositionManager _egoVehiclePositionManager;
        private TrafficControlManager _trafficControlManager;

        private void Awake()
        {
            _egoVehiclePositionManager = GetComponent<EgoVehiclePositionManager>();
            _trafficControlManager = GetComponent<TrafficControlManager>();
        }

        public void ResetEgoToSpawnPoint()
        {
            _egoVehiclePositionManager.ResetEgoToSpawnPoint();
        }

        public void TrafficManagerPlayToggle(bool isOn)
        {
            _trafficControlManager.TrafficManagerPlayToggle();
        }

        public void TrafficManagerVisibilityToggle(bool isOn)
        {
            _trafficControlManager.TrafficManagerVisibilityToggle();
        }

        public void TrafficManagerReset()
        {
            _trafficControlManager.TrafficManagerReset();
        }

        public void TrafficMangerSetSeed()
        {
            if (!trafficSeedInputField)
            {
                Debug.LogWarning("TrafficSeedInputField reference is missing. Can't set seed without it!");
                return;
            }

            _trafficControlManager.SeedInput = Convert.ToInt32(trafficSeedInputField.textComponent.text);
            _trafficControlManager.TrafficManagerUpdate();
        }
    }
}
