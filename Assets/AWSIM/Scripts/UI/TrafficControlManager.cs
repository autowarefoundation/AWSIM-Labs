using System;
using AWSIM.TrafficSimulation;
using UnityEngine;

namespace AWSIM.Scripts.UI
{
    public class TrafficControlManager : MonoBehaviour
    {
        public TrafficManager TrafficManager { private get; set; }

        public void TrafficManagerPauseResumeToggle()
        {
            if (TrafficManager.gameObject.activeSelf)
            {
                TrafficManager.enabled = !TrafficManager.enabled;
            }
            else
                Debug.Log("TrafficManager is not active. Can't pause/resume!");
        }

        public void TrafficManagerEnable()
        {
            TrafficManager.gameObject.SetActive(true);
            TrafficManager.enabled = true;
        }

        public void TrafficManagerDisable()
        {
            TrafficManager.gameObject.SetActive(false);
        }

        public void TrafficManagerReset()
        {
            if (TrafficManager.gameObject.activeSelf)
            {
                TrafficManager.enabled = true;
                TrafficManager.npcVehicleSimulator.ClearAll();
            }
            else
            {
                TrafficManager.gameObject.SetActive(true);
                TrafficManager.enabled = true;
                TrafficManager.npcVehicleSimulator.ClearAll();
            }
        }

        public void TrafficMangerSetSeed(int seed)
        {
            if (!TrafficManager.gameObject.activeSelf)
            {
                Debug.LogWarning("TrafficManager is not active. Can't set seed!");
            }

            TrafficManager.seed = seed;
        }
    }
}
