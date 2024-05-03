using AWSIM.TrafficSimulation;
using UnityEngine;
using UnityEngine.Events;

namespace AWSIM.Scripts.UI
{
    public class TrafficPlayToggleEvent : UnityEvent<bool>
    {
    }

    public class TrafficVisibilityToggleEvent : UnityEvent<bool>
    {
    }

    public class TrafficControlManager : MonoBehaviour
    {
        public TrafficManager TrafficManager { private get; set; }

        public TrafficPlayToggleEvent trafficPlayToggleEvent;
        public TrafficVisibilityToggleEvent trafficVisibilityToggleEvent;

        private void Awake()
        {
            trafficPlayToggleEvent ??= new TrafficPlayToggleEvent();
            trafficVisibilityToggleEvent ??= new TrafficVisibilityToggleEvent();
        }

        public void TrafficManagerPlayToggle()
        {
            if (TrafficManager.gameObject.activeSelf)
            {
                TrafficManager.enabled = !TrafficManager.enabled;
                trafficPlayToggleEvent.Invoke(TrafficManager.enabled);
            }
            else
                Debug.Log("TrafficManager is not active. Can't pause/resume!");
        }

        public void TrafficManagerVisibilityToggle()
        {
            if (!TrafficManager.gameObject.activeSelf)
            {
                TrafficManager.gameObject.SetActive(true);
            }
            else
            {
                TrafficManager.enabled = false;
                TrafficManager.gameObject.SetActive(false);

                // Invoke event
                trafficPlayToggleEvent.Invoke(TrafficManager.enabled);
            }

            //Invoke event
            trafficVisibilityToggleEvent.Invoke(TrafficManager.gameObject.activeSelf);
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

                // Invoke event
                trafficVisibilityToggleEvent.Invoke(TrafficManager.gameObject.activeSelf);
            }

            // Invoke event
            trafficPlayToggleEvent.Invoke(TrafficManager.enabled);
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
