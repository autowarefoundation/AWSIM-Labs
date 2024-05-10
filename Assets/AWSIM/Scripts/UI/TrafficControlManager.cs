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

        public TrafficPlayToggleEvent TrafficPlayToggleEvent;
        public TrafficVisibilityToggleEvent TrafficVisibilityToggleEvent;
        public int SeedInput;
        public int TargetVehicleCount;

        private void Awake()
        {
            TrafficPlayToggleEvent ??= new TrafficPlayToggleEvent();
            TrafficVisibilityToggleEvent ??= new TrafficVisibilityToggleEvent();
        }

        public void TrafficManagerPlayToggle()
        {
            if (TrafficManager.gameObject.activeSelf)
            {
                TrafficManager.enabled = !TrafficManager.enabled;
                TrafficPlayToggleEvent.Invoke(TrafficManager.enabled);
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
                TrafficPlayToggleEvent.Invoke(TrafficManager.enabled);
            }

            //Invoke event
            TrafficVisibilityToggleEvent.Invoke(TrafficManager.gameObject.activeSelf);
        }

        public void TrafficManagerReset()
        {
            if (TrafficManager.gameObject.activeSelf)
            {
                TrafficManager.enabled = true;
                TrafficManager.RestartTraffic();
            }
            else
            {
                TrafficManager.gameObject.SetActive(true);
                TrafficManager.enabled = true;
                TrafficManager.RestartTraffic();

                // Invoke event
                TrafficVisibilityToggleEvent.Invoke(TrafficManager.gameObject.activeSelf);
            }

            // Invoke event
            TrafficPlayToggleEvent.Invoke(TrafficManager.enabled);
        }

        public void TrafficManagerUpdate()
        {
            if (!TrafficManager.gameObject.activeSelf)
            {
                Debug.LogWarning("TrafficManager is not active. Can't update traffic density!");
            }
            else
            {
                TrafficManager.enabled = true;
                TrafficPlayToggleEvent.Invoke(TrafficManager.enabled);

                TrafficManager.targetVehicleCount = TargetVehicleCount;
                TrafficManager.seed = SeedInput;
                TrafficManager.RestartTraffic();
            }
        }

        public int GetMaxVehicleCount() => TrafficManager.maxVehicleCount;
        public int GetTargetVehicleCount() => TrafficManager.targetVehicleCount;
    }
}
