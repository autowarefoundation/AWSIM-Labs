using UnityEngine;

namespace AWSIM.Scripts.UI
{
    public class HotkeyHandler : MonoBehaviour
    {
        [SerializeField] private UISideBarHandler uiSideBarHandler;

        private EgoVehiclePositionManager egoVehiclePositionManager;
        private TrafficControlManager trafficControlManager;

        private void Awake()
        {
            egoVehiclePositionManager = GetComponent<EgoVehiclePositionManager>();
            trafficControlManager = GetComponent<TrafficControlManager>();
        }

        void Update()
        {
            // Toggle the main menu
            if (Input.GetKeyDown(KeyCode.Escape))
            {
                if (!uiSideBarHandler)
                {
                    Debug.LogWarning(
                        "The mainMenu reference is null. Please ensure the mainMenu Canvas object is assigned to mainMenu in the Inspector. Without it, the GUI cannot be toggled on/off.");
                }
                else
                {
                    uiSideBarHandler.ToggleSideBar();
                }
            }

            // Reset the ego vehicle to the spawn point
            if (Input.GetKey(KeyCode.LeftControl))
            {
                if (Input.GetKeyDown(KeyCode.R))
                {
                    egoVehiclePositionManager.ResetEgoToSpawnPoint();
                }
            }

            // Toggle traffic visibility
            if (Input.GetKey(KeyCode.LeftControl))
            {
                if (Input.GetKeyDown(KeyCode.T))
                {
                    trafficControlManager.TrafficManagerVisibilityToggle();
                }
            }
        }
    }
}
