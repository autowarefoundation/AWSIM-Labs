using UnityEngine;

namespace AWSIM.Scripts.UI
{
    public class HotkeyHandler : MonoBehaviour
    {
        [SerializeField] private Canvas mainMenu;

        private EgoVehiclePositionManager egoVehiclePositionManager;

        private void Awake()
        {
            egoVehiclePositionManager = GetComponent<EgoVehiclePositionManager>();
        }

        void Update()
        {
            // Toggle the main menu
            if (Input.GetKeyDown(KeyCode.Escape))
            {
                if (!mainMenu)
                {
                    Debug.LogWarning(
                        "The mainMenu reference is null. Please ensure the mainMenu Canvas object is assigned to mainMenu in the Inspector. Without it, the GUI cannot be toggled on/off.");
                }
                else
                {
                    mainMenu.enabled = !mainMenu.enabled;
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
        }
    }
}
