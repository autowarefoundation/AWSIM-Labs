using UnityEngine;

namespace AWSIM.Scripts.UI
{
    public class UIBridge : MonoBehaviour
    {
        private EgoVehiclePositionManager egoVehiclePositionManager;

        private void Awake()
        {
            egoVehiclePositionManager = GetComponent<EgoVehiclePositionManager>();
        }

        public void ResetEgoToSpawnPoint()
        {
            egoVehiclePositionManager.ResetEgoToSpawnPoint();
        }
    }
}