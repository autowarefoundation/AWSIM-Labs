using UnityEngine;

namespace AWSIM.Scripts.UI.Toggle
{
    public class UIKeyboardControlToggle : MonoBehaviour
    {
        private GameObject _egoVehicle;
        private VehicleKeyboardInput _vehicleKeyboardInput;

        public void Activate()
        {
            _egoVehicle = GameObject.FindGameObjectWithTag("Ego");
            _vehicleKeyboardInput = _egoVehicle.GetComponent<VehicleKeyboardInput>();

            // Set the toggle to the current state of the keyboard control
            GetComponent<UnityEngine.UI.Toggle>().isOn = _vehicleKeyboardInput.enabled;
        }

        // Toggle the keyboard control
        public void OnClick(bool isOn)
        {
            _vehicleKeyboardInput.enabled = isOn;
        }
    }
}
