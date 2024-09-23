using AWSIM.Scripts.Vehicles.VPP_Integration;
using AWSIM.Scripts.Vehicles.VPP_Integration.Enums;
using UnityEngine;
using VehiclePhysics;

namespace AWSIM.Scripts.UI.Toggle
{
    public class UIKeyboardControlToggle : MonoBehaviour
    {
        private GameObject _egoVehicle;
        private AutowareVPPAdapter _adapter;
        private VPVehicleController _controller;

        private void Start()
        {
            _egoVehicle = GameObject.FindWithTag("Ego");
            _adapter = _egoVehicle.GetComponent<AutowareVPPAdapter>();
            _controller = _egoVehicle.GetComponent<VPVehicleController>();

            // Set the toggle to the current state of the keyboard control mode
            GetComponent<UnityEngine.UI.Toggle>().isOn = _adapter.ControlModeInput != VPPControlMode.Autonomous;
        }

        public void Activate()
        {
            enabled = true;
            Start();
        }

        private void Update()
        {
            if (_adapter.ControlModeInput == VPPControlMode.Manual)
            {
                if (Input.GetKeyDown(KeyCode.P))
                {
                    _controller.data.bus[Channel.Input][InputData.AutomaticGear] = (int)Gearbox.AutomaticGear.P;
                }
            }
        }

        // Toggle the keyboard control
        public void OnClick(bool isOn)
        {
            _adapter.ControlModeInput = _adapter.ControlModeInput == VPPControlMode.Autonomous
                ? VPPControlMode.Manual
                : VPPControlMode.Autonomous;
        }
    }
}
