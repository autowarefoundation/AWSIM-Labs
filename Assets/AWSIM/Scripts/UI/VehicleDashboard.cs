using AWSIM.Scripts.Vehicles.VPP_Integration;
using AWSIM.Scripts.Vehicles.VPP_Integration.Enums;
using UnityEngine;
using UnityEngine.UI;
using VehiclePhysics;

namespace AWSIM.Scripts.UI
{
    public class VehicleDashboard : MonoBehaviour
    {
        private VPVehicleController _vehicleController;
        private AutowareVPPAdapter _adapter;

        [SerializeField] private Text _speedText;
        [SerializeField] private Text _transmissionModeText;
        [SerializeField] private Text _gearText;
        [SerializeField] private Text _absText;
        [SerializeField] private Text _tcsText;
        [SerializeField] private Text _escText;
        [SerializeField] private Text _asrText;
        [SerializeField] private Text _controlModeText;

        [SerializeField] private Color _systemActiveColor = Color.green;
        [SerializeField] private Color _systemInactiveColor = Color.gray;

        [SerializeField] private Color _conrtolModeAutonomousColor = Color.cyan;
        [SerializeField] private Color _controlModeManualColor = Color.yellow;

        private const float MsToKmH = 3.6f;

        private void Start()
        {
            _vehicleController = FindObjectOfType<VPVehicleController>();
            _adapter = FindObjectOfType<AutowareVPPAdapter>();
        }

        private void FixedUpdate()
        {
            UpdateDashboard();
        }

        private void UpdateDashboard()
        {
            _speedText.text = UpdateSpeed(_vehicleController);
            _transmissionModeText.text = UpdateTransmissionMode(_vehicleController);
            _gearText.text = UpdateGear(_vehicleController);
            UpdateAbsState(_vehicleController, _absText);
            UpdateAsrState(_vehicleController, _asrText);
            UpdateEscState(_vehicleController, _escText);
            UpdateTcsState(_vehicleController, _tcsText);
            UpdateControlMode(_adapter, _controlModeText);
        }

        private void UpdateControlMode(AutowareVPPAdapter adapter, Text text)
        {
            if (adapter == null)
            {
                text.text = "N/A";
                text.color = _systemInactiveColor;
            }
            else
            {
                switch (adapter.ControlModeInput)
                {
                    case VPPControlMode.Autonomous:
                        text.text = "Autonomous";
                        text.color = _conrtolModeAutonomousColor;
                        break;
                    case VPPControlMode.Manual:
                        text.text = "Manual";
                        text.color = _controlModeManualColor;
                        break;
                    case VPPControlMode.NoCommand:
                        text.text = "NoCommand";
                        text.color = _systemInactiveColor;
                        break;
                    case VPPControlMode.AutonomousSteerOnly:
                        text.text = "AutonomousSteerOnly";
                        text.color = _systemInactiveColor;
                        break;
                    case VPPControlMode.AutonomousVelocityOnly:
                        text.text = "AutonomousVelocityOnly";
                        text.color = _systemInactiveColor;
                        break;
                    case VPPControlMode.Disengaged:
                        text.text = "Disengaged";
                        text.color = _systemInactiveColor;
                        break;
                    case VPPControlMode.NotReady:
                        text.text = "Not Ready";
                        text.color = _systemInactiveColor;
                        break;
                    default:
                        text.text = "N/A";
                        text.color = _systemInactiveColor;
                        break;
                }
            }
        }

        private static string UpdateSpeed(VPVehicleController vehicle)
        {
            if (vehicle == null)
            {
                return "N/A";
            }

            var speedVal = (int)(vehicle.speed * MsToKmH);
            // Always return positive
            if (speedVal < 0)
            {
                speedVal = -speedVal;
            }

            string speed = speedVal.ToString("F0");
            return speed;
        }

        private string UpdateTransmissionMode(VPVehicleController vehicle)
        {
            if (vehicle == null)
            {
                return "N/A";
            }

            int modeVal = _vehicleController.data.bus[Channel.Vehicle][VehicleData.GearboxMode];

            return modeVal switch
            {
                0 => "Manual",
                1 => "Park",
                2 => "Reverse",
                3 => "Neutral",
                4 => "Auto",
                5 => "Auto 1",
                6 => "Auto 2",
                7 => "Auto 3",
                8 => "Auto 4",
                9 => "Auto 5",
                _ => "N/A"
            };
        }

        private string UpdateGear(VPVehicleController vehicle)
        {
            if (vehicle == null)
            {
                return "N/A";
            }

            int gearVal = _vehicleController.data.bus[Channel.Vehicle][VehicleData.GearboxGear];

            return gearVal switch
            {
                < 0 => "R",
                0 => "N",
                > 0 => gearVal.ToString()
            };
        }

        private void UpdateAbsState(VPVehicleController vehicle, Text text)
        {
            if (vehicle.data.bus[Channel.Vehicle][VehicleData.AbsEngaged] != 0)
            {
                text.color = _systemActiveColor;
            }
            else
            {
                text.color = _systemInactiveColor;
            }
        }

        private void UpdateAsrState(VPVehicleController vehicle, Text text)
        {
            if (vehicle.data.bus[Channel.Vehicle][VehicleData.AsrEngaged] != 0)
            {
                text.color = _systemActiveColor;
            }
            else
            {
                text.color = _systemInactiveColor;
            }
        }

        private void UpdateEscState(VPVehicleController vehicle, Text text)
        {
            if (vehicle.data.bus[Channel.Vehicle][VehicleData.EscEngaged] != 0)
            {
                text.color = _systemActiveColor;
            }
            else
            {
                text.color = _systemInactiveColor;
            }
        }

        private void UpdateTcsState(VPVehicleController vehicle, Text text)
        {
            if (vehicle.data.bus[Channel.Vehicle][VehicleData.TcsEngaged] != 0)
            {
                text.color = _systemActiveColor;
            }
            else
            {
                text.color = _systemInactiveColor;
            }
        }
    }
}
