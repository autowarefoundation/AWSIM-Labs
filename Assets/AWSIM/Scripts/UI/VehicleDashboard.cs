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
        private int[] _vehicleDataBus;
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
            _vehicleDataBus = _vehicleController.data.bus[Channel.Vehicle];
            _adapter = FindObjectOfType<AutowareVPPAdapter>();

            if (_vehicleController == null || _adapter == null)
            {
                Debug.LogError("VehicleController or AutowareVPPAdapter component not found!");
            }
        }

        public void Activate()
        {
            Start();
        }

        private void FixedUpdate()
        {
            if (_vehicleController == null) return;

            UpdateDashboard();
        }

        private void UpdateDashboard()
        {
            _speedText.text = UpdateSpeed(_vehicleController);
            _transmissionModeText.text = UpdateTransmissionMode(_vehicleDataBus[VehicleData.GearboxMode]);
            _gearText.text = UpdateGear(_vehicleDataBus[VehicleData.GearboxGear]);

            UpdateSystemState(_vehicleDataBus[VehicleData.AbsEngaged], _absText);
            UpdateSystemState(_vehicleDataBus[VehicleData.AsrEngaged], _asrText);
            UpdateSystemState(_vehicleDataBus[VehicleData.EscEngaged], _escText);
            UpdateSystemState(_vehicleDataBus[VehicleData.TcsEngaged], _tcsText);

            UpdateControlMode();
        }

        private void UpdateControlMode()
        {
            if (_adapter == null)
            {
                SetControlModeText("N/A", _systemInactiveColor);
                return;
            }

            var (text, color) = _adapter.ControlModeInput switch
            {
                VPPControlMode.Autonomous => ("Autonomous", _conrtolModeAutonomousColor),
                VPPControlMode.Manual => ("Manual", _controlModeManualColor),
                VPPControlMode.NoCommand => ("NoCommand", _systemInactiveColor),
                VPPControlMode.AutonomousSteerOnly => ("AutonomousSteerOnly", _systemInactiveColor),
                VPPControlMode.AutonomousVelocityOnly => ("AutonomousVelocityOnly", _systemInactiveColor),
                VPPControlMode.Disengaged => ("Disengaged", _systemInactiveColor),
                VPPControlMode.NotReady => ("Not Ready", _systemInactiveColor),
                _ => ("N/A", _systemInactiveColor)
            };

            SetControlModeText(text, color);
        }

        private void SetControlModeText(string mode, Color color)
        {
            _controlModeText.text = mode;
            _controlModeText.color = color;
        }

        private static string UpdateSpeed(VPVehicleController vehicle)
        {
            if (vehicle == null)
            {
                return "N/A";
            }

            var speedVal = Mathf.Abs((int)(vehicle.speed * MsToKmH));

            return speedVal.ToString("F0");
        }

        private static string UpdateTransmissionMode(int modeVal)
        {
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

        private static string UpdateGear(int gearVal)
        {
            return gearVal switch
            {
                < 0 => "R",
                0 => "N",
                > 0 => gearVal.ToString()
            };
        }

        private void UpdateSystemState(int systemState, Text systemText)
        {
            systemText.color = systemState != 0 ? _systemActiveColor : _systemInactiveColor;
        }
    }
}
