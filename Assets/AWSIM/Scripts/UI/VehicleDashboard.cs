using AWSIM.Scripts.Vehicles.VPP_Integration;
using AWSIM.Scripts.Vehicles.VPP_Integration.Enums;
using UnityEngine;
using UnityEngine.UI;
using VehiclePhysics;

namespace AWSIM.Scripts.UI
{
    /// <summary>
    /// Script for displaying vehicle status in the UI.
    /// Commented out the text version of ControlMode since is not utilized completely from Autoware side yet.
    /// Added more simplistic image representation for ControlMode as requested.
    /// Disabled updating GearBoxMode in UI.
    /// </summary>
    public class VehicleDashboard : MonoBehaviour
    {
        private VPVehicleController _vehicleController;
        private AutowareVPPAdapter _adapter;
        [Header("Speed")]
        [SerializeField] private Text _speedText;
        [Header("Transmission")]
        [SerializeField] private Text _gearText;
        [Header("Systems")]
        [SerializeField] private Text _absText;
        [SerializeField] private Text _tcsText;
        [SerializeField] private Text _escText;
        [SerializeField] private Text _asrText;
        // [SerializeField] private Text _transmissionModeText;
        // [SerializeField] private Text _controlModeText;

        [SerializeField] private Color _systemActiveColor = Color.green;
        [SerializeField] private Color _systemInactiveColor = Color.gray;
        // [SerializeField] private Color _controlModeAutonomousColor = Color.cyan;
        // [SerializeField] private Color _controlModeManualColor = Color.yellow;
        [Header("Control Mode")]
        [SerializeField] private Image _controlModeImage;
        [SerializeField] private Sprite[] _controlModeImages;

        private const float MsToKmH = 3.6f;

        private void GetVariables()
        {
            _vehicleController = FindObjectOfType<VPVehicleController>();
            _adapter = FindObjectOfType<AutowareVPPAdapter>();
        }

        public void Activate()
        {
            GetVariables();
        }

        private void FixedUpdate()
        {
            if (_vehicleController == null) return;
            if (_adapter == null) return;

            UpdateDashboard();
        }

        private void UpdateDashboard()
        {
            _speedText.text = UpdateSpeed(_vehicleController);
            // _transmissionModeText.text = UpdateTransmissionMode(_vehicleDataBus[VehicleData.GearboxMode]);
            _gearText.text = UpdateGear(_vehicleController.data.bus[Channel.Vehicle][VehicleData.GearboxGear]);

            UpdateSystemState(_vehicleController.data.bus[Channel.Vehicle][VehicleData.AbsEngaged], _absText);
            UpdateSystemState(_vehicleController.data.bus[Channel.Vehicle][VehicleData.AsrEngaged], _asrText);
            UpdateSystemState(_vehicleController.data.bus[Channel.Vehicle][VehicleData.EscEngaged], _escText);
            UpdateSystemState(_vehicleController.data.bus[Channel.Vehicle][VehicleData.TcsEngaged], _tcsText);

            // UpdateControlMode();
            UpdateControlModeImage();
        }

        private void UpdateControlModeImage()
        {
            if (_adapter == null)
            {
                SetControlModeImage(_controlModeImages[0]);
                return;
            }

            Sprite texture = _adapter.ControlModeInput switch
            {
                VPPControlMode.NoCommand => _controlModeImages[0],
                VPPControlMode.Autonomous => _controlModeImages[1],
                VPPControlMode.Manual => _controlModeImages[0],
                _ => _controlModeImages[0]
            };

            SetControlModeImage(texture);
        }

        private void SetControlModeImage(Sprite texture)
        {
            _controlModeImage.sprite = texture;
        }

        // private void UpdateControlMode()
        // {
        //     if (_adapter == null)
        //     {
        //         SetControlModeText("N/A", _systemInactiveColor);
        //         return;
        //     }
        //
        //     var (text, color) = _adapter.ControlModeInput switch
        //     {
        //         VPPControlMode.NoCommand => ("NoCommand", _systemInactiveColor),
        //         VPPControlMode.Autonomous => ("Autonomous", _conrtolModeAutonomousColor: _controlModeAutonomousColor),
        //         VPPControlMode.AutonomousSteerOnly => ("AutonomousSteerOnly", _systemInactiveColor),
        //         VPPControlMode.AutonomousVelocityOnly => ("AutonomousVelocityOnly", _systemInactiveColor),
        //         VPPControlMode.Manual => ("Manual", _controlModeManualColor),
        //         VPPControlMode.Disengaged => ("Disengaged", _systemInactiveColor),
        //         VPPControlMode.NotReady => ("Not Ready", _systemInactiveColor),
        //         _ => ("N/A", _systemInactiveColor)
        //     };
        //
        //     SetControlModeText(text, color);
        // }

        // private void SetControlModeText(string mode, Color color)
        // {
        //     _controlModeText.text = mode;
        //     _controlModeText.color = color;
        // }

        private static string UpdateSpeed(VPVehicleController vehicle)
        {
            if (vehicle == null)
            {
                return "N/A";
            }

            var speedVal = Mathf.Abs((int)(vehicle.speed * MsToKmH));

            return speedVal.ToString("F0");
        }

        // private static string UpdateTransmissionMode(int modeVal)
        // {
        //     return modeVal switch
        //     {
        //         0 => "Manual",
        //         1 => "Park",
        //         2 => "Reverse",
        //         3 => "Neutral",
        //         4 => "Auto",
        //         5 => "Auto 1",
        //         6 => "Auto 2",
        //         7 => "Auto 3",
        //         8 => "Auto 4",
        //         9 => "Auto 5",
        //         _ => "N/A"
        //     };
        // }

        private string UpdateGear(int gearVal)
        {
            return gearVal switch
            {
                < 0 => "R",
                0 => _vehicleController.data.bus[Channel.Vehicle][VehicleData.GearboxMode] switch
                {
                    (int)Gearbox.AutomaticGear.P => "P",
                    (int)Gearbox.AutomaticGear.N => "N",
                    _ => "D"
                },
                _ => gearVal.ToString()
            };
        }

        private void UpdateSystemState(int systemState, Text systemText)
        {
            systemText.color = systemState != 0 ? _systemActiveColor : _systemInactiveColor;
        }
    }
}
