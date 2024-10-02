using System;
using System.Collections.Generic;
using AWSIM.Scripts.UI;
using AWSIM.Scripts.Vehicles.VPP_Integration.Enums;
using AWSIM.Scripts.Vehicles.VPP_Integration.IVehicleControlModes;
using UnityEngine;
using VehiclePhysics;

namespace AWSIM.Scripts.Vehicles.VPP_Integration
{
    public class AutowareVPPAdapter : MonoBehaviour
    {
        /// <summary>
        /// This script applies the VPP vehicle inputs.
        /// Vehicle inputs are updated from the `Ros2ToVPPInput.cs`.
        /// Results from this script are sent to the `VPPtoRos2Publisher.cs`.
        /// </summary>

        // Initial Position inputs from Rviz
        [NonSerialized] public bool WillUpdatePositionInput;

        [NonSerialized] public Vector3 PositionInput;
        [NonSerialized] public Quaternion RotationInput;

        /// Control inputs from Autoware
        [NonSerialized] public float SteerAngleInput;

        private float _steerAngleInput => SteerAngleInput;

        public bool IsDefinedSteeringTireRotationRateInput { get; set; }
        public float SteeringTireRotationRateInput { get; set; }

        // Gear input from Autoware
        [NonSerialized] public Gearbox.AutomaticGear AutomaticShiftInput = Gearbox.AutomaticGear.P;
        private Gearbox.AutomaticGear _automaticShiftInput => AutomaticShiftInput;

        // Signal input from Autoware
        [NonSerialized] public VPPSignal VehicleSignalInput;
        private VPPSignal _vehicleSignalInput => VehicleSignalInput;

        // Emergency input from Autoware
        public bool IsEmergencyInput { get; set; }

        // Control mode input from Autoware
        public VPPControlMode ControlModeInput { get; set; }

        // Actuation commands from Autoware
        public double SteerInput { get; set; }
        [NonSerialized] public double BrakeInput;
        private double _brakeInput => BrakeInput;
        [NonSerialized] public double ThrottleInput;
        private double _throttleInput => ThrottleInput;

        // Outputs from Unity/VPP
        public VPPControlMode VpControlModeReport { get; private set; }
        public int VpGearReport { get; private set; }
        public Vector3 VpVelocityReport { get; private set; }
        public Vector3 VpAngularVelocityReport { get; private set; }
        public float VpSteeringReport { get; private set; }
        public VPPSignal VpTurnIndicatorReport { get; private set; }
        public VPPSignal VpHazardLightsReport { get; private set; }
        public double VpThrottleStatusReport { get; private set; }
        public double VpBrakeStatusReport { get; private set; }
        public double VpSteerStatusReport { get; private set; }

        // For ss2
        // I feel like some of these are same with the ones used in VPReports but will see (mozzz)

        /// <summary>
        /// Vehcile local velocity (m/s)
        /// </summary>
        public Vector3 LocalVelocity => VpVelocityReport;

        /// <summary>
        /// Acceleration(m/s^2) in the local coordinate system of the vehicle.
        /// </summary>
        public Vector3 LocalAcceleration { get; private set; }

        /// <summary>
        /// Vehicle angular velocity (rad/s)
        /// </summary>
        public Vector3 AngularVelocity { get; private set; }

        /// <summary>
        /// Vehicle angular velocity in the local coordinate system of the vehicle (rad/s)
        /// </summary>
        public Vector3 LocalAngularVelocity => VpAngularVelocityReport;

        /// <summary>
        /// Vehicle angular acceleration (rad/s^2)
        /// </summary>
        public Vector3 AngularAcceleration { get; private set; }

        /// <summary>
        /// Vehicle angular acceleration in the local coordinate system of the vehicle (rad/s^2)
        /// </summary>
        public Vector3 LocalAngularAcceleration => transform.InverseTransformDirection(AngularAcceleration);

        // VPP components
        private VPVehicleController _vehicleController;
        private VPCameraController _cameraController;

        [SerializeField] private VPWheelCollider[] _frontWheels;

        private Rigidbody _rigidbody;

        [Tooltip("Whether set wheel angle directly from Autoware or simulate with additive steering wheel input")]
        [SerializeField]
        private bool _simulateSteering;

        [Tooltip("Change applied to steering wheel per fixed update in degrees")]
        [Range(0f, 100f)]
        [SerializeField]
        private float _steerWheelInput = 5f;

        private int _vppSteerFromLastFrame;

        [Tooltip("Brake pedal percent on emergency brake. This value is mapped to [0, 10000] to match VPP input")]
        [Range(0f, 100f)]
        [SerializeField]
        private float _emergencyBrakePercent = 100f;

        public float CurrentSpeed { get; private set; }
        public float CurrentJerk { get; private set; }
        // private float _previousAcceleration;

        // Control mode variables
        private Dictionary<VPPControlMode, IVehicleControlMode> _controlModes;
        private IVehicleControlMode _currentMode;

        // RViz2 Update position variables
        [Header("RViz2 Update Position")]
        [SerializeField]
        private float _updatePositionOffsetY = 0.5f;

        [SerializeField] private float _updatePositionRayOriginY = 1000f;

        [Header("Calibration Mode")]
        [Tooltip(
            "Enable pedal calibration mode. Use Numpad(+,-,0) keys to set constant throttle/brake values for ease of calibration")]
        [SerializeField]
        private bool _doPedalCalibration;

        private int _brakeAmount;
        private int _throttleAmount;

        // Constants used for conversion between VPP and Autoware
        private const float VppToAutowareMultiplier = 0.0001f;
        private const int AutowareToVppMultiplier = 10000;

        // Event holders to subscribe
        private EgoVehiclePositionManager _egoVehiclePositionManager;

        private void Awake()
        {
            //TODO: Implement the control mode switch from simulator (mozzz)
            // Initialize the control mode as Autonomous
            ControlModeInput = VPPControlMode.Autonomous;
        }

        private void Start()
        {
            // get components
            _vehicleController = GetComponent<VPVehicleController>();
            _rigidbody = GetComponent<Rigidbody>();

            // set camera target to this vehicle
            _cameraController = FindObjectOfType<VPCameraController>();
            _cameraController.target = transform;

            InitializeControlModes();

            // Set initial vehicle gear to Park to prevent movement
            _vehicleController.data.bus[Channel.Input][InputData.AutomaticGear] = (int)Gearbox.AutomaticGear.P;

            // reset vpp stuff for multi-scene loading (no idea why, but it works)
            _vehicleController.enabled = false;
            _vehicleController.enabled = true;

            // Subscribe to events
            _egoVehiclePositionManager = FindObjectOfType<EgoVehiclePositionManager>();
            if (_egoVehiclePositionManager != null)
            {
                _egoVehiclePositionManager.OnEgoReset += ResetEgoPosition;
            }
        }

        // private void Update()
        // {
        //     // TODO: Implement the control mode switch from simulator (mozzz)
        //     UserSwitchControlMode();
        // }

        private void FixedUpdate()
        {
            // Update the ego position depending on RViz Input.
            if (WillUpdatePositionInput)
            {
                UpdateEgoPosition();
                WillUpdatePositionInput = false;
            }

            if (_doPedalCalibration)
            {
                PedalCalibrationMode();
            }
            else
            {
                // If keyboard enabled stop Autoware control.
                // if (_standardInput.enabled) return;

                // Control the vehicle based on the control mode
                ControlVehicle(ControlModeInput);
            }

            // Update the publisher values for VPPToRos2Publisher.cs
            CalculateCurrentValues();
            ReportVehicleState();
        }

        private void InitializeControlModes()
        {
            _controlModes = new Dictionary<VPPControlMode, IVehicleControlMode>
            {
                { VPPControlMode.NoCommand, new ControlMode.NoCommand() },
                { VPPControlMode.Autonomous, new ControlMode.Autonomous() },
                { VPPControlMode.AutonomousSteerOnly, new ControlMode.AutonomousSteerOnly() },
                { VPPControlMode.AutonomousVelocityOnly, new ControlMode.AutonomousVelocityOnly() },
                { VPPControlMode.Manual, new ControlMode.Manual() },
                { VPPControlMode.Disengaged, new ControlMode.Disengaged() },
                { VPPControlMode.NotReady, new ControlMode.NotReady() }
            };
        }

        private void ControlVehicle(VPPControlMode controlMode)
        {
            if (_controlModes.TryGetValue(controlMode, out _currentMode))
            {
                _currentMode.ExecuteControlMode(this);
            }
            else
            {
                Debug.LogWarning("Control mode is not recognized.");
            }
        }

        public void HandleHazardLights()
        {
            // TODO: this is implemented in Ros2ToVPPInput.cs, move it here (mozzz)
        }

        public void HandleTurnSignal()
        {
            // TODO: this is implemented in Ros2ToVPPInput.cs, move it here (mozzz)
        }

        public void HandleSteer()
        {
            if (_simulateSteering)
            {
                SimulateSteeringWheelInput();
            }
            else
            {
                SetWheelSteerAngleDirectly(_steerAngleInput);
            }
        }

        private void SetWheelSteerAngleDirectly(float angle)
        {
            foreach (var wheel in _frontWheels)
            {
                wheel.steerAngle = angle;
            }
        }

        // temp method to simulate steering wheel input with delay, also inaccurate (mozzz)
        private void SimulateSteeringWheelInput()
        {
            if (SteerAngleInput == 0) return;

            int steerAdjustment = SteerAngleInput > _vehicleController.wheelState[0].steerAngle
                ? (int)_steerWheelInput
                : -(int)_steerWheelInput;

            _vehicleController.data.bus[Channel.Input][InputData.Steer] = _vppSteerFromLastFrame + steerAdjustment;
            _vppSteerFromLastFrame = _vehicleController.data.bus[Channel.Input][InputData.Steer];
        }

        public void HandleGear()
        {
            _vehicleController.data.bus[Channel.Input][InputData.AutomaticGear] = (int)_automaticShiftInput;
        }

        public void HandleAcceleration()
        {
            SetThrottle((int)(_throttleInput * AutowareToVppMultiplier));
            SetBrake((int)(_brakeInput * AutowareToVppMultiplier));
        }

        /// <summary>
        /// Range:[0-10000]
        /// </summary>
        private void SetThrottle(int amount)
        {
            _vehicleController.data.bus[Channel.Input][InputData.Throttle] = amount;
        }

        /// <summary>
        /// Range:[0-10000]
        /// </summary>
        private void SetBrake(int amount)
        {
            _vehicleController.data.bus[Channel.Input][InputData.Brake] = amount;
        }

        private void CalculateCurrentValues()
        {
            // Store & calculate current values
            CurrentSpeed = _vehicleController.speed;
            LocalAcceleration = _vehicleController.localAcceleration;

            var lastAngularVelocity = AngularVelocity;
            AngularVelocity = _rigidbody.angularVelocity;
            AngularAcceleration = (AngularVelocity - lastAngularVelocity) / Time.deltaTime;
        }

        // TODO: report jerk state (mozzz)
        private void ReportVehicleState()
        {
            VpControlModeReport = ControlModeInput;
            VpHazardLightsReport = _vehicleSignalInput;
            VpTurnIndicatorReport = _vehicleSignalInput;
            VpSteeringReport = _frontWheels[0].steerAngle;
            VpGearReport = _vehicleController.data.bus[Channel.Vehicle][VehicleData.GearboxMode];
            VpVelocityReport =
                transform.InverseTransformDirection(_rigidbody.velocity);
            VpAngularVelocityReport = transform.InverseTransformDirection(AngularVelocity);
            VpThrottleStatusReport =
                _vehicleController.data.bus[Channel.Input][InputData.Throttle] * VppToAutowareMultiplier;
            VpBrakeStatusReport = _vehicleController.data.bus[Channel.Input][InputData.Brake] * VppToAutowareMultiplier;
            VpSteerStatusReport = _vehicleController.data.bus[Channel.Input][InputData.Steer] * VppToAutowareMultiplier;
        }

        private void UpdateEgoPosition()
        {
            // Method to update the position based on PositionInput
            Vector3 rayOrigin = new Vector3(PositionInput.x, _updatePositionRayOriginY, PositionInput.z);
            Vector3 rayDirection = Vector3.down;

            if (Physics.Raycast(rayOrigin, rayDirection, out RaycastHit hit, Mathf.Infinity))
            {
                PositionInput = new Vector3(PositionInput.x, hit.point.y + _updatePositionOffsetY, PositionInput.z);
                _vehicleController.HardReposition(PositionInput, RotationInput);
                _vehicleController.data.bus[Channel.Input][InputData.AutomaticGear] = (int)Gearbox.AutomaticGear.P;
                _cameraController.ResetCamera();
            }
            else
            {
                Debug.LogWarning(
                    "No mesh or collider detected on target location. Please ensure that the target location is on a mesh or collider.");
            }
        }

        /// <summary>
        /// Reset the ego vehicle to the spawn point.
        /// </summary>
        private void ResetEgoPosition(Vector3 position, Quaternion rotation)
        {
            _vehicleController.HardReposition(position, rotation, true);
            _vehicleController.data.bus[Channel.Input][InputData.AutomaticGear] = (int)Gearbox.AutomaticGear.P;
            _cameraController.ResetCamera();
        }

        // TODO: Method to switch control mode based on user input (mozzz)
        // Currently it is updated from UI with keyboard toggle.
        private void UserSwitchControlMode()
        {
        }

        // TODO: Add UI and documentation for this method (mozzz)
        /// <summary>
        /// Used to enable Numpad(+,-,0) keys to set constant throttle/brake values for ease of calibration. %10 increments
        /// </summary>
        private void PedalCalibrationMode()
        {
            _vehicleController.data.bus[Channel.Input][InputData.Throttle] = _throttleAmount;
            _vehicleController.data.bus[Channel.Input][InputData.Brake] = _brakeAmount;

            if (Input.GetKeyDown(KeyCode.KeypadPlus))
            {
                _throttleAmount += 1000;
            }

            if (Input.GetKeyDown(KeyCode.KeypadMinus))
            {
                _brakeAmount += 1000;
            }

            if (Input.GetKeyDown(KeyCode.Keypad0))
            {
                _throttleAmount = 0;
                _brakeAmount = 0;
            }
        }

        private void OnDestroy()
        {
            // Unsubscribe from events
            _egoVehiclePositionManager.OnEgoReset -= ResetEgoPosition;
        }
    }
}
