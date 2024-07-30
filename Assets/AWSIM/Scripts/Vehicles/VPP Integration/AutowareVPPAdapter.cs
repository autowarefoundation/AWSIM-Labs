using System;
using System.Collections.Generic;
using AWSIM.Scripts.Vehicles.VPP_Integration.Enums;
using AWSIM.Scripts.Vehicles.VPP_Integration.IVehicleControlModes;
using UnityEngine;
using VehiclePhysics;
using Quaternion = UnityEngine.Quaternion;
using Vector3 = UnityEngine.Vector3;

namespace AWSIM.Scripts.Vehicles.VPP_Integration
{
    public class AutowareVPPAdapter : MonoBehaviour
    {
        /// <summary>
        /// Inputs
        /// </summary>

        // Initial Position inputs from Rviz
        [NonSerialized] public bool WillUpdatePositionInput;

        [NonSerialized] public Vector3 PositionInput;
        [NonSerialized] public Quaternion RotationInput;

        /// Control inputs from Autoware

        //longitudinal
        [NonSerialized] public float VelocityInput;

        [NonSerialized] public bool IsAccelerationDefinedInput;
        [NonSerialized] public float AccelerationInput;
        [NonSerialized] public bool IsJerkDefinedInput;
        [NonSerialized] public float JerkInput;
        private float _velocityInput => VelocityInput;
        private bool _isAccelerationDefinedInput => IsAccelerationDefinedInput;
        private float _accelerationInput => AccelerationInput;
        private bool _isJerkDefinedInput => IsJerkDefinedInput;
        private float _jerkInput => JerkInput;

        //lateral
        [NonSerialized] public float SteerAngleInput;
        [NonSerialized] public bool IsDefinedSteeringTireRotationRateInput;
        [NonSerialized] public float SteeringTireRotationRateInput;
        private float _steerAngleInput => SteerAngleInput;
        private bool _isDefinedSteeringTireRotationRateInput => IsDefinedSteeringTireRotationRateInput;
        private float _steeringTireRotationRateInput => SteeringTireRotationRateInput;

        // Gear input from Autoware
        [NonSerialized] public Gearbox.AutomaticGear AutomaticShiftInput;
        private Gearbox.AutomaticGear _automaticShiftInput => AutomaticShiftInput;

        // Signal input from Autoware
        [NonSerialized] public VPPSignal VehicleSignalInput;
        private VPPSignal _vehicleSignalInput => VehicleSignalInput;

        // Emergency input from Autoware
        [NonSerialized] public bool IsEmergencyInput;
        private bool _isEmergencyInput => IsEmergencyInput;

        // Control mode input from Autoware
        [NonSerialized] public VPPControlMode ControlModeInput;
        private VPPControlMode _controlModeInput => ControlModeInput;

        // Outputs from Unity/VPP
        [NonSerialized] public VPPControlMode VPControlModeReport;
        [NonSerialized] public int VPGearReport;
        [NonSerialized] public Vector3 VPVelocityReport;
        [NonSerialized] public Vector3 VPAngularVelocityReport;
        [NonSerialized] public float VPSteeringReport;
        [NonSerialized] public VPPSignal VPTurnIndicatorReport;
        [NonSerialized] public VPPSignal VPHazardLightsReport;

        // VPP components
        private VPVehicleController _vehicleController;

        [Header("Lateral")][SerializeField] private VPWheelCollider[] _frontWheels;

        private Rigidbody _rigidbody;
        private VehiclePedalMapLoader _pedalMap;

        /// <summary>
        /// Whether set wheel angle directly from Autoware or simulate with additive steering wheel input
        /// </summary>
        [SerializeField] private bool _simulateSteering;

        /// <summary>
        /// Change applied to steering wheel per fixed update
        /// </summary>
        [Range(0f, 100f)][SerializeField] private float _steerWheelInput = 5f;

        private int _vppSteerFromLastFrame;

        /// <summary>
        /// Use pedal maps to control throttle and brake. If set to false, will use velocity and acceleration input to control throttle and brake
        /// </summary>
        [Header("Longitudinal")]
        [SerializeField]
        private bool _doUsePedalMaps;

        private int _vppThrottleFromLastFrame;
        private int _vppBrakeFromLastFrame;

        /// <summary>
        /// Brake pedal percent on emergency brake
        /// This value is mapped to [0, 10000] to match VPP input
        /// </summary>
        [Range(0f, 100f)][SerializeField] private float _emergencyBrakePercent = 100f;

        private float _currentSpeed;
        private float _previousAcceleration;
        private float _currentJerk;

        // Control mode variables
        private Dictionary<VPPControlMode, IVehicleControlMode> _controlModes;
        private IVehicleControlMode _currentMode;

        // RViz2 Update position variables
        [Header("RViz2 Update Position")]
        [SerializeField]
        private float _updatePositionOffsetY = 1.33f;

        [SerializeField] private float _updatePositionRayOriginY = 1000f;

        // private static float GainAdjuster(float error)
        // {
        //     // Example: Reduce gains as error magnitude increases
        //     return 1.0f / (1.0f + Mathf.Abs(error));
        // }
        //
        // // PID stuff
        // private PIDController _pidController;
        // private VehicleState _vehicleState = VehicleState.Braking;
        //
        // // Threshold to determine if the vehicle is close enough to the target velocity
        // [Header("PID Params")] [SerializeField]
        // private float _velocityThreshold = 0.01f;
        //
        // [SerializeField] private float kp;
        // [SerializeField] private float ki;
        // [SerializeField] private float kd;
        // public float targetVel;
        private void Awake()
        {
            // Initialize the control mode to Autonomous
            ControlModeInput = VPPControlMode.Autonomous;
        }

        private void Start()
        {
            _vehicleController = GetComponent<VPVehicleController>();
            _rigidbody = GetComponent<Rigidbody>();
            _pedalMap = GetComponent<VehiclePedalMapLoader>();
            // _pidController = new PIDController(kp, ki, kd, 0, 10000, 5000, 0.01f, 0.2f, true,
            //     gainAdjuster: GainAdjuster);

            // Initialize the control modes
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

        private void Update()
        {
            // TODO: Implement the control mode switch from simulator (mozzz)
            UserSwitchControlMode();

            // // update pid
            // if (Input.GetKey(KeyCode.K))
            // {
            //     _pidController = new PIDController(kp, ki, kd, 0, 10000, 5000, 0.01f, 0.2f, true,
            //         gainAdjuster: GainAdjuster);
            // }
            //
            // if (Input.GetKey(KeyCode.L))
            // {
            //     AutomaticShiftInput = Gearbox.AutomaticGear.D;
            //     VelocityInput = targetVel;
            // }
            //
            // if (Input.GetKey(KeyCode.J))
            // {
            //     IsEmergencyInput = !IsEmergencyInput;
            // }
        }

        private void FixedUpdate()
        {
            // Update the ego position depending on RViz Input.
            if (WillUpdatePositionInput)
            {
                UpdateEgoPosition();
                WillUpdatePositionInput = false;
            }

            // Control the vehicle based on the control mode
            ControlVehicle(_controlModeInput);
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

            ReportVehicleState();
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
                // set wheel angle directly
                _vehicleController.wheelState[0].steerAngle = _steerAngleInput;
                _vehicleController.wheelState[1].steerAngle = _steerAngleInput;
                foreach (var wheel in _frontWheels)
                {
                    wheel.steerAngle = _steerAngleInput;
                }
            }
        }

        private void SimulateSteeringWheelInput()
        {
            // simulate steering wheel input
            if (_steerAngleInput == 0)
                return;

            if (_steerAngleInput > _vehicleController.wheelState[0].steerAngle)
            {
                _vehicleController.data.bus[Channel.Input][InputData.Steer] =
                    _vppSteerFromLastFrame + (int)_steerWheelInput;
            }
            else if (_steerAngleInput < _vehicleController.wheelState[0].steerAngle)
            {
                _vehicleController.data.bus[Channel.Input][InputData.Steer] =
                    _vppSteerFromLastFrame - (int)_steerWheelInput;
            }

            _vppSteerFromLastFrame = _vehicleController.data.bus[Channel.Input][InputData.Steer];
        }

        public void HandleGear()
        {
            _vehicleController.data.bus[Channel.Input][InputData.AutomaticGear] = (int)_automaticShiftInput;
        }

        public void HandleAcceleration()
        {
            // Store current values
            _currentSpeed = _vehicleController.speed;
            if (_isJerkDefinedInput)
            {
                float currentAcceleration = _vehicleController.localAcceleration.magnitude;
                _currentJerk = (currentAcceleration - _previousAcceleration) / Time.fixedDeltaTime;
                _previousAcceleration = currentAcceleration;
            }

            // return after applying pedals if using pedal maps
            if (_doUsePedalMaps)
            {
                if (_isEmergencyInput)
                {
                    // set emergency brake
                    var emergencyBrakePercent = RemapValue(_emergencyBrakePercent, 0f, 100f, 0, 10000);
                    SetBrake(emergencyBrakePercent);
                }
                else
                {
                    // TODO: for some reason "isAccelerationDefinedInput" is always false from the Autoware side, can't use it (mozzz)
                    // set accel
                    if (_velocityInput > 0)
                    {
                        var throttlePercent =
                            _pedalMap.GetPedalPercent(_pedalMap.AccelMap, _accelerationInput, _currentSpeed);
                        throttlePercent = RemapValue(throttlePercent, 0f, 0.5f, 0, 5000);
                        SetThrottle((int)throttlePercent);
                    }

                    // set brake
                    if (_velocityInput < 0)
                    {
                        var brakePercent =
                            _pedalMap.GetPedalPercent(_pedalMap.BrakeMap, _accelerationInput, _currentSpeed);
                        brakePercent = RemapValue(brakePercent, 0f, 0.8f, 0, 8000);
                        SetBrake((int)brakePercent);
                    }
                }

                return;
            }

            // handle pedals if not using pedal maps
            // if (_isEmergencyInput)
            // {
            //     // set emergency brake
            //     var emergencyBrakePercent = RemapValue(_emergencyBrakePercent, 0f, 100f, 0, 10000);
            //     SetBrake(emergencyBrakePercent);
            // }
            // else
            // {
            //     // Calculate the velocity error
            //     float velocityError = _velocityInput - _currentSpeed;
            //
            //     // Compute the required action from PID controller
            //     float controlOutput = _pidController.Compute(velocityError, Time.fixedDeltaTime) * 100000;
            //
            //     // Determine the desired state based on control output
            //     if (Mathf.Abs(velocityError) < _velocityThreshold)
            //     {
            //         _vehicleState = VehicleState.Coasting;
            //     }
            //     else if (controlOutput > 0)
            //     {
            //         _vehicleState = VehicleState.Accelerating;
            //     }
            //     else
            //     {
            //         _vehicleState = VehicleState.Braking;
            //     }
            //
            //     Debug.Log("Velocity diff: " + velocityError);
            //     Debug.Log("PID output throttle: " + controlOutput);
            //     Debug.Log("VehicleState: " + _vehicleState);
            //
            //
            //     // Execute actions based on current state
            //     switch (_vehicleState)
            //     {
            //         case VehicleState.Accelerating:
            //             ApplyThrottle(controlOutput);
            //             ApplyBrake(0);
            //             break;
            //
            //         case VehicleState.Braking:
            //             ApplyThrottle(0);
            //             ApplyBrake(-controlOutput);
            //             break;
            //
            //         case VehicleState.Coasting:
            //             // Coasting state: apply coast throttle
            //             // float coastThrottle = _pidController.ComputeCoastThrottle(_currentSpeed);
            //             // ApplyThrottle(coastThrottle);
            //             // ApplyBrake(0); // Ensure brakes are not applied
            //             break;
            //         case VehicleState.Idle:
            //             break;
            //         default:
            //             throw new ArgumentOutOfRangeException();
            //     }
            // }

            // Debug.Log("Speed Input | Current Speed : " +
            //           _velocityInput + " | " + _currentSpeed);
            // Debug.Log("Accel Input | Current Accel : " +
            //           _accelerationInput + " | " + _vehicleController.localAcceleration.magnitude);
            // Debug.Log("Throttle % |Brake % : " +
            //           _vehicleController.data.bus[Channel.Input][InputData.Throttle] + " | " +
            //           _vehicleController.data.bus[Channel.Input][InputData.Brake]);
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

        // TODO: report jerk state (mozzz)
        private void ReportVehicleState()
        {
            VPControlModeReport = _controlModeInput;
            VPHazardLightsReport = _vehicleSignalInput;
            VPTurnIndicatorReport = _vehicleSignalInput;
            VPSteeringReport = _frontWheels[0].steerAngle;
            VPGearReport = _vehicleController.data.bus[Channel.Vehicle][VehicleData.GearboxMode];
            VPVelocityReport = transform.InverseTransformDirection(_rigidbody.velocity.normalized * _currentSpeed);
            VPAngularVelocityReport = transform.InverseTransformDirection(_rigidbody.angularVelocity);
        }

        private void UpdateEgoPosition()
        {
            // Method to update the position based on PositionInput
            Vector3 rayOrigin = new Vector3(PositionInput.x, _updatePositionRayOriginY, PositionInput.z);
            Vector3 rayDirection = Vector3.down;

            if (Physics.Raycast(rayOrigin, rayDirection, out RaycastHit hit, Mathf.Infinity))
            {
                PositionInput = new Vector3(PositionInput.x, hit.point.y + _updatePositionOffsetY, PositionInput.z);
                transform.SetPositionAndRotation(PositionInput, RotationInput);
            }
            else
            {
                Debug.LogWarning(
                    "No mesh or collider detected on target location. Please ensure that the target location is on a mesh or collider.");
            }
        }

        // TODO: Method to switch control mode based on user input (mozzz)
        private void UserSwitchControlMode()
        {
        }

        /// <summary>
        /// Remap Autoware input value to VPP input
        /// Range for pedals: [0,10000]
        /// Range for steer: [-10000,10000]
        /// </summary>
        private static int RemapValue(float value, float from1, float to1, int from2, int to2)
        {
            return (int)((value - from1) / (to1 - from1) * (to2 - from2) + from2);
        }
    }
}
