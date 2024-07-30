using ROS2;
using UnityEngine;

namespace AWSIM.Scripts.Vehicles.VPP_Integration
{
    public class VPPToRos2Publisher : MonoBehaviour
    {
        [Header("AutowareVPPAdapter")]
        [SerializeField]
        private AutowareVPPAdapter _adapter;

        [Header("Ros2 fields")]
        [SerializeField] private string _controlModeReportTopic = "/vehicle/status/control_mode";
        [SerializeField] private string _gearReportTopic = "/vehicle/status/gear_status";
        [SerializeField] private string _steeringReportTopic = "/vehicle/status/steering_status";
        [SerializeField] private string _turnIndicatorsReportTopic = "/vehicle/status/turn_indicators_status";
        [SerializeField] private string _hazardLightsReportTopic = "/vehicle/status/hazard_lights_status";
        [SerializeField] private string _velocityReportTopic = "/vehicle/status/velocity_status";
        [SerializeField] private string _frameId = "base_link";

        [Range(1, 60)][SerializeField] private int _publishHz = 30;
        [SerializeField] private QoSSettings _qosSettings;

        // messages
        private autoware_vehicle_msgs.msg.ControlModeReport _controlModeReportMsg;
        private autoware_vehicle_msgs.msg.GearReport _gearReportMsg;
        private autoware_vehicle_msgs.msg.SteeringReport _steeringReportMsg;
        private autoware_vehicle_msgs.msg.TurnIndicatorsReport _turnIndicatorsReportMsg;
        private autoware_vehicle_msgs.msg.HazardLightsReport _hazardLightsReportMsg;
        private autoware_vehicle_msgs.msg.VelocityReport _velocityReportMsg;

        // publisher
        private IPublisher<autoware_vehicle_msgs.msg.ControlModeReport> _controlModeReportPublisher;
        private IPublisher<autoware_vehicle_msgs.msg.GearReport> _gearReportPublisher;
        private IPublisher<autoware_vehicle_msgs.msg.SteeringReport> _steeringReportPublisher;
        private IPublisher<autoware_vehicle_msgs.msg.TurnIndicatorsReport> _turnIndicatorsReportPublisher;
        private IPublisher<autoware_vehicle_msgs.msg.HazardLightsReport> _hazardLightsReportPublisher;
        private IPublisher<autoware_vehicle_msgs.msg.VelocityReport> _velocityReportPublisher;

        private bool _isInitialized;
        private float _timer;

        private void Start()
        {
            // Create publisher
            var qos = _qosSettings.GetQoSProfile();
            _controlModeReportPublisher =
                SimulatorROS2Node.CreatePublisher<autoware_vehicle_msgs.msg.ControlModeReport>(_controlModeReportTopic,
                    qos);
            _gearReportPublisher =
                SimulatorROS2Node.CreatePublisher<autoware_vehicle_msgs.msg.GearReport>(_gearReportTopic, qos);
            _steeringReportPublisher =
                SimulatorROS2Node.CreatePublisher<autoware_vehicle_msgs.msg.SteeringReport>(_steeringReportTopic, qos);
            _turnIndicatorsReportPublisher =
                SimulatorROS2Node.CreatePublisher<autoware_vehicle_msgs.msg.TurnIndicatorsReport>(
                    _turnIndicatorsReportTopic, qos);
            _hazardLightsReportPublisher =
                SimulatorROS2Node.CreatePublisher<autoware_vehicle_msgs.msg.HazardLightsReport>(
                    _hazardLightsReportTopic, qos);
            _velocityReportPublisher =
                SimulatorROS2Node.CreatePublisher<autoware_vehicle_msgs.msg.VelocityReport>(_velocityReportTopic, qos);

            // Create msg.
            _controlModeReportMsg = new autoware_vehicle_msgs.msg.ControlModeReport();
            _gearReportMsg = new autoware_vehicle_msgs.msg.GearReport();
            _steeringReportMsg = new autoware_vehicle_msgs.msg.SteeringReport();
            _turnIndicatorsReportMsg = new autoware_vehicle_msgs.msg.TurnIndicatorsReport();
            _hazardLightsReportMsg = new autoware_vehicle_msgs.msg.HazardLightsReport();
            _velocityReportMsg = new autoware_vehicle_msgs.msg.VelocityReport()
            {
                Header = new std_msgs.msg.Header()
                {
                    Frame_id = _frameId,
                }
            };

            _isInitialized = true;
        }

        private void FixedUpdate()
        {
            if (_isInitialized == false)
                return;

            // Update timer.
            _timer += Time.fixedDeltaTime;

            // Matching publish to hz.
            var interval = 1.0f / _publishHz;
            interval -= 0.00001f; // Allow for accuracy errors.
            if (_timer < interval)
                return;
            _timer = 0;

            // ControlModeReport
            _controlModeReportMsg.Mode = Ros2ToVPPUtilities.VPPToRos2ControlMode(_adapter.VPControlModeReport);

            // GearReport
            _gearReportMsg.Report = Ros2ToVPPUtilities.VPPToRos2Shift(_adapter.VPGearReport);

            // SteeringReport
            _steeringReportMsg.Steering_tire_angle = -1 * _adapter.VPSteeringReport * Mathf.Deg2Rad;

            // TurnIndicatorsReport
            _turnIndicatorsReportMsg.Report = Ros2ToVPPUtilities.VPPToRos2TurnSignal(_adapter.VehicleSignalInput);

            // HazardLightsReport
            _hazardLightsReportMsg.Report = Ros2ToVPPUtilities.VPPToRos2Hazard(_adapter.VehicleSignalInput);

            // VelocityReport
            var rosLinearVelocity = ROS2Utility.UnityToRosPosition(_adapter.VPVelocityReport);
            var rosAngularVelocity = ROS2Utility.UnityToRosPosition(_adapter.VPAngularVelocityReport);
            _velocityReportMsg.Longitudinal_velocity = rosLinearVelocity.x;
            _velocityReportMsg.Lateral_velocity = rosLinearVelocity.y;
            _velocityReportMsg.Heading_rate = rosAngularVelocity.z;

            // Update Stamp
            var time = SimulatorROS2Node.GetCurrentRosTime();
            _controlModeReportMsg.Stamp = time;
            _gearReportMsg.Stamp = time;
            _steeringReportMsg.Stamp = time;
            _turnIndicatorsReportMsg.Stamp = time;
            _hazardLightsReportMsg.Stamp = time;
            var velocityReportMsgHeader = _velocityReportMsg as MessageWithHeader;
            SimulatorROS2Node.UpdateROSTimestamp(ref velocityReportMsgHeader);

            // publish
            _controlModeReportPublisher.Publish(_controlModeReportMsg);
            _gearReportPublisher.Publish(_gearReportMsg);
            _steeringReportPublisher.Publish(_steeringReportMsg);
            _turnIndicatorsReportPublisher.Publish(_turnIndicatorsReportMsg);
            _hazardLightsReportPublisher.Publish(_hazardLightsReportMsg);
            _velocityReportPublisher.Publish(_velocityReportMsg);
        }

        private void OnDestroy()
        {
            SimulatorROS2Node.RemovePublisher<autoware_vehicle_msgs.msg.ControlModeReport>(_controlModeReportPublisher);
            SimulatorROS2Node.RemovePublisher<autoware_vehicle_msgs.msg.GearReport>(_gearReportPublisher);
            SimulatorROS2Node.RemovePublisher<autoware_vehicle_msgs.msg.SteeringReport>(_steeringReportPublisher);
            SimulatorROS2Node.RemovePublisher<autoware_vehicle_msgs.msg.TurnIndicatorsReport>(
                _turnIndicatorsReportPublisher);
            SimulatorROS2Node.RemovePublisher<autoware_vehicle_msgs.msg.HazardLightsReport>(
                _hazardLightsReportPublisher);
            SimulatorROS2Node.RemovePublisher<autoware_vehicle_msgs.msg.VelocityReport>(_velocityReportPublisher);
        }
    }
}
