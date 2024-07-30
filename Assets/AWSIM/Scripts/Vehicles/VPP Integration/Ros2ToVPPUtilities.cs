using AWSIM.Scripts.Vehicles.VPP_Integration.Enums;
using VehiclePhysics;

namespace AWSIM.Scripts.Vehicles.VPP_Integration
{
    public static class Ros2ToVPPUtilities
    {
        public static Gearbox.AutomaticGear Ros2ToVPPShift(autoware_vehicle_msgs.msg.GearCommand gearCommand)
        {
            return gearCommand.Command switch
            {
                autoware_vehicle_msgs.msg.GearReport.NONE or autoware_vehicle_msgs.msg.GearReport.PARK => Gearbox
                    .AutomaticGear.P,
                autoware_vehicle_msgs.msg.GearReport.REVERSE => Gearbox.AutomaticGear.R,
                autoware_vehicle_msgs.msg.GearReport.NEUTRAL => Gearbox.AutomaticGear.N,
                autoware_vehicle_msgs.msg.GearReport.DRIVE or autoware_vehicle_msgs.msg.GearReport.LOW => Gearbox
                    .AutomaticGear.D,
                _ => Gearbox.AutomaticGear.P
            };
        }

        public static byte VPPToRos2Shift(int shift)
        {
            return shift switch
            {
                1 => autoware_vehicle_msgs.msg.GearReport.PARK,
                2 => autoware_vehicle_msgs.msg.GearReport.REVERSE,
                3 => autoware_vehicle_msgs.msg.GearReport.NEUTRAL,
                4 or 5 or 6 or 7 or 8 => autoware_vehicle_msgs.msg.GearReport.DRIVE,
                _ => autoware_vehicle_msgs.msg.GearReport.PARK
            };
        }

        // Turn Signal conversion
        public static VPPSignal Ros2ToVPPTurnSignal(
            autoware_vehicle_msgs.msg.TurnIndicatorsCommand turnIndicatorsCommand)
        {
            return turnIndicatorsCommand.Command switch
            {
                autoware_vehicle_msgs.msg.TurnIndicatorsCommand.DISABLE => VPPSignal.None,
                autoware_vehicle_msgs.msg.TurnIndicatorsCommand.ENABLE_LEFT => VPPSignal.Left,
                autoware_vehicle_msgs.msg.TurnIndicatorsCommand.ENABLE_RIGHT => VPPSignal.Right,
                _ => VPPSignal.None
            };
        }

        public static byte VPPToRos2TurnSignal(VPPSignal turnSignal)
        {
            return turnSignal switch
            {
                VPPSignal.None => autoware_vehicle_msgs.msg.TurnIndicatorsReport.DISABLE,
                VPPSignal.Left => autoware_vehicle_msgs.msg.TurnIndicatorsReport.ENABLE_LEFT,
                VPPSignal.Right => autoware_vehicle_msgs.msg.TurnIndicatorsReport.ENABLE_RIGHT,
                _ => autoware_vehicle_msgs.msg.TurnIndicatorsReport.DISABLE
            };
        }

        // Hazard Lights conversion
        public static VPPSignal Ros2ToVPPHazard(
            autoware_vehicle_msgs.msg.HazardLightsCommand hazardLightsCommand)
        {
            return hazardLightsCommand.Command switch
            {
                autoware_vehicle_msgs.msg.HazardLightsCommand.ENABLE => VPPSignal.Hazard,
                autoware_vehicle_msgs.msg.HazardLightsCommand.DISABLE => VPPSignal.None,
                _ => VPPSignal.None
            };
        }

        public static byte VPPToRos2Hazard(VPPSignal turnSignal)
        {
            return turnSignal switch
            {
                VPPSignal.Hazard => autoware_vehicle_msgs.msg.HazardLightsReport.ENABLE,
                VPPSignal.None => autoware_vehicle_msgs.msg.HazardLightsReport.DISABLE,
                _ => autoware_vehicle_msgs.msg.HazardLightsReport.DISABLE
            };
        }

        // Control Mode conversion
        public static VPPControlMode Ros2ToVPPControlMode(
            autoware_vehicle_msgs.msg.ControlModeReport controlMode)
        {
            return controlMode.Mode switch
            {
                autoware_vehicle_msgs.msg.ControlModeReport.NO_COMMAND => VPPControlMode.NoCommand,
                autoware_vehicle_msgs.msg.ControlModeReport.AUTONOMOUS => VPPControlMode.Autonomous,
                autoware_vehicle_msgs.msg.ControlModeReport.AUTONOMOUS_STEER_ONLY => VPPControlMode.AutonomousSteerOnly,
                autoware_vehicle_msgs.msg.ControlModeReport.AUTONOMOUS_VELOCITY_ONLY => VPPControlMode.AutonomousVelocityOnly,
                autoware_vehicle_msgs.msg.ControlModeReport.MANUAL => VPPControlMode.Manual,
                autoware_vehicle_msgs.msg.ControlModeReport.DISENGAGED => VPPControlMode.Disengaged,
                autoware_vehicle_msgs.msg.ControlModeReport.NOT_READY => VPPControlMode.NotReady,
                _ => VPPControlMode.NoCommand
            };
        }

        public static byte VPPToRos2ControlMode(VPPControlMode controlMode)
        {
            return controlMode switch
            {
                VPPControlMode.NoCommand => autoware_vehicle_msgs.msg.ControlModeReport.NO_COMMAND,
                VPPControlMode.Autonomous => autoware_vehicle_msgs.msg.ControlModeReport.AUTONOMOUS,
                VPPControlMode.AutonomousSteerOnly => autoware_vehicle_msgs.msg.ControlModeReport.AUTONOMOUS_STEER_ONLY,
                VPPControlMode.AutonomousVelocityOnly => autoware_vehicle_msgs.msg.ControlModeReport.AUTONOMOUS_VELOCITY_ONLY,
                VPPControlMode.Manual => autoware_vehicle_msgs.msg.ControlModeReport.MANUAL,
                VPPControlMode.Disengaged => autoware_vehicle_msgs.msg.ControlModeReport.DISENGAGED,
                VPPControlMode.NotReady => autoware_vehicle_msgs.msg.ControlModeReport.NOT_READY,
                _ => autoware_vehicle_msgs.msg.ControlModeReport.NO_COMMAND
            };
        }
    }
}
