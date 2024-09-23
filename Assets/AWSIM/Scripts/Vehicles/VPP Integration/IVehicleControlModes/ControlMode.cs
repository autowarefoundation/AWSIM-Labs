namespace AWSIM.Scripts.Vehicles.VPP_Integration.IVehicleControlModes
{
    // Vehicle control mode classes
    public abstract class ControlMode
    {
        public class NoCommand : IVehicleControlMode
        {
            public void ExecuteControlMode(AutowareVPPAdapter adapter)
            {
                // Debug.Log("Control mode: No command");
            }
        }

        public class Autonomous : IVehicleControlMode
        {
            public void ExecuteControlMode(AutowareVPPAdapter adapter)
            {
                adapter.HandleHazardLights();
                adapter.HandleTurnSignal();
                adapter.HandleAcceleration();
                adapter.HandleGear();
                adapter.HandleSteer();
            }
        }

        public class AutonomousSteerOnly : IVehicleControlMode
        {
            public void ExecuteControlMode(AutowareVPPAdapter adapter)
            {
                adapter.HandleHazardLights();
                adapter.HandleTurnSignal();
                adapter.HandleGear();
                adapter.HandleSteer();
            }
        }

        public class AutonomousVelocityOnly : IVehicleControlMode
        {
            public void ExecuteControlMode(AutowareVPPAdapter adapter)
            {
                adapter.HandleHazardLights();
                adapter.HandleTurnSignal();
                adapter.HandleAcceleration();
                adapter.HandleGear();
            }
        }

        public class Manual : IVehicleControlMode
        {
            public void ExecuteControlMode(AutowareVPPAdapter adapter)
            {
                // Debug.Log("Control mode: Manual");
            }
        }

        public class Disengaged : IVehicleControlMode
        {
            public void ExecuteControlMode(AutowareVPPAdapter adapter)
            {
                // Debug.Log("Control mode: Disengaged");
            }
        }

        public class NotReady : IVehicleControlMode
        {
            public void ExecuteControlMode(AutowareVPPAdapter adapter)
            {
                // Debug.Log("Control mode: Not ready");
            }
        }
    }
}
