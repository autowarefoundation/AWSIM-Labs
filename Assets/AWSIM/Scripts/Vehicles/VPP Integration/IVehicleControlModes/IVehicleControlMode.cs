namespace AWSIM.Scripts.Vehicles.VPP_Integration.IVehicleControlModes
{
    // Vehicle control mode interface
    public interface IVehicleControlMode
    {
        void ExecuteControlMode(AutowareVPPAdapter adapter);
    }
}
