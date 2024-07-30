using System.Collections.Generic;

namespace AWSIM.Scripts.Vehicles.VPP_Integration
{
    /// <summary>
    /// Pedal map struct
    /// </summary>
    public struct PedalMap
    {
        public Dictionary<float, List<float>> Map;
        public Dictionary<float, List<float>> MapVertical;
        public List<float> MapHeaders;
    }
}
