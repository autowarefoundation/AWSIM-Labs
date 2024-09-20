using UnityEngine;
using UnityEngine.UI;
using VehiclePhysics;

namespace AWSIM.Scripts.UI
{
    public class VehicleDashboard : MonoBehaviour
    {
        private VPVehicleController _vehicleController;
        private Text _speedText;
        private Text _gearText;
        private Text _absText;
        private Text _tcsText;
        private Text _espText;
        private Text _asrText;

        private void Start()
        {
            _vehicleController = FindObjectOfType<VPVehicleController>();
            _speedText = transform.Find("Speed").GetComponent<Text>();
            _gearText = transform.Find("Gear").GetComponent<Text>();
            _absText = transform.Find("ABS").GetComponent<Text>();
            _tcsText = transform.Find("TCS").GetComponent<Text>();
            _espText = transform.Find("ESP").GetComponent<Text>();
            _asrText = transform.Find("ASR").GetComponent<Text>();
        }

        private void Update()
        {
            _speedText.text = _vehicleController.speed.ToString("F0");
            // _gearText.text = _vehicleController.data.bus[][VehicleData.GearboxGear].ToString();
        }
    }
}