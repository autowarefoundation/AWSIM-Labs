using System;
using System.Globalization;
using UnityEngine;
using UnityEngine.UI;

namespace AWSIM.Scripts.UI
{
    public class UITrafficVehicleDensity : MonoBehaviour
    {
        [SerializeField] private Text _textField;

        private TrafficControlManager _trafficControlManager;
        private Slider _trafficTargetVehicleCountSlider;

        private void Start()
        {
            _trafficTargetVehicleCountSlider = GetComponent<Slider>();
            _trafficControlManager = FindObjectOfType<TrafficControlManager>();

            InitializeSlider();
        }

        public void TrafficManagerSetTargetVehicleCount()
        {
            var targetValue = Convert.ToInt32(_trafficTargetVehicleCountSlider.value);
            _trafficControlManager.TargetVehicleCount = targetValue;
            _textField.text = targetValue.ToString();

            _trafficControlManager.TrafficManagerUpdate();
        }
        private void InitializeSlider()
        {
            _trafficTargetVehicleCountSlider.maxValue = _trafficControlManager.GetMaxVehicleCount();
            _trafficTargetVehicleCountSlider.value = _trafficControlManager.GetTargetVehicleCount();
            _textField.text = _trafficTargetVehicleCountSlider.value.ToString(CultureInfo.InvariantCulture);
            _textField.Rebuild(CanvasUpdate.PostLayout);
        }
    }
}
