using UnityEngine;
using UnityEngine.UI;

namespace AWSIM.Scripts.UI.Toggle
{
    public class UIMainCameraToggle : MonoBehaviour
    {
        private Camera _mainCam;
        private UnityEngine.UI.Toggle _toggle;
        private Image _mainCanvasImage;

        private void Start()
        {
            // Get components
            _mainCam = Camera.main;
            _mainCanvasImage = GameObject.FindGameObjectWithTag("MainCanvas").GetComponent<Image>();
            _toggle = GetComponent<UnityEngine.UI.Toggle>();
            if (_mainCam != null) _toggle.isOn = _mainCam.enabled;
            _toggle.onValueChanged.AddListener(ToggleMainCamera);
        }

        // Simple toggle for main camera
        private void ToggleMainCamera(bool value)
        {
            _mainCam.enabled = value;
            _mainCanvasImage.enabled = !value;
        }
    }
}
