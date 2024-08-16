using UnityEngine;
using UnityEngine.UI;

namespace AWSIM.Scripts.UI.Toggle
{
    public class UIMainCameraToggle : MonoBehaviour
    {
        private Camera _mainCam;
        private UnityEngine.UI.Toggle _toggle;
        private Image _mainCanvasImage;

        /// <summary>
        /// Key for storing user's main camera render preference:
        /// Not set = -1 | Disabled = 0 | Enabled = 1
        /// </summary>
        private const string UserMainCameraRenderKey = "UserMainCameraRender";

        private void Awake()
        {
            if (!PlayerPrefs.HasKey(UserMainCameraRenderKey))
            {
                PlayerPrefs.SetInt(UserMainCameraRenderKey, -1);
            }
        }

        private void Start()
        {
            // Get components
            _mainCam = Camera.main;
            _mainCanvasImage = GameObject.FindGameObjectWithTag("MainCanvas").GetComponent<Image>();
            _toggle = GetComponent<UnityEngine.UI.Toggle>();

            // Set main camera render preference
            if (_mainCam && PlayerPrefs.GetInt(UserMainCameraRenderKey) != -1)
            {
                if (PlayerPrefs.GetInt(UserMainCameraRenderKey) == 1)
                {
                    _mainCam.enabled = true;
                }
                else if (PlayerPrefs.GetInt(UserMainCameraRenderKey) == 0)
                {
                    _mainCam.enabled = false;
                }
            }

            if (_mainCam) _toggle.isOn = _mainCam.enabled;
            _toggle.onValueChanged.AddListener(ToggleMainCamera);
        }

        public void Activate()
        {
            enabled = true;
            Start();
        }

        // Simple toggle for main camera
        private void ToggleMainCamera(bool value)
        {
            _mainCam.enabled = value;
            _mainCanvasImage.enabled = !value;

            // Save user's preference
            PlayerPrefs.SetInt(UserMainCameraRenderKey, value ? 1 : 0);
        }
    }
}
