using UnityEngine;
using UnityEngine.UI;

namespace AWSIM.Scripts.UI.Toggle
{
    public class UITrafficControlVisibilityToggle : MonoBehaviour
    {
        [SerializeField] private Sprite sprite1;
        [SerializeField] private Sprite sprite2;

        private TrafficControlManager trafficControlManager;
        private Image image;

        private void Start()
        {
            image = GetComponent<Image>();
            trafficControlManager = FindObjectOfType<TrafficControlManager>();
            trafficControlManager.trafficVisibilityToggleEvent.AddListener(OnStatusChangeUpdateImage);
        }

        private void OnStatusChangeUpdateImage(bool isToggled)
        {
            image.sprite = isToggled ? sprite1 : sprite2;
        }

        private void OnDestroy()
        {
            trafficControlManager.trafficVisibilityToggleEvent.RemoveListener(OnStatusChangeUpdateImage);
        }
    }
}
