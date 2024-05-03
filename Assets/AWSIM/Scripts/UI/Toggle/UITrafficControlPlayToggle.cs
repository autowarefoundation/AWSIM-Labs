using UnityEngine;
using UnityEngine.UI;

namespace AWSIM.Scripts.UI.Toggle
{
    public class UITrafficControlPlayToggle : MonoBehaviour
    {
        [SerializeField] public Sprite sprite1;
        [SerializeField] public Sprite sprite2;

        private TrafficControlManager trafficControlManager;
        private Image image;

        private void Start()
        {
            image = GetComponent<Image>();
            trafficControlManager = FindObjectOfType<TrafficControlManager>();
            trafficControlManager.trafficPlayToggleEvent.AddListener(OnStatusChangeUpdateImage);
        }

        private void OnStatusChangeUpdateImage(bool isToggled)
        {
            image.sprite = isToggled ? sprite1 : sprite2;
        }

        private void OnDestroy()
        {
            trafficControlManager.trafficPlayToggleEvent.RemoveListener(OnStatusChangeUpdateImage);
        }
    }
}
