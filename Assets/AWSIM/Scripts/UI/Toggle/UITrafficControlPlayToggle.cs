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
            trafficControlManager.TrafficPlayToggleEvent.AddListener(OnStatusChangeUpdateImage);
        }

        public void Activate()
        {
            enabled = true;
            Start();
        }

        private void OnStatusChangeUpdateImage(bool isToggled)
        {
            image.sprite = isToggled ? sprite1 : sprite2;
        }

        private void OnDestroy()
        {
            trafficControlManager.TrafficPlayToggleEvent.RemoveListener(OnStatusChangeUpdateImage);
        }
    }
}
