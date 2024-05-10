using UnityEngine;
using UnityEngine.UI;

namespace AWSIM.Scripts.UI
{
    public class UISideBarHandler : MonoBehaviour
    {
        [SerializeField] public float uiAnimationLerpValue;

        [SerializeField] private GameObject sideBar;
        [SerializeField] private Vector2 sideBarPositionActive;
        [SerializeField] private Vector2 sideBarPositionDisabled;

        [SerializeField] private GameObject sideBarChevronButton;
        [SerializeField] private Vector2 sideChevronPositionOpen;
        [SerializeField] private Vector2 sideChevronPositionClosed;

        [SerializeField] public Sprite chevronSpriteRight;
        [SerializeField] public Sprite chevronSpriteLeft;

        private void Start()
        {
            // Disable sidebar if enabled by default
            sideBar.SetActive(false);
        }

        public void ToggleSideBar()
        {
            sideBarChevronButton.transform.GetChild(0).GetComponent<Image>().sprite = sideBar.activeSelf
                ? chevronSpriteLeft
                : chevronSpriteRight;
            var sideBarRect = sideBar.GetComponent<RectTransform>();
            var sideBarChevronRect = sideBarChevronButton.GetComponent<RectTransform>();

            if (!sideBar.activeSelf)
            {
                sideBar.SetActive(true);
                StartCoroutine(UIFunctions.LerpUIRectPosition(sideBarRect, sideBarPositionActive,
                    uiAnimationLerpValue, false));
                StartCoroutine(UIFunctions.LerpUIRectPosition(sideBarChevronRect, sideChevronPositionOpen,
                    uiAnimationLerpValue, false));
            }
            else
            {
                StartCoroutine(UIFunctions.LerpUIRectPosition(sideBarRect, sideBarPositionDisabled,
                    uiAnimationLerpValue, true));
                StartCoroutine(UIFunctions.LerpUIRectPosition(sideBarChevronRect, sideChevronPositionClosed,
                    uiAnimationLerpValue, false));
            }
        }
    }
}
