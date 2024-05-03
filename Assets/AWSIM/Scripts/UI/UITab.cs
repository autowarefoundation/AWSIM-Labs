using UnityEngine;
using UnityEngine.UI;

namespace AWSIM.Scripts.UI
{
    public class UITab : MonoBehaviour
    {
        [SerializeField] private GameObject tabBackground;
        [SerializeField] private GameObject barChevron;

        [SerializeField] private int numberOfLines;
        [SerializeField] private float elementHeight = 30f;
        [SerializeField] private float elementSpacing = 10f;

        [SerializeField] private Sprite chevronSpriteUp;
        [SerializeField] private Sprite chevronSpriteDown;

        private float tabOffBgHeight;
        private float tabOnBgHeight;

        private void Awake()
        {
            CalculateTabBackgroundHeight();
        }

        public void ClickTabBar()
        {
            var lerpValue = GetComponentInParent<UISideBarHandler>().uiAnimationLerpValue;

            if (!tabBackground.gameObject.activeSelf)
            {
                tabBackground.gameObject.SetActive(true);
                StartCoroutine(UIFunctions.LerpUIRectHeight(tabBackground.GetComponent<RectTransform>(),
                    tabOnBgHeight, lerpValue, false));
                barChevron.transform.GetComponent<Image>().sprite = chevronSpriteUp;
            }
            else
            {
                StartCoroutine(UIFunctions.LerpUIRectHeight(tabBackground.GetComponent<RectTransform>(),
                    tabOffBgHeight, lerpValue, true));
                barChevron.transform.GetComponent<Image>().sprite = chevronSpriteDown;
            }
        }

        // Calculates the background on/off height
        private void CalculateTabBackgroundHeight()
        {
            tabOffBgHeight = elementHeight;
            if (numberOfLines != 0)
            {
                tabOnBgHeight = tabOffBgHeight + (numberOfLines * elementHeight) + (numberOfLines * elementSpacing) +
                                elementSpacing;
            }
            else
            {
                tabOnBgHeight = tabOffBgHeight;
            }
        }
    }
}
