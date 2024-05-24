using System.Collections;
using UnityEngine;

namespace AWSIM.Scripts.UI
{
    // Functions used by the UI elements
    public static class UIFunctions
    {
        // Lerp UI objects position
        public static IEnumerator LerpUIRectPosition(RectTransform uiRect, Vector2 targetPosition, float lerpValue,
            bool willDisableAtEnd)
        {
            var elapsedTime = 0f;
            var currentPosition = uiRect.anchoredPosition;

            while (elapsedTime < 1f)
            {
                elapsedTime += Time.deltaTime * lerpValue;
                uiRect.anchoredPosition = Vector2.Lerp(currentPosition, targetPosition, elapsedTime);
                yield return null;
            }

            // Ensure the UI element is exactly at the target position
            uiRect.anchoredPosition = targetPosition;

            // Disable at end if wanted
            if (willDisableAtEnd)
            {
                uiRect.gameObject.SetActive(false);
            }
        }

        // Lerp UI objects height
        public static IEnumerator LerpUIRectHeight(RectTransform uiRect, float targetHeight, float lerpValue,
            bool willDisableAtEnd)
        {
            var elapsedTime = 0f;
            var currentHeight = uiRect.sizeDelta.y;

            while (elapsedTime < 1f)
            {
                elapsedTime += Time.deltaTime * lerpValue;
                float lerpedHeight = Mathf.Lerp(currentHeight, targetHeight, elapsedTime);
                uiRect.sizeDelta = new Vector2(uiRect.sizeDelta.x, lerpedHeight);
                yield return null;
            }

            // Ensure the UI element has the target height
            uiRect.sizeDelta = new Vector2(uiRect.sizeDelta.x, targetHeight);

            // Disable at end if wanted
            if (willDisableAtEnd)
            {
                uiRect.gameObject.SetActive(false);
            }
        }
    }
}
