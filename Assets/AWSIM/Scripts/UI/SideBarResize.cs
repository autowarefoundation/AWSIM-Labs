using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

namespace AWSIM.Scripts.UI
{
    public class SideBarResize : MonoBehaviour, IDragHandler, IPointerDownHandler, IPointerUpHandler
    {
        [SerializeField] private float _sidebarMinimumWidth = 220;
        [SerializeField] private float _sidebarDragBorderSize = 5;
        [SerializeField] private RectTransform _staticObjects;
        [SerializeField] private Texture2D _cursorResizeTexture2D;
        private readonly Vector2 _cursorResizeTexture2DHotspot = new(16, 16);
        private Vector2 _initialDragPointOffset;
        private Vector2 _initialSizeDelta;
        private RectTransform _sideBarRectTransform;
        private UISideBarHandler _sideBarHandler;
        private Image _sideBarImage;
        private Canvas _canvas;

        private float _dragAreaRightBound;
        private float _dragAreaLeftBound;
        private float _pointerPositionX;

        private bool _isDragActive;
        private bool _isCursorInDragArea;

        private void Start()
        {
            _sideBarRectTransform = GetComponent<RectTransform>();
            _sideBarHandler = GetComponentInParent<UISideBarHandler>();
            _sideBarImage = GetComponent<Image>();
            _sideBarImage.raycastPadding = new Vector4(_sidebarMinimumWidth - _sidebarDragBorderSize, 0, 0, 0);
            _canvas = _sideBarRectTransform.GetComponentInParent<Canvas>();
        }

        private void Update()
        {
            UpdateCursor();
        }

        private void UpdateCursor()
        {
            if (!_canvas) return;

            var pointerPosition = Input.mousePosition;
            RectTransformUtility.ScreenPointToLocalPointInRectangle(_sideBarRectTransform, pointerPosition,
                _canvas.worldCamera, out var localPointerPosition);

            var scaleFactor = _canvas.scaleFactor;
            _pointerPositionX = localPointerPosition.x * scaleFactor;
            _dragAreaRightBound = _sideBarRectTransform.rect.xMax * scaleFactor;
            _dragAreaLeftBound = (_sideBarRectTransform.rect.xMin + _sideBarImage.raycastPadding.x) * scaleFactor;

            if (_dragAreaRightBound >= _pointerPositionX && _pointerPositionX >= _dragAreaLeftBound)
            {
                if (!_isCursorInDragArea)
                {
                    Cursor.SetCursor(_cursorResizeTexture2D, _cursorResizeTexture2DHotspot, CursorMode.Auto);
                    _isCursorInDragArea = true;
                }
            }
            else
            {
                if (_isCursorInDragArea)
                {
                    Cursor.SetCursor(null, Vector2.zero, CursorMode.Auto);
                    _isCursorInDragArea = false;
                }
            }
        }

        public void OnPointerDown(PointerEventData eventData)
        {
            if (_isCursorInDragArea)
            {
                _isDragActive = true;
                RectTransformUtility.ScreenPointToLocalPointInRectangle(_sideBarRectTransform, eventData.position,
                    eventData.pressEventCamera, out var localPointerPosition);
                var scaleFactor = _canvas.scaleFactor;
                _initialDragPointOffset = new Vector2(localPointerPosition.x * scaleFactor - _pointerPositionX, 0);
                _initialSizeDelta = _sideBarRectTransform.sizeDelta;
            }
        }

        public void OnDrag(PointerEventData eventData)
        {
            if (_isDragActive)
            {
                if (!_canvas) return;

                RectTransformUtility.ScreenPointToLocalPointInRectangle(_sideBarRectTransform, eventData.position,
                    eventData.pressEventCamera, out var localPointerPosition);

                _sideBarRectTransform.sizeDelta = new Vector2(localPointerPosition.x, _initialSizeDelta.y) -
                                                  new Vector2(_initialDragPointOffset.x, 0);

                _staticObjects.sizeDelta = new Vector2(_sideBarRectTransform.sizeDelta.x, _staticObjects.sizeDelta.y);

                var padding = _sideBarImage.raycastPadding;
                padding.x = _sideBarRectTransform.sizeDelta.x - _sidebarDragBorderSize;
                _sideBarImage.raycastPadding = padding;

                _sideBarHandler.SideBarPositionDisabled = new Vector2(-_sideBarRectTransform.sizeDelta.x, 0);
            }
        }

        public void OnPointerUp(PointerEventData eventData)
        {
            _isDragActive = false;
        }

        public void OnRectTransformDimensionsChange()
        {
            if (!_sideBarRectTransform) return;
            if (_sideBarRectTransform.sizeDelta.x < _sidebarMinimumWidth)
            {
                _sideBarRectTransform.sizeDelta = new Vector2(_sidebarMinimumWidth, _initialSizeDelta.y);
                _staticObjects.sizeDelta = new Vector2(_sideBarRectTransform.sizeDelta.x, _staticObjects.sizeDelta.y);
            }
        }
    }
}
