using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

namespace AWSIM.Scripts.UI
{
    public class BirdEyeView : MonoBehaviour
    {
        [SerializeField] private float _birdEyeCameraInitialHeight = 200;

        [SerializeField] private float _cameraOrthographicSizeMinimum = 3;
        [SerializeField] private float _cameraOrthographicSizeMaximum = 150;

        [SerializeField] private float _zoomChangeMultiplier = 10f;
        [SerializeField] private float _zoomTime = 0.1f;

        [SerializeField] private float _dragPanMultiplier = 0.2f;

        [SerializeField] private float _keyPanSpeed = 50f;
        [SerializeField] private float _keyPanLerp = 0.5f;

        [SerializeField] private float _targetFollowOffsetX;
        [SerializeField] private float _targetFollowOffsetZ;

        private Camera _birdEyeCamera;
        private Camera _vehicleCamera;

        private Transform _cameraFollowTarget;
        private GameObject _vehicleTransform;

        private float _targetSize;
        private float _zoomChangeVelocity;
        private bool _willFollowEgo;

        private Vector3 _initialMousePosition;

        private void Start()
        {
            // Find the camera follow target
            _willFollowEgo = false;
            _vehicleTransform = GameObject.FindWithTag("Ego");
            foreach (Transform tf in _vehicleTransform.transform)
            {
                if (tf.CompareTag("BEVTarget"))
                {
                    _cameraFollowTarget = tf;
                }
            }

            _vehicleCamera = Camera.main;

            // Initialize the BEV camera
            _birdEyeCamera = InitializeBirdEyeCamera();
            _birdEyeCamera.enabled = false;
        }

        private void Update()
        {
            // Snap top-down camera to vehicle position
            if (Input.GetKeyDown(KeyCode.Space) && _birdEyeCamera.enabled)
            {
                _birdEyeCamera.transform.position = new Vector3(_cameraFollowTarget.position.x,
                    _birdEyeCamera.transform.position.y, _cameraFollowTarget.position.z);
            }
        }

        private void LateUpdate()
        {
            if (_birdEyeCamera.enabled)
            {
                CameraMovement();
            }

            if (Input.GetKeyDown(KeyCode.B))
            {
                if (!_birdEyeCamera.enabled)
                {
                    _birdEyeCamera.enabled = true;
                    _birdEyeCamera.depth = 1;
                    _vehicleCamera.depth = 0;
                    _vehicleCamera.enabled = false;
                }
                else
                {
                    _birdEyeCamera.enabled = false;
                    _birdEyeCamera.depth = 0;
                    _vehicleCamera.depth = 1;
                    _vehicleCamera.enabled = true;
                }
            }

            if (Input.GetKey(KeyCode.LeftControl))
            {
                if (Input.GetKeyDown(KeyCode.Space) && _birdEyeCamera.enabled)
                {
                    _willFollowEgo = !_willFollowEgo;
                }
            }

            // Follow the ego vehicle if toggled
            FollowEgoVehicle(_willFollowEgo);
        }

        private Camera InitializeBirdEyeCamera()
        {
            if (_birdEyeCamera == null)
            {
                // Create object with components
                var birdEyeObject = new GameObject("BirdEyeCamera",
                    typeof(Camera),
                    typeof(Volume),
                    typeof(BoxCollider)
                );

                // Setup Camera component
                _birdEyeCamera = birdEyeObject.GetComponent<Camera>();
                _birdEyeCamera.tag = "BEVCamera";
                _birdEyeCamera.gameObject.layer = 0;
                _birdEyeCamera.depth = 0;
                _birdEyeCamera.orthographic = true;
                _birdEyeCamera.allowMSAA = true;
                _birdEyeCamera.GetUniversalAdditionalCameraData().renderShadows = false;
                _birdEyeCamera.GetUniversalAdditionalCameraData().renderPostProcessing = true;
                _birdEyeCamera.transform.position =
                    new Vector3(_cameraFollowTarget.position.x, _birdEyeCameraInitialHeight,
                        _cameraFollowTarget.position.z);
                _birdEyeCamera.transform.LookAt(_cameraFollowTarget.position);

                //Setup Volume component
                var volume = birdEyeObject.GetComponent<Volume>();
                volume.isGlobal = false;
                volume.blendDistance = 0;
                volume.weight = 0.7f;
                volume.priority = 2;
                volume.profile = Resources.Load<VolumeProfile>("URP Presets/VolumeProfiles/MainCameraProfile");

                // Setup BoxCollider component
                var boxCollider = birdEyeObject.GetComponent<BoxCollider>();
                boxCollider.isTrigger = true;
                boxCollider.center = Vector3.zero;
                boxCollider.size = new Vector3(0.5f, 0.5f, 0.5f);
            }

            return _birdEyeCamera;
        }

        private void FollowEgoVehicle(bool willFollow)
        {
            if (willFollow)
            {
                _birdEyeCamera.transform.position = new Vector3(_cameraFollowTarget.position.x + _targetFollowOffsetX,
                    _birdEyeCamera.transform.position.y, _cameraFollowTarget.position.z + _targetFollowOffsetZ);
            }
        }

        private void CameraMovement()
        {
            var newCameraPosition = _birdEyeCamera.transform.position;

            // Calculate the target size
            _targetSize -= Input.GetAxis("Mouse ScrollWheel") * _zoomChangeMultiplier;
            _targetSize = Mathf.Clamp(_targetSize, _cameraOrthographicSizeMinimum, _cameraOrthographicSizeMaximum);

            // Interpolate current size to target size
            _birdEyeCamera.orthographicSize =
                Mathf.SmoothDamp(_birdEyeCamera.orthographicSize, _targetSize, ref _zoomChangeVelocity, _zoomTime);

            // Do not pan the camera if following the ego vehicle
            if (_willFollowEgo) return;

            // Pan the camera with the keys
            if (Input.GetAxis("Horizontal") != 0f || Input.GetAxis("Vertical") != 0f)
            {
                var cameraPan = new Vector2(Input.GetAxis("Horizontal"), Input.GetAxis("Vertical")) *
                                (_keyPanSpeed * Time.deltaTime);
                newCameraPosition.x += cameraPan.x;
                newCameraPosition.z += cameraPan.y;
                _birdEyeCamera.transform.position =
                    Vector3.Lerp(_birdEyeCamera.transform.position, newCameraPosition, _keyPanLerp);
            }

            // Pan the camera by dragging with middle mouse button
            if (Input.GetMouseButtonDown(2))
            {
                _initialMousePosition = Input.mousePosition;
            }

            if (Input.GetMouseButton(2))
            {
                var mousePositionDelta = Input.mousePosition - _initialMousePosition;
                newCameraPosition.x += mousePositionDelta.x * (_dragPanMultiplier * Time.deltaTime);
                newCameraPosition.z += mousePositionDelta.y * (_dragPanMultiplier * Time.deltaTime);
                _birdEyeCamera.transform.position = newCameraPosition;
            }
        }
    }
}
