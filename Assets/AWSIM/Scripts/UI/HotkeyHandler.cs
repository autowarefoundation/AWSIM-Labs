using UnityEngine;


//This script will be improved in the future. Temporarily it will be a bit dirty.
namespace AWSIM
{
    public class HotkeyHandler : MonoBehaviour
    {
        private AutowareSimulation _simulation;
        private Transform _egoTransform;
        private Rigidbody _egoRigidbody;
        private Vector3 _initialEgoPosition;
        private Quaternion _initialEgoRotation;


        private void Start()
        {
            _egoTransform = gameObject.GetComponent<AutowareSimulation>().egoTransform;
            // Store the initial position and rotation of the ego.
            _initialEgoPosition = _egoTransform.position;
            _initialEgoRotation = _egoTransform.rotation;
            // Get the rigidbody of the ego
            _egoRigidbody = _egoTransform.GetComponent<Rigidbody>();
        }

        void Update()
        {
            // If the escape key is pressed, toggle the GUI panel.
            if (Input.GetKeyDown(KeyCode.R))
            {
                ResetEgoToSpawnPoint();
            }
        }

        // If the ego transform reference is present, reset the ego to the initial position and rotation.
        public void ResetEgoToSpawnPoint()
        {
            if (!_egoTransform)
            {
                Debug.LogWarning("Ego transform reference is missing. No ego to reset here!");
                return;
            }

            _egoTransform.SetPositionAndRotation(_initialEgoPosition, _initialEgoRotation);
            _egoRigidbody.Sleep();
        }
    }
}