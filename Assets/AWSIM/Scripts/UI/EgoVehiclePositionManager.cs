using System;
using UnityEngine;

namespace AWSIM.Scripts.UI
{
    public class EgoVehiclePositionManager : MonoBehaviour
    {
        public Transform EgoTransform { private get; set; }
        private Vector3 _initialEgoPosition;
        private Quaternion _initialEgoRotation;

        // Event to notify subscribers that the ego vehicle has been reset to the spawn point.
        public event Action<Vector3, Quaternion> OnEgoReset;

        private void Start()
        {
            InitializeEgoTransform(EgoTransform);
        }

        public void Activate()
        {
            enabled = true;
            Start();
        }

        private void InitializeEgoTransform(Transform egoTransform)
        {
            EgoTransform = egoTransform;
            _initialEgoPosition = EgoTransform.position;
            _initialEgoRotation = EgoTransform.rotation;
        }

        // If the ego transform reference is present, reset the ego to the initial position and rotation.
        public void ResetEgoToSpawnPoint()
        {
            if (!EgoTransform)
            {
                Debug.LogWarning("Ego transform reference is missing. No ego to reset here!");
                return;
            }

            // Reset the ego vehicle to the initial position and rotation.
            OnEgoReset?.Invoke(_initialEgoPosition, _initialEgoRotation);
        }
    }
}
