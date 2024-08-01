using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;


namespace AWSIM
{
    /// <summary>
    /// This class subscribes to the /simulation/dummy_perception_publisher/object_info topic
    /// and spawn an NPC pedestrian or vehicle based on the value of "Label"
    /// </summary>
    public class RVIZNPCSpawner : MonoBehaviour
    {
        // collision layer
        private const int NPC_LAYER = 12;

        [Header("ROS Config")]
        [SerializeField] string dummyPerceptionTopic = "/simulation/dummy_perception_publisher/object_info";
        [SerializeField] QoSSettings qosSettings = new QoSSettings();

        [Header("NPC Prefabs")]
        [SerializeField] GameObject npcPedestrianPrefab;
        [SerializeField] GameObject[] npcCarPrefabs;
        [SerializeField] GameObject[] npcBusPrefabs;

        [Header("NPC Config")]
        public Transform npcPedestrianParent;
        public Transform npcVehicleParent;
        [SerializeField] float despawnTime = 30;

        private bool _willSpawnNpc = false;
        private bool _willDespawnAllNPCs = false;
        private int _npcLabel;
        private float _npcVelocity;
        private Quaternion _npcSpawnRotation;
        private Vector3 _npcSpawnPosition;
        private List<GameObject> _spawnedNPCs = new List<GameObject>();

        // Subscriber
        ISubscription<dummy_perception_publisher.msg.Object> dummyPerceptionSubscriber;

        void Start()
        {
            // Initialize the ROS2 node and create the subscription
            dummyPerceptionSubscriber
                = SimulatorROS2Node.CreateSubscription<dummy_perception_publisher.msg.Object>(
                        dummyPerceptionTopic, OnObjectInfoReceived, qosSettings.GetQoSProfile());
        }

        private void FixedUpdate()
        {
            if (_willSpawnNpc)
            {
                // check where the ground collider is and spawn the NPC above the ground
                Vector3 rayOrigin = new Vector3(_npcSpawnPosition.x, 1000.0f, _npcSpawnPosition.z);
                Vector3 rayDirection = Vector3.down;

                if (Physics.Raycast(rayOrigin, rayDirection, out RaycastHit hit, Mathf.Infinity))
                {
                    _npcSpawnPosition = new Vector3(_npcSpawnPosition.x, hit.point.y + 1.33f, _npcSpawnPosition.z);
                    if (_npcLabel == 7)
                    {
                        SpawnPedestrians(_npcSpawnPosition, _npcSpawnRotation);
                    }
                    else
                    {
                        SpawnVehicles(_npcSpawnPosition, _npcSpawnRotation, _npcLabel);
                    }
                }
                else
                {
                    Debug.LogWarning("No mesh or collider detected on target location. Please ensure that the target location is on a mesh or collider.");
                }

                _willSpawnNpc = false;
            }

            if (_willDespawnAllNPCs)
            {
                DespawnAllNPCs();
                _willDespawnAllNPCs = false;
            }
        }

        /// <summary>
        /// Callback method to handle incoming ROS messages
        /// </summary>
        /// <param name="msg">Received Object message</param>
        void OnObjectInfoReceived(dummy_perception_publisher.msg.Object msg)
        {
            if (msg.Classification.Label == 0)
            {
                // despawn all the rviz NPCs
                Debug.Log("delete spawned NPCs");
                _willDespawnAllNPCs = true;
                return;
            }
            _willSpawnNpc = true;
            _npcSpawnPosition = ROS2Utility.RosMGRSToUnityPosition(msg.Initial_state.Pose_covariance.Pose.Position);
            _npcSpawnRotation = ROS2Utility.RosToUnityRotation(msg.Initial_state.Pose_covariance.Pose.Orientation);
            _npcLabel = msg.Classification.Label;
            _npcVelocity = (float)msg.Initial_state.Twist_covariance.Twist.Linear.X;
        }

        /// <summary>
        /// Method to spawn NPC Pedestrians
        /// </summary>
        private void SpawnPedestrians(Vector3 spawnPoint, Quaternion spawnOrientation)
        {
            GameObject npcPedestrian = Instantiate(npcPedestrianPrefab, new Vector3(spawnPoint.x, spawnPoint.y, spawnPoint.z), spawnOrientation, npcPedestrianParent);
            SimplePedestrianWalkerController walker = npcPedestrian.AddComponent<SimplePedestrianWalkerController>();
            _spawnedNPCs.Add(npcPedestrian);

            walker.GetType().GetField("duration", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance).SetValue(walker, 30);
            walker.GetType().GetField("speed", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance).SetValue(walker, _npcVelocity);
            StartCoroutine(DespawnNPC(npcPedestrian, despawnTime));
        }

        /// <summary>
        /// Method to spawn NPC vehicles
        /// </summary>
        private void SpawnVehicles(Vector3 spawnPoint, Quaternion spawnOrientation, int vehicleType)
        {
            GameObject vehiclePrefab = npcCarPrefabs[Random.Range(0, npcCarPrefabs.Length - 1)];
            if (vehicleType == 3)
            {
                vehiclePrefab = npcBusPrefabs[Random.Range(0, npcBusPrefabs.Length)];
            }

            GameObject npcVehicle = Instantiate(vehiclePrefab, new Vector3(spawnPoint.x, spawnPoint.y, spawnPoint.z), spawnOrientation, npcVehicleParent);
            _spawnedNPCs.Add(npcVehicle);

            SetNPCLayer(npcVehicle, NPC_LAYER);

            // make the vehicle move a constant velocity
            NPCMovement npcMovement = npcVehicle.AddComponent<NPCMovement>();
            npcMovement.Initialize(_npcVelocity);

            StartCoroutine(DespawnNPC(npcVehicle, despawnTime));
        }

        /// <summary>
        /// Recursively set layer of the NPC and its children to layer 12: "NPC Vehicle"
        /// </summary>
        private void SetNPCLayer(GameObject obj, int newLayer)
        {
            obj.layer = newLayer;

            foreach (Transform child in obj.transform)
            {
                SetNPCLayer(child.gameObject, newLayer);
            }
        }

        /// <summary>
        /// Despawn NPC after a delay
        /// </summary>
        private IEnumerator DespawnNPC(GameObject npc, float delay)
        {
            yield return new WaitForSeconds(delay);
            Destroy(npc);
        }

        /// <summary>
        /// Despawn all NPCs
        /// </summary>
        private void DespawnAllNPCs()
        {
            foreach (var npc in _spawnedNPCs)
            {
                Destroy(npc);
            }
            _spawnedNPCs.Clear();
        }

        void OnDestroy()
        {
            // Clean up the subscription when the GameObject is destroyed
            SimulatorROS2Node.RemoveSubscription<dummy_perception_publisher.msg.Object>(dummyPerceptionSubscriber);
        }
    }
}
