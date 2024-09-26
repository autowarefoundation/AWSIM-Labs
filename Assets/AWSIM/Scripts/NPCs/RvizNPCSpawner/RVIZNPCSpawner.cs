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
        private bool _willDespawnInteractiveNPCs = false;
        private bool _allNPCsDespawned = false;
        private float _npcVelocity;
        private int _npcLabel;
        private int _interactiveAction = 0;
        private int _activeNPCCount = 0;
        private GameObject interactiveVehicle;
        private List<GameObject> _spawnedNPCs = new List<GameObject>();
        private Quaternion _npcSpawnRotation;
        private Vector3 _npcSpawnPosition;
        private Vector3 _previousPosition;
        private float _raycastStart = 1.33f;

        // Subscriber
        ISubscription<tier4_simulation_msgs.msg.DummyObject> dummyPerceptionSubscriber;

        void Start()
        {
            // Initialize the ROS2 node and create the subscription
            dummyPerceptionSubscriber
                = SimulatorROS2Node.CreateSubscription<tier4_simulation_msgs.msg.DummyObject>(
                        dummyPerceptionTopic, OnObjectInfoReceived, qosSettings.GetQoSProfile());
        }

        private void FixedUpdate()
        {
            // clear the _spawnedNPCs array if all the npcs are despawned
            if (_allNPCsDespawned)
            {
                DespawnAllNPCs();
                _allNPCsDespawned = false;
                return;
            }

            // spawn a new NPC
            if (_willSpawnNpc)
            {
                SpawnNewNPC();
            }


            // interactive action = 1: update the position of the last spawned interactive NPC
            if (_interactiveAction == 1 && _spawnedNPCs != null && _spawnedNPCs.Count > 0)
            {
                if (Mathf.Abs(_previousPosition.x - _npcSpawnPosition.x) >= 0.1f || Mathf.Abs(_previousPosition.z - _npcSpawnPosition.z) >= 0.1f)
                {
                    MoveNPC();
                }
            }


            if (_willDespawnAllNPCs)
            {
                DespawnAllNPCs();
                _willDespawnAllNPCs = false;
                return;
            }
            if (_willDespawnInteractiveNPCs)
            {
                DespawnInteractiveNPCs();
                _willDespawnInteractiveNPCs = false;
                return;
            }
        }

        /// <summary>
        /// Callback method to handle incoming ROS messages
        /// </summary>
        /// <param name="msg">Received Object message</param>
        void OnObjectInfoReceived(tier4_simulation_msgs.msg.DummyObject msg)
        {
            _npcLabel = msg.Classification.Label; // Label: 0 = delete All Npcs, 7 = Spawn pedestrians, 3 = spawn vehicle
            _interactiveAction = msg.Action; // Action: 0 = uninteractive mode, 1 = interactive mode, 2 = delete interactive NPCs
            _npcSpawnPosition = ROS2Utility.RosMGRSToUnityPosition(msg.Initial_state.Pose_covariance.Pose.Position);
            _npcSpawnRotation = ROS2Utility.RosToUnityRotation(msg.Initial_state.Pose_covariance.Pose.Orientation);
            _npcVelocity = (float)msg.Initial_state.Twist_covariance.Twist.Linear.X;

            // interactive mode;
            if (_interactiveAction == 1)
            {
                return;
            }

            if (_npcLabel == 0)
            {
                _willDespawnAllNPCs = true;
                return;
            }


            // spawn normal npcs
            if (_interactiveAction == 0)
            {
                _willSpawnNpc = true;
                return;
            }

            // delete only interactive npcs
            if (_interactiveAction == 2)
            {
                _willDespawnInteractiveNPCs = true;
                return;
            }

        }

        private void SpawnNewNPC()
        {
            {
                // check where the ground collider is and spawn the NPC above the ground
                Vector3 rayOrigin = new Vector3(_npcSpawnPosition.x, _raycastStart, _npcSpawnPosition.z);
                Vector3 rayDirection = Vector3.down;

                if (Physics.Raycast(rayOrigin, rayDirection, out RaycastHit hit, Mathf.Infinity))
                {
                    _npcSpawnPosition = new Vector3(_npcSpawnPosition.x, hit.point.y + _raycastStart, _npcSpawnPosition.z);
                    if (_npcLabel == 7)
                    {
                        SpawnPedestrians(_npcSpawnPosition, _npcSpawnRotation);
                    }
                    else
                    {
                        SpawnVehicles(_npcSpawnPosition, _npcSpawnRotation, _npcLabel);
                    }
                    _activeNPCCount++;
                }
                else
                {
                    Debug.LogWarning("No mesh or collider detected on target location. Please ensure that the target location is on a mesh or collider.");
                }

                _willSpawnNpc = false;
                return;
            }
        }

        /// <summary>
        /// Method to spawn NPC Pedestrians
        /// </summary>
        private void SpawnPedestrians(Vector3 spawnPoint, Quaternion spawnOrientation)
        {
            GameObject npcPedestrian = Instantiate(npcPedestrianPrefab, new Vector3(spawnPoint.x, spawnPoint.y, spawnPoint.z), spawnOrientation, npcPedestrianParent);

            // in interactive mode I don't need the following, but I need it in normal mode
            SimplePedestrianWalkerController walker = npcPedestrian.AddComponent<SimplePedestrianWalkerController>();
            walker.GetType().GetField("duration", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance).SetValue(walker, 30);
            walker.GetType().GetField("speed", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance).SetValue(walker, _interactiveAction == 1 ? 0 : _npcVelocity);

            if (_interactiveAction == 1)
            {
                NPCPedestrian pedestrianScript = npcPedestrian.GetComponent<NPCPedestrian>();
                if (pedestrianScript != null)
                {
                    Destroy(walker);
                    Destroy(pedestrianScript);
                }
            }

            _spawnedNPCs.Add(npcPedestrian);
            StartCoroutine(DespawnNPC(npcPedestrian, despawnTime));
        }

        /// <summary>
        /// Method to spawn NPC vehicles
        /// </summary>
        private void SpawnVehicles(Vector3 spawnPoint, Quaternion spawnOrientation, int vehicleType)
        {
            // default vehicle is car
            GameObject vehiclePrefab = npcCarPrefabs[Random.Range(0, npcCarPrefabs.Length - 1)];

            // change the vehicle to bus
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

            // despawn automatically after some time.
            StartCoroutine(DespawnNPC(npcVehicle, despawnTime));
        }

        private void MoveNPC()
        {
            int groundLayerMask = LayerMask.GetMask("Ground");

            Vector3 rayOrigin = new Vector3(_npcSpawnPosition.x, _raycastStart, _npcSpawnPosition.z);
            Vector3 rayDirection = Vector3.down;

            if (Physics.Raycast(rayOrigin, rayDirection, out RaycastHit hit, Mathf.Infinity, groundLayerMask))
            {
                interactiveVehicle = _spawnedNPCs[_spawnedNPCs.Count - 1];
                interactiveVehicle.tag = "InteractiveNpcs";
                if (interactiveVehicle != null)
                {
                    if (_npcLabel == 7)
                    {
                        // remove unwanted components for interactive pedestrians
                        NPCPedestrian pedestrianScript = interactiveVehicle.GetComponent<NPCPedestrian>();
                        SimplePedestrianWalkerController walker = interactiveVehicle.GetComponent<SimplePedestrianWalkerController>();
                        if (walker != null)
                        {
                            Destroy(walker); // Destroy the SimplePedestrianWalkerController script
                        }

                        if (pedestrianScript != null)
                        {
                            Destroy(pedestrianScript); // Destroy the NPCPedestrian script
                        }
                    }

                    interactiveVehicle.transform.position = new Vector3(_npcSpawnPosition.x, hit.point.y, _npcSpawnPosition.z);
                    _previousPosition = interactiveVehicle.transform.position;
                    interactiveVehicle.transform.rotation = _npcSpawnRotation;
                }
                return;
            }
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
            _activeNPCCount--;

            if (_activeNPCCount == 0)
            {
                _allNPCsDespawned = true; // Set flag to trigger despawn check in FixedUpdate
            }
        }

        /// <summary>
        /// Despawn all NPCs
        /// </summary>
        private void DespawnAllNPCs()
        {
            _activeNPCCount = 0;
            foreach (var npc in _spawnedNPCs)
            {
                Destroy(npc);
            }
            DespawnInteractiveNPCs();
            _spawnedNPCs.Clear();
        }

        /// <summary>
        /// Despawn only interactive NPCs
        /// </summary>
        private void DespawnInteractiveNPCs()
        {
            GameObject[] npcs = GameObject.FindGameObjectsWithTag("InteractiveNpcs");
            foreach (GameObject npc in npcs)
            {
                _activeNPCCount--;
                Destroy(npc);
            }
            if (_activeNPCCount == 0)
            {
                _allNPCsDespawned = true;
            }
        }


        /// <summary>
        /// Clean up the subscription when the GameObject is destroyed
        /// </summary>
        void OnDestroy()
        {
            SimulatorROS2Node.RemoveSubscription<tier4_simulation_msgs.msg.DummyObject>(dummyPerceptionSubscriber);
        }
    }
}
