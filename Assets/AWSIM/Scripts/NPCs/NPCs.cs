using UnityEngine;
using System;
using System.Reflection;

namespace AWSIM
{
    public class NPCs : MonoBehaviour
    {
        public unique_identifier_msgs.msg.UUID  uuid;
        public virtual Vector3 LinearVelocity { get; }
        public virtual Vector3 AngularVelocity { get; }
        public void SetUUID(){
            Guid guid = Guid.NewGuid();
            uuid = new unique_identifier_msgs.msg.UUID();
            PropertyInfo prop = uuid.GetType().GetProperty("Uuid", BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.Static);
            prop.SetValue(uuid, guid.ToByteArray());
        }

        public int spawnSec;
        public uint spawnNanosec;
        public void SetSpawnTime(){
            SimulatorROS2Node.TimeSource.GetTime(out spawnSec, out spawnNanosec);
        }
    }

}