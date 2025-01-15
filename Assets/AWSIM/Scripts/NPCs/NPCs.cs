using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Reflection;
using ROS2;

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
    }

}