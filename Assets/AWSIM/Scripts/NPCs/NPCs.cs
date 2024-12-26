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
        public string uuid;
        public virtual Vector3 LinearVelocity { get; }
        public virtual Vector3 AngularVelocity { get; }
        public void SetUUID(){
            Guid guid = Guid.NewGuid();
            uuid = guid.ToString();
        }
    }

}