using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class NPCMovement : MonoBehaviour
{
    private float velocity = 0;
    Rigidbody rb;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    public void Initialize(float velocity)
    {
        this.velocity = velocity;
    }

    void Update()
    {

        if (rb != null)
        {
            rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationY | RigidbodyConstraints.FreezeRotationZ;
            Vector3 forwardMovement = transform.forward * velocity;
            Vector3 currentVelocity = rb.velocity;
            rb.velocity = new Vector3(forwardMovement.x, rb.velocity.y, forwardMovement.z);
        }
    }
}
