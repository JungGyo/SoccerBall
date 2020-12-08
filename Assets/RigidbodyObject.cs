using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(Collider))]
public class RigidbodyObject : MonoBehaviour
{
    const float AirDensity = 1f; //무시 //1.293f at 0 degrees Celsius;
    Vector3 gravity {get{return Physics.gravity;}} //const gravity
    float mass {get{return rigidbody.mass;}}
    bool isMoving {get{return velocity.magnitude != 0;}}

    public float bounceness = 0.5f; //콘크리트에서 축구공의 반발계수 0.77~0.9 (콘크리트)
    public float dynamicFriction = 0.6f;
    public float staticFriction = 0.4f;
    [HideInInspector] public Vector3 velocity = new Vector3();
    Vector3 pos {
        get{ return this.transform.position; }
        set{ this.transform.position = value; }
    }
    Vector3 acc = new Vector3();

    Rigidbody rigidbody;
    void Awake(){
        rigidbody = GetComponent<Rigidbody>();
    }

    // Update is called once per frame
    void FixedUpdate(){
        var delta = Time.fixedDeltaTime;

        var friction = Vector3.zero;
        var reaction = Vector3.zero;
        if (onContacted){
            foreach(var point in contactDic.Values){
                var nVector = point.normal.normalized;
                velocity = Vector3.Scale(Vector3.Reflect(velocity, nVector), new Vector3(1, bounceness, 1));

                friction += CalcFriction(velocity, acc, nVector);
                reaction += CalcReactionForce(acc, nVector);
            }
        }

        pos = CalcPos(pos, velocity, acc, delta);
        velocity = CalcVel(velocity, acc, delta);
        acc = gravity + (friction + reaction)/mass;

        this.transform.position = pos;
    }

    Vector3 CalcPos(Vector3 p0, Vector3 v0, Vector3 a0, float t){
        return p0 + v0 * t + 0.5f * a0 * Mathf.Pow(t, 2);
    }

    Vector3 CalcVel(Vector3 v0, Vector3 a0, float t){
        return v0 + a0 * t;
    }

    // public void AddForce(Vector3 force, Vector3 Point){
    //     acc = force/mass;
    // }

    //List<ContactPoint> contactPoints = new List<ContactPoint>();
    Dictionary<Collider, ContactPoint> contactDic = new Dictionary<Collider, ContactPoint>();
    public bool onContacted {get{return contactDic.Count > 0;}}

    void OnCollisionEnter(Collision col){
        if (isMoving == false)
            return;
        //contactPoints.Add(col.GetContact(0));
        contactDic[col.collider] = col.GetContact(0);
        // if (onFootPrint) footPrint.Draw(pos);
        Debug.Log($"name: {col.gameObject.name} - OnCollisionEnter({col.contacts.Length})/ contact nv: {col.GetContact(0).normal.normalized}");
    }
    void OnCollisionStay(Collision col){
        //lastContactPoint.normal.normalized;
        // reflectionVectors.Add(contactNormalVector);
        // Debug.Log($"time: {time} - OnCollisionStay({col.contacts.Length})");
    }

    void OnCollisionExit(Collision col){
        if (contactDic.ContainsKey(col.collider))
            contactDic.Remove(col.collider);
        Debug.Log($"name: {col.gameObject.name} - OnCollisionExit({col.contacts.Length})");
    }

    Vector3 CalcFriction(Vector3 vel, Vector3 acc, Vector3 nVector){
        var n = -1 * vel.normalized;
        var normalForceScale = Vector3.Dot(acc, nVector.normalized);
        var force = dynamicFriction * normalForceScale * mass * n;
        // Debug.Log($"CalcFriction - F: {force}/accMag: {normalForceScale}/normal vector:{n}");
        return force;
    }

    //Wall은 질량이 무하다고 가정. 벽의 수직 방향 가속도는 0이 되어야함.(수직항력 or 반력)
    Vector3 CalcReactionForce(Vector3 acc, Vector3 nVector){
        var n = nVector.normalized;
        var tangentForce = (acc - Vector3.Dot(acc, n) * n) * mass;
        // Debug.Log($"CalcReactionForceAcc - acc: {acc}/accNew: {newAcc}/normal vector:{n}");
        return tangentForce - (acc * mass);
    }
}
