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

    public float Boundness {
        get {return bounceness;}
        set {bounceness = Mathf.Clamp(value, 0, 1);}
    }
    public float contactReward = 1f;
    public float bounceness = 0.5f; //콘크리트에서 축구공의 반발계수 0.77~0.9 (콘크리트)
    public float dynamicFriction = 0.6f;
    public float staticFriction = 0.4f;
    [HideInInspector] public Vector3 velocity = new Vector3();
    Vector3 pos {
        get{ return this.transform.position; }
        set{
// #if UNITY_EDITOR
//             if (UnityEditor.Tools.current == UnityEditor.Tool.Move &&
//                 UnityEditor.Tools.handlePosition == this.transform.position)
//                 return;
// #endif

            this.transform.position = value;
            }
    }
    Vector3 acc = new Vector3();

    Rigidbody rigidbody;
    void Awake(){
        rigidbody = GetComponent<Rigidbody>();
        rigidbody.isKinematic = true;
    }

    void FixedUpdate(){
        var delta = Time.fixedDeltaTime;

        var friction = Vector3.zero;
        var reaction = Vector3.zero;
        var collisionReact = Vector3.zero;
        if (onContacted){
            foreach(var key in contactDic.Keys){
                var point = contactDic[key]; 
                var nVector = point.normal.normalized;
                if (reflectionDone.Contains(key) == false){
                    velocity = Vector3.Scale(Vector3.Reflect(velocity, nVector), new Vector3(1, bounceness, 1));
                    reflectionDone.Add(key);
                }
                friction += CalcFriction(velocity, gravity, nVector);
                reaction += CalcReactionForce(gravity, nVector);
            }
        }

        acc = gravity + (friction + reaction)/mass;
        pos = CalcPos(pos, velocity, acc, delta);
        velocity = CalcVel(velocity, acc, delta);

        // Debug.Log($"Friction: {friction}/ Reaction:{reaction} /vel: {velocity}/acc: {acc}");
        Debug.Log($"pos: {pos.y} /vel: {velocity.y} /acc: {acc.y} /fr: {friction}/ ra:{reaction}");
    }

    float time;
    void Update(){
        if((int)(time * 10 % 5) == 0){
            //Debug.Log($"vel: {velocity}/acc: {acc}");
        }

        time += Time.deltaTime;
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
    HashSet<Collider> reflectionDone = new HashSet<Collider>();
    public bool onContacted {get{return contactDic.Count > 0;}}

    void OnCollisionEnter(Collision col){
        if (isMoving == false)
            return;

        contactDic[col.collider] = col.GetContact(0);

        //RigidbodyObject가 아닌 isKinematic rigidbody인 경우 위치 보정.
        if (col.gameObject.GetComponent<RigidbodyObject>() == null)
            pos -= col.GetContact(0).normal.normalized * col.GetContact(0).separation;

        Debug.Log($"name: {col.gameObject.name} - OnCollisionEnter({col.contacts.Length})/ contact nv: {col.GetContact(0).normal.normalized}");
    }
    void OnCollisionStay(Collision col){
        if (reflectionDone.Contains(col.collider)){
            contactDic[col.collider] = col.GetContact(0);
        }
    }

    void OnCollisionExit(Collision col){
        var collider = col.collider;
        if (contactDic.ContainsKey(collider))
            contactDic.Remove(collider);
        if (reflectionDone.Contains(collider))
            reflectionDone.Remove(collider);
        Debug.Log($"name: {col.gameObject.name} - OnCollisionExit({col.contacts.Length})");
    }

    Vector3 CalcFriction(Vector3 vel, Vector3 acc, Vector3 nVector){
        var n = nVector.normalized;
        var normalForceScale = Vector3.Dot(acc, -n);
        var u = vel - Vector3.Project(vel, n);
        var force = normalForceScale > 0? dynamicFriction * normalForceScale * mass * -u : Vector3.zero;
        // Debug.Log($"CalcFriction - F: {force}/nVector:{n}/ uVector:{-u}");
        return force;
    }

    //TODO: Wall이 아닌 RigidObject에 대한 반력 계산으로 수정되어야함.
    //Wall은 질량이 무한하다고 가정시 수직 방향 가속도는 0이 되어야함.(수직항력 or 반력)
    Vector3 CalcReactionForce(Vector3 acc, Vector3 nVector){
        var n = nVector.normalized;
        var tangentForce = (acc - Vector3.Project(acc, n)) * mass;
        // Debug.Log($"CalcReactionForce - F: {tangentForce}/nVector:{n}");
        return tangentForce - (acc * mass);
    }

    Vector3 CalcContactRewardVelocity(Vector3 nVector, float dist, float t){
        var n = nVector.normalized;
        return  (n * dist) / t * contactReward;
    }
}
