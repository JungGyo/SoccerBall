﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(SphereCollider))]
public class SphereRigidBody : MonoBehaviour
{
    const float VelocityThreshold = 0.01f;
    const float AngularVelocityThreshold = 0.01f;

    Vector3 gravity {get{return Physics.gravity;}} //const gravity
    float mass {get{return rigidbody.mass;}}
    bool isMoving {get{return velocity.magnitude != 0;}}

    public float Bounciness {
        get {
            var mat = (PhysicMaterial)collider.material;
            if (mat == null)
                return 0;

            return ((PhysicMaterial)collider.material).bounciness;
            }
    }
    public float DynamicFriction {        
        get {
            var mat = (PhysicMaterial)collider.material;
            if (mat == null)
                return 0;
                
            return ((PhysicMaterial)collider.material).dynamicFriction;
            }
    }
    public float StaticFriction {        
        get {
            var mat = (PhysicMaterial)collider.material;
            if (mat == null)
                return 0;
                
            return ((PhysicMaterial)collider.material).staticFriction;
            }
    }

    public float magnusEffect = 1f;
    public float rotationalResistance = 0.025f; 

    public float Drag {get{ return rigidbody.drag; }}
    public float AngularDrag {get{ return rigidbody.angularDrag; }}
    public Vector3 velocity = new Vector3();
    Vector3 velocityLocal {get{ return WorldToLocal(velocity); }}

    Vector3 pos {
        get{ return this.transform.position; }
        set{
            this.transform.position = value;
            }
    }

    Vector3 force = new Vector3();
    Vector3 acceration {get{ return force / mass; }}

    Vector3 accerationLocal{
        get{ return WorldToLocal(acceration); }
    }

    public Vector3 angularAcc = new Vector3();

    public Vector3 angularVelocity = new Vector3();

    public Vector3 angularVelocityLocal {get{ return WorldToLocal(angularVelocity); }}
    public Vector3 angularAccLocal {get{ return WorldToLocal(angularAcc); }}

    public Matrix4x4 Inertia{
        get{
            if (rigidbody == null)
                return Matrix4x4.zero;

            var rotTensor = rigidbody.inertiaTensorRotation.eulerAngles;
            
            return new Matrix4x4(
            new Vector4(rigidbody.inertiaTensor.x, -rotTensor.z, -rotTensor.y, 0),
            new Vector4(-rotTensor.z,rigidbody.inertiaTensor.y, -rotTensor.x, 0),
            new Vector4(-rotTensor.y, -rotTensor.x,rigidbody.inertiaTensor.z, 0),
            new Vector4(0, 0, 0, 1));
        }
    }

    float Radius{
        get{
            return collider.radius * transform.lossyScale.x;
        }
    }

    Rigidbody rigidbody;
    SphereCollider collider;
    void Awake(){
        rigidbody = GetComponent<Rigidbody>();
        rigidbody.isKinematic = false;
        rigidbody.ResetInertiaTensor();
        rigidbody.isKinematic = true;

        collider = GetComponent<SphereCollider>();

        Time.timeScale = 0.1f;
    }

    Quaternion QuaternionConjugate(Quaternion q){
        return new Quaternion(-q.x, -q.y, -q.z, q.w);
    }
    public Vector3 t_drag;
    public Vector3 t_dragMomnet;

    void FixedUpdate(){
        var delta = Time.fixedDeltaTime;

        var exForce = Vector3.zero;
        var moment = Vector3.zero;
        var friction = Vector3.zero;
        var reaction = Vector3.zero;
        var collisionReact = Vector3.zero;

        //Gravity
        exForce += gravity * mass;

        //Drag
        exForce += LocalToWorld(CalcDragForce(velocity));
        moment += CalcAngularDragMoment(angularVelocityLocal);

        t_drag = exForce - gravity * mass;
        t_dragMomnet = moment;

        if (onContacted){
            foreach(var key in contactDic.Keys){
                var contact = contactDic[key]; 
                if (reflectionDone.Contains(key) == false){
                    velocity = contact.velocity;
                    angularVelocity = contact.angularVelocity;
                    reflectionDone.Add(key);
                }

                //Cotact(reaction + friction)
                reaction = CalcReactionForce(exForce, contact.normal);
                moment += CalcWorldMoment(reaction, contact.point - pos);

                friction = CalcFriction(exForce, contact.relativeVelocity, contact.normal, contact.tangent);
                moment += CalcWorldMoment(friction, contact.point - pos);

                //Rotation resistance
                if (Mathf.Abs(angularVelocityLocal.magnitude) > AngularVelocityThreshold){
                    var mag = Vector3.Cross(contact.tangent * (rotationalResistance * Radius), reaction).magnitude;
                    moment += -angularVelocityLocal.normalized * mag;
                }
            }
        }else{
            //Curve-ball(MagnusEffect)
            exForce += LocalToWorld(CalcMagnusEffecForce(velocityLocal, angularVelocityLocal));
        }

        if (onAddForce){
            exForce += externalForce;
            moment += CalcWorldMoment(externalForce, externalForcePoint);
            onAddForce = false;
        }

        force = exForce + friction + reaction;
        velocity += CalcDeltaVelocity(acceration, delta);
        pos += CalcDeltaPos(velocity, delta);

        var angAccLocal = CalcAngularAcc(moment, angularVelocityLocal);
        var angVelLocal = angularVelocityLocal + CalcDeltaAngularVelocity(angAccLocal, delta);
        transform.Rotate(angVelLocal * delta * Mathf.Rad2Deg, Space.Self);
        angularAcc = LocalToWorld(angAccLocal);
        angularVelocity = LocalToWorld(angVelLocal);
    }

    float time;
    void Update(){
        if((int)(time * 10 % 5) == 0){
            //Debug.Log($"vel: {velocity}/acc: {acc}");
        }

        time += Time.deltaTime;
    }

    Vector3 CalcDeltaPos(Vector3 v0, float t){
        return v0 * t;
    }

    Vector3 CalcDeltaVelocity(Vector3 a0, float t){
        return a0 * t;
    }

    Vector3 CalcDeltaAngularVelocity(Vector3 a0, float t){
        return a0 * t;
    }
    
    public class ContactInfo{
        public Vector3 point;
        public Vector3 normal;
        public Vector3 tangent;
        public Vector3 relativeVelocity;
        public Vector3 velocity;
        public Vector3 angularVelocity;
    }

    Dictionary<Collider, ContactInfo> contactDic = new Dictionary<Collider, ContactInfo>();
    HashSet<Collider> reflectionDone = new HashSet<Collider>();
    public bool onContacted {get{return contactDic.Count > 0;}}

    void OnCollisionEnter(Collision col){
        if (isMoving == false)
            return;

        var point = col.GetContact(0);
        var contactInfo = new ContactInfo(){point = point.point, normal = point.normal.normalized};
        var other = col.gameObject.GetComponent<SphereRigidBody>();
        
        if (other != null){
            UpdateCotactInfo(this, other, ref contactInfo);
        }else{
            //SphereRigidBody가 아닌 isKinematic rigidbody인 경우 위치 보정.
            pos -= point.normal.normalized * col.GetContact(0).separation;
            UpdateCotactInfo(this, null, ref contactInfo);
        }
        contactDic[col.collider] = contactInfo;

        Debug.Log($"name: {col.gameObject.name} - OnCollisionEnter({col.contacts.Length})/ p1:{contactInfo.point - transform.position}/ v1:{velocity}/ rv:{contactInfo.relativeVelocity}/ nv: {contactInfo.normal}, tv:{contactInfo.tangent}, cv:{contactInfo.velocity}, cv2:{Vector3.Reflect(velocity, contactInfo.normal)}");
    }

    void OnCollisionStay(Collision col){
        if (reflectionDone.Contains(col.collider)){
            var point = col.GetContact(0);
            var contactInfo = new ContactInfo(){point = point.point, normal = point.normal.normalized};
            var other = col.gameObject.GetComponent<SphereRigidBody>();
            
            if (other != null){
                UpdateCotactInfo(this, other, ref contactInfo);
            }else{
                //SphereRigidBody가 아닌 isKinematic rigidbody인 경우 위치 보정.
                UpdateCotactInfo(this, null, ref contactInfo);
                if (Vector3.Dot(contactInfo.relativeVelocity, contactInfo.normal) < 0.5f)
                    pos -= point.normal.normalized * col.GetContact(0).separation;
            }
            contactDic[col.collider] = contactInfo;
            // Debug.Log($"name: {col.gameObject.name} - OnCollisionStay({col.contacts.Length})/ p1:{contactInfo.point - transform.position}/ v1:{velocity}/ rv:{contactInfo.relativeVelocity}/ nv: {contactInfo.normal}, tv:{contactInfo.tangent}, j:{contactInfo.j}, cv:{contactInfo.velocity}");
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

    public bool IgnoreCollisionFriction = true;
    void UpdateCotactInfo(SphereRigidBody b1, SphereRigidBody b2, ref ContactInfo contact){
        if (b2 != null){//contact with RigidbodyObject.
            // TODO: else case 에서 테스트한 내용으로 수정 필요. 
            // var p1 = contact.point - b1.transform.position;
            // var p2 = contact.point - b2.transform.position;
            // var v1 = b1.velocity + Vector3.Cross(b1.angularVelocityLocal, p1);
            // var v2 = b2.velocity +  Vector3.Cross(b2.angularVelocityLocal, p2);
            // var rv = v2 - v1;
            
            // var j = (-(1 + (b1.bounceness + b2.bounceness)/2f) * Vector3.Dot(rv, contact.normal))/
            //     ((1/b1.mass + 1/b2.mass) + (Vector3.Dot(contact.normal, (Vector3.Cross(Matrix4x4.Inverse(b1.Inertia).transpose * Vector3.Cross(p1, contact.normal), p1))))
            //     + (Vector3.Dot(contact.normal, (Vector3.Cross(Matrix4x4.Inverse(b2.Inertia).transpose * Vector3.Cross(p2, contact.normal), p2)))));

            // contact.relativeVelocity = rv;
            // contact.tangent = Vector3.Cross(Vector3.Cross(contact.normal, rv), contact.normal).normalized;

            // var rvt = Vector3.Dot(rv, contact.tangent);

            // if (Mathf.Abs(rvt) > 0 && IgnoreCollisionFriction == false){
            //     contact.velocity = b1.velocity + ((j * contact.normal) + ((b1.dynamicFriction * j) * contact.tangent)) / b1.mass;
            //     contact.angularVelocityLocal = b1.angularVelocityLocal + (Vector3)(Matrix4x4.Inverse(b1.Inertia).transpose * Vector3.Cross(p1, ((j * contact.normal) + ((b1.dynamicFriction * j) * contact.tangent))));
            // }else{
            //     contact.velocity = b1.velocity + (j * contact.normal) / b1.mass;
            //     contact.angularVelocityLocal = b1.angularVelocityLocal + (Vector3)(Matrix4x4.Inverse(b1.Inertia).transpose * Vector3.Cross(p1, ((j * contact.normal))));
            // }
        }else{//contact with non-RigidbodyObject.
            var p1 = contact.point - b1.transform.position;
            var v1 = b1.velocity + Vector3.Cross(b1.angularVelocity, p1);

            var rv = v1;
            contact.relativeVelocity = rv;
            contact.tangent = -Vector3.Cross(Vector3.Cross(contact.normal, rv), contact.normal).normalized;

            var j = (-(1 + b1.Bounciness / 2f) * Vector3.Dot(rv, contact.normal))/
                ((1/b1.mass) + (Vector3.Dot(contact.normal, (Vector3.Cross(Matrix4x4.Inverse(b1.Inertia).transpose * Vector3.Cross(p1, contact.normal), p1)))));
           
            var rvt = Vector3.Dot(rv, contact.tangent);
            if (Mathf.Abs(rvt) > 0 && IgnoreCollisionFriction == false){
                contact.velocity = b1.velocity + ((j * contact.normal) + ((b1.DynamicFriction * j) * contact.tangent)) / b1.mass;
                var worldAngVel = b1.angularVelocity + (Vector3)(Matrix4x4.Inverse(b1.Inertia).transpose * Vector3.Cross(p1, ((j * contact.normal) + ((b1.DynamicFriction * j) * contact.tangent))));
                contact.angularVelocity = QuaternionConjugate(b1.transform.rotation) * worldAngVel;
            }else{
                contact.velocity = b1.velocity + (j * contact.normal) / b1.mass;
                var worldAngVel = b1.angularVelocity + (Vector3)(Matrix4x4.Inverse(b1.Inertia).transpose * Vector3.Cross(p1, ((j * contact.normal))));
                contact.angularVelocity = QuaternionConjugate(b1.transform.rotation) * worldAngVel;
            }
        }
    }


    public Vector3 t_Friction;
    public Vector3 t_Tangent;
    Vector3 CalcFriction(Vector3 force, Vector3 relativeVelocity, Vector3 normal, Vector3 tangent){
        var normalForceMag = Mathf.Abs(Vector3.Dot(force, normal.normalized));
        var tangentVel = Vector3.Project(relativeVelocity, tangent);
        var fc = DynamicFriction;//Mathf.Abs(tangentVel.magnitude) > 0.1? dynamicFriction : 0;
        var f = fc * mass * normalForceMag * tangent.normalized;
        // Debug.Log($"CalcFriction - F: {force}/nVector:{n}/ uVector:{tangent}");
        t_Friction = f;
        t_Tangent = tangent;
        return f;
    }

    public Vector3 t_Reaction;
    //TODO: Wall이 아닌 RigidObject에 대한 반력 계산으로 수정되어야함.
    //Wall은 질량이 무한하다고 가정시 수직 방향 가속도는 0이 되어야함.(수직항력 or 반력)
    Vector3 CalcReactionForce(Vector3 force, Vector3 normal){
        var f = -Vector3.Project(force,  normal);
        t_Reaction = f;
        // Debug.Log($"CalcReactionForce - F: {tangentForce}/nVector:{n}");
        return f;
    }

    public float t_rotInertia;
    Vector3 CalcAngularAcc(Vector3 moment, Vector3 angularVelocity){
        var invInertia = Matrix4x4.Inverse(Inertia);
        var rotInertia = Vector3.Cross(angularVelocity, (Inertia * angularVelocity));
        //var rotInertia = Vector3.Scale(angularVelocityLocal, (Vector3)(Inertia * new Vector3(Mathf.Abs(angularVelocityLocal.x),  Mathf.Abs(angularVelocityLocal.y),Mathf.Abs(angularVelocityLocal.z)))); 
        var a = invInertia * (moment - rotInertia);

        t_rotInertia = rotInertia.magnitude / moment.magnitude;
        //Debug.Log($"CalcTorque - T: {momentSum}/ I:{rotInertia}/ a:{a}/ w:{w}/ (q * w):{(inertia * w)}/ i:{inertia}/ inv: {invInertia}");
        // Debug.Log($"a:({a.x},{a.y},{a.z})/ w:({acc.x},{acc.y},{acc.z})/i:{rotInertia}");
        return a;
    }

    Vector3 CalcWorldMoment(Vector3 localForce, Vector3 localPoint){
        var worldPoint = LocalToWorld(localPoint);
        var worldForce = LocalToWorld(localForce);
        var m = Vector3.Cross(worldPoint, worldForce);
        return m;
    }

    public Vector3 t_Magnus;
    // Magnus effect(회전에 의해 생성된 커브 볼 효과)
    Vector3 CalcMagnusEffecForce(Vector3 velocity, Vector3 angularVelocity){
        var d = 1f; //무시 air density = 1.293f at 0 degrees Celsius;
        var area = Mathf.Pow(Radius, 2) * Mathf.PI;
        var v = Vector3.Cross(angularVelocity, velocity);
        var f = 2 * Mathf.PI * Radius * v * d * area * magnusEffect;
        t_Magnus = f;
        return f;
    }

    Vector3 CalcDragForce(Vector3 velocity){
        var d = 1f; //무시 air density = 1.293f at 0 degrees Celsius;
        var cd = Radius; //0.47f 완전한 구의 항력계수
        var u = -velocity.normalized;
        var area = Mathf.Pow(Radius, 2) * Mathf.PI;
        var f = 0.5f * d * Mathf.Pow(velocity.magnitude,  2) * cd * area * u * Drag;

        return f;
    }
    Vector3 CalcAngularDragMoment(Vector3 angularVelocity){
        var d = 1f; //무시 air density = 1.293f at 0 degrees Celsius;
        var cd = 0.47f; //완전한 구의 항력계수
        var a = Mathf.Pow(Radius, 2) * Mathf.PI;
        var u = -angularVelocity.normalized;
        var m = u * d * Mathf.Pow(angularVelocity.magnitude, 2) * cd * 4 * a * AngularDrag;

        return m;
    }

    public void Sleep(){
        force = Vector3.zero;
        velocity = Vector3.zero;

        angularAcc = Vector3.zero;
        angularVelocity = Vector3.zero;
    }

    bool onAddForce = false;
    Vector3 externalForce;
    Vector3 externalForcePoint;
    public void AddForce(Vector3 force, Vector3 localPoint = new Vector3()){
        externalForce = force;
        externalForcePoint = localPoint;
        onAddForce = true;
    }

    public Vector3 WorldToLocal(Vector3 world){
        return QuaternionConjugate(transform.rotation) * world;
    }

    public Vector3 LocalToWorld(Vector3 local){
        return  transform.rotation * local;
    }
}