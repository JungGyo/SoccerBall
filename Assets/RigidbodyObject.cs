using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(Collider))]
public class RigidbodyObject : MonoBehaviour
{
    const float VelocityThreshold = 0.001f;
    const float AngularVelocityThreshold = 0.001f;

    Vector3 gravity {get{return Physics.gravity;}} //const gravity
    float mass {get{return rigidbody.mass;}}
    bool isMoving {get{return velocity.magnitude != 0;}}

    public float Boundness {
        get {return bounceness;}
        set {bounceness = Mathf.Clamp(value, 0, 1);}
    }

    public float rotationalResistance = 0.025f; 
    public float bounceness = 0.5f; //콘크리트에서 축구공의 반발계수 0.77~0.9 (콘크리트)
    public float dynamicFriction = 0.6f;
    public float staticFriction = 0.4f;
    public float Drag = 1f;
    public float AngularDrag = 1f;
    public Vector3 velocity = new Vector3();
    Vector3 pos {
        get{ return this.transform.position; }
        set{
            this.transform.position = value;
            }
    }

    Vector3 force = new Vector3();
    Vector3 acceration {
        get{ return force / mass;}
        }

    Quaternion rotPos {
        get{ return transform.rotation; }
    }
    public Vector3 angularVelocity = new Vector3();
    Vector3 angularAcc = new Vector3();

    public Vector3 angularVelocityWorld {
        get{ return transform.rotation * angularVelocity;}
        }

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
            if(collider.GetType() != typeof(SphereCollider)){
                Debug.LogError($"{this.gameObject.name} needs <SphereCollider> component for Radius");
                return 0;
            }
            return ((SphereCollider)collider).radius * transform.lossyScale.x;
        }
    }

    Rigidbody rigidbody;
    Collider collider;
    void Awake(){
        rigidbody = GetComponent<Rigidbody>();
        rigidbody.isKinematic = false;
        rigidbody.ResetInertiaTensor();
        rigidbody.isKinematic = true;

        collider = GetComponent<Collider>();

        //Time.timeScale = 0.1f;
    }

    Quaternion QuaternionConjugate(Quaternion q){
        return new Quaternion(-q.x, -q.y, -q.z, q.w);
    }

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
        exForce += CalcDragForce(velocity);
        moment += CalcAngularDragMoment(angularVelocity);

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

                if (Mathf.Abs(angularVelocity.magnitude) > AngularVelocityThreshold){
                    var mag = Vector3.Cross(contact.tangent * (rotationalResistance * Radius), reaction).magnitude;
                    moment += -angularVelocity.normalized * mag;
                }

            }
        }

        force = (exForce + friction + reaction);
        velocity += CalcDeltaVelocity(acceration, delta);
        pos += CalcDeltaPos(velocity, delta);

        angularAcc = CalcAngularAcc(moment, angularVelocityWorld);
        angularVelocity += CalcDeltaAngularVelocity(angularAcc, delta);
        transform.Rotate(angularVelocity * delta * Mathf.Rad2Deg, Space.Self);
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

    // public void AddForce(Vector3 force, Vector3 Point){
    //     acc = force/mass;
    // }

    //List<ContactPoint> contactPoints = new List<ContactPoint>();

    
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
        var other = col.gameObject.GetComponent<RigidbodyObject>();
        
        if (other != null){
            UpdateCotactInfo(this, other, ref contactInfo);
        }else{
            //RigidbodyObject가 아닌 isKinematic rigidbody인 경우 위치 보정.
            if (Mathf.Abs(Vector3.Dot(pos - point.point, point.normal.normalized)) < Radius){
                pos -= point.normal.normalized * col.GetContact(0).separation;
            }
            UpdateCotactInfo(this, null, ref contactInfo);
        }
        contactDic[col.collider] = contactInfo;

        Debug.Log($"name: {col.gameObject.name} - OnCollisionEnter({col.contacts.Length})/ p1:{contactInfo.point - transform.position}/ v1:{velocity}/ rv:{contactInfo.relativeVelocity}/ nv: {contactInfo.normal}, tv:{contactInfo.tangent}, cv:{contactInfo.velocity}, cv2:{Vector3.Reflect(velocity, contactInfo.normal)}");
    }

    public float t_Dist;
    void OnCollisionStay(Collision col){
        if (reflectionDone.Contains(col.collider)){
            var point = col.GetContact(0);
            var contactInfo = new ContactInfo(){point = point.point, normal = point.normal.normalized};
            var other = col.gameObject.GetComponent<RigidbodyObject>();
            
            if (other != null){
                UpdateCotactInfo(this, other, ref contactInfo);
            }else{
                //RigidbodyObject가 아닌 isKinematic rigidbody인 경우 위치 보정.
                t_Dist = Mathf.Abs(Vector3.Dot(pos - point.point, point.normal.normalized));
                if (Mathf.Abs(Vector3.Dot(pos - point.point, point.normal.normalized)) < Radius){
                    pos -= point.normal.normalized * col.GetContact(0).separation;
                }
                UpdateCotactInfo(this, null, ref contactInfo);
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
    void UpdateCotactInfo(RigidbodyObject b1, RigidbodyObject b2, ref ContactInfo contact){
        if (b2 != null){//contact with RigidbodyObject.
            // TODO: else case 에서 테스트한 내용으로 수정 필요. 
            // var p1 = contact.point - b1.transform.position;
            // var p2 = contact.point - b2.transform.position;
            // var v1 = b1.velocity + Vector3.Cross(b1.angularVelocity, p1);
            // var v2 = b2.velocity +  Vector3.Cross(b2.angularVelocity, p2);
            // var rv = v2 - v1;
            
            // var j = (-(1 + (b1.bounceness + b2.bounceness)/2f) * Vector3.Dot(rv, contact.normal))/
            //     ((1/b1.mass + 1/b2.mass) + (Vector3.Dot(contact.normal, (Vector3.Cross(Matrix4x4.Inverse(b1.Inertia).transpose * Vector3.Cross(p1, contact.normal), p1))))
            //     + (Vector3.Dot(contact.normal, (Vector3.Cross(Matrix4x4.Inverse(b2.Inertia).transpose * Vector3.Cross(p2, contact.normal), p2)))));

            // contact.relativeVelocity = rv;
            // contact.tangent = Vector3.Cross(Vector3.Cross(contact.normal, rv), contact.normal).normalized;

            // var rvt = Vector3.Dot(rv, contact.tangent);

            // if (Mathf.Abs(rvt) > 0 && IgnoreCollisionFriction == false){
            //     contact.velocity = b1.velocity + ((j * contact.normal) + ((b1.dynamicFriction * j) * contact.tangent)) / b1.mass;
            //     contact.angularVelocity = b1.angularVelocity + (Vector3)(Matrix4x4.Inverse(b1.Inertia).transpose * Vector3.Cross(p1, ((j * contact.normal) + ((b1.dynamicFriction * j) * contact.tangent))));
            // }else{
            //     contact.velocity = b1.velocity + (j * contact.normal) / b1.mass;
            //     contact.angularVelocity = b1.angularVelocity + (Vector3)(Matrix4x4.Inverse(b1.Inertia).transpose * Vector3.Cross(p1, ((j * contact.normal))));
            // }
        }else{//contact with non-RigidbodyObject.
            var p1 = contact.point - b1.transform.position;
            var v1 = b1.velocity + Vector3.Cross(b1.angularVelocityWorld, p1);

            var rv = v1;
            contact.relativeVelocity = rv;
            contact.tangent = -Vector3.Cross(Vector3.Cross(contact.normal, rv), contact.normal).normalized;

            var j = (-(1 + b1.bounceness/2f) * Vector3.Dot(rv, contact.normal))/
                ((1/b1.mass) + (Vector3.Dot(contact.normal, (Vector3.Cross(Matrix4x4.Inverse(b1.Inertia).transpose * Vector3.Cross(p1, contact.normal), p1)))));
           
            var rvt = Vector3.Dot(rv, contact.tangent);
            if (Mathf.Abs(rvt) > 0 && IgnoreCollisionFriction == false){
                contact.velocity = b1.velocity + ((j * contact.normal) + ((b1.dynamicFriction * j) * contact.tangent)) / b1.mass;
                var worldAngVel = b1.angularVelocityWorld + (Vector3)(Matrix4x4.Inverse(b1.Inertia).transpose * Vector3.Cross(p1, ((j * contact.normal) + ((b1.dynamicFriction * j) * contact.tangent))));
                contact.angularVelocity = QuaternionConjugate(b1.transform.rotation) * worldAngVel;
            }else{
                contact.velocity = b1.velocity + (j * contact.normal) / b1.mass;
                var worldAngVel = b1.angularVelocityWorld + (Vector3)(Matrix4x4.Inverse(b1.Inertia).transpose * Vector3.Cross(p1, ((j * contact.normal))));
                contact.angularVelocity = QuaternionConjugate(b1.transform.rotation) * worldAngVel;
            }
        }
    }


    public Vector3 t_Friction;
    public Vector3 t_Tangent;
    Vector3 CalcFriction(Vector3 force, Vector3 relativeVelocity, Vector3 normal, Vector3 tangent){
        var normalForceMag = Mathf.Abs(Vector3.Dot(force, normal.normalized));
        var tangentVel = Vector3.Project(relativeVelocity, tangent);
        var fc = dynamicFriction;//Mathf.Abs(tangentVel.magnitude) > 0.1? dynamicFriction : 0;
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
    public Vector3 t_angularAcc;
    Vector3 CalcAngularAcc(Vector3 moment, Vector3 worldAngVel){
        var invInertia = Matrix4x4.Inverse(Inertia);
        var rotInertia = Vector3.Cross(worldAngVel, (Inertia * worldAngVel));
        //var rotInertia = Vector3.Scale(angularVelocity, (Vector3)(Inertia * new Vector3(Mathf.Abs(angularVelocity.x),  Mathf.Abs(angularVelocity.y),Mathf.Abs(angularVelocity.z)))); 
        var a = invInertia * (moment - rotInertia);

        if(moment.magnitude > 0)
            t_rotInertia = rotInertia.magnitude / moment.magnitude;
        t_angularAcc = a;
        //Debug.Log($"CalcTorque - T: {momentSum}/ I:{rotInertia}/ a:{a}/ w:{w}/ (q * w):{(inertia * w)}/ i:{inertia}/ inv: {invInertia}");
        // Debug.Log($"a:({a.x},{a.y},{a.z})/ w:({acc.x},{acc.y},{acc.z})/i:{rotInertia}");
        return QuaternionConjugate(transform.rotation) * a;
    }

    Vector3 CalcMoment(Vector3 force, Vector3 actPoint){
        var center = pos;
        var localPos = center - actPoint;
        var m = Vector3.Cross(force, localPos);

        return m;
    }

    Vector3 t_moment;
    Vector3 t_momentWorld;
    Vector3 CalcWorldMoment(Vector3 localForce, Vector3 localPoint){
        var worldPoint = QuaternionConjugate(transform.rotation) * localPoint;
        var worldForce = QuaternionConjugate(transform.rotation) * localForce;
        var m = Vector3.Cross(worldPoint, worldForce);

        t_moment = CalcMoment(localForce, localPoint);
        t_momentWorld = m;

        return m;
    }

    Vector3 CalcDragForce(Vector3 velocity){
        var d = 1f; //무시 air density = 1.293f at 0 degrees Celsius;
        var cd = 0.47f; //완전한 구의 항력계수
        var u = -velocity.normalized;
        var area = Mathf.Pow(Radius, 2) * Mathf.PI;
        var f = 0.5f * d * Mathf.Pow(velocity.magnitude, 2) * cd * area * u * Drag;

        return f;
    }
    Vector3 CalcAngularDragMoment(Vector3 angularVelocity){
        var d = 1f; //무시 air density = 1.293f at 0 degrees Celsius;
        var cd = 0.47f; //완전한 구의 항력계수
        var a = Mathf.Pow(Radius, 2) * Mathf.PI;
        var u = -angularVelocity.normalized;
        var m = u * d * Mathf.Pow(angularVelocity.magnitude, 2) * cd * a * AngularDrag;

        return m;
    }

    public void Sleep(){
        force = Vector3.zero;
        velocity = Vector3.zero;

        angularAcc = Vector3.zero;
        angularVelocity = Vector3.zero;
    }
}
