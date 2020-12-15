using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(Collider))]
public class RigidbodyObject : MonoBehaviour
{
    const float VelocityThreshold = 0.001f;
    const float AirDensity = 1f; //무시 //1.293f at 0 degrees Celsius;
    Vector3 gravity {get{return Physics.gravity;}} //const gravity
    float mass {get{return rigidbody.mass;}}
    bool isMoving {get{return velocity.magnitude != 0;}}

    public float Boundness {
        get {return bounceness;}
        set {bounceness = Mathf.Clamp(value, 0, 1);}
    }

    public float bounceness = 0.5f; //콘크리트에서 축구공의 반발계수 0.77~0.9 (콘크리트)
    public float dynamicFriction = 0.6f;
    public float staticFriction = 0.4f;
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

    float radius{
        get{
            if(collider.GetType() != typeof(SphereCollider)){
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
    }

    bool t_on = false;
    void FixedUpdate(){
        var delta = Time.fixedDeltaTime;

        var exForce = gravity * mass;

        var moments = new List<Moment>();
        var friction = Vector3.zero;
        var reaction = Vector3.zero;
        var collisionReact = Vector3.zero;
        if (onContacted){
            foreach(var key in contactDic.Keys){
                var contact = contactDic[key]; 
                var nVector = contact.normal.normalized;
                if (reflectionDone.Contains(key) == false){
                    //velocity = Vector3.Scale(Vector3.Reflect(velocity, nVector), new Vector3(1, bounceness, 1));
                    velocity = contact.velocity;
                    angularVelocity = contact.angularVelocity;
                    reflectionDone.Add(key);
                    t_on = true;
                }

                //Cotact
                float tangentVel = Mathf.Abs(Vector3.Project(contact.relativeVelocity, contact.tangent).magnitude);
                if (tangentVel > VelocityThreshold){
                    friction += CalcFriction(exForce, contact.normal, contact.tangent);
                    moments.Add(new Moment(){force = friction, actPoint = pos - nVector * radius});
                }
                reaction += CalcReactionForce(exForce, nVector);
                moments.Add(new Moment(){force = reaction, actPoint = pos - nVector * radius});

            }
        }

        force = (exForce + friction + reaction);
        var acc = force/mass;
        velocity += CalcDeltaVelocity(acc, delta);
        pos += CalcDeltaPos(velocity, acc, delta);

        angularAcc = CalcangularAcc(moments, angularVelocity);
        angularVelocity += CalcDeltaVelocity(angularAcc, delta);
        transform.Rotate(angularVelocity, Space.Self);
    }

    float time;
    void Update(){
        if((int)(time * 10 % 5) == 0){
            //Debug.Log($"vel: {velocity}/acc: {acc}");
        }

        time += Time.deltaTime;
    }

    Vector3 CalcDeltaPos(Vector3 v0, Vector3 a0, float t){
        return v0 * t;
    }

    Vector3 CalcDeltaVelocity(Vector3 a0, float t){
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
        public float j;
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
            pos -= point.normal.normalized * col.GetContact(0).separation;
            UpdateCotactInfo(this, null, ref contactInfo);
        }
        contactDic[col.collider] = contactInfo;

        t_j = contactInfo.j;
        t_cv = contactInfo.velocity;
        t_cav = contactInfo.angularVelocity;

        var cv2 = Vector3.Scale(Vector3.Reflect(velocity, contactInfo.normal), new Vector3(1, bounceness, 1));

        Debug.Log($"name: {col.gameObject.name} - OnCollisionEnter({col.contacts.Length})/ p1:{contactInfo.point - transform.position}/ v1:{velocity}/ rv:{contactInfo.relativeVelocity}/ nv: {contactInfo.normal}, tv:{contactInfo.tangent}, j:{contactInfo.j}, cv:{contactInfo.velocity}, cv2:{cv2}");
    }

    public bool IgnoreCollisionFriction = true;
    public float t_j;
    public Vector3 t_cv;
    public Vector3 t_cav;
    void UpdateCotactInfo(RigidbodyObject b1, RigidbodyObject b2, ref ContactInfo contact){
            if (b2 != null){//contact with RigidbodyObject.
                var p1 = contact.point - b1.transform.position;
                var p2 = contact.point - b2.transform.position;
                var v1 = b1.velocity + Vector3.Cross(b1.angularVelocity, p1);
                var v2 = b2.velocity +  Vector3.Cross(b2.angularVelocity, p2);
                var rv = v2 - v1;
                
                var j = (-(1 + (b1.bounceness + b2.bounceness)/2f) * Vector3.Dot(rv, contact.normal))/
                    ((1/b1.mass + 1/b2.mass) + (Vector3.Dot(contact.normal, (Vector3.Cross(Matrix4x4.Inverse(b1.Inertia).transpose * Vector3.Cross(p1, contact.normal), p1))))
                    + (Vector3.Dot(contact.normal, (Vector3.Cross(Matrix4x4.Inverse(b2.Inertia).transpose * Vector3.Cross(p2, contact.normal), p2)))));

                contact.relativeVelocity = rv;
                contact.tangent = Vector3.Cross(Vector3.Cross(contact.normal, rv), contact.normal).normalized;
                contact.j = j;

                var rvt = Vector3.Dot(rv, contact.tangent);

                if (Mathf.Abs(rvt) > 0 && IgnoreCollisionFriction == false){
                    contact.velocity = b1.velocity + ((j * contact.normal) + ((b1.dynamicFriction * j) * contact.tangent)) / b1.mass;
                    contact.angularVelocity = b1.angularVelocity + (Vector3)(Matrix4x4.Inverse(b1.Inertia).transpose * Vector3.Cross(p1, ((j * contact.normal) + ((b1.dynamicFriction * j) * contact.tangent))));
                }else{
                    contact.velocity = b1.velocity + (j * contact.normal) / b1.mass;
                    contact.angularVelocity = b1.angularVelocity + (Vector3)(Matrix4x4.Inverse(b1.Inertia).transpose * Vector3.Cross(p1, ((j * contact.normal))));
                }
            }else{//contact with non-RigidbodyObject.
                var p1 = contact.point - b1.transform.position;
                var v1 = b1.velocity + Vector3.Cross(b1.angularVelocity, p1);

                var rv = v1;
                contact.relativeVelocity = rv;
                contact.tangent = Vector3.Cross(Vector3.Cross(contact.normal, rv), contact.normal).normalized;

                var j = (-(1 + b1.bounceness/2f) * Vector3.Dot(rv, contact.normal))/
                    ((1/b1.mass) + (Vector3.Dot(contact.normal, (Vector3.Cross(Matrix4x4.Inverse(b1.Inertia) * Vector3.Cross(p1, contact.normal), p1))))
                    + 0);
                var rvt = Vector3.Dot(rv, contact.tangent);

                if (Mathf.Abs(rvt) > 0 && IgnoreCollisionFriction == false){
                    contact.velocity = b1.velocity + ((j * contact.normal) + ((b1.dynamicFriction * j) * contact.tangent)) / b1.mass;
                    contact.angularVelocity = b1.angularVelocity + (Vector3)(Matrix4x4.Inverse(b1.Inertia).transpose * Vector3.Cross(p1, ((j * contact.normal) + ((b1.dynamicFriction * j) * contact.tangent))));
                }else{
                    contact.velocity = b1.velocity + (j * contact.normal) / b1.mass;
                    contact.angularVelocity = b1.angularVelocity + (Vector3)(Matrix4x4.Inverse(b1.Inertia).transpose * Vector3.Cross(p1, ((j * contact.normal))));
                }
            }
    }

    void OnCollisionStay(Collision col){
        if (reflectionDone.Contains(col.collider)){
            var point = col.GetContact(0);
            var contactInfo = new ContactInfo(){point = point.point, normal = point.normal.normalized, relativeVelocity = -velocity};
            var other = col.gameObject.GetComponent<RigidbodyObject>();
            
            if (other != null){
                UpdateCotactInfo(this, other, ref contactInfo);
            }else{
                //RigidbodyObject가 아닌 isKinematic rigidbody인 경우 위치 보정.
                pos -= point.normal.normalized * col.GetContact(0).separation;
                UpdateCotactInfo(this, null, ref contactInfo);
            }
            contactDic[col.collider] = contactInfo; 
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

    public Vector3 t_Friction;
    Vector3 CalcFriction(Vector3 force, Vector3 normal, Vector3 tangent){
        var normalForceMag = Vector3.Project(force, normal.normalized).magnitude;
        var f = dynamicFriction * mass * normalForceMag * tangent.normalized;
        // Debug.Log($"CalcFriction - F: {force}/nVector:{n}/ uVector:{tangent}");
        t_Friction = f;
        return f;
    }

    public Vector3 t_Reaction;
    //TODO: Wall이 아닌 RigidObject에 대한 반력 계산으로 수정되어야함.
    //Wall은 질량이 무한하다고 가정시 수직 방향 가속도는 0이 되어야함.(수직항력 or 반력)
    Vector3 CalcReactionForce(Vector3 force, Vector3 nVector){
        var f = -Vector3.Project(force,  nVector.normalized);
        t_Reaction = f;
        // Debug.Log($"CalcReactionForce - F: {tangentForce}/nVector:{n}");
        return f;
    }

    class Moment{
        public Vector3 force;
        public Vector3 actPoint;
    }

    // class Matrix3x3{
    //     public Vector3 e1, e2, e3;
    //     public Matrix3x3(Vector3 v1, Vector3 v2, Vector3 v3){
    //         e1 = v1;
    //         e2 = v2;
    //         e3 = v3;
    //     }
    // }

    public Vector3 qVector {
        get{return q.eulerAngles;}
    }
    Quaternion q;
    public Vector3 rotInertia;
    public Vector3 momentSum;
    public Vector3 t_angularAcc;
    Vector3 CalcangularAcc(IEnumerable<Moment> moments, Vector3 angularVelocity){
        var invInertia = Matrix4x4.Inverse(Inertia);
        momentSum = CalcMomentSum(moments);
        rotInertia = Vector3.Scale(angularVelocity, (Inertia * angularVelocity));
        var a = invInertia * (-momentSum + rotInertia);
        t_angularAcc = a;
        //Debug.Log($"CalcTorque - T: {momentSum}/ I:{rotInertia}/ a:{a}/ w:{w}/ (q * w):{(inertia * w)}/ i:{inertia}/ inv: {invInertia}");
        // Debug.Log($"a:({a.x},{a.y},{a.z})/ w:({acc.x},{acc.y},{acc.z})/i:{rotInertia}");
        return a; 
    }

    Vector3 CalcMomentSum(IEnumerable<Moment> moments){
        var sum = Vector3.zero;
        foreach(var m in moments){
            sum += CalcMoment(m.force, m.actPoint);
        }
        t_on = false;
        return sum;
    }

    Vector3 CalcMoment(Vector3 force, Vector3 actPoint){
        var center = pos;
        var localPos = center - actPoint;
        var torque = Vector3.Cross(localPos, force);

        if (t_on)
            Debug.Log($"CalcTorque - T: {torque}/ r:{localPos}/ f:{force}");
            
        return torque;
    }


    public void Sleep(){
        force = Vector3.zero;
        velocity = Vector3.zero;

        angularAcc = Vector3.zero;
        angularVelocity = Vector3.zero;
    }
}
