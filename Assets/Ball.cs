using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class Ball : MonoBehaviour
{
    const float AirDensity = 1.293f;
    Vector3 gravity {get{return new Vector3(0, -9.81f, 0);}} //const gravity

    public FootPrint footPrint;
    public bool onFootPrint;
    public bool onDrag;
    public bool onSpinDrag;
    public bool onSpinForce;
    public float spinFactor = 1f; 
    float bounceness = 0.5f; //축구공 반발계수 0.77~0.9 (콘크리트)
    bool isMoving = false;
    float mass = 0.45f;
    float radius = 0.11f;
    float footPrintLoopTime = .2f;
    
    Vector3 vel0 = new Vector3();  
    Vector3 rotVel0 = new Vector3();

    public void Spawn(Vector3 position){
        pos = position;
        isMoving = true;
    }

    public void Shoot(Vector3 velocity, Vector3 rotationVelocity){
        this.vel0 = velocity;
        this.rotVel0 = rotationVelocity;
        
        time = 0;
        vel = vel0;
        rotVel = rotVel0;
        acc = gravity;
        if (onFootPrint) footPrint.Draw(pos);
    }

    float time = 0;
    float lastFootPrintTime = 0;
    public Vector3 vel = Vector3.zero;
    public Vector3 pos = Vector3.zero;
    public Vector3 acc = Vector3.zero;

    Vector3 rotVel = Vector3.zero;
    Vector3 rotAcc = Vector3.zero;
    Quaternion rotPos = Quaternion.Euler(Vector3.zero);

    void FixedUpdate(){
        if (isMoving == false)
            return;

        var delta = Time.fixedDeltaTime;

        if (onContact){
            if(reflectionVectors.Count > 0){
                reflectionVectors.ForEach(nVector =>{
                    vel = CalcReflectionVelocity(vel, nVector) * bounceness;
                    acc = CalcReactionForceAcc(acc, nVector);
                });
                reflectionVectors.Clear();
            }else{
                acc = CalcReactionForceAcc(acc, contactNormalVector);
            }
        }

        // Debug.Log($"time: {time}/vel0: {vel}/acc0: {acc}");

        pos = CalcPos(pos, vel, acc, delta);
        vel = CalcVel(vel, acc, delta);

        if (onSpinDrag)
            rotAcc = CalcSpinDragRotAcc(rotVel);
        rotVel = CalcVel(rotVel, rotAcc, delta);

        var dragAcc = onDrag? CalcDragAcc(vel) : Vector3.zero;
        var spinForceAcc = onSpinForce? CalcSpinForceAcc(vel, rotVel) : Vector3.zero;
        acc = dragAcc + gravity + spinForceAcc;

        this.transform.Rotate(rotVel, Space.Self);

        if (time >= lastFootPrintTime + footPrintLoopTime){
            if (onFootPrint) footPrint.Draw(pos);
            lastFootPrintTime = time;
        }

        this.transform.position = pos;
        time += delta;
        // Debug.Log($"time: {time}/vel1: {vel}/acc1: {acc}");
    }

    Vector3 CalcPos(Vector3 p0, Vector3 v0, Vector3 a0, float t){
        return p0 + v0 * t + 0.5f * a0 * Mathf.Pow(t, 2);
    }

    Vector3 CalcVel(Vector3 v0, Vector3 a0, float t){
        return v0 + a0 * t;
    }
    
    //가정1. 완전한 구(항력계수: 0.47)
    //가정2. 온도: 0도(공기밀도: 1.293)
    //가정3. 5호 축구공(지름: 0.22, 무게: 0.45)
    Vector3 CalcDragAcc(Vector3 vel){
        var cd = 0.47f;
        var a = Mathf.Pow(radius, 2) * Mathf.PI;
        var u = vel.normalized * -1;
        var force = 0.5f * AirDensity * Mathf.Pow(vel.magnitude, 2) * cd * a * u;

        // Debug.Log($"drag force: {force}, acc: {force/mass}, vel: {vel}");
        return force / mass;
    }

    //가정1. 속이 빈 구(관성모멘트: 2/3 * m * r^2)
    //가정2. 반구에 저항 작용(항력계수: 0.42)
    //5호 축구공(지름: 0.22, 무게: 0.45)
    Vector3 CalcSpinDragRotAcc(Vector3 w){
        var inertia = 2/3f * mass * Mathf.Pow(radius, 2);
        var cd = 0.42f;
        var a = Mathf.Pow(radius, 2) * Mathf.PI /2;
        var u = w.normalized * -1;
        var force = 0.5f * AirDensity * Mathf.Pow(w.magnitude * radius, 2) * cd * a * u;

        // Debug.Log($"drag force: {force}, acc: {force/mass}, vel: {vel}");
        return force / inertia;
    }

    //완전 탄성 충돌
    Vector3 CalcReflectionVelocity(Vector3 vel, Vector3 nVector){
        var n = nVector.normalized;
        return Vector3.Reflect(vel, n); //(vel - 2f * Vector3.Dot(vel, n) * n);
    }

    //Wall은 질량이 무하다고 가정. 벽의 수직 방향 가속도는 0이 되어야함.(수직항력)
    Vector3 CalcReactionForceAcc(Vector3 acc, Vector3 nVector){
        var n = nVector.normalized;
        var newAcc = (acc - Vector3.Dot(acc, n) * n);
        Debug.Log($"CalcReactionForceAcc - acc: {acc}/accNew: {newAcc}/normal vector:{n}");
        return newAcc;
    }

    // Magnus effect(회전에 의해 생성된 커브 볼 효과)
    Vector3 CalcSpinForceAcc(Vector3 vel, Vector3 rotVel){
        var w = rotVel; //* Mathf.Deg2Rad;
        var a = Mathf.Pow(radius, 2) * Mathf.PI;
        var cv = Vector3.Cross(w.normalized, vel.normalized);
        var force = 0.5f * spinFactor * w.magnitude * vel.magnitude * radius * a * cv;

        return force / mass;
    }

    List<Vector3> reflectionVectors = new List<Vector3>();
    public bool onContact;
    Vector3 contactNormalVector;

    void OnCollisionEnter(Collision col){
        if (isMoving == false)
            return;

        contactNormalVector = col.GetContact(0).normal.normalized;
        reflectionVectors.Add(contactNormalVector);
        onContact = true;
        // if (onFootPrint) footPrint.Draw(pos);
        Debug.Log($"time: {time} - OnCollisionEnter({col.contacts.Length})/ contact nv: {contactNormalVector}");
    }
    void OnCollisionStay(Collision col){
        contactNormalVector = col.GetContact(0).normal.normalized;
        onContact = true;
        Debug.Log($"time: {time} - OnCollisionStay({col.contacts.Length})");
    }

    void OnCollisionExit(Collision col){
        onContact = false;
        Debug.Log($"time: {time} - OnCollisionExit({col.contacts.Length})");
    }

    void OnDestroy(){
        footPrint.Reset();
    }

    // Vector3 CalibContactPos(Vector3 normal, float dist){
    //     var newPos = pos + normal.normalized * dist;
    //     Debug.Log($"pos: {pos}/ new: {newPos}");
    //     return newPos;
    // }
}
