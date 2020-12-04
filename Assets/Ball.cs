using System.Collections;
using System.Collections.Generic;
using UnityEngine;

//TODO: Ball을 범용 RigidBody로 구현. 
public class Ball : MonoBehaviour
{
    const float AirDensity = 1;
    Vector3 gravity {get{return new Vector3(0, -9.81f, 0);}} //const gravity

    public FootPrint footPrint;
    public bool onDrag;
    public bool onSpinForce;
    public float spinFactor = 1f; 
    public float bounceness = (0.77f + 0.9f)/2f; //축구공 반발계수 0.77~0.9.
    bool shoot = false;
    float mass = 0.45f;
    float radius = 0.11f;
    float footPrintLoopTime = .2f;
    
    Vector3 vel0 = new Vector3();  
    Vector3 rotVel0 = new Vector3();

    public void Spawn(Vector3 position){
        pos = position;
    }

    public void Shoot(Vector3 velocity, Vector3 rotationVelocity){
        this.vel0 = velocity;
        this.rotVel0 = rotationVelocity;
        
        time = 0;
        vel = vel0;
        rotVel = rotVel0;
        acc = gravity;
        footPrint.Draw(pos);
        shoot = true;
    }

    float time = 0;
    float lastFootPrintTime = 0;
    public Vector3 vel = Vector3.zero;
    public Vector3 pos = Vector3.zero;
    public Vector3 acc = Vector3.zero;

    Vector3 rotVel = Vector3.zero;
    Vector3 rotAcc = Vector3.zero;
    Quaternion rotPos = Quaternion.Euler(Vector3.zero);

    public float posMax = 10;
    void Update(){
        if (shoot == false)
            return;

        var delta = Time.deltaTime;

        pos = CalcPos(pos, vel, acc, delta);
        vel = CalcVel(vel, acc, delta);

        posMax = Mathf.Max(posMax, pos.y);

        rotAcc = CalcSpinDragAcc(rotVel);
        rotVel = CalcVel(rotVel, rotAcc, delta);

        var drag = onDrag? CalcDragAcc(vel) : Vector3.zero;
        var spinForce = onSpinForce? CalcSpinForceAcc(vel, rotVel) : Vector3.zero;
        acc = drag + gravity + spinForce;

        this.transform.Rotate(rotVel, Space.Self);


        if (time >= lastFootPrintTime + footPrintLoopTime){
            footPrint.Draw(pos);
            lastFootPrintTime = time;
        }

        this.transform.position = pos;
        time += delta;
        // Debug.Log($"time: {time}/pos: {pos.x},{pos.y},{pos.z}/");
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

    Vector3 CalcSpinDragAcc(Vector3 w){
        var cd = 0.47f;
        var a = Mathf.Pow(radius, 2) * Mathf.PI;
        var u = w.normalized * -1;
        var force = 0.5f * AirDensity * Mathf.Pow(w.magnitude, 2) * cd * a * u;

        // Debug.Log($"drag force: {force}, acc: {force/mass}, vel: {vel}");
        return force / mass;
    }

    //완전 탄성 충돌
    Vector3 CalcReflectionVelocity(Vector3 vel, Vector3 nVector){
        var n = nVector.normalized;
        return  (vel - 2f * Vector3.Dot(vel, n) * n); //Vector3.Reflect(vel, n);
    }

    Vector3 CalcSpinForceAcc(Vector3 vel, Vector3 rotVel){
        var w = rotVel; //* Mathf.Deg2Rad;
        var a = Mathf.Pow(radius, 2) * Mathf.PI;
        var cv = Vector3.Cross(w.normalized, vel.normalized);
        var force = 0.5f * spinFactor * w.magnitude * vel.magnitude * radius * a * cv;

        return force / mass;
    }

    public bool onReflection;

    void OnCollisionEnter(Collision col){
        if (col.gameObject.tag == "WALL"){
            var contact = col.GetContact(0);
            //pos = CalibContactPos(-contact.normal.normalized, contact.separation);
            vel = CalcReflectionVelocity(vel, contact.normal.normalized) * bounceness;
            footPrint.Draw(pos);
            Debug.Log($"time: {time} - OnCollisionEnter");
        }
    }
    void OnTriggerStay(Collider col){
        if (col.gameObject.tag == "WALL"){
            //onGround = true;
            // Debug.Log($"OnTriggerStay");
        }
    }

    void OnTriggerExit(Collider col){
        if (col.gameObject.tag == "WALL"){
            //onGround = false;
            // Debug.Log($"OnTriggerExit");
        }
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
