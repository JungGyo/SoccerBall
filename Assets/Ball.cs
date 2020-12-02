using System.Collections;
using System.Collections.Generic;
using UnityEngine;

//TODO: Ball을 범용 RigidBody로 구현. 
public class Ball : MonoBehaviour
{
    const float AirDensity = 1;
    Vector3 gravity = new Vector3(0, -9.81f, 0);
    // Start is called before the first frame update
    void Start()
    {
        AddSpin(rotVel0);
        Shoot();
    }

    public FootPrint footPrint;
    public bool onDrag;
    public bool onSpinForce;
    public float spinFactor = 1f; 
    public float mass = 0.45f;
    public float radius = 0.11f;
    public float footPrintLoopTime = .5f;
    public float bounceness = .5f;
    public Vector3 pos0 = new Vector3();
    public Vector3 vel0 = new Vector3();
    
    public Vector3 rotVel0 = new Vector3();
    bool shoot = false;

    public void AddSpin(Vector3 vel){
        rotVel = vel;
    }
    
    public void Shoot(){
        time = 0;
        vel = vel0;
        pos = pos0;
        acc = gravity;
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

    void FixedUpdate(){
        if (shoot == false)
            return;

        var delta = Time.fixedDeltaTime;
            
        pos = CalcPos(pos, vel, acc, delta);
        vel = CalcVel(vel, acc, delta);

        rotAcc = CalcSpinDragAcc(rotVel);
        rotVel = CalcVel(rotVel, rotAcc, delta);

        var drag = onDrag? CalcDragAcc(vel) : Vector3.zero;
        var spinForce = onSpinForce? CalcSpinForceAcc(vel, rotVel) : Vector3.zero;
        if (onGround)
            acc = drag + spinForce;
        else
            acc = drag + gravity + spinForce;

        this.transform.Rotate(rotVel, Space.Self);

        if (onReflection){
            footPrint.Draw(pos);
            vel = CalcReflectionVelocity(vel, Vector3.up) * bounceness;
            pos = CalcPos(pos, vel, acc, delta);
            onReflection = false;
            onGround = false;
        }

        if (time >= lastFootPrintTime + footPrintLoopTime){
            footPrint.Draw(pos);
            lastFootPrintTime = time;
        }

        this.transform.position = pos;
        time += delta;

        // Debug.Log($"time: {time}/pos: {pos.x},{pos.y},{pos.z}/");
    }

    Vector3 CalcPos(Vector3 p0, Vector3 v0, Vector3 acc, float t){
        var p1 = p0 + v0 * t + 0.5f * acc * Mathf.Pow(t, 2);
        if (onGround)
            p1 = CalibGroundPos(p1);

        return p1;
    }

    Vector3 CalcVel(Vector3 v0,  Vector3 acc, float t){
        return v0 + acc * t;
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
        return  (vel - 2f * Vector3.Dot(vel, n) * n);
    }

    Vector3 CalcSpinForceAcc(Vector3 vel, Vector3 rotVel){
        var w = rotVel; //* Mathf.Deg2Rad;
        var a = Mathf.Pow(radius, 2) * Mathf.PI;
        var cv = Vector3.Cross(w.normalized, vel.normalized);
        var force = 0.5f * spinFactor * w.magnitude * vel.magnitude * radius * a * cv;

        return force / mass;
    }

    public bool onGround;
    public bool onReflection;
    void OnTriggerEnter(Collider col){
        if (col.gameObject.tag == "GROUND"){
            if (onGround == false && vel.y < 0){
                onReflection = true;
            }
            onGround = true;
            // Debug.Log($"OnTriggerEnter");
        }
    }

    void OnTriggerStay(Collider col){
        if (col.gameObject.tag == "GROUND"){
            onGround = true;
            // Debug.Log($"OnTriggerStay");
        }
    }

    void OnTriggerExit(Collider col){
        if (col.gameObject.tag == "GROUND"){
            onGround = false;
            // Debug.Log($"OnTriggerExit");
        }
    }

    Vector3 CalibGroundPos(Vector3 p){
        var pp = p;
        if (p.y < 1)
            pp = new Vector3(p.x, 1, p.z);

        return pp;
    }
}
