using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Ball : MonoBehaviour
{
    Vector3 Gravity = new Vector3(0, -9.81f, 0);
    // Start is called before the first frame update
    void Start()
    {
        Shoot();
    }

    public FootPrint footPrint;
    public float footPrintLoopTime = .5f;
    public float bounceness = .5f;
    public Vector3 pos0 = new Vector3();
    public Vector3 vel0 = new Vector3();
    bool shoot = false;
    
    public void Shoot(){
        time = 0;
        vel = vel0;
        pos = pos0;
        shoot = true;
    }

    float time = 0;
    float lastFootPrintTime = 0;
    Vector3 vel = Vector3.zero;
    Vector3 pos = Vector3.zero;

    void FixedUpdate(){
        if (shoot == false)
            return;

        var delta = Time.fixedDeltaTime;

        pos = CalcPos(pos, vel, delta);
        vel = CalcVel(vel, delta);

        if (onReflection){
            footPrint.Draw(pos);
            vel = CalcReflectionVelocity(vel, Vector3.up);
            pos = CalcPos(pos, vel, delta);
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

    Vector3 CalcPos(Vector3 p0, Vector3 v0, float t){
        if (onGround)
            return p0 + v0 * t;

        return p0 + v0 * t + 0.5f * Gravity * Mathf.Pow(t, 2);
    }

    Vector3 CalcVel(Vector3 v0, float t){
        if (onGround)
            return v0;

        return v0 + Gravity * t;
    }

    //완전탄성충돌
    Vector3 CalcReflectionVelocity(Vector3 vel, Vector3 nVector){
        var n = nVector.normalized;
        return  (vel - 2f * Vector3.Dot(vel, n) * n) * bounceness;
    }

    bool onGround;
    bool onReflection;
    void OnTriggerEnter(Collider col){
        if (col.gameObject.tag == "GROUND"){
            if (onGround == false && vel.y < 0){
                onReflection = true;
            }
            onGround = true;
            Debug.Log($"OnTriggerEnter");
        }
    }

    void OnTriggerStay(Collider col){
        if (col.gameObject.tag == "GROUND"){
            //여기
            Debug.Log($"OnTriggerStay");
        }
    }

    void OnTriggerExit(Collider col){
        if (col.gameObject.tag == "GROUND"){
            onGround = false;
            Debug.Log($"OnTriggerExit");
        }
    }
}
