using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GameManager : MonoBehaviour
{
    public Ball ballPrefab;
    public Transform ballParent;
    Ball ball;

    public RigidbodyObject rigidbodyObject;
    public Rigidbody untiyPhysicsObject;

    public Vector3 pos0;
    public Vector3 posUnity;
    public Vector3 vel0 = new Vector3(0, 10f, -20f);  
    public Vector3 rotVel0 = new Vector3();

    public bool onFootPrint = false;
    public bool onDrag = true;
    public bool onAngularDrag = true;
    public bool onAngularForce = true;
    public float Drag = 1f;
    public float AngularDrag = 1f;

    void Awake(){
        Time.fixedDeltaTime = 0.005f;
    }

    void Start(){
        ballPrefab.gameObject.SetActive(false);
        pos0 = rigidbodyObject.transform.position;
        posUnity = untiyPhysicsObject.transform.position;
        OnSpawnBall();
    }

    public void OnSpawnBall(){
        // if (ball != null){
        //     Destroy(ball.gameObject);
        // }
        // ball = Instantiate(ballPrefab, ballParent);
        // ball.gameObject.SetActive(true);
        // ball.Spawn(ball.transform.position);

        rigidbodyObject.transform.position = pos0;
        untiyPhysicsObject.transform.position = posUnity;
    }

    public void OnShoot(){
        // ball.onDrag = onDrag;
        // ball.onAngularDrag = onAngularDrag;
        // ball.onAngularForce = onAngularForce;
        // ball.drag = Drag;
        // ball.angularDrag = AngularDrag;
        // ball.onFootPrint = onFootPrint;
        // ball.Shoot(vel0, rotVel0);

        rigidbodyObject.velocity = vel0;
        untiyPhysicsObject.velocity = vel0;
    }
}
