using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GameManager : MonoBehaviour
{
    public Ball ballPrefab;
    public Transform ballParent;
    Ball ball;

    public Vector3 vel0 = new Vector3(0, 10f, -20f);  
    public Vector3 rotVel0 = new Vector3();

    public bool onFootPrint = false;
    public bool onDrag = true;
    public bool onSpinDrag = true;
    public bool onSpinForce = true;
    public float spinFactor = 1f;

    void Awake(){
        Time.fixedDeltaTime = 0.005f;
    }

    void Start(){
        ballPrefab.gameObject.SetActive(false);
        OnSpawnBall();
    }

    public void OnSpawnBall(){
        if (ball != null){
            Destroy(ball.gameObject);
        }
        ball = Instantiate(ballPrefab, ballParent);
        ball.gameObject.SetActive(true);
        ball.Spawn(ball.transform.position);
    }

    public void OnShoot(){
        ball.onDrag = onDrag;
        ball.onSpinDrag = onSpinDrag;
        ball.onSpinForce = onSpinForce;
        ball.spinFactor = spinFactor;
        ball.onFootPrint = onFootPrint;
        ball.Shoot(vel0, rotVel0);
    }
}
