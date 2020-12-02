using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FootPrint : MonoBehaviour
{
    public Transform footPrintGroup;
    public GameObject footPrint;
    List<GameObject> footPrintPool;

    void Awake(){
        if (footPrintPool == null){
            footPrintPool = new List<GameObject>();
            for(int i = 0; i < 20; i++)
                footPrintPool.Add(Instantiate(footPrint, footPrintGroup));
        }
    }

    public void Draw(Vector3 pos){
        var obj = Instantiate(footPrint, footPrintGroup);
        footPrintPool.Add(obj);
        footPrint.transform.position = pos;
    }
}
