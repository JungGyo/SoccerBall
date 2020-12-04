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
            for(int i = 0; i < 20; i++){
                var obj = Instantiate(footPrint, footPrintGroup);
                footPrintPool.Add(obj);
                obj.gameObject.SetActive(false);
            }
        }
    }

    int num = 0;
    public void Draw(Vector3 pos){
        if (footPrintPool.Count <= num){
            var obj = Instantiate(footPrint, footPrintGroup);
            footPrintPool.Add(obj);
        }
        footPrintPool[num].gameObject.SetActive(true);
        footPrintPool[num].transform.position = pos;
        num++;
    }
}
