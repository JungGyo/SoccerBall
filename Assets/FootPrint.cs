using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FootPrint : MonoBehaviour
{
    List<GameObject> pool;
    public Transform parent;
    public GameObject footPrint;
    List<GameObject> footPrintList;

    void Awake(){
        footPrintList = new List<GameObject>();
        if (pool == null){
            pool = new List<GameObject>();
            for(int i = 0; i < 20; i++){
                var obj = Instantiate(footPrint, parent);
                pool.Add(obj);
                obj.gameObject.SetActive(false);
            }
        }
    }

    GameObject PullFootPrint(){
        var obj = pool.FirstOrDefault(t => t.activeSelf == false);
        if (obj == null){
            var newObj = Instantiate(footPrint, parent);
            pool.Add(newObj);
            obj = newObj;
        }
        footPrintList.Add(obj);
        return obj;
    }

    public void Draw(Vector3 pos){
        var obj = PullFootPrint();
        obj.gameObject.SetActive(true);
        obj.transform.position = pos; 
    }

    public void Reset(){
        footPrintList.ForEach(t => {
            if (t.gameObject == null)
                return;
            t.gameObject.SetActive(false);
        });
        footPrintList.Clear();
    }
}
