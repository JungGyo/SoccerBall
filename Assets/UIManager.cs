using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class UIManager : MonoBehaviour
{
    public GameManager gameManager;
    public CanvasScaler canvasScaler;
    public RectTransform targetBase;
    public RectTransform target;
    public RectTransform selected;

    
    public void OnClickTarget(){
        selected.position = target.position;
        var radius = 0.75f;
        var range = 45f; 
        var pos = selected.localPosition/range * radius;
        gameManager.SetShootPoint(new Vector3(-pos.x, pos.y, Mathf.Sqrt(radius * radius - (pos.x * pos.x + pos.y * pos.y))));
    }

    bool onPointer;
    void Update(){
        if(onPointer == false)
            return;

        target.position = new Vector2(Input.mousePosition.x * canvasScaler.referenceResolution.x / Screen.width, Input.mousePosition.y * canvasScaler.referenceResolution.y / Screen.height);
    }

    public void OnPointerEnter(){
        target.gameObject.SetActive(true);
        onPointer = true;
    }

    public void OnPointerExit(){
        target.gameObject.SetActive(false);
        onPointer = false;
    }
}
