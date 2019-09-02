using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class wallMove : MonoBehaviour {

    // Use this for initialization
    void Start()
    {

    }
    float speed = 4f;//
    // Update is called once per frame
    void FixedUpdate()
    {
        //speed = speed * 1.1f;
        float x =   speed* Time.deltaTime;
        //float x = 1f * speed;
        //float z =  Time.deltaTime * speed;
        transform.Translate(x, 0, 0);
        //print(transform.position.x);
    }
}
