using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class keyboardmove : MonoBehaviour {

	// Use this for initialization
	void Start () {
        
    }
    int speed = 10;
    // Update is called once per frame
    void Update () {
        float x = Input.GetAxis("Horizontal") * Time.deltaTime * speed;
        float z = Input.GetAxis("Vertical") * Time.deltaTime * speed;
        transform.Translate(x, 0, z);
        print(transform.position.x);
    }
}
