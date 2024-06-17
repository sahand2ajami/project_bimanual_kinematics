using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class drawPath : MonoBehaviour {

	private LineRenderer path;
	private float theta = 0f;
	private Transform leftHand;

	private int pathLength = 0;

	public float thetaScale = 0.01f;
	public float xRadius = 10f;
	public float yRadius = 10f;
	public float xLoc = 0.1f;
	public float yLoc = -0.35f;


	// Use this for initialization
	void Start () {
		path = GetComponent<LineRenderer> ();
		leftHand = GameObject.Find ("leftHand").GetComponent<Transform>();
	}

	// Update is called once per frame
	void Update () {
		pathLength += 1;
		path.SetVertexCount(pathLength);
//		theta = 0f;
//		int size = (int)((1f / thetaScale) + 1f);
//		targetCircle.SetVertexCount (size);
//		for (int i = 0; i < size; i++) {
//			theta += 2f * Mathf.PI * thetaScale;
//			targetCircle.SetPosition (i, new Vector3 (xRadius * Mathf.Cos (theta) + transform.position.x + xLoc*positionScale,
//				yRadius * Mathf.Sin (theta) + transform.position.y + yLoc*positionScale, 0));
//		}

	}
}
