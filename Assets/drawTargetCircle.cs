using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class drawTargetCircle : MonoBehaviour {

	private LineRenderer targetCircle;
	private float theta = 0f;
	private float positionScale;

	public float thetaScale = 0.01f;
	public float xRadius = 10f;
	public float yRadius = 10f;
	public float xLoc = 0.1f;
	public float yLoc = -0.35f;


	// Use this for initialization
	void Start () {
		targetCircle = GetComponent<LineRenderer> ();
		positionScale = GameObject.Find ("robotConnection").GetComponent<robotController>().positionScale;
	}

	// Update is called once per frame
	void Update () {
		theta = 0f;
		int size = (int)((1f / thetaScale) + 1f);
		targetCircle.SetVertexCount (size);
		for (int i = 0; i < size; i++) {
			theta += 2f * Mathf.PI * thetaScale;
			targetCircle.SetPosition (i, new Vector3 (xRadius * Mathf.Cos (theta) + transform.position.x + xLoc*positionScale,
				yRadius * Mathf.Sin (theta) + transform.position.y + yLoc*positionScale, 0));
		}
	}
}
