using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RayCast : MonoBehaviour
{
    bool showLines;
    [Header("Visualization Options")]
    [SerializeField]
    private GameObject linePrefab;
    [SerializeField]
    private float vanishingTime;
    
    private GameObject line;
    private LineRenderer lineRender;

    private IEnumerator coroutine;

    // Start is called before the first frame update
    //void Start()
    //{
    //    Debug.Log("Activated Ray Cast");

    //}

    private IEnumerator WaitAndRemoveLine(float waitTime, LineRenderer x)
    {
        yield return new WaitForSeconds(waitTime);
        x.positionCount = 0;
    }

    // Update is called once per frame
    void Update()
    {
        if (showLines) 
        {
            Ray ray = new Ray(transform.position, transform.up);
            RaycastHit hitData;

            if (Physics.Raycast(ray, out hitData))
            {
                GameObject line = Instantiate(linePrefab);
                LineRenderer lineRender = line.GetComponent<LineRenderer>();

                Vector3[] positions = new Vector3[2];
                positions[0] = ray.origin;
                positions[1] = hitData.point;

                lineRender.positionCount = positions.Length;
                lineRender.SetPositions(positions);

                coroutine = WaitAndRemoveLine(vanishingTime, lineRender);
                StartCoroutine(coroutine);

            }
        }
        
    }

    public void ToggleLines() { showLines = !showLines; }
}
