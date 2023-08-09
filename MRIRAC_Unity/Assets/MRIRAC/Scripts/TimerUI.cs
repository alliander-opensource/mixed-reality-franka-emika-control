using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class TimerUI : MonoBehaviour
{
    [SerializeField]
    private GameObject TimerPrefab;

    [SerializeField]
    private GameObject TimerUIObject;

    private bool TimerActive;

    // Start is called before the first frame update
    void Start()
    {
        TimerActive = false;
    }

    // Update is called once per frame
    void Update()
    {
        if (TimerActive)
        {
            Vector3 inFrontOfCameraPos = new Vector3(-1.5f, 1.0f, 2.0f) + Camera.main.transform.position; // (Camera.main.transform.forward * 2.0f) + (Camera.main.transform.up * 1.0f) + (Camera.main.transform.right * -1.5f) 
            transform.position = inFrontOfCameraPos;
            transform.rotation = Quaternion.identity;
        }
    }

    public void ActivateTimer()
    {
        foreach (Transform timerTransform in TimerUIObject.transform)
        {
            Destroy(timerTransform.gameObject);
        }

        GameObject timer = Instantiate(TimerPrefab, TimerUIObject.transform, true);
        timer.name = "Timer";
        TimerActive = true;
    }

    public void DeactivateTimer()
    {
        foreach (Transform timerTransform in TimerUIObject.transform)
        {
            Destroy(timerTransform.gameObject);
        }

        TimerActive = false;
    }
}
