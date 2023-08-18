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
    public GameObject timer;

    // Start is called before the first frame update
    void Start()
    {
        TimerActive = false;
        TimerUIObject.transform.SetParent(Camera.main.transform, false);
    }

    // Update is called once per frame
    void Update()
    {
        if (TimerActive)
        {
            Vector3 inFrontOfCameraPos = new Vector3(-0.1f, 0.01f, 0.6f); // + Camera.main.transform.position; // (Camera.main.transform.forward * 2.0f) + (Camera.main.transform.up * 1.0f) + (Camera.main.transform.right * -1.5f) 
            transform.localPosition = inFrontOfCameraPos;
            transform.localRotation = Quaternion.identity; //Camera.main.transform.rotation;
        }
    }

    public void ActivateTimer()
    {
        foreach (Transform timerTransform in TimerUIObject.transform)
        {
            Destroy(timerTransform.gameObject);
        }

        timer = Instantiate(TimerPrefab, TimerUIObject.transform, false);
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

    public void PauzeTimer()
    {
        Timer timer_script = timer.transform.GetChild(1).gameObject.GetComponent<Timer>();
        timer_script.enabled = false;
    }

    public void ResumeTimer()
    {
        Timer timer_script = timer.transform.GetChild(1).gameObject.GetComponent<Timer>();
        timer_script.enabled = true;
    }
}
