using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ExperimentDataMessage : MonoBehaviour
{
    [SerializeField]
    private GameObject EnvironmentsObject;

    [SerializeField]
    private CollisionTriggerEvent CollisionCounter;

    [SerializeField]
    private GameObject TimerUI;

    [SerializeField]
    private GameObject ExperimentUI;

    private string ControlMethod;

    public void ExperimentData()
    {
        // Environment number
        string EnvironmentName = EnvironmentsObject.transform.GetChild(0).gameObject.name[12].ToString();
        Debug.Log(EnvironmentName);

        // Environment condition

        // Environment control method
        FindControlMethod();
        Debug.Log(ControlMethod);

        // Number of collisions
        int AmountCollisions = CollisionCounter.counter;
        Debug.Log(AmountCollisions);

        // Time of human operation
        float time =  TimerUI.transform.GetChild(0).GetChild(1).gameObject.GetComponent<Timer>().currentTime;
        Debug.Log(time);
    }

    void FindControlMethod()
    {
        ControlMethod = "Error";

        if (ExperimentUI.transform.GetChild(1).gameObject.activeSelf)
        {
            ControlMethod = "Waypoint Control";
        }

        if (ExperimentUI.transform.GetChild(2).gameObject.activeSelf)
        {
            ControlMethod = "Direct Control";
        }

        if (ExperimentUI.transform.GetChild(3).gameObject.activeSelf)
        {
            ControlMethod = "Command Control";
        }
    }
}
