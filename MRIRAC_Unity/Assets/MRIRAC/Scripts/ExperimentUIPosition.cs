using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ExperimentUIPosition : MonoBehaviour
{
    [SerializeField]
    private GameObject ExperimentUIObject;

    [SerializeField]
    private GameObject ExperimentWaypointControlUIObject;

    [SerializeField]
    private GameObject ExperimentDirectControlUIObject;

    [SerializeField]
    private GameObject ExperimentCommandControlUIObject;

    [SerializeField]
    private GameObject ExperimentDirectControlEnvironmentObject;

    [SerializeField]
    private GameObject ExperimentEnvironmentObject;

    // Update is called once per frame
    public void setUITransforms()
    {
        ExperimentUIObject.transform.SetParent(ExperimentEnvironmentObject.transform, true);
        ExperimentWaypointControlUIObject.transform.SetParent(ExperimentEnvironmentObject.transform, true);
        ExperimentDirectControlUIObject.transform.SetParent(ExperimentEnvironmentObject.transform, true);
        ExperimentCommandControlUIObject.transform.SetParent(ExperimentEnvironmentObject.transform, true);

        ExperimentUIObject.transform.localPosition = ExperimentDirectControlEnvironmentObject.transform.localPosition + new Vector3(-0.5f, 0.2f, 0f);
        ExperimentWaypointControlUIObject.transform.localPosition = ExperimentDirectControlEnvironmentObject.transform.localPosition +  new Vector3(-0.8f, 0.2f, 0f);
        ExperimentDirectControlUIObject.transform.localPosition = ExperimentDirectControlEnvironmentObject.transform.localPosition +  new Vector3(-0.8f, 0.2f, 0f);
        ExperimentCommandControlUIObject.transform.localPosition = ExperimentDirectControlEnvironmentObject.transform.localPosition +  new Vector3(-0.8f, 0.2f, 0f);

        ExperimentUIObject.transform.localRotation = new Quaternion(0f, 1f, 0f, 0f);
        ExperimentWaypointControlUIObject.transform.localRotation = new Quaternion(0f, 1f, 0f, 0f);
        ExperimentDirectControlUIObject.transform.localRotation = new Quaternion(0f, 1f, 0f, 0f);
        ExperimentCommandControlUIObject.transform.localRotation = new Quaternion(0f, 1f, 0f, 0f);
    }
}
