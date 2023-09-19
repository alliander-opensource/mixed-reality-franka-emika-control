using UnityEngine;

public class SummonObject : MonoBehaviour
{

    public void Summon()
    {
        Vector3 inFrontOfCameraPos = (Camera.main.transform.forward * 0.5f) + Camera.main.transform.position;
        transform.position = inFrontOfCameraPos;
        transform.rotation = Quaternion.identity;
    }

    public void ExperimentTargetLocation()
    {
        transform.localRotation = new Quaternion(0.0f, -0.7f, -0.7f, 0.0f);
        transform.localPosition = new Vector3(0.0f, 0.4f, 0.8f);
    }

}
