using UnityEngine;

public class SummonObject : MonoBehaviour
{

    public void Summon()
    {
        Vector3 inFrontOfCameraPos = (Camera.main.transform.forward * 0.5f) + Camera.main.transform.position;
        transform.position = inFrontOfCameraPos;
        transform.rotation = Quaternion.identity;
    }

    public void Rotate90Degrees()
    {
        transform.rotation = new Quaternion(0.0f, -0.7f, -0.7f, 0.0f);
    }

    // void Update()
    // {
    //     Debug.Log(transform.rotation);
    // }
}
