using System.Collections.Generic;
using UnityEngine;

public class PositionCorrection : MonoBehaviour
{
    [SerializeField]
    private float moveDistance;
    [SerializeField]
    private float rotationDistance;

    [SerializeField]
    private GameObject controlPanel;


    public void Move(int direction)
    {
        Vector3 newPosition = transform.position;
        switch (direction)
        {
            case 0:
                newPosition += moveDistance * transform.forward;
                break;
            case 1:
                newPosition -= moveDistance * transform.forward;
                break;
            case 2:
                newPosition += moveDistance * transform.right;
                break;
            case 3:
                newPosition -= moveDistance * transform.right;
                break;
            case 4:
                newPosition += moveDistance * transform.up;
                break;
            case 5:
                newPosition -= moveDistance * transform.up;
                break;

            default:
                Debug.Log("invalid move direction specified");
                break;
        }

        transform.position = newPosition;

    }

    public void Rotate(int direction)
    {
        Quaternion newRotation = transform.rotation;
        switch (direction)
        {
            case 0: //Around x axis positive
                newRotation = Quaternion.AngleAxis(rotationDistance, transform.right) * newRotation;
                break;
            case 1: //Around x axis negative
                newRotation = Quaternion.AngleAxis(-rotationDistance, transform.right) * newRotation;
                break;
            case 2: //Around y axis positive
                newRotation = Quaternion.AngleAxis(rotationDistance, transform.up) * newRotation;
                break;
            case 3: //Around y axis negative
                newRotation = Quaternion.AngleAxis(-rotationDistance, transform.up) * newRotation;
                break;
            case 4: //Around z axis positive
                newRotation = Quaternion.AngleAxis(rotationDistance, transform.forward) * newRotation;
                break;
            case 5: //Around z axis negative
                newRotation = Quaternion.AngleAxis(-rotationDistance, transform.forward) * newRotation;
                break;
            default:
                Debug.Log("invalid rotation direction specified");
                break;
        }

        transform.rotation = newRotation;
    }

    public void SetControlPanel()
    {
        controlPanel.SetActive(true);

        controlPanel.transform.rotation = gameObject.transform.rotation * Quaternion.Euler(0f, -90f, 0f);
        controlPanel.transform.position = gameObject.transform.position + gameObject.transform.right * 0.2f; //0.1f;
    }
}
