using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.UI;

public class PositionCorrection : MonoBehaviour
{
    [SerializeField]
    private float moveDistance;
    [SerializeField]
    private float rotationDistance;
    [SerializeField]
    private float sliderMultiplier;

    [SerializeField]
    private GameObject controlPanel;

    [SerializeField]
    private GameObject sliderXAxis;
    [SerializeField]
    private GameObject sliderYAxis;
    [SerializeField]
    private GameObject sliderZAxis;

    [SerializeField]
    private GameObject CalibrationStorageObject;


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

    public void SliderMove(int direction) 
    {
        Vector3 newPosition = transform.position;
        switch (direction)
        {
            case 0: //X axis
                PinchSlider sliderX = sliderXAxis.GetComponent<PinchSlider>();
                newPosition += (sliderX.SliderValue-0.5f) * sliderMultiplier * transform.right;
                break;
            case 1: //Y axis
                PinchSlider sliderY = sliderYAxis.GetComponent<PinchSlider>();
                newPosition += (sliderY.SliderValue-0.5f) * sliderMultiplier * transform.up;
                break;
            case 2: //Z axis
                PinchSlider sliderZ = sliderZAxis.GetComponent<PinchSlider>();
                newPosition += (sliderZ.SliderValue-0.5f) * sliderMultiplier * transform.forward;
                break;
            default:
                Debug.Log("invalid slider move direction specified");
                break;
        }

        transform.position = newPosition;
    }

    public void SetSavedPosition()
    {
        CalibrationStorage CalibrationStorageScript = CalibrationStorageObject.GetComponent<CalibrationStorage>();

        CalibrationStorageScript.ReadCallibrationPosition();

        transform.position = CalibrationStorageScript.position;
        transform.rotation = CalibrationStorageScript.rotation;
    }

    public void SetControlPanel()
    {
        controlPanel.SetActive(true);

        controlPanel.transform.rotation = gameObject.transform.rotation * Quaternion.Euler(0f, -90f, 0f);
        controlPanel.transform.position = gameObject.transform.position + gameObject.transform.right * 0.2f; //0.1f;
    }
}
