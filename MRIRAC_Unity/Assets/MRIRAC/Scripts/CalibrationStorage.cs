using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json;
using System.Text;

public class CalibrationStorage : MonoBehaviour
{
    public Vector3 position;
    public Quaternion rotation;

    [SerializeField]
    public GameObject robothandle;

    [SerializeField]
    public GameObject ObjectToSave;

    public void SaveCallibrationPosition()
    {
        position = robothandle.transform.position;
        rotation = robothandle.transform.rotation;

        string path = string.Format("{0}/mydata/stored_callibration_position.json", Application.persistentDataPath);

        string json = JsonConvert.SerializeObject(ObjectToSave);
        byte[] data = Encoding.ASCII.GetBytes(json);

        UnityEngine.Windows.File.WriteAllBytes(path, data);
    }

    public void ReadCallibrationPosition()
    {
        string path = string.Format("{0}/mydata/stored_callibration_position.json", Application.persistentDataPath);

        byte[] data = UnityEngine.Windows.File.ReadAllBytes(path);
        string json = Encoding.ASCII.GetString(data);

        GameObject obj = JsonConvert.DeserializeObject<GameObject>(json);

        position = obj.GetComponent<CalibrationStorage>().position;
        rotation = obj.GetComponent<CalibrationStorage>().rotation;
    }
}
