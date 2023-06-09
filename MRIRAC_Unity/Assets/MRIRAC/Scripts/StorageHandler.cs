using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json;
using System.Text;

public class StorageHandler : MonoBehaviour
{
    [SerializeField]
    public GameObject ObjectToSave;

    public void SaveCallibrationPosition()
    {
        ObjectToSave.GetComponent<CalibrationStorage>().position = transform.position;
        ObjectToSave.GetComponent<CalibrationStorage>().rotation = transform.rotation;

        string path = string.Format("{0}/mydata/stored_callibration_position.json", Application.persistentDataPath);

        string json = JsonConvert.SerializeObject(ObjectToSave.GetComponent<CalibrationStorage>(), Formatting.Indented, new JsonSerializerSettings
        {
            ReferenceLoopHandling = ReferenceLoopHandling.Ignore
        });

        Debug.Log(json);

        byte[] data = Encoding.ASCII.GetBytes(json);

        UnityEngine.Windows.File.WriteAllBytes(path, data);
    }

    public void SetCallibrationPosition()
    {
        string path = string.Format("{0}/mydata/stored_callibration_position.json", Application.persistentDataPath);

        byte[] data = UnityEngine.Windows.File.ReadAllBytes(path);
        string json = Encoding.ASCII.GetString(data);

        CalibrationStorage obj = JsonConvert.DeserializeObject<CalibrationStorage>(json);

        transform.position = obj.position;
        transform.rotation = obj.rotation;
    }
}
