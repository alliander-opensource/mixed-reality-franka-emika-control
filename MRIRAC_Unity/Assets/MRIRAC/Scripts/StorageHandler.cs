using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json;
using System.Text;
using System.Windows;

public class StorageHandler : MonoBehaviour
{
   
    public void SaveCallibrationPosition()
    {
        string json_pos = JsonConvert.SerializeObject(transform.position, Formatting.Indented, new JsonSerializerSettings
        {
            ReferenceLoopHandling = ReferenceLoopHandling.Ignore
        });

        string json_rot = JsonConvert.SerializeObject(transform.rotation, Formatting.Indented, new JsonSerializerSettings
        {
            ReferenceLoopHandling = ReferenceLoopHandling.Ignore
        });

        Debug.Log(json_pos);
        Debug.Log(json_rot);

        PlayerPrefs.SetString("Position", json_pos);
        PlayerPrefs.SetString("Rotation", json_rot);
        PlayerPrefs.Save();
    }

    public void SetCallibrationPosition()
    {
        string json_pos = PlayerPrefs.GetString("Position");
        Vector3 pos = JsonConvert.DeserializeObject<Vector3>(json_pos);

        string json_rot = PlayerPrefs.GetString("Rotation");
        Quaternion rot = JsonConvert.DeserializeObject<Quaternion>(json_rot);

        Debug.Log(pos);
        Debug.Log(rot);

        transform.position = pos;
        transform.rotation = rot;
    }

// #if WINDOWS_UWP
    // public async void SaveCallibrationPosition()
    // {
    //     // string json_pos = JsonConvert.SerializeObject(transform.position, Formatting.Indented, new JsonSerializerSettings
    //     // {
    //     //     ReferenceLoopHandling = ReferenceLoopHandling.Ignore
    //     // });

    //     // string json_rot = JsonConvert.SerializeObject(transform.rotation, Formatting.Indented, new JsonSerializerSettings
    //     // {
    //     //     ReferenceLoopHandling = ReferenceLoopHandling.Ignore
    //     // });

    //     // Debug.Log(json_pos);
    //     // Debug.Log(json_rot);

    //     // PlayerPrefs.SetString("Position", json_pos);
    //     // PlayerPrefs.SetString("Rotation", json_rot);
    //     // PlayerPrefs.Save();

    //     Windows.Storage.StorageFolder storageFolder = Windows.Storage.KnownFolders.CameraRoll;
    //     Windwos.Storage.StorageFile sampleFile = await storageFolder.CreateFileAsync("test.dat", Windows.Storage.CreationCollisionOption.ReplaceExisting);
    // }
// #endif
}



