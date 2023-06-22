using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json;
using System.Text;
//using System.Threading.Tasks;

public class StorageHandler : MonoBehaviour
{
   
    //public void SaveCallibrationPosition()
    //{
        //string json_pos = JsonConvert.SerializeObject(transform.position, Formatting.Indented, new JsonSerializerSettings
        //{
        //    ReferenceLoopHandling = ReferenceLoopHandling.Ignore
        //});

        //string json_rot = JsonConvert.SerializeObject(transform.rotation, Formatting.Indented, new JsonSerializerSettings
        //{
        //    ReferenceLoopHandling = ReferenceLoopHandling.Ignore
        //});

        //Debug.Log(json_pos);
        //Debug.Log(json_rot);

        //PlayerPrefs.SetString("Position", json_pos);
        //PlayerPrefs.SetString("Rotation", json_rot);
        //PlayerPrefs.Save();
    //}

    //public void SetCallibrationPosition()
    //{
    //    string json_pos = PlayerPrefs.GetString("Position");
    //    Vector3 pos = JsonConvert.DeserializeObject<Vector3>(json_pos);

    //    string json_rot = PlayerPrefs.GetString("Rotation");
    //    Quaternion rot = JsonConvert.DeserializeObject<Quaternion>(json_rot);

    //    Debug.Log(pos);
    //    Debug.Log(rot);

    //    transform.position = pos;
    //    transform.rotation = rot;
    //}


    public void SaveCallibrationPosition()
    {
        Debug.Log("In Save Function");
        // Debug.Log(json_rot);

        // PlayerPrefs.SetString("Position", json_pos);
        // PlayerPrefs.SetString("Rotation", json_rot);
        // PlayerPrefs.Save();

        //Windows.Storage.StorageFolder storageFolder = Windows.Storage.KnownFolders.CameraRoll;
        //Windwos.Storage.StorageFile sampleFile = await storageFolder.CreateFileAsync("test.dat", Windows.Storage.CreationCollisionOption.ReplaceExisting);

        //#if ENABLE_WINMD_SUPPORT
#if WINDOWS_UWP
            //Windows.Storage.StorageFolder storageFolder = Windows.Storage.KnownFolders.CameraRoll;
            //Windows.Storage.StorageFile sampleFile = await storageFolder.CreateFileAsync("test.dat", Windows.Storage.CreationCollisionOption.ReplaceExisting);

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

            Windows.Storage.ApplicationDataContainer localSettings = Windows.Storage.ApplicationData.Current.LocalSettings;
            localSettings.Values["Position"] = json_pos;
            localSettings.Values["Rotation"] = json_rot;
#endif

    }

    public void SetCallibrationPosition()
    {
        Debug.Log("In Set Function");
        //#if ENABLE_WINMD_SUPPORT
#if WINDOWS_UWP
            Windows.Storage.ApplicationDataContainer localSettings = Windows.Storage.ApplicationData.Current.LocalSettings;
            string json_pos = localSettings.Values["Position"].ToString();
            string json_rot = localSettings.Values["Rotation"].ToString();

            Vector3 pos = JsonConvert.DeserializeObject<Vector3>(json_pos);
            Quaternion rot = JsonConvert.DeserializeObject<Quaternion>(json_rot);

            Debug.Log(pos);
            Debug.Log(rot);

            transform.position = pos;
            transform.rotation = rot;
#endif
    }
}



