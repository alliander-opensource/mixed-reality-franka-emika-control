using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json;
using System.Text;
using Microsoft.MixedReality.SampleQRCodes;
//using System.Threading.Tasks;

public class StorageHandler : MonoBehaviour
{
   
    [SerializeField]
    private GameObject QRCodesManager;

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
        var qrcodelist = QRCodesManager.GetComponent<QRCodesVisualizer>().qrCodesObjectsList;

        foreach (var qrcode in qrcodelist)
            {
                if (qrcode.Value.GetComponent<QRCode>().CodeText == "Kinova")
                {
                    transform.parent = qrcode.Value.transform;
                }
            }
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
            Debug.Log("In if");
            // string json_pos = JsonConvert.SerializeObject(transform.position, Formatting.Indented, new JsonSerializerSettings
            // {
            //     ReferenceLoopHandling = ReferenceLoopHandling.Ignore
            // });

            // string json_rot = JsonConvert.SerializeObject(transform.rotation, Formatting.Indented, new JsonSerializerSettings
            // {
            //     ReferenceLoopHandling = ReferenceLoopHandling.Ignore
            // });

            // Debug.Log(json_pos);
            // Debug.Log(json_rot);

            Windows.Storage.ApplicationDataContainer localSettings = Windows.Storage.ApplicationData.Current.LocalSettings;

            Debug.Log("Position_x");
            Debug.Log(transform.localPosition[0]);

            localSettings.Values["Position_x"] = transform.localPosition[0];
            localSettings.Values["Position_y"] = transform.localPosition[1];
            localSettings.Values["Position_z"] = transform.localPosition[2];
            localSettings.Values["Rotation_x"] = transform.localRotation[0];
            localSettings.Values["Rotation_y"] = transform.localRotation[1];
            localSettings.Values["Rotation_z"] = transform.localRotation[2];
            localSettings.Values["Rotation_w"] = transform.localRotation[3];
#endif

    }

    public void SetCallibrationPosition()
    {
        Debug.Log("In Set Function");

        var qrcodelist = QRCodesManager.GetComponent<QRCodesVisualizer>().qrCodesObjectsList;

        foreach (var qrcode in qrcodelist)
            {
                if (qrcode.Value.GetComponent<QRCode>().CodeText == "Kinova")
                {
                    transform.parent = qrcode.Value.transform;
                }
            }

        //#if ENABLE_WINMD_SUPPORT
#if WINDOWS_UWP
            Debug.Log("In if");
            Windows.Storage.ApplicationDataContainer localSettings = Windows.Storage.ApplicationData.Current.LocalSettings;
            var position_x = localSettings.Values["Position_x"].ToString();
            var position_y = localSettings.Values["Position_y"].ToString();
            var position_z = localSettings.Values["Position_z"].ToString();
            var rotation_x = localSettings.Values["Rotation_x"].ToString();
            var rotation_y = localSettings.Values["Rotation_y"].ToString();
            var rotation_z = localSettings.Values["Rotation_z"].ToString();
            var rotation_w = localSettings.Values["Rotation_w"].ToString();

            float position_x_float = float.Parse(position_x);
            float position_y_float = float.Parse(position_y);
            float position_z_float = float.Parse(position_z);
            float rotation_x_float = float.Parse(rotation_x);
            float rotation_y_float = float.Parse(rotation_y);
            float rotation_z_float = float.Parse(rotation_z);
            float rotation_w_float = float.Parse(rotation_w);

            Debug.Log("Position_x");
            Debug.Log(position_x);
            Debug.Log(position_x.ToString());
            Debug.Log(position_x_float);

            //Object json_rot = localSettings.Values["Rotation"].ToString();

            //Vector3 pos = JsonConvert.DeserializeObject<Vector3>(json_pos);
            //Quaternion rot = JsonConvert.DeserializeObject<Quaternion>(json_rot);

            //Debug.Log(pos);
            //Debug.Log(rot);

            Vector3 pos = new Vector3(position_x_float, position_y_float, position_z_float);
            Quaternion rot = new Quaternion(rotation_x_float, rotation_y_float, rotation_z_float, rotation_w_float);

            transform.localPosition = pos;
            transform.localRotation = rot;
#endif
    }
}



