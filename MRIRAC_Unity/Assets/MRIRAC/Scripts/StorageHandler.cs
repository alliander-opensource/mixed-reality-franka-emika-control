using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Text;
using Microsoft.MixedReality.SampleQRCodes;

public class StorageHandler : MonoBehaviour
{
   
    [SerializeField]
    private GameObject QRCodesManager;

    public void SaveCallibrationPosition()
    {
        //Debug.Log("In Save Function");
        var qrcodelist = QRCodesManager.GetComponent<QRCodesVisualizer>().qrCodesObjectsList;

        foreach (var qrcode in qrcodelist)
            {
                if (qrcode.Value.GetComponent<QRCode>().CodeText == "Kinova")
                {
                    transform.parent = qrcode.Value.transform;
                }
            }

        #if WINDOWS_UWP
            //Debug.Log("In if");
           
            Windows.Storage.ApplicationDataContainer localSettings = Windows.Storage.ApplicationData.Current.LocalSettings;

            //Debug.Log("Position_x");
            //Debug.Log(transform.localPosition[0]);

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
        //Debug.Log("In Set Function");

        var qrcodelist = QRCodesManager.GetComponent<QRCodesVisualizer>().qrCodesObjectsList;

        foreach (var qrcode in qrcodelist)
            {
                if (qrcode.Value.GetComponent<QRCode>().CodeText == "Kinova")
                {
                    transform.parent = qrcode.Value.transform;
                }
            }

        #if WINDOWS_UWP
            //Debug.Log("In if");
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

            //Debug.Log("Position_x");
            //Debug.Log(position_x);
            //Debug.Log(position_x.ToString());
            //Debug.Log(position_x_float);

            Vector3 pos = new Vector3(position_x_float, position_y_float, position_z_float);
            Quaternion rot = new Quaternion(rotation_x_float, rotation_y_float, rotation_z_float, rotation_w_float);

            transform.localPosition = pos;
            transform.localRotation = rot;
        #endif
    }
}



