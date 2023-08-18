using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class ToggleRobotArmMesh : MonoBehaviour
{
    private Renderer[] renders;
    private bool RenderActive;

    [SerializeField]
    private GameObject EndEffectorObject;

    // Start is called before the first frame update
    void Start()
    {
        RenderActive = true;
        renders = GetComponentsInChildren<Renderer>();
    }

    public void ToggleRobotMesh()
    {
        if (RenderActive)
        {
            foreach(Renderer render in renders)
            {
                render.enabled = false;
            }

            RenderActive = false;
        }

        else
        {
            foreach(Renderer render in renders)
            {
                render.enabled = true;
            }

            RenderActive = true;
        }
    }

    public void ActivateEndEffectorMesh()
    {
        Renderer[] EndEffectorRenders = EndEffectorObject.GetComponentsInChildren<Renderer>();

        foreach(Renderer render in EndEffectorRenders)
        {
            render.enabled = true;
        }
    }
}
