using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Environments : MonoBehaviour
{

    [SerializeField]
    private GameObject StartAndGoalEnvironmentObjects;

    [SerializeField]
    private GameObject StartPositionPrefab;

    [SerializeField]
    private Vector3 StartPositionCoordinates;

     [SerializeField]
    private GameObject GoalPositionPrefab;

    [SerializeField]
    private Vector3 GoalPositionCoordinates;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void CreateStartAndGoalPosition()
    {
        GameObject StartPosition = Instantiate(StartPositionPrefab, StartAndGoalEnvironmentObjects.transform, false);
        StartPosition.name = "StartPosition";
        // StartPosition.transform.SetParent(StartAndGoalEnvironmentObjects.transform, false);

        StartPosition.transform.localPosition = StartPositionCoordinates;

        GameObject GoalPosition = Instantiate(GoalPositionPrefab, StartAndGoalEnvironmentObjects.transform, false);
        GoalPosition.name = "GoalPosition";
        Debug.Log(GoalPosition.transform.position);
        // GoalPosition.transform.SetParent(StartAndGoalEnvironmentObjects.transform, false);

        GoalPosition.transform.localPosition = GoalPositionCoordinates;
        Debug.Log(GoalPosition.transform.position);
    }

    public void DestroyStartAndGoalPosition()
    {
        foreach (Transform objectTransform in StartAndGoalEnvironmentObjects.transform)
        {
            Destroy(objectTransform.gameObject);
        }
    }
}
