using System;
using UnityEngine;

// We are going to use the value of the Z if the camera to pass from 2D to 3D

[RequireComponent(typeof(Collider))] // attribute to ensure that the GameObject this script is attached to has a Collider component. This is because the script relies on mouse events, which require a Collider for interaction

public class MouseFollower : MonoBehaviour // In Unity, MonoBehaviour is a base class that provides a framework for creating scripts that can be attached to GameObjects.
{
    // Definition of variables:
    private Camera main_camera;    // variable to store a reference to the main camera
    private float object_distance_Z;// store the z-axis distance of the GameObject from the camera in screen coordinates

    void Start()  //  called on the frame when the script is enabled
    {
        InitializeVariables();
    }
    void OnMouseDrag() // called when the user drags the mouse over the collider of the GameObject.
    {
        MoveObjectWithMouse();
    }

    private void InitializeVariables()
    {
        main_camera = Camera.main;  // make a copy to do not overwrite it
        object_distance_Z = CalculateObjectDistanceZ();
    }
    private float CalculateObjectDistanceZ()
    {
        return main_camera.WorldToScreenPoint(transform.position).z; //z-axis distance of the GameObject from the camera in screen coordinates
    }

    private void MoveObjectWithMouse()
    {
        Vector3 ScreenPosition = new Vector3(Input.mousePosition.x, Input.mousePosition.y, object_distance_Z); //z axis added to screen point 
        Vector3 NewWorldPosition = main_camera.ScreenToWorldPoint(ScreenPosition); //Converte the screen position(2D) to a world position(3D)
        transform.position = NewWorldPosition; //Setting the position of the GameObject to the calculated world position, to follow the mouse
    }
}