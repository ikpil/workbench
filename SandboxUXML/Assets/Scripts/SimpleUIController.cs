using System;
using UnityEngine;
using UnityEngine.UIElements;

public class SimpleUIController : MonoBehaviour
{
    public UIDocument document;

    private void Awake()
    {
        document = GetComponent<UIDocument>();
    }

    void Start()
    {
        var root = document.rootVisualElement;

        // var button = root.Q<Button>("start-button");

        // button.clicked += () => { Debug.Log("Start Clicked"); };
    }
}