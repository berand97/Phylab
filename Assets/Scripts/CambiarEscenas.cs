using System;
using System.Collections;
using UnityEngine;
using UnityEngine.UI;

public class CambiarEscenas : MonoBehaviour
{

    void OnMouseDown()
    {
        this.gameObject.GetComponent<SpriteRenderer>().color = new Color(0.3f, 0.3f, 0.3f, 1);
    }
    void OnMouseOver()
    {
        this.gameObject.GetComponent<SpriteRenderer>().color = new Color(0.5f, 0.5f, 0.5f, 1);
    }
    void OnMouseExit()
    {
        this.gameObject.GetComponent<SpriteRenderer>().color = new Color(1, 1, 1, 1);
    }
    void OnMouseUp()
    {
        this.gameObject.GetComponent<SpriteRenderer>().color = new Color(1, 1, 1, 1);
        this.gameObject.GetComponent<BoxCollider2D>().enabled = false;
        StartCoroutine(CargoEScena());
    }

    IEnumerator CargoEScena()
    {
        yield return new WaitForSeconds(1);
        Application.LoadLevel("Main");
    }



}