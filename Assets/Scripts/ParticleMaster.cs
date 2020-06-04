using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Adjunte esto a cualquier objeto que sea un ParticleSystem o que tenga uno o más ParticleSystems
/// como objetos secundarios. Después de que todos los ParticleSystems (incluidos los hijos y este objeto) hayan terminado de emitirse
/// el objeto se autodestruirá, o alternativamente, si se está reproduciendo una fuente de audio, se autodestruirá después de ambos
/// y los sistemas de partículas han terminado de jugar
/// </summary>
public class ParticleMaster : MonoBehaviour {

    [SerializeField]
    bool waitForAudioSource = true;

    [SerializeField]
    bool emitOnAwake = true;

    private List<ParticleSystem> particles;

    void Awake()
    {
        particles = new List<ParticleSystem>();

        foreach (Transform child in transform)
        {
            if (child.GetComponent<ParticleSystem>() != null)
            {
                particles.Add(child.GetComponent<ParticleSystem>());
            }
        }

        if (gameObject.GetComponent<ParticleSystem>() != null)
        {
            particles.Add(gameObject.GetComponent<ParticleSystem>());
        }

        if (!emitOnAwake)
        {
            foreach (var particle in particles)
            {
                particle.enableEmission = false;
            }
        }
    }

    public void BeginEmission()
    {
        foreach (var particle in particles)
        {
            particle.enableEmission = true;
        }
    }

    public void Emit(bool emit)
    {
        foreach (var particle in particles)
        {
            particle.loop = emit;
        }
    }

    void Update()
    {
        if (waitForAudioSource && GetComponent<AudioSource>() && GetComponent<AudioSource>().isPlaying)
            return;

        foreach (var particle in particles)
        {
            if (particle.IsAlive())
            {
                return;
            }
        }

        Destroy(gameObject);
    }
}
