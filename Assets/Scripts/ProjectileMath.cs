using UnityEngine;

public static class ProjectileMath
{
    /// <summary>
    /// Calcula los dos ángulos iniciales posibles que podrían usarse para disparar un proyectil al
    /// velocidad para recorrer la distancia deseada
    /// </summary>
    /// <param name="speed">Velocidad inicial del proyectil.</param>
    /// <param name="distance">Distancia a lo largo del eje horizontal que viajará el proyectil</param>
    /// <param name="yOffset">    Elevación del objetivo con respecto a la posición de disparo inicial</param>
    /// <param name="gravity">    Aceleración descendente en m / s ^ 2</param>
    /// <param name="angle0"></param>
    /// <param name="angle1"></param>
    /// <returns>Falso si el objetivo está fuera de rango</returns>
    public static bool LaunchAngle(float speed, float distance, float yOffset, float gravity, out float angle0, out float angle1)
    {
        angle0 = angle1 = 0;

        float speedSquared = speed * speed;

        float operandA = Mathf.Pow(speed, 4);
        float operandB = gravity * (gravity * (distance * distance) + (2 * yOffset * speedSquared));

        //         El objetivo no está dentro del rango
        if (operandB > operandA)
            return false;

        float root = Mathf.Sqrt(operandA - operandB);

        angle0 = Mathf.Atan((speedSquared + root) / (gravity * distance));
        angle1 = Mathf.Atan((speedSquared - root) / (gravity * distance));

        return true;
    }

    /// <summary>
    ///     Calcula la velocidad de lanzamiento inicial requerida para alcanzar un objetivo a distancia con la elevación y Offset.
    /// </summary>
    /// <param name="distance">Distancia plana desde el origen hasta el objetivo</param>
    /// <param name="yOffset">Elevación del origen con respecto al objetivo.</param>
    /// <param name="gravity">    Aceleración descendente en m / s ^ 2</param>
    /// <param name="angle">Ángulo de lanzamiento inicial en radianes</param>
    /// <returns>Velocidad de lanzamiento inicial</returns>
    public static float LaunchSpeed(float distance, float yOffset, float gravity, float angle)
    {
        float speed = (distance * Mathf.Sqrt(gravity) * Mathf.Sqrt(1 / Mathf.Cos(angle))) / Mathf.Sqrt(2 * distance * Mathf.Sin(angle) + 2 * yOffset * Mathf.Cos(angle));

        return speed;
    }

    /// <summary>
    /// Calcula cuánto tiempo permanecerá un proyectil en el aire antes de alcanzar su objetivo

    /// </summary>
    /// <param name="speed">Velocidad inicial del proyectil.</param>
    /// <param name="angle">Ángulo de lanzamiento inicial en radianes</param>
    /// <param name="yOffset">    Elevación del objetivo con respecto a la posición de disparo inicial</param>
    /// <param name="gravity">Aceleración descendente en m / s ^ 2</param>
    /// <returns></returns>
    public static float TimeOfFlight(float speed, float angle, float yOffset, float gravity)
    {
        float ySpeed = speed * Mathf.Sin(angle);

        float time = (ySpeed + Mathf.Sqrt((ySpeed * ySpeed) + 2 * gravity * yOffset)) / gravity;

        return time;
    }

    /// <summary>
    ///     Muestra una serie de puntos a lo largo de un arco de proyectil
    /// </summary>
    /// <param name="iterations">Número de puntos para muestrear</param>
    /// <param name="speed">Initial speed of the projectile</param>
    /// <param name="distance">Distancia que recorrerá el proyectil a lo largo del eje horizontal</param>
    /// <param name="gravity">Aceleración descendente en m / s ^ 2</param>
    /// <param name="angle">    Ángulo de lanzamiento inicial en radianes</param>
    /// <returns>Matriz de puntos muestreados con la longitud de las iteraciones proporcionadas</returns>
    public static Vector2[] ProjectileArcPoints(int iterations, float speed, float distance, float gravity, float angle)
    {
        float iterationSize = distance / iterations;

        float radians = angle;

        Vector2[] points = new Vector2[iterations + 1];

        for (int i = 0; i <= iterations; i++)
        {
            float x = iterationSize * i;
            float t = x / (speed * Mathf.Cos(radians));
            float y = -0.5f * gravity * (t * t) + speed * Mathf.Sin(radians) * t;

            Vector2 p = new Vector2(x, y);

            points[i] = p;
        }

        return points;
    }
}
