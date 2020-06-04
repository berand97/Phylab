using UnityEngine;
using System.Collections;
using System;

public class Math3d : MonoBehaviour
{

    private static Transform tempChild = null;
    private static Transform tempParent = null;

    public static void Init()
    {
        tempChild = (new GameObject("Math3d_TempChild")).transform;
        tempParent = (new GameObject("Math3d_TempParent")).transform;

        tempChild.gameObject.hideFlags = HideFlags.HideAndDontSave;
        DontDestroyOnLoad(tempChild.gameObject);

        tempParent.gameObject.hideFlags = HideFlags.HideAndDontSave;
        DontDestroyOnLoad(tempParent.gameObject);

        //set the parent
        tempChild.parent = tempParent;
    }


    //aumentar o disminuir la longitud del vector por tamaño
    public static Vector3 AddVectorLength(Vector3 vector, float size)
    {

        //obtener la longitud del vector
        float magnitude = Vector3.Magnitude(vector);

        // cambiar la longitud
                magnitude += size;

        //Normalizar el  vector
        Vector3 vectorNormalized = Vector3.Normalize(vector);

        //Escalar el vector
        return Vector3.Scale(vectorNormalized, new Vector3(magnitude, magnitude, magnitude));
    }

    //crear un vector de dirección "vector" con longitud "tamaño"
    public static Vector3 SetVectorLength(Vector3 vector, float size)
    {

        //normalizar el vector
        Vector3 vectorNormalized = Vector3.Normalize(vector);

        //escalar el vector
        return vectorNormalized *= size;
    }


    //calcular la diferencia rotacional de A a B
    public static Quaternion SubtractRotation(Quaternion B, Quaternion A)
    {

        Quaternion C = Quaternion.Inverse(A) * B;
        return C;
    }

    // Encuentra la línea de intersección entre dos planos. Los planos están definidos por una normal y un punto en ese plano.
    // Las salidas son un punto en la línea y un vector que indica su dirección. Si los planos no son paralelos,
    // la función genera verdadero, de lo contrario falso.
    public static bool PlanePlaneIntersection(out Vector3 linePoint, out Vector3 lineVec, Vector3 plane1Normal, Vector3 plane1Position, Vector3 plane2Normal, Vector3 plane2Position)
    {

        linePoint = Vector3.zero;
        lineVec = Vector3.zero;

        // Podemos obtener la dirección de la línea de intersección de los dos planos calculando el
        // producto cruzado de las normales de los dos planos. Tenga en cuenta que esto es solo una dirección y la línea
        // aún no está arreglado en el espacio. Necesitamos un punto para que vaya con el vector de línea.
        lineVec = Vector3.Cross(plane1Normal, plane2Normal);

        // Lo siguiente es calcular un punto en la línea para fijar su posición en el espacio. Esto se hace buscando un vector de
        // la ubicación del plano 2, moviéndose paralela a su plano e intersectando el plano 1. Para evitar el redondeo
        // errores, este vector también tiene que ser perpendicular a lineDirection. Para obtener este vector, calcule
        // el producto cruzado de la normalidad de plane2 y lineDirection.	
        Vector3 ldir = Vector3.Cross(plane2Normal, lineVec);

        float denominator = Vector3.Dot(plane1Normal, ldir);

        // Evita dividir entre cero y errores de redondeo al requerir un ángulo de aproximadamente 5 grados entre los planos.
        if (Mathf.Abs(denominator) > 0.006f)
        {

            Vector3 plane1ToPlane2 = plane1Position - plane2Position;
            float t = Vector3.Dot(plane1Normal, plane1ToPlane2) / denominator;
            linePoint = plane2Position + t * ldir;

            return true;
        }

        // salida no válida
        else
        {
            return false;
        }
    }

    // Obtener la intersección entre una línea y un plano.
    // Si la línea y el plano no son paralelos, la función genera verdadero, de lo contrario falso.
    public static bool LinePlaneIntersection(out Vector3 intersection, Vector3 linePoint, Vector3 lineVec, Vector3 planeNormal, Vector3 planePoint)
    {

        float length;
        float dotNumerator;
        float dotDenominator;
        Vector3 vector;
        intersection = Vector3.zero;

        // calcula la distancia entre el punto de línea y el punto de intersección del plano de línea
        dotNumerator = Vector3.Dot((planePoint - linePoint), planeNormal);
        dotDenominator = Vector3.Dot(lineVec, planeNormal);

        // línea y plano no son paralelos
        if (dotDenominator != 0.0f)
        {
            length = dotNumerator / dotDenominator;

            // crea un vector desde el LinePoint hasta el punto de intersección
            vector = SetVectorLength(lineVec, length);

            //get the coordinates of the line-plane intersection point
            intersection = linePoint + vector;

            return true;
        }

        // salida no válida
        else
        {
            return false;
        }
    }

    // Calcular el punto de intersección de dos líneas. Devuelve verdadero si las líneas se cruzan, de lo contrario falso.
    // Tenga en cuenta que en 3d, dos líneas no se cruzan la mayor parte del tiempo. Entonces, si las dos líneas no están en el
    // mismo plano, use ClosestPointsOnTwoLines () en su lugar.
    public static bool LineLineIntersection(out Vector3 intersection, Vector3 linePoint1, Vector3 lineVec1, Vector3 linePoint2, Vector3 lineVec2)
    {

        intersection = Vector3.zero;

        Vector3 lineVec3 = linePoint2 - linePoint1;
        Vector3 crossVec1and2 = Vector3.Cross(lineVec1, lineVec2);
        Vector3 crossVec3and2 = Vector3.Cross(lineVec3, lineVec2);

        float planarFactor = Vector3.Dot(lineVec3, crossVec1and2);

        // Las líneas no son coplanares. Tener en cuenta los errores de redondeo.
        if ((planarFactor >= 0.00001f) || (planarFactor <= -0.00001f))
        {

            return false;
        }

        // Nota: sqrMagnitude hace x * x + y * y + z * z en el vector de entrada.
        float s = Vector3.Dot(crossVec3and2, crossVec1and2) / crossVec1and2.sqrMagnitude;

        if ((s >= 0.0f) && (s <= 1.0f))
        {

            intersection = linePoint1 + (lineVec1 * s);
            return true;
        }

        else
        {
            return false;
        }
    }

    // Dos líneas no paralelas que pueden o no tocarse entre sí tienen un punto en cada línea más cercano
    // el uno al otro. Esta función encuentra esos dos puntos. Si las líneas no son paralelas, la función
    // produce verdadero, de lo contrario falso.
    public static bool ClosestPointsOnTwoLines(out Vector3 closestPointLine1, out Vector3 closestPointLine2, Vector3 linePoint1, Vector3 lineVec1, Vector3 linePoint2, Vector3 lineVec2)
    {

        closestPointLine1 = Vector3.zero;
        closestPointLine2 = Vector3.zero;

        float a = Vector3.Dot(lineVec1, lineVec1);
        float b = Vector3.Dot(lineVec1, lineVec2);
        float e = Vector3.Dot(lineVec2, lineVec2);

        float d = a * e - b * b;

        // las líneas no son paralelas
        if (d != 0.0f)
        {

            Vector3 r = linePoint1 - linePoint2;
            float c = Vector3.Dot(lineVec1, r);
            float f = Vector3.Dot(lineVec2, r);

            float s = (b * f - c * e) / d;
            float t = (a * f - c * b) / d;

            closestPointLine1 = linePoint1 + lineVec1 * s;
            closestPointLine2 = linePoint2 + lineVec2 * t;

            return true;
        }

        else
        {
            return false;
        }
    }

    // Esta función devuelve un punto que es una proyección de un punto a una línea.
    // La línea se considera infinita. Si la línea es finita, use ProjectPointOnLineSegment () en su lugar.
    public static Vector3 ProjectPointOnLine(Vector3 linePoint, Vector3 lineVec, Vector3 point)
    {

        // obtener un vector de punto en línea a punto en el espacio
        Vector3 linePointToPoint = point - linePoint;

        float t = Vector3.Dot(linePointToPoint, lineVec);

        return linePoint + lineVec * t;
    }

    // Esta función devuelve un punto que es una proyección de un punto a un segmento de línea.
    // Si el punto proyectado se encuentra fuera del segmento de línea, el punto proyectado
    // se sujetará al borde de línea apropiado.
    // Si la línea es infinita en lugar de un segmento, use ProjectPointOnLine () en su lugar.
    public static Vector3 ProjectPointOnLineSegment(Vector3 linePoint1, Vector3 linePoint2, Vector3 point)
    {

        Vector3 vector = linePoint2 - linePoint1;

        Vector3 projectedPoint = ProjectPointOnLine(linePoint1, vector.normalized, point);

        int side = PointOnWhichSideOfLineSegment(linePoint1, linePoint2, projectedPoint);

        // El punto proyectado está en el segmento de línea.
        if (side == 0)
        {

            return projectedPoint;
        }

        if (side == 1)
        {

            return linePoint1;
        }

        if (side == 2)
        {

            return linePoint2;
        }

        // la salida no es válida
        return Vector3.zero;
    }

    // Esta función devuelve un punto que es una proyección de un punto a un plano.
    public static Vector3 ProjectPointOnPlane(Vector3 planeNormal, Vector3 planePoint, Vector3 point)
    {

        float distance;
        Vector3 translationVector;

        // Primero calcule la distancia desde el punto al plano:
        distance = SignedDistancePlanePoint(planeNormal, planePoint, point);

        // Invierte el signo de la distancia
        distance *= -1;

        // Obtenga un vector de traducción
        translationVector = SetVectorLength(planeNormal, distance);

        // Traducir el punto para formar una proyección
        return point + translationVector;
    }

    // Proyecta un vector en un plano. La salida no está normalizada.
    public static Vector3 ProjectVectorOnPlane(Vector3 planeNormal, Vector3 vector)
    {

        return vector - (Vector3.Dot(vector, planeNormal) * planeNormal);
    }

    // Obtenga la distancia más corta entre un punto y un plano. La salida se firma para que contenga información
    // en cuanto a qué lado del plano normal es el punto.
    public static float SignedDistancePlanePoint(Vector3 planeNormal, Vector3 planePoint, Vector3 point)
    {

        return Vector3.Dot(planeNormal, (point - planePoint));
    }

    // Esta función calcula un producto de puntos con signo (+ o - en lugar de ser ambiguo). Básicamente se usa
    // para determinar si un vector está posicionado a la izquierda o derecha de otro vector. La forma en que se hace esto es
    // calculando un vector perpendicular a uno de los vectores y utilizándolo como referencia. Esto es porque
    // el resultado de un producto de puntos solo tiene información firmada cuando un ángulo está en transición entre más o menos
    // luego 90 grados.
    public static float SignedDotProduct(Vector3 vectorA, Vector3 vectorB, Vector3 normal)
    {

        Vector3 perpVector;
        float dot;

        // Usa el objeto de geometría normal y uno de los vectores de entrada para calcular el vector perpendicular
        perpVector = Vector3.Cross(normal, vectorA);

        // Ahora calcule el producto punto entre el vector perpendicular (perpVector) y el otro vector de entrada
        dot = Vector3.Dot(perpVector, vectorB);

        return dot;
    }

    public static float SignedVectorAngle(Vector3 referenceVector, Vector3 otherVector, Vector3 normal)
    {
        Vector3 perpVector;
        float angle;

        // Usa el objeto de geometría normal y uno de los vectores de entrada para calcular el vector perpendicular
        perpVector = Vector3.Cross(normal, referenceVector);

        // Ahora calcule el producto punto entre el vector perpendicular (perpVector) y el otro vector de entrada
        angle = Vector3.Angle(referenceVector, otherVector);
        angle *= Mathf.Sign(Vector3.Dot(perpVector, otherVector));

        return angle;
    }

    // Calcula el ángulo entre un vector y un plano. El plano está hecho por un vector normal.
    // La salida está en radianes.
    public static float AngleVectorPlane(Vector3 vector, Vector3 normal)
    {

        float dot;
        float angle;

        // calcula el producto punto entre los dos vectores de entrada. Esto da el coseno entre los dos vectores.
        dot = Vector3.Dot(vector, normal);

        // esto está en radianes
        angle = (float)Math.Acos(dot);

        return 1.570796326794897f - angle; // 90 grados - ángulo
    }

    // Calcula el producto escalar como un ángulo
    public static float DotProductAngle(Vector3 vec1, Vector3 vec2)
    {

        double dot;
        double angle;

        // obtener el producto punto
        dot = Vector3.Dot(vec1, vec2);

        // Abrazadera para evitar errores de NaN. No debería necesitar esto en primer lugar, pero podría haber un problema de error de redondeo.
        if (dot < -1.0f)
        {
            dot = -1.0f;
        }
        if (dot > 1.0f)
        {
            dot = 1.0f;
        }

        // Calcula el ángulo. La salida está en radianes.
        // Este paso se puede omitir para la optimización ...
        angle = Math.Acos(dot);

        return (float)angle;
    }

    // Convierte un plano definido por 3 puntos en un plano definido por un vector y un punto.
    // El punto plano es el centro del triángulo definido por los 3 puntos.
    public static void PlaneFrom3Points(out Vector3 planeNormal, out Vector3 planePoint, Vector3 pointA, Vector3 pointB, Vector3 pointC)
    {

        planeNormal = Vector3.zero;
        planePoint = Vector3.zero;

        // Hacer dos vectores a partir de los 3 puntos de entrada, originados desde el punto A
        Vector3 AB = pointB - pointA;
        Vector3 AC = pointC - pointA;

        // Calcular la normal
        planeNormal = Vector3.Normalize(Vector3.Cross(AB, AC));

        // Obtenga los puntos en el medio AB y AC
        Vector3 middleAB = pointA + (AB / 2.0f);
        Vector3 middleAC = pointA + (AC / 2.0f);

        // Obtener vectores desde el medio de AB y AC hasta el punto que no está en esa línea.
        Vector3 middleABtoC = pointC - middleAB;
        Vector3 middleACtoB = pointB - middleAC;

        // Calcula la intersección entre las dos líneas. Este será el centro
        // del triángulo definido por los 3 puntos.
        // Podríamos usar LineLineIntersection en lugar de ClosestPointsOnTwoLines pero debido a errores de redondeo
        // esto a veces no funciona.
        Vector3 temp;
        ClosestPointsOnTwoLines(out planePoint, out temp, middleAB, middleABtoC, middleAC, middleACtoB);
    }

    // Devuelve el vector directo de un cuaternión
    public static Vector3 GetForwardVector(Quaternion q)
    {

        return q * Vector3.forward;
    }

    // Devuelve el vector directo de un cuaternión
    public static Vector3 GetUpVector(Quaternion q)
    {

        return q * Vector3.up;
    }

    // Devuelve el vector correcto de un cuaternión
    public static Vector3 GetRightVector(Quaternion q)
    {

        return q * Vector3.right;
    }

    // Obtiene un cuaternión de una matriz
    public static Quaternion QuaternionFromMatrix(Matrix4x4 m)
    {

        return Quaternion.LookRotation(m.GetColumn(2), m.GetColumn(1));
    }

    // Obtiene una posición de una matriz
    public static Vector3 PositionFromMatrix(Matrix4x4 m)
    {

        Vector4 vector4Position = m.GetColumn(3);
        return new Vector3(vector4Position.x, vector4Position.y, vector4Position.z);
    }

    // Esta es una alternativa para Quaternion.LookRotation. En lugar de alinear el vector hacia adelante y hacia arriba del juego
    // objeto con los vectores de entrada, se puede usar una dirección personalizada en lugar de los vectores fijos hacia adelante y hacia arriba.
    // alignWithVector y alignWithNormal están en el espacio mundial.
    // customForward y customUp están en el espacio de objetos.
    // Uso: use alignWithVector y alignWithNormal como si estuviera usando la función LookRotation predeterminada.
    // Establezca customForward y customUp en los vectores que desea utilizar en lugar de los vectores predeterminados hacia adelante y hacia arriba.
    public static void LookRotationExtended(ref GameObject gameObjectInOut, Vector3 alignWithVector, Vector3 alignWithNormal, Vector3 customForward, Vector3 customUp)
    {

        // Establecer la rotación del destino
        Quaternion rotationA = Quaternion.LookRotation(alignWithVector, alignWithNormal);

        // Establece la rotación de los vectores normal y superior personalizados.
        // Cuando se usa la función LookRotation predeterminada, esto estaría codificado en el vector hacia adelante y hacia arriba.
        Quaternion rotationB = Quaternion.LookRotation(customForward, customUp);

        //Calculate the rotation
        gameObjectInOut.transform.rotation = rotationA * Quaternion.Inverse(rotationB);
    }

    // Esta función transforma un objeto como si estuviera parental al otro.
    // Antes de usar esta función, la función Init () debe llamarse
    // Entrada: parentRotation y parentPosition: la transformación principal actual.
    // Entrada: startParentRotation y startParentPosition: la transformación del objeto primario en el momento en que los objetos son parentales.
    // Entrada: startChildRotation y startChildPosition: la transformación del objeto secundario en el momento en que los objetos son parentales.
    // Salida: childRotation y childPosition.
    // Todas las transformaciones están en el espacio mundial.
    public static void TransformWithParent(out Quaternion childRotation, out Vector3 childPosition, Quaternion parentRotation, Vector3 parentPosition, Quaternion startParentRotation, Vector3 startParentPosition, Quaternion startChildRotation, Vector3 startChildPosition)
    {

        childRotation = Quaternion.identity;
        childPosition = Vector3.zero;

        // establece la transformación de inicio principal
        tempParent.rotation = startParentRotation;
        tempParent.position = startParentPosition;
        tempParent.localScale = Vector3.one; //to prevent scale wandering

        // establece la transformación de inicio secundario
        tempChild.rotation = startChildRotation;
        tempChild.position = startChildPosition;
        tempChild.localScale = Vector3.one; // para evitar el desplazamiento de la escala

        // traduce y rota al niño moviendo al padre
        tempParent.rotation = parentRotation;
        tempParent.position = parentPosition;

        // consigue que el niño se transforme
        childRotation = tempChild.rotation;
        childPosition = tempChild.position;
    }

    // Con esta función puedes alinear un triángulo de un objeto con cualquier transformación.
    // Uso: gameObjectInOut es el objeto del juego que quieres transformar.
    // alignWithVector, alignWithNormal y alignWithPosition es la transformación con la que se debe alinear el triángulo del objeto.
    // triangleForward, triangleNormal, y trianglePosition es la transformación del triángulo del objeto.
    // alignWithVector, alignWithNormal y alignWithPosition están en el espacio mundial.
    // triangleForward, triangleNormal, y trianglePosition están en el espacio de objetos.
    // trianglePosition es la posición de malla del triángulo. El efecto de la escala del objeto se maneja automáticamente.
    // trianglePosition se puede establecer en cualquier posición, no tiene que estar en un vértice o en el medio del triángulo.
    public static void PreciseAlign(ref GameObject gameObjectInOut, Vector3 alignWithVector, Vector3 alignWithNormal, Vector3 alignWithPosition, Vector3 triangleForward, Vector3 triangleNormal, Vector3 trianglePosition)
    {

        // Establecer la rotación.
        LookRotationExtended(ref gameObjectInOut, alignWithVector, alignWithNormal, triangleForward, triangleNormal);

        // Obtener la posición espacial mundial de trianglePosition
        Vector3 trianglePositionWorld = gameObjectInOut.transform.TransformPoint(trianglePosition);

        //Get a vector from trianglePosition to alignWithPosition
        Vector3 translateVector = alignWithPosition - trianglePositionWorld;

        // Ahora transforma el objeto para que el triángulo se alinee correctamente.
        gameObjectInOut.transform.Translate(translateVector, Space.World);
    }


    // Convierte una posición, dirección y vector normal en una transformación
    void VectorsToTransform(ref GameObject gameObjectInOut, Vector3 positionVector, Vector3 directionVector, Vector3 normalVector)
    {

        gameObjectInOut.transform.position = positionVector;
        gameObjectInOut.transform.rotation = Quaternion.LookRotation(directionVector, normalVector);
    }

    // Esta función descubre en qué lado de un segmento de línea se encuentra el punto.
    // Se supone que el punto está en una línea creada por linePoint1 y linePoint2. Si el punto no está encendido
    // el segmento de línea, proyecte primero en la línea usando ProjectPointOnLine ().
    // Devuelve 0 si el punto está en el segmento de línea.
    // Devuelve 1 si el punto está fuera del segmento de línea y se encuentra al lado de linePoint1.
    // Devuelve 2 si el punto está fuera del segmento de línea y se encuentra al lado de linePoint2.
    public static int PointOnWhichSideOfLineSegment(Vector3 linePoint1, Vector3 linePoint2, Vector3 point)
    {

        Vector3 lineVec = linePoint2 - linePoint1;
        Vector3 pointVec = point - linePoint1;

        float dot = Vector3.Dot(pointVec, lineVec);

        // el punto está del lado de linePoint2, en comparación con linePoint1
        if (dot > 0)
        {

            // el punto está en el segmento de línea
            if (pointVec.magnitude <= lineVec.magnitude)
            {

                return 0;
            }

            // el punto no está en el segmento de línea y está en el lado de linePoint2
            else
            {

                return 2;
            }
        }

        // El punto no está del lado de linePoint2, en comparación con linePoint1.
        // El punto no está en el segmento de línea y está en el lado de linePoint1.
        else
        {

            return 1;
        }
    }


    // Devuelve la distancia de píxeles desde el puntero del mouse a una línea.
    // Alternativa para HandleUtility.DistanceToLine (). Funciona tanto en modo Editor como en modo Play.
    // No llame a esta función desde OnGUI () ya que la posición del mouse será incorrecta.
    public static float MouseDistanceToLine(Vector3 linePoint1, Vector3 linePoint2)
    {

        Camera currentCamera;
        Vector3 mousePosition;

#if UNITY_EDITOR
        if (Camera.current != null)
        {

            currentCamera = Camera.current;
        }

        else
        {

            currentCamera = Camera.main;
        }

        //convert format because y is flipped
        mousePosition = new Vector3(Event.current.mousePosition.x, currentCamera.pixelHeight - Event.current.mousePosition.y, 0f);

#else
		currentCamera = Camera.main;
		mousePosition = Input.mousePosition;
#endif

        Vector3 screenPos1 = currentCamera.WorldToScreenPoint(linePoint1);
        Vector3 screenPos2 = currentCamera.WorldToScreenPoint(linePoint2);
        Vector3 projectedPoint = ProjectPointOnLineSegment(screenPos1, screenPos2, mousePosition);

        // establece z en cero
        projectedPoint = new Vector3(projectedPoint.x, projectedPoint.y, 0f);

        Vector3 vector = projectedPoint - mousePosition;
        return vector.magnitude;
    }


    // Devuelve la distancia en píxeles desde el puntero del mouse a una cámara que mira hacia el círculo.
    // Alternativa para HandleUtility.DistanceToCircle (). Funciona tanto en modo Editor como en modo Play.
    // No llame a esta función desde OnGUI () ya que la posición del mouse será incorrecta.
    // Si quieres la distancia a un punto en lugar de un círculo, establece el radio en 0.
    public static float MouseDistanceToCircle(Vector3 point, float radius)
    {

        Camera currentCamera;
        Vector3 mousePosition;

#if UNITY_EDITOR
        if (Camera.current != null)
        {

            currentCamera = Camera.current;
        }

        else
        {

            currentCamera = Camera.main;
        }

        //convert format because y is flipped
        mousePosition = new Vector3(Event.current.mousePosition.x, currentCamera.pixelHeight - Event.current.mousePosition.y, 0f);
#else
		currentCamera = Camera.main;
		mousePosition = Input.mousePosition;
#endif

        Vector3 screenPos = currentCamera.WorldToScreenPoint(point);

        //set z to zero
        screenPos = new Vector3(screenPos.x, screenPos.y, 0f);

        Vector3 vector = screenPos - mousePosition;
        float fullDistance = vector.magnitude;
        float circleDistance = fullDistance - radius;

        return circleDistance;
    }

    // Devuelve verdadero si un segmento de línea (compuesto por linePoint1 y linePoint2) está total o parcialmente en un rectángulo
    // compuesto de RectA a RectD. Se supone que el segmento de línea está en el mismo plano que el rectángulo. Si la linea es
    // no en el avión, use ProjectPointOnPlane () en linePoint1 y linePoint2 primero.
    public static bool IsLineInRectangle(Vector3 linePoint1, Vector3 linePoint2, Vector3 rectA, Vector3 rectB, Vector3 rectC, Vector3 rectD)
    {

        bool pointAInside = false;
        bool pointBInside = false;

        pointAInside = IsPointInRectangle(linePoint1, rectA, rectC, rectB, rectD);

        if (!pointAInside)
        {

            pointBInside = IsPointInRectangle(linePoint2, rectA, rectC, rectB, rectD);
        }

        // ninguno de los puntos está dentro, así que verifique si una línea se cruza
        if (!pointAInside && !pointBInside)
        {

            bool lineACrossing = AreLineSegmentsCrossing(linePoint1, linePoint2, rectA, rectB);
            bool lineBCrossing = AreLineSegmentsCrossing(linePoint1, linePoint2, rectB, rectC);
            bool lineCCrossing = AreLineSegmentsCrossing(linePoint1, linePoint2, rectC, rectD);
            bool lineDCrossing = AreLineSegmentsCrossing(linePoint1, linePoint2, rectD, rectA);

            if (lineACrossing || lineBCrossing || lineCCrossing || lineDCrossing)
            {

                return true;
            }

            else
            {

                return false;
            }
        }

        else
        {

            return true;
        }
    }

    // Devuelve verdadero si "punto" está en un rectángulo en lo alto de RectA a RectD. Se supone que el punto de línea está en el mismo
    // plano como el rectángulo. Si el punto no está en el plano, use ProjectPointOnPlane () primero.
    public static bool IsPointInRectangle(Vector3 point, Vector3 rectA, Vector3 rectC, Vector3 rectB, Vector3 rectD)
    {

        Vector3 vector;
        Vector3 linePoint;

        // obtener el centro del rectángulo
        vector = rectC - rectA;
        float size = -(vector.magnitude / 2f);
        vector = AddVectorLength(vector, size);
        Vector3 middle = rectA + vector;

        Vector3 xVector = rectB - rectA;
        float width = xVector.magnitude / 2f;

        Vector3 yVector = rectD - rectA;
        float height = yVector.magnitude / 2f;

        linePoint = ProjectPointOnLine(middle, xVector.normalized, point);
        vector = linePoint - point;
        float yDistance = vector.magnitude;

        linePoint = ProjectPointOnLine(middle, yVector.normalized, point);
        vector = linePoint - point;
        float xDistance = vector.magnitude;

        if ((xDistance <= width) && (yDistance <= height))
        {

            return true;
        }

        else
        {

            return false;
        }
    }

    // Devuelve verdadero si el segmento de línea compuesto por el punto A1 y el punto A2 es un segmento de línea cruzada compuesto por
    // puntoB1 y puntoB2. Se supone que las dos líneas están en el mismo plano.
    public static bool AreLineSegmentsCrossing(Vector3 pointA1, Vector3 pointA2, Vector3 pointB1, Vector3 pointB2)
    {

        Vector3 closestPointA;
        Vector3 closestPointB;
        int sideA;
        int sideB;

        Vector3 lineVecA = pointA2 - pointA1;
        Vector3 lineVecB = pointB2 - pointB1;

        bool valid = ClosestPointsOnTwoLines(out closestPointA, out closestPointB, pointA1, lineVecA.normalized, pointB1, lineVecB.normalized);

        // las líneas no son paralelas
        if (valid)
        {

            sideA = PointOnWhichSideOfLineSegment(pointA1, pointA2, closestPointA);
            sideB = PointOnWhichSideOfLineSegment(pointB1, pointB2, closestPointB);

            if ((sideA == 0) && (sideB == 0))
            {

                return true;
            }

            else
            {

                return false;
            }
        }

        // las líneas son paralelas
        else
        {

            return false;
        }
    }

    public static bool LineSphereIntersection(Ray ray, Vector3 center, float radius, out float t0, out float t1)
    {
        t0 = t1 = 0;

        Vector3 L =  center - ray.origin;
        float tca = Vector3.Dot(L, ray.direction);

        /*if (tca < 0)
        {
            return false;
        }*/

        float d2 = Vector3.Dot(L, L) - tca * tca;

        float radiusSquared = radius * radius;

        if (d2 > radiusSquared)
            return false;

        float thc = Mathf.Sqrt(radiusSquared - d2);

        t0 = tca - thc;
        t1 = tca + thc;

        if (t0 > t1)
        {
            float temp = t0;
            t0 = t1;
            t1 = temp;
        }

        return true;
    }

    /// <summary>
    /// Determina el punto más cercano entre un punto y un triángulo.
    /// Tomado de la clase RPGMesh del paquete RPGController para Unity, por fholm
    /// El código de este método está protegido por copyright de SlimDX Group bajo la licencia MIT:
    ///
    ///
    /// El aviso de copyright anterior y este aviso de permiso se incluirán en
    /// todas las copias o partes sustanciales del Software.
    /// 
    public static void ClosestPointOnTriangleToPoint(Vector3 vertex1, Vector3 vertex2, Vector3 vertex3, Vector3 point, out Vector3 result)
    {
        //Source: Real-Time Collision Detection by Christer Ericson
        //Reference: Page 136

        // Comprueba si P en la región del vértice fuera de A
        Vector3 ab = vertex2 - vertex1;
        Vector3 ac = vertex3 - vertex1;
        Vector3 ap = point - vertex1;

        float d1 = Vector3.Dot(ab, ap);
        float d2 = Vector3.Dot(ac, ap);
        if (d1 <= 0.0f && d2 <= 0.0f)
        {
            result = vertex1; // Coordenadas barcéntricas (1,0,0)
            return;
        }

        // Comprueba si P en la región del vértice fuera de B
        Vector3 bp = point - vertex2;
        float d3 = Vector3.Dot(ab, bp);
        float d4 = Vector3.Dot(ac, bp);
        if (d3 >= 0.0f && d4 <= d3)
        {
            result = vertex2; // coordenadas barcéntricas (0,1,0)
            return;
        }

        // Verifique si P en la región del borde de AB, si es así, regrese la proyección de P sobre AB
        float vc = d1 * d4 - d3 * d2;
        if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
        {
            float v = d1 / (d1 - d3);
            result = vertex1 + v * ab; // Coordenadas barcéntricas (1-v, v, 0)
            return;
        }

        // Comprobar si P en la región del vértice fuera de C
        Vector3 cp = point - vertex3;
        float d5 = Vector3.Dot(ab, cp);
        float d6 = Vector3.Dot(ac, cp);
        if (d6 >= 0.0f && d5 <= d6)
        {
            result = vertex3; // Coordenadas barcéntricas (0,0,1)
            return;
        }

        // Verifique si P en la región del borde de AC, si es así, regrese la proyección de P sobre AC
        float vb = d5 * d2 - d1 * d6;
        if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
        {
            float w = d2 / (d2 - d6);
            result = vertex1 + w * ac; //Barycentric coordinates (1-w,0,w)
            return;
        }

        // Verifique si P en la región del borde de BC, si es así, regrese la proyección de P sobre BC
        float va = d3 * d6 - d5 * d4;
        if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
        {
            float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            result = vertex2 + w * (vertex3 - vertex2); //Barycentric coordinates (0,1-w,w)
            return;
        }

        // P dentro de la región de la cara. Calcule Q a través de sus coordenadas barcéntricas (u, v, w)
        float denom = 1.0f / (va + vb + vc);
        float v2 = vb * denom;
        float w2 = vc * denom;
        result = vertex1 + ab * v2 + ac * w2; // = u * vértice1 + v * vértice2 + w * vértice3, u = va * denom = 1.0f - v - w
    }
}