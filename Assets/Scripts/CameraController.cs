using System.Collections;
using UnityEngine;
using UnityEngine;

public class CameraController : MonoBehaviour
{
    public Transform target;

    public Transform target2;

    private Transform actTarget;

    public Vector3 adjustPos;

    public Vector3 adjustPos2;

    public bool isSecondTarget = false;

    public float distance = 2.0f;

    public float distance2 = 2.0f;

    private float actDistance;

    public float xSpeed = 20.0f;

    public float ySpeed = 20.0f;

    public float yMinLimit = -90f;

    public float yMaxLimit = 90f;

    public float distanceMin = 10f;

    public float distanceMax = 10f;

    public float smoothTime = 2f;

    float rotationYAxis = 0.0f;

    float rotationXAxis = 0.0f;

    float velocityX = 0.0f;

    float velocityY = 0.0f;

    // Use this for initialization
    void Start()
    {
        if (isSecondTarget)
        {
            actTarget = target2;
            actDistance = distance2;
        }
        else
        {
            actTarget = target;
            actDistance = distance;
        }

        Vector3 angles = transform.eulerAngles;
        rotationYAxis = angles.y;
        rotationXAxis = angles.x;

        // Make the rigid body not change rotation
        if (GetComponent<Rigidbody>())
        {
            GetComponent<Rigidbody>().freezeRotation = true;
        }
    }

    private bool trigger = false;

    void LateUpdate()
    {
        if (actTarget)
        {
            if (Input.GetMouseButton(1) && trigger == false)
            {
                isSecondTarget = !isSecondTarget;
                if (isSecondTarget)
                {
                    distance = actDistance;
                    actTarget = target2;
                    actDistance = distance2;
                }
                else
                {
                    distance2 = actDistance;
                    actDistance = distance;
                    actTarget = target;
                }
                trigger = true;
            }
            else if (Input.GetMouseButton(1))
            {
                trigger = true;
            }
            else
            {
                trigger = false;
            }
            if (Input.GetMouseButton(0))
            {
                velocityX +=
                    xSpeed * Input.GetAxis("Mouse X") * actDistance * 0.02f;
                velocityY += ySpeed * Input.GetAxis("Mouse Y") * 0.02f;
            }
            rotationYAxis += velocityX;
            rotationXAxis -= velocityY;
            rotationXAxis = ClampAngle(rotationXAxis, yMinLimit, yMaxLimit);
            Quaternion fromRotation =
                Quaternion
                    .Euler(transform.rotation.eulerAngles.x,
                    transform.rotation.eulerAngles.y,
                    0);
            Quaternion toRotation =
                Quaternion.Euler(rotationXAxis, rotationYAxis, 0);
            Quaternion rotation = toRotation;

            actDistance =
                Mathf
                    .Clamp(actDistance - Input.GetAxis("Mouse ScrollWheel") * 5,
                    distanceMin,
                    distanceMax);
            RaycastHit hit;
            if (
                Physics
                    .Linecast(actTarget.position, transform.position, out hit)
            )
            {
                actDistance -= hit.distance;
            }
            Vector3 negDistance = new Vector3(0.0f, 0.0f, -actDistance);
            Vector3 position = rotation * negDistance + actTarget.position;
            transform.rotation = rotation;

            if (!isSecondTarget)
            {
                transform.position = position + adjustPos;
            }
            else
            {
                transform.position = position + adjustPos2;
            }

            velocityX = Mathf.Lerp(velocityX, 0, Time.deltaTime * smoothTime);
            velocityY = Mathf.Lerp(velocityY, 0, Time.deltaTime * smoothTime);
        }
    }

    public static float ClampAngle(float angle, float min, float max)
    {
        if (angle < -360F) angle += 360F;
        if (angle > 360F) angle -= 360F;
        return Mathf.Clamp(angle, min, max);
    }
}
