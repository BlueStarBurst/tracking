using System;
using System.Threading;
using Cysharp.Threading.Tasks;
using TensorFlowLite;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// BlazePose form MediaPipe
/// https://github.com/google/mediapipe
/// https://viz.mediapipe.dev/demo/pose_tracking
/// </summary>
public sealed class ModelController : MonoBehaviour
{
    [SerializeField, FilePopup("*.tflite")]
    string poseDetectionModelFile = "coco_ssd_mobilenet_quant.tflite";

    [SerializeField, FilePopup("*.tflite")]
    string poseLandmarkModelFile = "coco_ssd_mobilenet_quant.tflite";

    [SerializeField, FilePopup("*.tflite")]
    string faceModelFile = "coco_ssd_mobilenet_quant.tflite";

    [SerializeField, FilePopup("*.tflite")]
    string faceMeshModelFile = "coco_ssd_mobilenet_quant.tflite";

    FaceDetect faceDetect;

    FaceMesh faceMesh;

    Vector3[] rtCorners = new Vector3[4];

    MeshFilter faceMeshFilter;

    [SerializeField]
    Vector3[] faceKeypoints;

    FaceDetect.Result detection;

    [SerializeField]
    RawImage cameraView = null;

    [SerializeField]
    RawImage debugView = null;

    [SerializeField]
    Canvas canvas = null;

    [SerializeField]
    bool useLandmarkFilter = true;

    [SerializeField]
    Vector3 filterVelocityScale = Vector3.one * 10;

    [SerializeField]
    bool runBackground;

    [SerializeField, Range(0f, 1f)]
    float visibilityThreshold = 0.5f;

    WebCamTexture webcamTexture;

    // Refs to models
    PoseDetect poseDetect;

    PoseLandmarkDetect poseLandmark;

    [SerializeField]
    public float rotSpeed = 10f;

    [SerializeField]
    public float slowRotSpeed = 10f;

    [SerializeField]
    public float smoothPoint = 0.1f;

    public GameObject modelObj;

    public Vector3 adjustPos;

    public Vector3 debugRot;

    public Quaternion debugRot2;

    [SerializeField]
    Part[] modelParts;

    // [SerializeField] // for debug raw data
    [SerializeField]
    Vector4[] worldJoints;

    PoseDetect.Result poseResult;

    PoseLandmarkDetect.Result landmarkResult;

    UniTask<bool> task;

    CancellationToken cancellationToken;

    bool NeedsDetectionUpdate => poseResult == null || poseResult.score < 0.5f;

    GameObject[] jointObjs;

    public GameObject smallPoint;

    public enum BodyParts
    {
        head = 0,
        chest = 1,
        hips = 2,
        larm = 3,
        lforearm = 4,
        rarm = 5,
        rforearm = 6,
        lleg = 7,
        lshin = 8,
        rleg = 9,
        rshin = 10
    }

    public BodyParts[] frozenParts = new BodyParts[0];

    public bool isFrozen = false;

    [System.Serializable]
    public class Part
    {
        public BodyParts bodyPart;

        public GameObject obj;

        public bool flip = false;

        public Vector3 adjustRot = new Vector3(0, 0, 0);

        private Vector3 initPos = new Vector3(0, 0, 0);

        private Quaternion initRot = new Quaternion(0, 0, 0, 0);

        private int[] indexes;

        public Vector3 getPos()
        {
            return initPos;
        }

        public Quaternion getRot()
        {
            return initRot;
        }

        public int[] getIndexes()
        {
            return indexes;
        }

        public void init()
        {
            initPos = obj.transform.position;
            initRot = obj.transform.rotation;
            switch ((int) bodyPart)
            {
                //note: these are the indexes for the pose joint array
                case 0:
                    // head
                    indexes = new int[] { 7, 8, 0 };
                    break;
                case 1:
                    // chest
                    indexes = new int[] { 11, 12, 23 };
                    break;
                case 2:
                    // hips
                    indexes = new int[] { 23, 24 };
                    break;
                case 3:
                    // larm
                    indexes = new int[] { 12, 14 };
                    break;
                case 4:
                    // lforearm
                    indexes = new int[] { 14, 16 };
                    break;
                case 5:
                    // rarm
                    indexes = new int[] { 11, 13 };
                    break;
                case 6:
                    // rforearm
                    indexes = new int[] { 13, 15 };
                    break;
                case 7:
                    // lleg
                    indexes = new int[] { 24, 26 };
                    break;
                case 8:
                    // lshin
                    indexes = new int[] { 26, 28 };
                    break;
                case 9:
                    // rleg
                    indexes = new int[] { 23, 25 };
                    break;
                case 10:
                    // rshin
                    indexes = new int[] { 26, 27 };
                    break;
                default:
                    indexes = new int[] { 0, 0 };
                    break;
            }
        }
    }

    void Start()
    {
        foreach (Part item in modelParts)
        {
            item.init();
        }

        // Init model
        poseDetect = new PoseDetect(poseDetectionModelFile);
        poseLandmark = new PoseLandmarkDetect(poseLandmarkModelFile);

        // Init camera
        string cameraName =
            WebCamUtil
                .FindName(new WebCamUtil.PreferSpec()
                { isFrontFacing = false, kind = WebCamKind.WideAngle });
        webcamTexture = new WebCamTexture(cameraName, 1280, 720, 30);

        cameraView.texture = webcamTexture;
        webcamTexture.Play();
        Debug.Log($"Starting camera: {cameraName}");

        //draw = new PrimitiveDraw(Camera.main, gameObject.layer);
        worldJoints = new Vector4[PoseLandmarkDetect.JointCount];

        jointObjs = new GameObject[PoseLandmarkDetect.JointCount];
        for (int i = 0; i < jointObjs.Length; i++)
        {
            jointObjs[i] = Instantiate(smallPoint);
        }

        cancellationToken = this.GetCancellationTokenOnDestroy();
    }

    void OnDestroy()
    {
        webcamTexture?.Stop();
        poseDetect?.Dispose();
        poseLandmark?.Dispose();
    }

    void Update()
    {
        if (runBackground)
        {
            if (task.Status.IsCompleted())
            {
                task = InvokeAsync();
            }
        }
        else
        {
            Invoke();
        }

        if (poseResult != null && poseResult.score > 0f)
        {
            //DrawFrame (poseResult);
        }

        if (landmarkResult != null && landmarkResult.score > 0.6f)
        {
            //DrawCropMatrix(poseLandmark.CropMatrix);
            DrawJoints(landmarkResult.joints);
        }

        ApplyRotModel();

        Vector3 p1 =
            new Vector3(worldJoints[11].x,
                worldJoints[11].y,
                worldJoints[11].z);
        Vector3 p2 =
            new Vector3(worldJoints[12].x,
                worldJoints[12].y,
                worldJoints[12].z);

        modelObj.transform.position =
            Vector3
                .MoveTowards(modelObj.transform.position,
                (p1 + p2) / 2 + adjustPos,
                10f);
    }

    void ApplyRotModel()
    {
        for (int i = 0; i < modelParts.Length; i++)
        {
            int[] indexes = modelParts[i].getIndexes();
            int thisPart = (int) modelParts[i].bodyPart;
            bool freeze = false;

            if (isFrozen)
            {
                foreach (BodyParts partInFrozenArray in frozenParts)
                {
                    if (thisPart == (int) partInFrozenArray)
                    {
                        freeze = true;
                        break;
                    }
                }
            }

            if (freeze)
            {
                modelParts[i].obj.transform.rotation = modelParts[i].getRot();
            }
            else
            {
                if (
                    worldJoints[indexes[0]].w < 0 ||
                    worldJoints[indexes[1]].w < 0
                )
                {
                    //modelParts[i].obj.transform.position = modelParts[i].getPos();
                    //modelParts[i].obj.transform.rotation = modelParts[i].getRot();
                }
                else
                {
                    Vector3 p1 =
                        new Vector3(worldJoints[indexes[0]].x,
                            worldJoints[indexes[0]].y,
                            worldJoints[indexes[0]].z);
                    Vector3 p2 =
                        new Vector3(worldJoints[indexes[1]].x,
                            worldJoints[indexes[1]].y,
                            worldJoints[indexes[1]].z);

                    if (indexes.Length < 3)
                    {
                        jointObjs[indexes[0]]
                            .transform
                            .LookAt(jointObjs[indexes[1]].transform.position,
                            Vector3.up);
                        Quaternion adjustRot = new Quaternion(0, 0, 0, 0);
                        adjustRot.eulerAngles = modelParts[i].adjustRot;
                        modelParts[i].obj.transform.rotation =
                            Quaternion
                                .RotateTowards(modelParts[i]
                                    .obj
                                    .transform
                                    .rotation,
                                jointObjs[indexes[0]].transform.rotation *
                                adjustRot,
                                rotSpeed);
                    }
                    else
                    {
                        jointObjs[indexes[0]]
                            .transform
                            .LookAt(jointObjs[indexes[1]].transform.position,
                            Vector3.up);
                        Quaternion adjustRot = new Quaternion(0, 0, 0, 0);

                        Vector3 p3 =
                            new Vector3(worldJoints[indexes[2]].x,
                                worldJoints[indexes[2]].y,
                                worldJoints[indexes[2]].z);
                        Vector3 origin = (p1 + p2) / 2;

                        debugRot2 = Quaternion.LookRotation(p3 - origin);

                        debugRot = p3 - origin;

                        int flop = 1;

                        if (modelParts[i].flip)
                        {
                            flop = -1;
                        }

                        if (indexes[2] == 0)
                        {
                            Vector3 newAdjustRot =
                                new Vector3(Mathf
                                        .Asin((
                                        (p3.y - origin.y) /
                                        Vector3.Distance(p3, origin)
                                        ) +
                                        0.4f) *
                                    -100 *
                                    flop +
                                    modelParts[i].adjustRot.x,
                                    modelParts[i].adjustRot.y,
                                    modelParts[i].adjustRot.z);
                            adjustRot.eulerAngles = newAdjustRot;

                            modelParts[i].obj.transform.rotation =
                                Quaternion
                                    .RotateTowards(modelParts[i]
                                        .obj
                                        .transform
                                        .rotation,
                                    jointObjs[indexes[0]].transform.rotation *
                                    adjustRot,
                                    slowRotSpeed);
                        }
                        else
                        {
                            Vector3 newAdjustRot =
                                new Vector3(Mathf
                                        .Asin((
                                        (p3.y - origin.y) /
                                        Vector3.Distance(p3, origin)
                                        ) +
                                        0.4f) *
                                    -200 *
                                    flop +
                                    modelParts[i].adjustRot.x,
                                    modelParts[i].adjustRot.y,
                                    modelParts[i].adjustRot.z);
                            adjustRot.eulerAngles = newAdjustRot;

                            modelParts[i].obj.transform.rotation =
                                Quaternion
                                    .RotateTowards(modelParts[i]
                                        .obj
                                        .transform
                                        .rotation,
                                    jointObjs[indexes[0]].transform.rotation *
                                    adjustRot,
                                    rotSpeed);
                        }

                        /*
                        // p1 = forward / p2 = left / p3 = right
                        Vector3 p3 =
                            new Vector3(worldJoints[indexes[2]].x,
                                worldJoints[indexes[2]].y,
                                worldJoints[indexes[2]].z);
                        Vector3 origin = (p2 + p3) / 2;
                        Vector3 forwardDirection = p1 - origin;
                        Vector3 rightDirection = p3 - origin;
                        Vector3 upDirection =
                            Vector3.Cross(forwardDirection, rightDirection);

                        Quaternion orientation =
                            Quaternion
                                .LookRotation(forwardDirection, upDirection);

                        Quaternion adjustRot = new Quaternion(0, 0, 0, 0);
                        adjustRot.eulerAngles = modelParts[i].adjustRot;

                        //modelParts[i].obj.transform.rotation =
                        //   orientation * adjustRot;
                        modelParts[i].obj.transform.rotation =
                            Quaternion
                                .RotateTowards(modelParts[i]
                                    .obj
                                    .transform
                                    .rotation,
                                orientation * adjustRot,
                                rotSpeed);*/
                    }
                }
            }
        }
    }

    void Invoke()
    {
        if (NeedsDetectionUpdate)
        {
            poseDetect.Invoke (webcamTexture);

            //cameraView.material = poseDetect.transformMat;
            //cameraView.rectTransform.GetWorldCorners(rtCorners);
            poseResult = poseDetect.GetResults(0.7f, 0.3f);
        }
        if (poseResult.score < 0)
        {
            poseResult = null;
            landmarkResult = null;
            return;
        }
        poseLandmark.Invoke (webcamTexture, poseResult);

        debugView.texture = poseLandmark.inputTex;
        if (useLandmarkFilter)
        {
            poseLandmark.FilterVelocityScale = filterVelocityScale;
        }
        landmarkResult = poseLandmark.GetResult(useLandmarkFilter);

        if (landmarkResult.score < 0.3f)
        {
            poseResult.score = landmarkResult.score;
        }
        else
        {
            poseResult = PoseLandmarkDetect.LandmarkToDetection(landmarkResult);
        }
    }

    async UniTask<bool> InvokeAsync()
    {
        if (NeedsDetectionUpdate)
        {
            // Note: `await` changes PlayerLoopTiming from Update to FixedUpdate.
            poseResult =
                await poseDetect
                    .InvokeAsync(webcamTexture,
                    cancellationToken,
                    PlayerLoopTiming.FixedUpdate);
        }
        if (poseResult.score < 0)
        {
            poseResult = null;
            landmarkResult = null;
            return false;
        }

        if (useLandmarkFilter)
        {
            poseLandmark.FilterVelocityScale = filterVelocityScale;
        }
        landmarkResult =
            await poseLandmark
                .InvokeAsync(webcamTexture,
                poseResult,
                useLandmarkFilter,
                cancellationToken,
                PlayerLoopTiming.Update);

        /*
        // Back to the update timing from now on 
        if (cameraView != null)
        {
            cameraView.material = poseDetect.transformMat;
            cameraView.rectTransform.GetWorldCorners(rtCorners);
        }*/
        if (debugView != null)
        {
            debugView.texture = poseLandmark.inputTex;
        }

        // Generate poseResult from landmarkResult
        if (landmarkResult.score < 0.3f)
        {
            poseResult.score = landmarkResult.score;
        }
        else
        {
            poseResult = PoseLandmarkDetect.LandmarkToDetection(landmarkResult);
        }

        return true;
    }

    void DrawJoints(Vector4[] joints)
    {
        //draw.color = Color.blue;
        // Vector3 min = rtCorners[0];
        // Vector3 max = rtCorners[2];
        // Debug.Log($"rtCorners min: {min}, max: {max}");
        // Apply webcam rotation to draw landmarks correctly
        Matrix4x4 mtx =
            WebCamUtil
                .GetMatrix(-webcamTexture.videoRotationAngle,
                false,
                webcamTexture.videoVerticallyMirrored);

        // float zScale = (max.x - min.x) / 2;
        float zScale = 1;
        float zOffset = canvas.planeDistance;
        float aspect = (float) Screen.width / (float) Screen.height;
        Vector3
            scale,
            offset;
        if (aspect > 1)
        {
            scale = new Vector3(1f / aspect, 1f, zScale);
            offset = new Vector3((1 - 1f / aspect) / 2, 0, zOffset);
        }
        else
        {
            scale = new Vector3(1f, aspect, zScale);
            offset = new Vector3(0, (1 - aspect) / 2, zOffset);
        }

        // Update world joints
        var camera = canvas.worldCamera;
        for (int i = 0; i < joints.Length; i++)
        {
            Vector3 p = mtx.MultiplyPoint3x4((Vector3) joints[i]);
            p = Vector3.Scale(p, scale) + offset;
            p = camera.ViewportToWorldPoint(p);

            // w is visibility
            worldJoints[i] = new Vector4(p.x, p.y, p.z, joints[i].w);

            //jointObjs[i].transform.position = new Vector3(p.x, p.y, p.z);
            if (joints[i].w > 0)
            {
                Vector3 tempPos = new Vector3(p.x, p.y, p.z);
                float step =
                    Vector3.Distance(tempPos, jointObjs[i].transform.position) /
                    smoothPoint;

                jointObjs[i].transform.position =
                    Vector3
                        .MoveTowards(jointObjs[i].transform.position,
                        tempPos,
                        step);
            }
        }

        // Draw
        /*
        for (int i = 0; i < worldJoints.Length; i++)
        {
            Vector4 p = worldJoints[i];
            if (p.w > visibilityThreshold)
            {
                draw.Cube(p, 0.2f);
            }
        }
        var connections = PoseLandmarkDetect.Connections;
        for (int i = 0; i < connections.Length; i += 2)
        {
            var a = worldJoints[connections[i]];
            var b = worldJoints[connections[i + 1]];
            if (a.w > visibilityThreshold || b.w > visibilityThreshold)
            {
                draw.Line3D(a, b, 0.05f);
            }
        }
        draw.Apply();
        */
    }
}
