using UnityEngine;
using RosSharp.RosBridgeClient;
using Sensor = RosSharp.RosBridgeClient.MessageTypes.Sensor;
using Std = RosSharp.RosBridgeClient.MessageTypes.Std;

// Unity Camera -> sensor_msgs/CompressedImage (JPEG), published through ros-sharp.
// URP-compatible: the camera renders to a RenderTexture and we read it back on a
// timer. (ros-sharp's built-in ImagePublisher uses Camera.onPostRender, which does
// not fire under the Universal Render Pipeline.)
//
// The shared RosConnector is found on a parent so this can share one connection
// with the LiDAR and other publishers.
[RequireComponent(typeof(Camera))]
public class CameraSensor : MonoBehaviour
{
    public string Topic = "camera/image_raw/compressed";
    public string FrameId = "camera_rgb_optical_frame";
    public int Width = 640;
    public int Height = 480;
    [Range(1, 100)] public int JpegQuality = 50;
    public float PublishRateHz = 15f;

    private RosConnector connector;
    private string publicationId;
    private Camera cam;
    private RenderTexture rt;
    private Texture2D tex;
    private Sensor.CompressedImage message;
    private float nextPublishTime;
    private bool advertised;

    private void Start()
    {
        connector = GetComponentInParent<RosConnector>();
        cam = GetComponent<Camera>();
        rt = new RenderTexture(Width, Height, 24);
        cam.targetTexture = rt;
        tex = new Texture2D(Width, Height, TextureFormat.RGB24, false);
        message = new Sensor.CompressedImage
        {
            header = new Std.Header { frame_id = FrameId },
            format = "jpeg"
        };
        nextPublishTime = Time.time;
    }

    private void LateUpdate()
    {
        if (connector == null || connector.RosSocket == null) return;
        if (!advertised)
        {
            publicationId = connector.RosSocket.Advertise<Sensor.CompressedImage>(Topic);
            advertised = true;
        }
        if (Time.time < nextPublishTime) return;
        nextPublishTime = Time.time + 1f / PublishRateHz;

        RenderTexture prev = RenderTexture.active;
        RenderTexture.active = rt;
        tex.ReadPixels(new Rect(0, 0, Width, Height), 0, 0);
        tex.Apply();
        RenderTexture.active = prev;

        SimTime.Stamp(message.header);
        message.data = tex.EncodeToJPG(JpegQuality);
        connector.RosSocket.Publish(publicationId, message);
    }

    private void OnDestroy()
    {
        if (cam != null) cam.targetTexture = null;
        if (rt != null) rt.Release();
    }
}
