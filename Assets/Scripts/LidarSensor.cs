using UnityEngine;
using RosSharp.RosBridgeClient;
using Sensor = RosSharp.RosBridgeClient.MessageTypes.Sensor;
using Std = RosSharp.RosBridgeClient.MessageTypes.Std;

// Raycast LiDAR -> sensor_msgs/LaserScan, published through ros-sharp.
// Defaults model a TurtleBot3 LDS-01: 360 samples, 0.12-3.5 m, 5 Hz, full circle.
// Angles follow the ROS convention (CCW about +Z). Unity is Y-up / left-handed
// (yaw is clockwise), so we negate the Unity yaw to keep the published scan
// geometrically correct for a real LDS-01.
//
// The shared RosConnector is found on a parent, so many sensors can share one
// connection. Self-hits are filtered by range_min (the body sits within 0.12 m
// of the sensor origin and below the horizontal scan plane).
public class LidarSensor : MonoBehaviour
{
    public string Topic = "scan";
    public string FrameId = "base_scan";
    public int Samples = 360;
    public float RangeMin = 0.12f;
    public float RangeMax = 3.5f;
    public float ScanRateHz = 5f;
    public LayerMask Mask = ~0;

    private RosConnector connector;
    private string publicationId;
    private Sensor.LaserScan message;
    private float angleIncrement;
    private float nextScanTime;
    private bool advertised;

    private void Start()
    {
        connector = GetComponentInParent<RosConnector>();
        angleIncrement = 2f * Mathf.PI / Samples;
        message = new Sensor.LaserScan
        {
            header = new Std.Header { frame_id = FrameId },
            angle_min = 0f,
            angle_max = 2f * Mathf.PI - angleIncrement,
            angle_increment = angleIncrement,
            time_increment = 0f,
            scan_time = 1f / ScanRateHz,
            range_min = RangeMin,
            range_max = RangeMax,
            ranges = new float[Samples],
            intensities = new float[Samples]
        };
        nextScanTime = Time.time;
    }

    private void FixedUpdate()
    {
        if (connector == null || connector.RosSocket == null) return;
        if (!advertised)
        {
            publicationId = connector.RosSocket.Advertise<Sensor.LaserScan>(Topic);
            advertised = true;
        }
        if (Time.time < nextScanTime) return;
        nextScanTime = Time.time + 1f / ScanRateHz;

        for (int i = 0; i < Samples; i++)
        {
            float rosAngleDeg = -(i * angleIncrement) * Mathf.Rad2Deg;
            Vector3 dir = Quaternion.AngleAxis(rosAngleDeg, transform.up) * transform.forward;
            if (Physics.Raycast(transform.position, dir, out RaycastHit hit, RangeMax, Mask) && hit.distance >= RangeMin)
                message.ranges[i] = hit.distance;
            else
                message.ranges[i] = 0f; // no return (consumers treat < range_min as invalid)
        }

        SimTime.Stamp(message.header);
        connector.RosSocket.Publish(publicationId, message);
    }
}
