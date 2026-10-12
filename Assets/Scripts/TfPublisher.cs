using System.Collections.Generic;
using UnityEngine;
using RosSharp;
using RosSharp.RosBridgeClient;
using Tf2 = RosSharp.RosBridgeClient.MessageTypes.Tf2;
using Geometry = RosSharp.RosBridgeClient.MessageTypes.Geometry;
using Std = RosSharp.RosBridgeClient.MessageTypes.Std;
using Nav = RosSharp.RosBridgeClient.MessageTypes.Nav;

// Publishes:
//   /tf        -> dynamic odom -> base_footprint
//   /tf_static -> fixed URDF link transforms
//   /<prefix>/odom -> nav_msgs/Odometry (same pose as the TF, plus twist; Nav2 needs it)
//
// Each robot uses a unique FramePrefix, for example:
//   robot1/odom -> robot1/base_footprint -> robot1/base_link -> robot1/base_scan
//   robot2/odom -> robot2/base_footprint -> robot2/base_link -> robot2/base_scan
//
// SLAM Toolbox owns map -> odom, so that transform is intentionally not
// published here.
public class TfPublisher : MonoBehaviour
{
    [Header("ROS Topics")]
    public string TfTopic = "/tf";
    public string TfStaticTopic = "/tf_static";

    [Header("Robot Frames")]
    public string FramePrefix = "robot1";
    public string OdomFrame = "odom";

    [Header("Odometry (nav_msgs/Odometry)")]
    public bool PublishOdom = true;
    // Leave empty for "/<FramePrefix>/odom", e.g. /robot1/odom
    public string OdomTopic = "";

    private string odomPublicationId;
    private bool odomAdvertised;
    private bool hasPrevOdom;
    private Vector3 prevOdomPos;   // ROS coordinates (odom frame)
    private float prevOdomYaw;
    private double prevOdomTime;
    private double odomVx, odomVy, odomWz;

    [Header("Publishing")]
    public float PublishRateHz = 30f;

    private RosConnector connector;

    private string tfPublicationId;
    private string tfStaticPublicationId;

    private float nextPublish;

    private bool tfAdvertised;
    private bool tfStaticAdvertised;
    private float nextStaticPublish;
    public float StaticPublishRateHz = 1f;

    private Vector3 initialPosition;
    private Quaternion initialRotation;

    private Transform baseFootprint;

    private struct Link
    {
        public string parent;
        public string child;
        public Transform transform;
    }

    private readonly List<Link> links = new List<Link>();

    private void Start()
    {
        connector = GetComponentInParent<RosConnector>();

        baseFootprint = FindDeep(transform, "base_footprint");

        if (baseFootprint != null)
        {
            initialPosition = baseFootprint.position;
            initialRotation = baseFootprint.rotation;
        }
        AddLink("base_footprint", "base_link");
        AddLink("base_link", "base_scan");
        AddLink("base_link", "camera_link");
        AddLink("base_link", "wheel_left_link");
        AddLink("base_link", "wheel_right_link");
        AddLink("base_link", "caster_back_link");
        AddLink("base_link", "imu_link");
    }

    private void AddLink(string parent, string child)
    {
        Transform t = FindDeep(transform, child);

        if (t != null)
        {
            links.Add(new Link
            {
                parent = parent,
                child = child,
                transform = t
            });
        }
    }

    private void Update()
    {
        if (connector == null ||
            connector.RosSocket == null ||
            baseFootprint == null)
        {
            return;
        }

        AdvertiseTopics();

        // Publish static URDF transforms once.
        if (Time.time >= nextStaticPublish)
        {
            nextStaticPublish = Time.time + 1f / StaticPublishRateHz;
            PublishStaticTransforms();
        }

        // Publish dynamic odom -> base_footprint at configured rate.
        if (Time.time < nextPublish)
        {
            return;
        }

        nextPublish = Time.time + 1f / PublishRateHz;

        PublishDynamicTransform();
    }

    private void AdvertiseTopics()
    {
        if (PublishOdom && !odomAdvertised)
        {
            string topic = string.IsNullOrEmpty(OdomTopic) ? "/" + Prefix("odom") : OdomTopic;
            odomPublicationId = connector.RosSocket.Advertise<Nav.Odometry>(topic);
            odomAdvertised = true;
        }

        if (!tfAdvertised)
        {
            tfPublicationId =
                connector.RosSocket.Advertise<Tf2.TFMessage>(TfTopic);

            tfAdvertised = true;
        }

        if (!tfStaticAdvertised)
        {
            tfStaticPublicationId =
                connector.RosSocket.Advertise<Tf2.TFMessage>(TfStaticTopic);

            tfStaticAdvertised = true;
        }
    }

    private void PublishDynamicTransform()
    {
        string odomFrame = Prefix(OdomFrame);
        string baseFootprintFrame = Prefix("base_footprint");

        Vector3 relativePosition =
            Quaternion.Inverse(initialRotation) *
            (baseFootprint.position - initialPosition);

        Quaternion relativeRotation =
            Quaternion.Inverse(initialRotation) *
            baseFootprint.rotation;

        Geometry.TransformStamped transform = Make(
            odomFrame,
            baseFootprintFrame,
            relativePosition,
            relativeRotation
        );

        Tf2.TFMessage message = new Tf2.TFMessage
        {
            transforms = new[]
            {
                transform
            }
        };

        connector.RosSocket.Publish(tfPublicationId, message);

        if (PublishOdom && odomAdvertised)
        {
            PublishOdometry(transform);
        }
    }

    // Publishes the same odom -> base_footprint pose as the TF above, with a
    // twist estimated from consecutive poses (expressed in base_footprint).
    // The robot moves in FixedUpdate, so velocities use fixed-step time.
    private void PublishOdometry(Geometry.TransformStamped ts)
    {
        Geometry.Vector3 tr = ts.transform.translation;
        Geometry.Quaternion q = ts.transform.rotation;

        Vector3 pos = new Vector3((float)tr.x, (float)tr.y, (float)tr.z);
        float yaw = (float)System.Math.Atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        double t = Time.fixedTimeAsDouble;

        if (hasPrevOdom && t > prevOdomTime)
        {
            float dt = (float)(t - prevOdomTime);
            Vector3 d = pos - prevOdomPos;
            float c = Mathf.Cos(yaw);
            float s = Mathf.Sin(yaw);
            odomVx = (c * d.x + s * d.y) / dt;
            odomVy = (-s * d.x + c * d.y) / dt;
            odomWz = Mathf.DeltaAngle(prevOdomYaw * Mathf.Rad2Deg, yaw * Mathf.Rad2Deg)
                     * Mathf.Deg2Rad / dt;
        }
        if (!hasPrevOdom || t > prevOdomTime)
        {
            prevOdomPos = pos;
            prevOdomYaw = yaw;
            prevOdomTime = t;
            hasPrevOdom = true;
        }

        // Simulated odometry is ground truth, so the covariance is small.
        double[] poseCov = new double[36];
        double[] twistCov = new double[36];
        for (int i = 0; i < 6; i++)
        {
            poseCov[i * 7] = 1e-4;
            twistCov[i * 7] = 1e-4;
        }

        Nav.Odometry odom = new Nav.Odometry
        {
            header = new Std.Header { frame_id = ts.header.frame_id },
            child_frame_id = ts.child_frame_id,
            pose = new Geometry.PoseWithCovariance
            {
                pose = new Geometry.Pose
                {
                    position = new Geometry.Point { x = tr.x, y = tr.y, z = tr.z },
                    orientation = new Geometry.Quaternion { x = q.x, y = q.y, z = q.z, w = q.w }
                },
                covariance = poseCov
            },
            twist = new Geometry.TwistWithCovariance
            {
                twist = new Geometry.Twist
                {
                    linear = new Geometry.Vector3 { x = odomVx, y = odomVy, z = 0 },
                    angular = new Geometry.Vector3 { x = 0, y = 0, z = odomWz }
                },
                covariance = twistCov
            }
        };
        odom.header.stamp = ts.header.stamp;

        connector.RosSocket.Publish(odomPublicationId, odom);
    }

    private void PublishStaticTransforms()
    {
        List<Geometry.TransformStamped> staticTransforms =
            new List<Geometry.TransformStamped>();

        foreach (Link link in links)
        {
            staticTransforms.Add(
                Make(
                    Prefix(link.parent),
                    Prefix(link.child),
                    link.transform.localPosition,
                    link.transform.localRotation
                )
            );
        }

        Tf2.TFMessage message = new Tf2.TFMessage
        {
            transforms = staticTransforms.ToArray()
        };

        connector.RosSocket.Publish(tfStaticPublicationId, message);
    }

    private string Prefix(string frame)
    {
        if (string.IsNullOrEmpty(FramePrefix))
        {
            return frame;
        }

        return FramePrefix.TrimEnd('/') + "/" + frame;
    }

    private Geometry.TransformStamped Make(
        string parent,
        string child,
        Vector3 pos,
        Quaternion rot)
    {
        Vector3 p = pos.Unity2Ros();
        Quaternion q = rot.Unity2Ros();

        Geometry.TransformStamped ts =
            new Geometry.TransformStamped
            {
                header = new Std.Header
                {
                    frame_id = parent
                },

                child_frame_id = child,

                transform = new Geometry.Transform
                {
                    translation = new Geometry.Vector3
                    {
                        x = p.x,
                        y = p.y,
                        z = p.z
                    },

                    rotation = new Geometry.Quaternion
                    {
                        x = q.x,
                        y = q.y,
                        z = q.z,
                        w = q.w
                    }
                }
            };

        SimTime.Stamp(ts.header);

        return ts;
    }

    private static Transform FindDeep(Transform root, string name)
    {
        if (root.name == name)
        {
            return root;
        }

        foreach (Transform child in root)
        {
            Transform result = FindDeep(child, name);

            if (result != null)
            {
                return result;
            }
        }

        return null;
    }
}