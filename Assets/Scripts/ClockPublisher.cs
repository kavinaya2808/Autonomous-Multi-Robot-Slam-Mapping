using UnityEngine;
using RosSharp.RosBridgeClient;
using Rosgraph = RosSharp.RosBridgeClient.MessageTypes.Rosgraph;

// Publishes rosgraph_msgs/Clock so the ROS2 side can run with use_sim_time:=true.
public class ClockPublisher : MonoBehaviour
{
    public string Topic = "clock";
    public float PublishRateHz = 100f;

    private RosConnector connector;
    private string publicationId;
    private Rosgraph.Clock message;
    private float nextPublish;
    private bool advertised;

    private void Start()
    {
        connector = GetComponentInParent<RosConnector>();
        message = new Rosgraph.Clock();
    }

    private void Update()
    {
        if (connector == null || connector.RosSocket == null) return;
        if (!advertised)
        {
            publicationId = connector.RosSocket.Advertise<Rosgraph.Clock>(Topic);
            advertised = true;
        }
        if (Time.time < nextPublish) return;
        nextPublish = Time.time + 1f / PublishRateHz;

        double t = SimTime.Now;
        int sec = (int)t;
        message.clock.sec = sec;
        message.clock.nanosec = (uint)((t - sec) * 1e9);
        connector.RosSocket.Publish(publicationId, message);
    }
}
