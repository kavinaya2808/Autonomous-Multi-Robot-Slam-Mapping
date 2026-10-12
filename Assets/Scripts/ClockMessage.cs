using RosSharp.RosBridgeClient.MessageTypes.BuiltinInterfaces;

// rosgraph_msgs/Clock is not shipped by ros-sharp; define it here (ROS2 form).
namespace RosSharp.RosBridgeClient.MessageTypes.Rosgraph
{
    public class Clock : RosSharp.RosBridgeClient.Message
    {
        public const string RosMessageName = "rosgraph_msgs/msg/Clock";
        public Time clock { get; set; }
        public Clock() { this.clock = new Time(); }
        public Clock(Time clock) { this.clock = clock; }
    }
}
