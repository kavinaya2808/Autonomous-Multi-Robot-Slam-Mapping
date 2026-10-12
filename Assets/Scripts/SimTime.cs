using UnityEngine;
using Std = RosSharp.RosBridgeClient.MessageTypes.Std;

// Single simulation-time source shared by /clock, the sensors, and TF so a
// use_sim_time SLAM stack sees consistent stamps. Unity play time since start.
public static class SimTime
{
    public static double Now => Time.timeAsDouble;

    public static void Stamp(Std.Header header)
    {
        double t = Now;
        int sec = (int)t;
        header.stamp.sec = sec;
        header.stamp.nanosec = (uint)((t - sec) * 1e9);
    }
}
