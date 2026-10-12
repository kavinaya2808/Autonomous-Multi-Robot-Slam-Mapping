using UnityEngine;
using RosSharp.RosBridgeClient;
using Geometry = RosSharp.RosBridgeClient.MessageTypes.Geometry;

// Subscribes to /cmd_vel (geometry_msgs/Twist) and drives the kinematic robot by
// integrating the base pose. ROS angular.z is CCW; Unity yaw is CW, so it's negated.
// Defaults are TurtleBot3 Burger velocity limits.
// The robot is moved kinematically (no physics forces), so forward/backward
// motion is checked with a sphere cast first: the robot stops at walls,
// furniture and the other robot instead of driving through them.
public class DiffDriveController : MonoBehaviour
{
    public string CmdVelTopic = "cmd_vel";
    public float MaxLinear = 0.22f;
    public float MaxAngular = 2.84f;
    public float CmdTimeout = 0.5f;

    [Header("Collision blocking")]
    public bool BlockOnCollision = true;
    public float BodyRadius = 0.08f;   // Burger footprint is ~0.105 m radius
    public float CastHeight = 0.12f;   // sphere centre above the robot origin
    public float Clearance = 0.02f;    // stop this far before contact
    public LayerMask CollisionMask = ~0;
    public float StepHeight = 0.05f;   // roll over anything lower (rugs, sills, floor seams)
    public bool LogBlocking = true;    // print what stopped the robot (Unity Console)

    private float nextBlockLog;

    private RosConnector connector;
    private volatile float cmdLinear;
    private volatile float cmdAngular;
    private volatile bool gotCmd;
    private float lastCmdTime;
    private bool subscribed;

    private void Start()
    {
        connector = GetComponentInParent<RosConnector>();
    }

    private void FixedUpdate()
    {
        if (connector == null || connector.RosSocket == null) return;
        if (!subscribed)
        {
            connector.RosSocket.Subscribe<Geometry.Twist>(CmdVelTopic, OnCmdVel);
            subscribed = true;
        }
        if (gotCmd) { lastCmdTime = Time.time; gotCmd = false; }

        float lin = cmdLinear, ang = cmdAngular;
        if (Time.time - lastCmdTime > CmdTimeout) { lin = 0f; ang = 0f; }
        lin = Mathf.Clamp(lin, -MaxLinear, MaxLinear);
        ang = Mathf.Clamp(ang, -MaxAngular, MaxAngular);

        float dt = Time.fixedDeltaTime;
        transform.Rotate(0f, -ang * Mathf.Rad2Deg * dt, 0f, Space.World);

        float step = lin * dt;
        if (BlockOnCollision && Mathf.Abs(step) > 1e-6f)
        {
            Vector3 dir = transform.forward * Mathf.Sign(step);
            float free = FreeDistance(dir, Mathf.Abs(step) + Clearance) - Clearance;
            step = Mathf.Sign(step) * Mathf.Clamp(free, 0f, Mathf.Abs(step));
        }
        transform.position += transform.forward * step;
    }

    // Distance the body can travel along dir before touching something that
    // is not part of this robot (colliders overlapping at the start are ignored).
    private float FreeDistance(Vector3 dir, float maxDistance)
    {
        Vector3 origin = transform.position + Vector3.up * CastHeight;
        float best = maxDistance;
        RaycastHit blocker = default;
        bool blocked = false;
        RaycastHit[] hits = Physics.SphereCastAll(
            origin, BodyRadius, dir, maxDistance, CollisionMask, QueryTriggerInteraction.Ignore);
        foreach (RaycastHit h in hits)
        {
            if (h.collider.transform.IsChildOf(transform)) continue;       // our own body
            if (h.distance <= 0f) continue;                                 // overlapping at start
            if (h.normal.y > 0.7f) continue;                                // floor-like surface
            if (h.point.y - transform.position.y < StepHeight) continue;    // low edge: roll over
            if (h.distance < best) { best = h.distance; blocker = h; blocked = true; }
        }
        if (blocked && LogBlocking && Time.time >= nextBlockLog)
        {
            nextBlockLog = Time.time + 1f;
            Debug.Log($"[{name}] blocked by '{blocker.collider.name}' " +
                      $"(contact {blocker.point.y - transform.position.y:F2} m above floor, {best:F2} m ahead)");
        }
        return best;
    }

    private void OnCmdVel(Geometry.Twist msg)
    {
        cmdLinear = (float)msg.linear.x;
        cmdAngular = (float)msg.angular.z;
        gotCmd = true;
    }
}
