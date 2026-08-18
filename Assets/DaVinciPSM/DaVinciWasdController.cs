using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.InputSystem;
using Unity.Robotics.UrdfImporter;
using DaVinci;

/// Drives the PSM1 ArticulationBody joints from the keyboard, in two modes:
///  - JointSpace: WASD/etc. jog individual joints directly (simple, always predictable).
///  - CartesianIK: WASD/etc. move a target point in world space, and a numerical IK solver
///    (DaVinciKinematics) drives yaw/pitch/insertion/roll/wrist_pitch/wrist_yaw to track it —
///    this is how a real PSM is actually teleoperated (surgeon moves in Cartesian space, the
///    controller solves joint angles), not by jogging joints one at a time.
/// Joint names below match the "joint name=" attributes in PSM1.urdf, not GameObject names
/// (the importer attaches each ArticulationBody to the joint's child *link* GameObject).
/// All physics-affecting writes happen in FixedUpdate on Unity's fixed timestep, since ArticulationBody
/// state only advances on physics steps — driving it from Update would make behavior framerate-dependent.
public class DaVinciWasdController : MonoBehaviour
{
    public enum ControlMode { JointSpace, CartesianIK }

    [System.Serializable]
    public class AxisBinding
    {
        public string jointName;
        public Key positiveKey;
        public Key negativeKey;
        public float speed; // degrees/sec for revolute joints, meters/sec for prismatic
    }

    [System.Serializable]
    public class MimicBinding
    {
        public string childJointName;
        public string masterJointName;
        public float multiplier;
    }

    [Header("Mode (Tab to toggle)")]
    public ControlMode mode = ControlMode.CartesianIK;

    [Header("Joint-space jog bindings (yaw/pitch/insertion/roll/wrist)")]
    public List<AxisBinding> axisBindings = new List<AxisBinding>
    {
        new AxisBinding{ jointName = "insertion",   positiveKey = Key.W,          negativeKey = Key.S,         speed = 0.08f },
        new AxisBinding{ jointName = "yaw",         positiveKey = Key.D,          negativeKey = Key.A,         speed = 25f },
        new AxisBinding{ jointName = "pitch",       positiveKey = Key.E,          negativeKey = Key.Q,         speed = 25f },
        new AxisBinding{ jointName = "wrist_yaw",   positiveKey = Key.RightArrow, negativeKey = Key.LeftArrow, speed = 40f },
        new AxisBinding{ jointName = "wrist_pitch", positiveKey = Key.UpArrow,    negativeKey = Key.DownArrow, speed = 40f },
        new AxisBinding{ jointName = "roll",        positiveKey = Key.X,          negativeKey = Key.Z,         speed = 60f },
    };

    [Header("Gripper (always active, both modes)")]
    public AxisBinding jawBinding = new AxisBinding{ jointName = "jaw", positiveKey = Key.V, negativeKey = Key.C, speed = 40f };

    // The dvrk_model URDF encodes these as <mimic joint="..." multiplier="..."/> on the parallelogram
    // linkage and gripper halves. Unity's URDF Importer parses <mimic> but does not drive it at runtime,
    // so it's reproduced here by copying the master joint's drive target every fixed step.
    public List<MimicBinding> mimicBindings = new List<MimicBinding>
    {
        new MimicBinding{ childJointName = "pitch_2", masterJointName = "pitch", multiplier = 1f },
        new MimicBinding{ childJointName = "pitch_3", masterJointName = "pitch", multiplier = 1f },
        new MimicBinding{ childJointName = "pitch_4", masterJointName = "pitch", multiplier = -1f },
        new MimicBinding{ childJointName = "pitch_5", masterJointName = "pitch", multiplier = -1f },
        new MimicBinding{ childJointName = "pitch_6", masterJointName = "pitch", multiplier = 1f },
        new MimicBinding{ childJointName = "jaw_1",   masterJointName = "jaw",   multiplier = 0.5f },
        new MimicBinding{ childJointName = "jaw_2",   masterJointName = "jaw",   multiplier = -0.5f },
    };

    [Header("Cartesian IK tuning")]
    public int ikIterationsPerStep = 6;
    public float ikPositionGain = 0.6f;
    public float ikRotationGain = 0.3f;
    public float cartesianMoveSpeed = 0.05f;   // m/s
    public float cartesianRotateSpeed = 30f;   // deg/s

    private static readonly string[] IkJointOrder =
        { "yaw", "pitch", "insertion", "roll", "wrist_pitch", "wrist_yaw" };

    private readonly Dictionary<string, ArticulationBody> joints = new Dictionary<string, ArticulationBody>();
    private List<JointFrame> ikChain;
    private Transform ikBaseParent;   // world-fixed reference: parent link of the first IK joint
    private Vector3 tipLocalPos;      // fixed tool-tip offset relative to the last chain link
    private Quaternion tipLocalRot;
    private Vector3 ikTargetPos;
    private Quaternion ikTargetRot;
    private Vector3 lastFkPos;        // for the on-screen tracking-error readout

    void Awake()
    {
        // The importer auto-attaches its own arrow-key jog controller (Controller.cs), which also
        // drives xDrive and zeroes forceLimit on Start(). Disable it so it doesn't fight this script.
        var builtInController = GetComponent<Unity.Robotics.UrdfImporter.Control.Controller>();
        if (builtInController != null) builtInController.enabled = false;

        foreach (var urdfJoint in GetComponentsInChildren<UrdfJoint>())
        {
            var ab = urdfJoint.GetComponent<ArticulationBody>();
            if (ab == null || string.IsNullOrEmpty(urdfJoint.jointName)) continue;

            joints[urdfJoint.jointName] = ab;

            var drive = ab.xDrive;
            drive.stiffness = 10000f;
            drive.damping = 500f;
            if (drive.forceLimit <= 0f) drive.forceLimit = 1000f;
            drive.target = 0f;
            ab.xDrive = drive;
        }

        // A 10+ body serial chain needs more solver iterations than Unity's default (6/1) to stay
        // stiff and avoid visible drift/jitter under fast IK-driven motion. This is set on the root
        // body, which controls solver quality for its whole articulation chain.
        var rootBody = GetComponentsInChildren<ArticulationBody>().FirstOrDefault(ab => ab.isRoot);
        if (rootBody != null)
        {
            rootBody.solverIterations = 30;
            rootBody.solverVelocityIterations = 10;
        }

        // A precision control loop warrants a tighter physics step than Unity's 50Hz default.
        Time.fixedDeltaTime = 1f / 200f;

        ikChain = DaVinciKinematics.BuildChain(gameObject, IkJointOrder);
        if (ikChain.Count == IkJointOrder.Length)
        {
            ikBaseParent = ikChain[0].body.transform.parent;
            Transform tipTransform = DaVinciKinematics.FindJointChildLink(gameObject, "tool_tip");
            if (tipTransform != null)
            {
                tipLocalPos = tipTransform.localPosition;
                tipLocalRot = tipTransform.localRotation;
            }
            SyncIkTargetToCurrentPose();
        }
        else
        {
            Debug.LogWarning("DaVinciWasdController: IK chain incomplete, Cartesian IK mode unavailable.");
        }
    }

    void Update()
    {
        var keyboard = Keyboard.current;
        if (keyboard == null) return;

        if (keyboard.tabKey.wasPressedThisFrame && ikChain != null && ikChain.Count == IkJointOrder.Length)
        {
            mode = mode == ControlMode.JointSpace ? ControlMode.CartesianIK : ControlMode.JointSpace;
            if (mode == ControlMode.CartesianIK) SyncIkTargetToCurrentPose();
        }
    }

    void FixedUpdate()
    {
        var keyboard = Keyboard.current;
        if (keyboard == null) return;

        ApplyBinding(keyboard, jawBinding);

        if (mode == ControlMode.JointSpace)
        {
            foreach (var b in axisBindings) ApplyBinding(keyboard, b);
        }
        else
        {
            StepCartesianIk(keyboard);
        }

        foreach (var m in mimicBindings)
        {
            if (!joints.TryGetValue(m.masterJointName, out var master)) continue;
            if (!joints.TryGetValue(m.childJointName, out var child)) continue;

            var drive = child.xDrive;
            drive.target = master.xDrive.target * m.multiplier;
            child.xDrive = drive;
        }
    }

    private void ApplyBinding(Keyboard keyboard, AxisBinding b)
    {
        if (!joints.TryGetValue(b.jointName, out var ab)) return;

        float dir = 0f;
        if (keyboard[b.positiveKey].isPressed) dir += 1f;
        if (keyboard[b.negativeKey].isPressed) dir -= 1f;
        if (dir == 0f) return;

        var drive = ab.xDrive;
        drive.target = Mathf.Clamp(drive.target + dir * b.speed * Time.fixedDeltaTime, drive.lowerLimit, drive.upperLimit);
        ab.xDrive = drive;
    }

    private void StepCartesianIk(Keyboard keyboard)
    {
        Vector3 move = Vector3.zero;
        if (keyboard[Key.W].isPressed) move += Vector3.forward;
        if (keyboard[Key.S].isPressed) move += Vector3.back;
        if (keyboard[Key.D].isPressed) move += Vector3.right;
        if (keyboard[Key.A].isPressed) move += Vector3.left;
        if (keyboard[Key.E].isPressed) move += Vector3.up;
        if (keyboard[Key.Q].isPressed) move += Vector3.down;
        if (move != Vector3.zero)
        {
            ikTargetPos += move.normalized * cartesianMoveSpeed * Time.fixedDeltaTime;
        }

        float rollDir = 0f;
        if (keyboard[Key.X].isPressed) rollDir += 1f;
        if (keyboard[Key.Z].isPressed) rollDir -= 1f;
        if (rollDir != 0f)
        {
            ikTargetRot *= Quaternion.AngleAxis(rollDir * cartesianRotateSpeed * Time.fixedDeltaTime, Vector3.forward);
        }

        float wristYawDir = 0f;
        if (keyboard[Key.RightArrow].isPressed) wristYawDir += 1f;
        if (keyboard[Key.LeftArrow].isPressed) wristYawDir -= 1f;
        float wristPitchDir = 0f;
        if (keyboard[Key.UpArrow].isPressed) wristPitchDir += 1f;
        if (keyboard[Key.DownArrow].isPressed) wristPitchDir -= 1f;
        if (wristYawDir != 0f || wristPitchDir != 0f)
        {
            ikTargetRot *= Quaternion.AngleAxis(wristYawDir * cartesianRotateSpeed * Time.fixedDeltaTime, Vector3.up)
                          * Quaternion.AngleAxis(wristPitchDir * cartesianRotateSpeed * Time.fixedDeltaTime, Vector3.right);
        }

        Matrix4x4 baseWorld = Matrix4x4.TRS(ikBaseParent.position, ikBaseParent.rotation, Vector3.one);
        float[] current = DaVinciKinematics.GetCurrentTargets(ikChain);
        float[] solved = DaVinciKinematics.SolveIK(
            ikChain, current, baseWorld, tipLocalPos, tipLocalRot,
            ikTargetPos, ikTargetRot, ikIterationsPerStep, ikPositionGain, ikRotationGain);

        for (int i = 0; i < ikChain.Count; i++)
        {
            var drive = ikChain[i].body.xDrive;
            drive.target = solved[i];
            ikChain[i].body.xDrive = drive;
        }

        var (fkPos, _) = DaVinciKinematics.ForwardKinematics(ikChain, solved, baseWorld, tipLocalPos, tipLocalRot);
        lastFkPos = fkPos;
    }

    private void SyncIkTargetToCurrentPose()
    {
        Matrix4x4 baseWorld = Matrix4x4.TRS(ikBaseParent.position, ikBaseParent.rotation, Vector3.one);
        float[] current = DaVinciKinematics.GetCurrentTargets(ikChain);
        var (pos, rot) = DaVinciKinematics.ForwardKinematics(ikChain, current, baseWorld, tipLocalPos, tipLocalRot);
        ikTargetPos = pos;
        ikTargetRot = rot;
        lastFkPos = pos;
    }

    void OnGUI()
    {
        string modeLabel = mode == ControlMode.CartesianIK ? "Cartesian IK (tool-tip space)" : "Joint Space (direct jog)";
        GUI.Label(new Rect(10, 10, 700, 20), "Mode: " + modeLabel + "   [Tab to toggle]");

        if (mode == ControlMode.CartesianIK)
        {
            GUI.Label(new Rect(10, 30, 700, 20), "W/A/S/D/Q/E move tip   Arrows: wrist   Z/X roll   C/V jaw");
            float error = Vector3.Distance(lastFkPos, ikTargetPos);
            GUI.Label(new Rect(10, 50, 700, 20), $"target={ikTargetPos:F3}  tip(FK)={lastFkPos:F3}  |error|={error:F4} m");
        }
        else
        {
            GUI.Label(new Rect(10, 30, 700, 20), "W/S insertion   A/D yaw   Q/E pitch   Arrows: wrist   Z/X roll   C/V jaw");
        }
    }
}
