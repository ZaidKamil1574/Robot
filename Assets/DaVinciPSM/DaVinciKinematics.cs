using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.UrdfImporter;

namespace DaVinci
{
    /// One revolute/prismatic degree of freedom in a serial joint chain, carrying the static
    /// (non-moving) anchor frames Unity's URDF importer baked into the ArticulationBody at import
    /// time. Per Unity's ArticulationBody design, motion always happens about/along local +X of the
    /// anchor frame, regardless of the joint's original axis in the URDF file.
    public class JointFrame
    {
        public string jointName;
        public ArticulationBody body;
        public bool isPrismatic;
        public float lowerLimit; // degrees (revolute) or meters (prismatic) — same units as xDrive.target
        public float upperLimit;
        public Vector3 parentAnchorPos;
        public Quaternion parentAnchorRot;
        public Vector3 anchorPos;
        public Quaternion anchorRot;

        // Finite-difference step for the numeric Jacobian: ~0.5mm for prismatic, ~0.5deg for revolute.
        public float Epsilon => isPrismatic ? 0.0005f : 0.5f;
    }

    /// Explicit joint-chain kinematics for the PSM arm, independent of whatever pose the physics
    /// engine currently happens to be in: forward kinematics via homogeneous-transform (Matrix4x4)
    /// composition, and a numerical Jacobian-transpose inverse-kinematics solver for driving the
    /// tool tip in Cartesian space.
    public static class DaVinciKinematics
    {
        public static List<JointFrame> BuildChain(GameObject robotRoot, IReadOnlyList<string> orderedJointNames)
        {
            var lookup = new Dictionary<string, UrdfJoint>();
            foreach (var uj in robotRoot.GetComponentsInChildren<UrdfJoint>())
            {
                if (!string.IsNullOrEmpty(uj.jointName)) lookup[uj.jointName] = uj;
            }

            var chain = new List<JointFrame>();
            foreach (var name in orderedJointNames)
            {
                if (!lookup.TryGetValue(name, out var uj))
                {
                    Debug.LogWarning($"DaVinciKinematics: joint '{name}' not found while building chain.");
                    continue;
                }
                var ab = uj.GetComponent<ArticulationBody>();
                if (ab == null) continue;

                chain.Add(new JointFrame
                {
                    jointName = name,
                    body = ab,
                    isPrismatic = ab.jointType == ArticulationJointType.PrismaticJoint,
                    lowerLimit = ab.xDrive.lowerLimit,
                    upperLimit = ab.xDrive.upperLimit,
                    parentAnchorPos = ab.parentAnchorPosition,
                    parentAnchorRot = ab.parentAnchorRotation,
                    anchorPos = ab.anchorPosition,
                    anchorRot = ab.anchorRotation,
                });
            }
            return chain;
        }

        /// Finds the child-link Transform for a given URDF joint name (UrdfJoint lives on the link
        /// GameObject it belongs to). Used to read fixed offsets, e.g. the tool tip past the last
        /// actuated joint.
        public static Transform FindJointChildLink(GameObject robotRoot, string jointName)
        {
            foreach (var uj in robotRoot.GetComponentsInChildren<UrdfJoint>())
            {
                if (uj.jointName == jointName) return uj.transform;
            }
            return null;
        }

        public static float[] GetCurrentTargets(List<JointFrame> chain)
        {
            var values = new float[chain.Count];
            for (int i = 0; i < chain.Count; i++) values[i] = chain[i].body.xDrive.target;
            return values;
        }

        /// Composes the homogeneous-transform chain from `baseWorld` (world pose of the first
        /// joint's parent link) through each joint's static anchor frames and its motion about
        /// local +X, ending at a fixed tip offset expressed relative to the chain's last link.
        public static (Vector3 pos, Quaternion rot) ForwardKinematics(
            List<JointFrame> chain, float[] jointValues, Matrix4x4 baseWorld,
            Vector3 tipLocalPos, Quaternion tipLocalRot)
        {
            Matrix4x4 t = baseWorld;
            for (int i = 0; i < chain.Count; i++)
            {
                JointFrame j = chain[i];
                Matrix4x4 parentAnchor = Matrix4x4.TRS(j.parentAnchorPos, j.parentAnchorRot, Vector3.one);
                Matrix4x4 motion = j.isPrismatic
                    ? Matrix4x4.Translate(Vector3.right * jointValues[i])
                    : Matrix4x4.Rotate(Quaternion.AngleAxis(jointValues[i], Vector3.right));
                Matrix4x4 childAnchorInv = Matrix4x4.TRS(j.anchorPos, j.anchorRot, Vector3.one).inverse;
                t *= parentAnchor * motion * childAnchorInv;
            }
            t *= Matrix4x4.TRS(tipLocalPos, tipLocalRot, Vector3.one);

            Vector3 pos = t.GetColumn(3);
            return (pos, t.rotation);
        }

        /// Iterative Jacobian-transpose resolved-rate IK: each pass estimates the 6xN task-space
        /// Jacobian by finite differences (perturbing a hypothetical joint vector, never touching
        /// physics), then nudges joints along J^T * error — gradient descent on task-space error.
        /// Simpler and more stable near singularities than a damped pseudo-inverse, at the cost of
        /// slower convergence; acceptable here since this runs every fixed timestep against a
        /// smoothly moving target rather than as a single one-shot solve. Joint limits are enforced
        /// every iteration, not just on the final result, so the solver can't walk through a limit
        /// partway through convergence. Known limitation: no null-space/active-constraint handling,
        /// so a joint sitting exactly at its limit can locally stall convergence on that axis.
        public static float[] SolveIK(
            List<JointFrame> chain, float[] jointValues, Matrix4x4 baseWorld,
            Vector3 tipLocalPos, Quaternion tipLocalRot,
            Vector3 targetPos, Quaternion targetRot,
            int iterations, float posGain, float rotGain)
        {
            int n = chain.Count;
            float[] q = (float[])jointValues.Clone();

            for (int iter = 0; iter < iterations; iter++)
            {
                var (curPos, curRot) = ForwardKinematics(chain, q, baseWorld, tipLocalPos, tipLocalRot);
                Vector3 posError = targetPos - curPos;
                Vector3 rotError = QuaternionError(targetRot, curRot);

                var dq = new float[n];
                for (int i = 0; i < n; i++)
                {
                    float eps = chain[i].Epsilon;
                    float[] qPerturbed = (float[])q.Clone();
                    qPerturbed[i] += eps;
                    var (pPos, pRot) = ForwardKinematics(chain, qPerturbed, baseWorld, tipLocalPos, tipLocalRot);

                    Vector3 dPos = (pPos - curPos) / eps;
                    Vector3 dRot = QuaternionError(pRot, curRot) / eps;

                    dq[i] = Vector3.Dot(dPos, posError) * posGain + Vector3.Dot(dRot, rotError) * rotGain;
                }

                for (int i = 0; i < n; i++)
                {
                    q[i] = Mathf.Clamp(q[i] + dq[i], chain[i].lowerLimit, chain[i].upperLimit);
                }
            }

            return q;
        }

        /// Small-angle (axis * radians) approximation of the rotation from `from` to `to` — the log
        /// map of the relative quaternion. Standard representation for velocity-/error-based
        /// orientation control.
        private static Vector3 QuaternionError(Quaternion to, Quaternion from)
        {
            Quaternion delta = to * Quaternion.Inverse(from);
            delta.ToAngleAxis(out float angleDeg, out Vector3 axis);
            if (float.IsNaN(axis.x) || float.IsInfinity(angleDeg)) return Vector3.zero;
            if (angleDeg > 180f) angleDeg -= 360f;
            return axis.normalized * (angleDeg * Mathf.Deg2Rad);
        }
    }
}
