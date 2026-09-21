using System;
using System.Collections.Generic;
using UnityEngine;
using static UnityEngine.Mathf;

namespace Preliy.Flange
{
    /// <summary>
    /// Kinematic model and deterministic analytic IK solver for KUKA LBR iiwa 14 R820.
    ///
    /// Unity-validated kinematic convention:
    ///   A1 = +Y, A2 = -Z, A3 = +Y, A4 = +Z,
    ///   A5 = +Y, A6 = -Z, A7 = +Y.
    ///
    /// Link offsets used by the validated Unity chain:
    ///   [0.15, 0.19, 0.22, 0.18, 0.22, 0.18, 0.09] m.
    ///
    /// IK is deterministic and analytic. The 7R redundancy is sampled on a configurable
    /// one-dimensional grid (default 0.5 deg), while q2=0 and q4=0 singular branches are
    /// handled with their dedicated analytic parametrizations.
    /// </summary>
    [AddComponentMenu("Robotics/Robot 7R LBR iiwa 14 R820")]
    [DisallowMultipleComponent]
    [SelectionBase]
    public class Robot7RLBRIIWA : Robot
    {
        private const int FRAME_COUNT = 9; // Base + 7 joints + Flange
        private const int JOINT_COUNT = 7;

        private const float Link5Plus6 = 0.40f;
        private const float BaseY = 0.34f;
        private const float Link7 = 0.09f;

        private const float AnalyticEpsilon = 1e-6f;
        private const float ReductionTolerance = 2e-5f;
        private const float PositionValidationTolerance = 1e-5f;
        private const float RotationValidationToleranceDeg = 0.001f;
        private const float DedupToleranceDeg = 0.001f;
        private const float Q4SingularityCosTolerance = 1e-7f;
        private const float Q4NearSingularitySinTolerance = 0.02f;

        [Tooltip("Sampling step in degrees for the 7R redundancy parameter. 0.5 is the validated default.")]
        [SerializeField]
        private float _redundancyStepDegrees = 0.5f;

        // Exact Unity chain convention established by the FK diagnostic.
        private static readonly Vector3[] JointAxes =
        {
            Vector3.up,
            Vector3.back,
            Vector3.up,
            Vector3.forward,
            Vector3.up,
            Vector3.back,
            Vector3.up
        };

        // Exact Unity link offsets established by the chain diagnostic.
        private static readonly Vector3[] JointOrigins =
        {
            new Vector3(0f, 0.15f, 0f),
            new Vector3(0f, 0.19f, 0f),
            new Vector3(0f, 0.22f, 0f),
            new Vector3(0f, 0.18f, 0f),
            new Vector3(0f, 0.22f, 0f),
            new Vector3(0f, 0.18f, 0f),
            new Vector3(0f, 0.09f, 0f)
        };

        private static readonly Vector2[] JointLimits =
        {
            new Vector2(-170f, 170f),
            new Vector2(-120f, 120f),
            new Vector2(-170f, 170f),
            new Vector2(-120f, 120f),
            new Vector2(-170f, 170f),
            new Vector2(-120f, 120f),
            new Vector2(-175f, 175f)
        };

        private void Reset()
        {
            _frames = new List<Frame>();
            for (var i = 0; i < FRAME_COUNT; i++)
                _frames.Add(null);

            _joints = new List<TransformJoint>();
            for (var i = 0; i < JOINT_COUNT; i++)
                _joints.Add(null);
        }

#if UNITY_EDITOR
        [ContextMenu("Create LBR iiwa Hierarchy", false, 500)]
        public void CreateDefaultHierarchy()
        {
            _frames = new List<Frame>();
            _joints = new List<TransformJoint>();

            var baseFrame = transform.GetOrCreate("Base").CreateMeshPlaceholder();
            _frames.Add(baseFrame);

            var parent = baseFrame.transform;
            for (var i = 0; i < JOINT_COUNT; i++)
            {
                var frame = parent.GetOrCreate("Joint_" + (i + 1)).CreateMeshPlaceholder();
                var joint = frame.GetComponent<TransformJoint>();
                if (joint == null)
                    joint = frame.gameObject.AddComponent<TransformJoint>();

                var frameConfig = new FrameConfig(0f, 0f, JointOrigins[i].y, 0f, "Joint_" + (i + 1));
                frame.Config = frameConfig;
                joint.Config = new JointConfig(
                    TransformJoint.JointType.Rotation,
                    JointLimits[i],
                    0f,
                    1f,
                    GetSpeedMax(i),
                    1000f,
                    "Joint_" + (i + 1));
                joint.ConfigureCustomAxis(JointAxes[i], Vector3.zero, true);

                _joints.Add(joint);
                parent = frame.transform;
                _frames.Add(frame);
            }

            var flangeFrame = parent.GetOrCreate("Flange");
            _frames.Add(flangeFrame);

            for (var i = 0; i < _joints.Count; i++)
                _joints[i].Position.Value = 0f;
        }
#endif

        public float RedundancyStepDegrees
        {
            get { return _redundancyStepDegrees; }
            set { _redundancyStepDegrees = Mathf.Max(0.01f, value); }
        }

        public override Matrix4x4 ComputeForward()
        {
            return ComputeForward(_joints.GetJointValues());
        }

        public override Matrix4x4 ComputeForward(float[] value)
        {
            if (value == null || value.Length != JOINT_COUNT)
                throw new ArgumentException("LBR iiwa requires exactly 7 joint values.", nameof(value));

            var result = Matrix4x4.identity;
            for (var i = 0; i < JOINT_COUNT; i++)
            {
                result *= Matrix4x4.Translate(JointOrigins[i]);
                result *= Matrix4x4.Rotate(Quaternion.AngleAxis(value[i], JointAxes[i]));
            }

            return result;
        }

        public override IKSolution ComputeInverse(Matrix4x4 target, Configuration configuration, SolutionIgnoreMask ignoreMask)
        {
            var current = _joints != null && _joints.Count == JOINT_COUNT
                ? _joints.GetJointValues()
                : new float[JOINT_COUNT];

            IKSolution bestSolution = null;
            var bestScore = float.MaxValue;
            var hadCandidate = false;

            EnumerateAnalyticSolutions(target, delegate(float[] q, int branchIndex)
            {
                hadCandidate = true;
                var branchConfiguration = configuration;
                branchConfiguration.SetIndex(branchIndex);

                var solution = CreateSolution(q, branchConfiguration, ignoreMask);
                if (!solution.IsValid)
                    return;

                var score = ScoreSolution(solution, current);
                if (bestSolution == null || score < bestScore)
                {
                    bestSolution = solution;
                    bestScore = score;
                }
            });

            if (bestSolution != null)
                return bestSolution;

            if (!hadCandidate)
                return new IKSolution("LBR iiwa analytic IK: target is unreachable for the current kinematic model.");

            return new IKSolution("LBR iiwa analytic IK: candidates were found but all failed solution validation.");
        }

        public override List<IKSolution> ComputeInverse(Matrix4x4 target, bool turn, SolutionIgnoreMask ignoreMask)
        {
            var solutions = new List<IKSolution>();

            EnumerateAnalyticSolutions(target, delegate(float[] q, int branchIndex)
            {
                var configuration = new Configuration(0, 0, 0, branchIndex);
                var solution = CreateSolution(q, configuration, ignoreMask);
                if (solution.IsValid)
                    solutions.Add(solution);
            });

            // The manifold is intentionally sampled, not collapsed to a single IK point.
            // Remove only numerical duplicates created by overlapping singular branches.
            return DeduplicateSolutions(solutions);
        }

        public override int GetConfigurationIndex(float[] jointValues)
        {
            if (jointValues == null || jointValues.Length < JOINT_COUNT)
                return 0;

            var index = 0;
            if (jointValues[3] >= 0f) index |= 1; // q4 branch
            if (jointValues[2] >= 0f) index |= 2; // q3 sign branch
            if (jointValues[5] >= 0f) index |= 4; // wrist q6 sign branch
            return index;
        }

        public override int GetConfigurationIndex()
        {
            return GetConfigurationIndex(_joints.GetJointValues());
        }

#if UNITY_EDITOR
        public override void DrawDebugGizmos()
        {
            if (_joints == null || _joints.Count != JOINT_COUNT)
                return;

            for (var i = 0; i < JOINT_COUNT; i++)
            {
                var joint = _joints[i];
                if (joint == null)
                    continue;

                Gizmos.matrix = joint.transform.localToWorldMatrix;
                GizmosUtils.DrawFrameOffset(joint.Frame, Color.white, Color.gray, 0.05f);
            }
        }
#endif

        private IKSolution CreateSolution(float[] q, Configuration configuration, SolutionIgnoreMask ignoreMask)
        {
            var values = new float[JOINT_COUNT];
            for (var i = 0; i < JOINT_COUNT; i++)
            {
                if (float.IsNaN(q[i]) || float.IsInfinity(q[i]))
                    return IKSolution.IKSolutionNaN;

                values[i] = (float)System.Math.Round(q[i], 5);
            }

            ClampToLimits(values);

            var target = new JointTarget(values);
            var solution = new IKSolution(target, configuration);
            solution.Validate(this);
            return solution;
        }

        private void EnumerateAnalyticSolutions(Matrix4x4 target, Action<float[], int> emit)
        {
            if (emit == null)
                return;

            var step = Mathf.Max(0.01f, _redundancyStepDegrees);
            var targetRotation = target.rotation;
            var targetPosition = target.GetPosition();
            var targetY = targetRotation * Vector3.up;

            // Remove the final link d7 along the target Y axis.
            var W = targetPosition - Link7 * targetY;

            var X = W.x / Link5Plus6;
            var H = (W.y - BaseY) / Link5Plus6;
            var Z = W.z / Link5Plus6;

            // Exact generalized reduction:
            //   c4 = (X^2 + H^2 + Z^2)/2 - 1.
            var c4 = 0.5f * (X * X + H * H + Z * Z) - 1f;
            if (c4 < -1f - ReductionTolerance || c4 > 1f + ReductionTolerance)
                return;

            c4 = Clamp(c4, -1f, 1f);
            var q4Abs = Acos(c4) * Rad2Deg;

            // q4 has physical limits [-120,+120].
            if (q4Abs > JointLimits[3].y + ReductionTolerance)
                return;

            // Unity float FK of an exact q4=0 target can produce a tiny residual
            // in the scalar reduction, typically corresponding to a q4Abs of a few
            // hundredths of a degree. Detect that as the q4=0 singular manifold.
            // Keep the generic branch as a fallback for genuinely near-singular
            // nonzero q4 targets.
            if (Abs(1f - c4) <= Q4SingularityCosTolerance)
            {
                EnumerateQ4ZeroSingularity(
                    target,
                    X,
                    H,
                    Z,
                    step,
                    emit);
            }

            if (q4Abs <= 0.0001f)
                return;

            EmitQ4Branch(target, X, H, Z, -q4Abs, step, emit, 0);
            EmitQ4Branch(target, X, H, Z, q4Abs, step, emit, 1);
        }

        private void EmitQ4Branch(
            Matrix4x4 target,
            float X,
            float H,
            float Z,
            float q4,
            float step,
            Action<float[], int> emit,
            int q4Branch)
        {
            var q4Rad = q4 * Deg2Rad;
            var c4 = Cos(q4Rad);
            var s4 = Sin(q4Rad);

            if (Abs(s4) < AnalyticEpsilon)
                return;

            // Near q4 = 0 the generic formulation is numerically ill-conditioned because
            // c3 = (H - c2(c4+1)) / (s2*s4) divides by a very small s4. In that region
            // parameterize the redundancy with q3 instead, then solve q2 analytically.
            if (Abs(s4) <= Q4NearSingularitySinTolerance)
            {
                EmitNearQ4Branch(target, X, H, Z, q4, step, emit, q4Branch);
                return;
            }

            // The generic branch excludes q2=0 because the formula for c3 divides by s2.
            foreach (var q2 in BuildGrid(JointLimits[1].x, JointLimits[1].y, step))
            {
                var s2 = Sin(q2 * Deg2Rad);
                if (Abs(s2) < AnalyticEpsilon)
                    continue;

                var c2 = Cos(q2 * Deg2Rad);
                var c3 = (H - c2 * (c4 + 1f)) / (s2 * s4);
                if (Abs(c3) > 1f + ReductionTolerance)
                    continue;

                c3 = Clamp(c3, -1f, 1f);
                var s3Abs = Sqrt(Max(0f, 1f - c3 * c3));

                for (var s3Sign = -1; s3Sign <= 1; s3Sign += 2)
                {
                    var s3 = s3Sign * s3Abs;

                    var A = -c2 * c3 * s4 + (c4 + 1f) * s2;
                    var B = s3 * s4;
                    var den = A * A + B * B;
                    if (den < AnalyticEpsilon)
                        continue;

                    var c1 = (A * X + B * Z) / den;
                    var s1 = (B * X - A * Z) / den;

                    var norm = Sqrt(c1 * c1 + s1 * s1);
                    if (norm < AnalyticEpsilon)
                        continue;

                    c1 /= norm;
                    s1 /= norm;

                    var q1 = Atan2(s1, c1) * Rad2Deg;
                    var q3 = Atan2(s3, c3) * Rad2Deg;

                    TryEmitFullPose(
                        target,
                        q1, q2, q3, q4,
                        q4Branch * 4 + (s3Sign > 0 ? 2 : 0),
                        emit);
                }
            }

            // Dedicated q2=0 singular branch. q3 is now the redundancy parameter and
            // q1+q3 is fixed by the target position.
            if (Abs(H - (c4 + 1f)) <= ReductionTolerance)
            {
                var cosPhi = -X / s4;
                var sinPhi = Z / s4;
                var normPhi = Sqrt(cosPhi * cosPhi + sinPhi * sinPhi);

                if (normPhi >= AnalyticEpsilon)
                {
                    cosPhi /= normPhi;
                    sinPhi /= normPhi;
                    var phi = Atan2(sinPhi, cosPhi) * Rad2Deg;

                    foreach (var q3 in BuildGrid(JointLimits[2].x, JointLimits[2].y, step))
                    {
                        var q1 = WrapDeg(phi - q3);
                        TryEmitFullPose(
                            target,
                            q1, 0f, q3, q4,
                            8 + q4Branch,
                            emit);
                    }
                }
            }
        }

        private void EmitNearQ4Branch(
            Matrix4x4 target,
            float X,
            float H,
            float Z,
            float q4,
            float step,
            Action<float[], int> emit,
            int q4Branch)
        {
            var q4Rad = q4 * Deg2Rad;
            var c4 = Cos(q4Rad);
            var s4 = Sin(q4Rad);
            var a = c4 + 1f;

            // For a sampled q3 we have:
            //   H = a*cos(q2) + s4*cos(q3)*sin(q2)
            // which is solved as R*cos(q2-alpha)=H.
            foreach (var q3 in BuildGrid(JointLimits[2].x, JointLimits[2].y, step))
            {
                var q3Rad = q3 * Deg2Rad;
                var c3 = Cos(q3Rad);
                var s3 = Sin(q3Rad);

                var k = s4 * c3;
                var radius = Sqrt(a * a + k * k);
                if (radius < AnalyticEpsilon || Abs(H) > radius + ReductionTolerance)
                    continue;

                var alpha = Atan2(k, a);
                var gamma = Acos(Clamp(H / radius, -1f, 1f));

                for (var root = -1; root <= 1; root += 2)
                {
                    var q2 = alpha + root * gamma;
                    var q2Deg = q2 * Rad2Deg;
                    if (!WithinLimit(q2Deg, JointLimits[1]))
                        continue;

                    var c2 = Cos(q2);
                    var s2 = Sin(q2);

                    var A = -c2 * c3 * s4 + a * s2;
                    var B = s3 * s4;
                    var den = A * A + B * B;
                    if (den < AnalyticEpsilon)
                        continue;

                    var c1 = (A * X + B * Z) / den;
                    var s1 = (B * X - A * Z) / den;
                    var norm = Sqrt(c1 * c1 + s1 * s1);
                    if (norm < AnalyticEpsilon)
                        continue;

                    c1 /= norm;
                    s1 /= norm;

                    var q1 = Atan2(s1, c1) * Rad2Deg;

                    TryEmitFullPose(
                        target,
                        q1, q2Deg, q3, q4,
                        24 + q4Branch * 2 + (root > 0 ? 1 : 0),
                        emit);
                }
            }
        }

        private void EnumerateQ4ZeroSingularity(
            Matrix4x4 target,
            float X,
            float H,
            float Z,
            float step,
            Action<float[], int> emit)
        {
            // For q4=0:
            //   c2 = H/2,
            //   q1 = atan2(-Z/(2 s2), X/(2 s2)),
            //   q3 is the redundancy parameter.
            var c2 = H * 0.5f;
            if (Abs(c2) > 1f + ReductionTolerance)
                return;

            c2 = Clamp(c2, -1f, 1f);
            var s2Abs = Sqrt(Max(0f, 1f - c2 * c2));

            // q2=0 and q4=0 simultaneously: position no longer fixes q1 or q3.
            // Use three fixed q1 values whose q1+q3 coverage spans [-180,+180] and
            // recover the wrist analytically for each sampled q3.
            if (s2Abs < AnalyticEpsilon)
            {
                if (Abs(X) > ReductionTolerance || Abs(Z) > ReductionTolerance ||
                    Abs(H - 2f) > ReductionTolerance)
                    return;

                var fixedQ1 = new[] { -10f, 0f, 10f };
                foreach (var q1 in fixedQ1)
                {
                    foreach (var q3 in BuildGrid(JointLimits[2].x, JointLimits[2].y, step))
                    {
                        TryEmitFullPose(target, q1, 0f, q3, 0f, 16 + (q1 > 0f ? 2 : q1 < 0f ? 1 : 0), emit);
                    }
                }

                return;
            }

            for (var sign = -1; sign <= 1; sign += 2)
            {
                var s2 = sign * s2Abs;
                var q2 = Atan2(s2, c2) * Rad2Deg;
                if (!WithinLimit(q2, JointLimits[1]))
                    continue;

                var q1 = Atan2(-Z / (2f * s2), X / (2f * s2)) * Rad2Deg;

                foreach (var q3 in BuildGrid(JointLimits[2].x, JointLimits[2].y, step))
                {
                    TryEmitFullPose(target, q1, q2, q3, 0f, 20 + (sign > 0 ? 1 : 0), emit);
                }
            }
        }

        private void TryEmitFullPose(
            Matrix4x4 target,
            float q1,
            float q2,
            float q3,
            float q4,
            int branchBase,
            Action<float[], int> emit)
        {
            q1 = WrapDeg(q1);
            q2 = WrapDeg(q2);
            q3 = WrapDeg(q3);
            q4 = WrapDeg(q4);

            if (!WithinLimit(q1, JointLimits[0]) ||
                !WithinLimit(q2, JointLimits[1]) ||
                !WithinLimit(q3, JointLimits[2]) ||
                !WithinLimit(q4, JointLimits[3]))
                return;

            var r4 = Quaternion.AngleAxis(q1, Vector3.up)
                   * Quaternion.AngleAxis(-q2, Vector3.forward)
                   * Quaternion.AngleAxis(q3, Vector3.up)
                   * Quaternion.AngleAxis(q4, Vector3.forward);

            var rw = Quaternion.Inverse(r4) * target.rotation;
            var wristCandidates = DecomposeWrist(rw);

            for (var i = 0; i < wristCandidates.Count; i++)
            {
                var wrist = wristCandidates[i];
                var q = new float[JOINT_COUNT]
                {
                    q1, q2, q3, q4,
                    wrist.x, wrist.y, wrist.z
                };

                if (!WithinLimits(q))
                    continue;

                var fk = ComputeForward(q);
                var pe = Vector3.Distance(fk.GetPosition(), target.GetPosition());
                var re = Quaternion.Angle(fk.rotation, target.rotation);

                if (pe > PositionValidationTolerance || re > RotationValidationToleranceDeg)
                    continue;

                emit(q, branchBase + i);
            }
        }

        private static List<Vector3> DecomposeWrist(Quaternion rw)
        {
            var m = Matrix4x4.Rotate(rw);
            var c6 = Clamp(m[1, 1], -1f, 1f);
            var s6Abs = Sqrt(Max(0f, 1f - c6 * c6));
            var result = new List<Vector3>(2);

            if (s6Abs > AnalyticEpsilon)
            {
                for (var sign = -1; sign <= 1; sign += 2)
                {
                    var s6 = sign * s6Abs;
                    var q5 = Atan2(-m[2, 1] / s6, m[0, 1] / s6) * Rad2Deg;
                    var q6 = Atan2(s6, c6) * Rad2Deg;
                    var q7 = Atan2(-m[1, 2] / s6, -m[1, 0] / s6) * Rad2Deg;
                    result.Add(new Vector3(WrapDeg(q5), WrapDeg(q6), WrapDeg(q7)));
                }
            }
            else if (c6 >= 0f)
            {
                // q6 = 0: only q5+q7 is observable. Choose q7=0 canonically.
                var q = Atan2(m[0, 2], m[0, 0]) * Rad2Deg;
                result.Add(new Vector3(WrapDeg(q), 0f, 0f));
            }
            else
            {
                // q6 = 180: only one Y-combination remains observable. Canonical q7=0.
                var q = Atan2(-m[0, 2], -m[0, 0]) * Rad2Deg;
                result.Add(new Vector3(WrapDeg(q), 180f, 0f));
            }

            return result;
        }

        private static List<float> BuildGrid(float min, float max, float step)
        {
            var result = new List<float>();
            var s = Mathf.Max(0.01f, step);
            var value = min;

            while (value <= max + 1e-4f)
            {
                result.Add(Mathf.Min(value, max));
                value += s;
            }

            if (result.Count == 0 || Abs(result[result.Count - 1] - max) > 1e-4f)
                result.Add(max);

            return result;
        }

        private static bool WithinLimit(float value, Vector2 limit)
        {
            return value >= limit.x - 1e-4f && value <= limit.y + 1e-4f;
        }

        private static bool WithinLimits(float[] q)
        {
            for (var i = 0; i < JOINT_COUNT; i++)
            {
                if (!WithinLimit(q[i], JointLimits[i]))
                    return false;
            }

            return true;
        }

        private static float WrapDeg(float value)
        {
            var result = (value + 180f) % 360f;
            if (result < 0f)
                result += 360f;
            return result - 180f;
        }

        private static void ClampToLimits(float[] q)
        {
            for (var i = 0; i < JOINT_COUNT; i++)
                q[i] = Clamp(q[i], JointLimits[i].x, JointLimits[i].y);
        }

        private static float ScoreSolution(IKSolution solution, float[] reference)
        {
            if (solution == null || !solution.IsValid ||
                solution.JointTarget == null || solution.JointTarget.RobJoint == null ||
                solution.JointTarget.RobJoint.Value == null)
                return float.MaxValue;

            var candidate = solution.JointTarget.RobJoint.Value;
            if (reference == null || reference.Length != JOINT_COUNT || candidate.Length != JOINT_COUNT)
                return float.MaxValue;

            var motionScore = 0f;
            var limitRisk = 0f;

            for (var i = 0; i < JOINT_COUNT; i++)
            {
                var delta = DeltaAngle(reference[i], candidate[i]);
                var minLimit = JointLimits[i].x;
                var maxLimit = JointLimits[i].y;
                var halfRange = Max(0.001f, (maxLimit - minLimit) * 0.5f);

                var normalizedMotion = delta / halfRange;
                motionScore += normalizedMotion * normalizedMotion;

                var distanceToLimit = Min(candidate[i] - minLimit, maxLimit - candidate[i]);
                var normalizedMargin = Clamp01(distanceToLimit / halfRange);

                const float limitMarginFloor = 0.05f;
                if (normalizedMargin < limitMarginFloor)
                    limitRisk += (limitMarginFloor - normalizedMargin) * 1000f;
                else
                    limitRisk += 1f / (normalizedMargin * normalizedMargin);
            }

            // First minimize motion from the current robot posture. Limit safety is
            // only a secondary preference when several solutions are otherwise close.
            return motionScore * 10000f + limitRisk;
        }

        private static List<IKSolution> DeduplicateSolutions(List<IKSolution> source)
        {
            var result = new List<IKSolution>();

            for (var i = 0; i < source.Count; i++)
            {
                var candidate = source[i];
                var duplicate = false;

                for (var j = 0; j < result.Count && !duplicate; j++)
                {
                    var a = candidate.JointTarget.RobJoint.Value;
                    var b = result[j].JointTarget.RobJoint.Value;
                    var distance = 0f;
                    for (var k = 0; k < JOINT_COUNT; k++)
                        distance += Abs(DeltaAngle(a[k], b[k]));

                    duplicate = distance < DedupToleranceDeg;
                }

                if (!duplicate)
                    result.Add(candidate);
            }

            return result;
        }

        private static float GetSpeedMax(int index)
        {
            switch (index)
            {
                case 0: return 85f;
                case 1: return 85f;
                case 2: return 100f;
                case 3: return 75f;
                case 4: return 130f;
                case 5: return 135f;
                case 6: return 135f;
                default: return 100f;
            }
        }
    }
}
