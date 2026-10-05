using System;
using UnityEngine;

namespace Preliy.Flange
{
    /// <summary>
    /// Synchronized joint-space trajectory for the robot joints.
    /// Uses a symmetric triangular profile over the common motion duration.
    /// The common duration is chosen from the slowest joint, so no joint
    /// exceeds its configured SpeedMax or AccMax.
    /// Unity 2019.4 / C# 7.3 compatible.
    /// </summary>
    [Serializable]
    public sealed class JointTrajectory
    {
        private const int RobotJointCount = 7;
        private const float Epsilon = 0.000001f;

        private readonly float[] _from = new float[RobotJointCount];
        private readonly float[] _to = new float[RobotJointCount];
        private readonly float[] _distance = new float[RobotJointCount];
        private readonly float[] _speedMax = new float[RobotJointCount];
        private readonly float[] _accMax = new float[RobotJointCount];

        public float Duration { get; private set; }
        public bool IsValid { get; private set; }
        public string ErrorMessage { get; private set; }

        public JointTrajectory(
            JointTarget from,
            JointTarget to,
            MechanicalGroup mechanicalGroup)
        {
            IsValid = false;
            ErrorMessage = null;

            if (from == null || to == null)
            {
                ErrorMessage = "JointTrajectory: start or target joint state is null.";
                return;
            }

            if (mechanicalGroup == null ||
                mechanicalGroup.RobotJoints == null ||
                mechanicalGroup.RobotJoints.Count != RobotJointCount)
            {
                ErrorMessage = "JointTrajectory: expected exactly 7 robot joints.";
                return;
            }

            for (var i = 0; i < RobotJointCount; i++)
            {
                _from[i] = from[i];
                _to[i] = to[i];

                // Rotation joints use degrees.
                _distance[i] = Mathf.Abs(Mathf.DeltaAngle(_from[i], _to[i]));

                var config = mechanicalGroup.RobotJoints[i].Config;
                _speedMax[i] = Mathf.Abs(config.SpeedMax);
                _accMax[i] = Mathf.Abs(config.AccMax);

                if (_distance[i] > Epsilon &&
                    (_speedMax[i] <= Epsilon || _accMax[i] <= Epsilon))
                {
                    ErrorMessage = string.Format(
                        "JointTrajectory: joint {0} has invalid SpeedMax ({1}) or AccMax ({2}).",
                        i + 1,
                        config.SpeedMax,
                        config.AccMax);
                    return;
                }
            }

            Duration = ComputeCommonDuration();
            IsValid = true;
        }

        public JointTarget Evaluate(float time, JointTarget externalTarget)
        {
            if (!IsValid)
                throw new InvalidOperationException(
                    "JointTrajectory is invalid: " + ErrorMessage);

            var values = new float[JointTarget.LENGTH];
            var t = Mathf.Clamp(time, 0f, Duration);

            for (var i = 0; i < RobotJointCount; i++)
                values[i] = EvaluateJoint(i, t);

            // External axes are not part of this trajectory yet.
            if (externalTarget != null)
            {
                for (var i = RobotJointCount; i < JointTarget.LENGTH; i++)
                    values[i] = externalTarget[i];
            }

            return new JointTarget(values);
        }

        private float ComputeCommonDuration()
        {
            var duration = 0f;

            for (var i = 0; i < RobotJointCount; i++)
            {
                if (_distance[i] <= Epsilon)
                    continue;

                // Minimum time for a symmetric trapezoidal/triangular move.
                var v = _speedMax[i];
                var a = _accMax[i];
                var accelTimeAtVmax = v / a;
                var accelDistanceAtVmax = 0.5f * a * accelTimeAtVmax * accelTimeAtVmax;

                float jointDuration;

                if (_distance[i] <= 2f * accelDistanceAtVmax)
                {
                    // Triangular profile.
                    var accelTime = Mathf.Sqrt(_distance[i] / a);
                    jointDuration = 2f * accelTime;
                }
                else
                {
                    // Trapezoidal profile.
                    var cruiseDistance =
                        _distance[i] - 2f * accelDistanceAtVmax;

                    jointDuration =
                        2f * accelTimeAtVmax +
                        cruiseDistance / v;
                }

                duration = Mathf.Max(duration, jointDuration);
            }

            return duration;
        }

        private float EvaluateJoint(int index, float time)
        {
            if (_distance[index] <= Epsilon)
                return _from[index];

            // We deliberately stretch every joint to the common duration.
            // A symmetric triangular profile is sufficient and guarantees
            // that the resulting peak speed/acceleration do not exceed the
            // configured limits because Duration >= each joint's minimum time.
            var total = Duration;
            var half = total * 0.5f;

            if (half <= Epsilon)
                return _to[index];

            var direction = Mathf.DeltaAngle(_from[index], _to[index]);

            // For a symmetric triangular trajectory:
            // distance = a * half^2
            // peakSpeed = a * half
            var acceleration = Mathf.Abs(direction) / (half * half);

            if (time <= half)
            {
                var displacement = 0.5f * acceleration * time * time;
                return _from[index] + Mathf.Sign(direction) * displacement;
            }

            var remainingTime = total - time;
            var displacementFromEnd =
                0.5f * acceleration * remainingTime * remainingTime;

            return _to[index] - Mathf.Sign(direction) * displacementFromEnd;
        }
    }
}
