using NUnit.Framework;
using UnityEngine;

namespace Preliy.Flange.Editor.Tests
{
    public class TestRobot7RLBRIIWA
    {
        private GameObject _gameObject;
        private Robot7RLBRIIWA _robot;

        private readonly float[] _jointValues = { 10f, -20f, 30f, 40f, -50f, 20f, 60f };

        [SetUp]
        public void SetUp()
        {
            _gameObject = new GameObject("Test LBR iiwa 14 R820");
            _robot = _gameObject.AddComponent<Robot7RLBRIIWA>();
            _robot.CreateDefaultHierarchy();
        }

        [TearDown]
        public void TearDown()
        {
            Object.DestroyImmediate(_gameObject);
        }

        [Test]
        public void HasSevenRobotJoints()
        {
            Assert.AreEqual(7, _robot.Joints.Count);
        }

        [Test]
        public void ForwardKinematicsMatchesHierarchy()
        {
            _robot.JointValue = new JointTarget(_jointValues);

            var actualFromHierarchy = _robot.ComputeForward();
            var actualFromValues = _robot.ComputeForward(_jointValues);

            AssertExtension.AssertEqualMatrix(actualFromHierarchy, actualFromValues, 1e-5f);
        }

        [Test]
        public void InverseKinematicsReachesForwardPose()
        {
            var target = _robot.ComputeForward(_jointValues);

            _robot.JointValue = new JointTarget(0f, 0f, 0f, 0f, 0f, 0f, 0f);
            var solution = _robot.ComputeInverse(target, Configuration.Default, SolutionIgnoreMask.All);

            Assert.IsTrue(solution.IsValid, solution.Exception == null ? "IK solution is invalid." : solution.Exception.Message);

            var solvedPose = _robot.ComputeForward(solution.JointTarget.RobJoint.Value);
            Assert.Less(Vector3.Distance(target.GetPosition(), solvedPose.GetPosition()), 0.002f);
            Assert.Less(Quaternion.Angle(target.rotation, solvedPose.rotation), 0.5f);
        }
    }
}
