using System;
using UnityEngine;
using Preliy.Flange;

namespace Preliy.Flange.Orchestration
{
    /// <summary>
    /// Muove il robot fino a una posa cartesiana riferita alla flangia o al TCP attivo.
    /// L'IK viene risolta quando lo stato entra, usando lo stato articolare corrente.
    /// </summary>
    public sealed class MoveToPoseState : IRobotState
    {
        public string Label { get; }
        public string ErrorMessage { get; private set; }

        private readonly CartesianTarget _target;
        private readonly int _frame;
        private readonly RobotPoseTargetKind _targetKind;
        private readonly SolutionIgnoreMask _ignoreMask;

        private JointTarget _startJoint;
        private JointTarget _endJoint;
        private JointTrajectory _trajectory;
        private float _elapsed;

        public MoveToPoseState(
            CartesianTarget target,
            int frame,
            SolutionIgnoreMask ignoreMask,
            string label = "Move",
            RobotPoseTargetKind targetKind = RobotPoseTargetKind.ActiveToolTcp)
        {
            if (target == null) throw new ArgumentNullException(nameof(target));

            _target = target.Clone();
            _frame = frame;
            _targetKind = targetKind;
            _ignoreMask = ignoreMask;
            Label = label;
        }

        public void Enter(Controller controller)
        {
            _elapsed = 0f;
            ErrorMessage = null;
            _trajectory = null;

            _startJoint = controller.MechanicalGroup.JointState.Clone();

            var target = _target.Clone();
            target.ExtJoint = _startJoint.ExtJoint;
            var solverToolIndex = _targetKind == RobotPoseTargetKind.Flange
                ? 0
                : controller.Tool.Value;

            if (Label == "Return to home")
            {
                var worldPose = controller.FrameToWorld(
                    target.Pose, _frame, target.ExtJoint);
                var basePose = controller.MechanicalGroup.GetRobotBaseWorld(
                    target.ExtJoint).inverse * worldPose;
                var flangePose = controller.RemoveToolOffset(
                    basePose, solverToolIndex);
            }

            var solution = controller.Solver.ComputeInverse(
                target,
                solverToolIndex,
                _frame,
                _ignoreMask);

            if (!solution.IsValid)
            {
                var candidates = -1;
                var effectiveWorldPose = Matrix4x4.identity;
                try
                {
                    // Same target/tool/frame path as ComputeInverse, but expose whether
                    // the analytic solver produced any valid solutions at all.
                    effectiveWorldPose = controller.FrameToWorld(
                        target.Pose, _frame, target.ExtJoint);
                    var allSolutions = controller.Solver.GetAllSolutions(
                        target,
                        solverToolIndex,
                        _frame,
                        false,
                        _ignoreMask);
                    candidates = allSolutions == null ? 0 : allSolutions.Count;
                }
                catch (Exception diagnosticException)
                {
                    ErrorMessage = string.Format(
                        "MoveToPoseState '{0}': IK failed; solution diagnostic also failed: {1}",
                        Label,
                        diagnosticException.Message);
                    return;
                }

                ErrorMessage = string.Format(
                    "MoveToPoseState '{0}': IK failed ({1}); valid candidates={2}; " +
                    "frame={3}, tool={4}, world pose={5}.",
                    Label,
                    solution.Exception != null ? solution.Exception.Message : "unknown IK error",
                    candidates,
                    _frame,
                    solverToolIndex,
                    effectiveWorldPose);
                return;
            }

            _endJoint = solution.JointTarget;
            _trajectory = new JointTrajectory(
                _startJoint,
                _endJoint,
                controller.MechanicalGroup);

            if (!_trajectory.IsValid)
            {
                ErrorMessage = string.Format(
                    "MoveToPoseState '{0}': {1}",
                    Label,
                    _trajectory.ErrorMessage);
            }
        }

        public RobotStateResult Tick(Controller controller, float deltaTime)
        {
            if (_trajectory == null || !_trajectory.IsValid)
            {
                if (ErrorMessage == null)
                    ErrorMessage = string.Format(
                        "MoveToPoseState '{0}': trajectory could not be created.",
                        Label);

                return RobotStateResult.Failed;
            }

            _elapsed += deltaTime;

            var evaluated = _trajectory.Evaluate(_elapsed, _startJoint);
            controller.MechanicalGroup.SetJoints(evaluated, true);

            if (_elapsed < _trajectory.Duration)
                return RobotStateResult.Running;

            controller.MechanicalGroup.SetJoints(_endJoint, true);
            return RobotStateResult.Completed;
        }

        public void Exit(Controller controller, RobotStateResult result)
        {
        }
    }
}
