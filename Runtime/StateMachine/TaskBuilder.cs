using System;
using System.Collections.Generic;
using UnityEngine;
using Preliy.Flange;

namespace Preliy.Flange.Orchestration
{
    public static class TaskBuilder
    {
        private const float ApproachOffset = 0.02f;
        // Repository Transform poses are world-space; Flange frame -1 is world.
        private const int WorldFrame = -1;

        public static Queue<IRobotState> Build(
            Controller controller,
            ManipulationTaskRequest request,
            IManipulationRepository repository)
        {
            if (controller == null) throw new ArgumentNullException(nameof(controller));
            if (repository == null) throw new ArgumentNullException(nameof(repository));

            if (request.ScrewColors == null || request.ScrewColors.Length == 0)
                throw new ArgumentException("ManipulationTaskRequest has no screws.", nameof(request));

            if (!Enum.IsDefined(typeof(ManipulationTool), request.Tool))
                throw new ArgumentOutOfRangeException(nameof(request), "Unknown manipulation tool.");

            if (request.UseGlue && !Enum.IsDefined(typeof(GlueColor), request.GlueColor))
                throw new ArgumentOutOfRangeException(nameof(request), "Unknown glue color.");

            var queue = new Queue<IRobotState>();
            ManipulationToolInfo mountedTool = null;
            ManipulationToolInfo requestedTool = null;
            var mountedRequestedTool = false;

            var currentToolIndex = controller.Tool.Value;
            if (currentToolIndex > 0 &&
                !repository.TryGetToolBySolverIndex(currentToolIndex, out mountedTool))
            {
                throw new InvalidOperationException(
                    "The currently mounted solver tool index " + currentToolIndex +
                    " is not mapped by the manipulation repository.");
            }
            if (mountedTool != null)
                ValidateSolverToolIndex(controller, mountedTool);

            if (request.Tool != ManipulationTool.None)
            {
                if (!repository.TryGetTool(request.Tool, out requestedTool) ||
                    requestedTool == null ||
                    requestedTool.Tool != request.Tool ||
                    requestedTool.FlangeMatingPose == null)
                {
                    throw new InvalidOperationException(
                        "Tool station pose not found for " + request.Tool + ".");
                }

                ValidateSolverToolIndex(controller, requestedTool);

                if (mountedTool == null || mountedTool.Tool != requestedTool.Tool)
                {
                    if (mountedTool != null)
                    {
                        if (mountedTool.FlangeMatingPose == null)
                            throw new InvalidOperationException(
                                "Mounted tool station pose is missing for " + mountedTool.Tool + ".");

                        queue.Enqueue(CreatePoseState(
                            mountedTool.FlangeMatingPose,
                            request.IgnoreMask,
                            "Move to " + mountedTool.Tool + " release station",
                            RobotPoseTargetKind.Flange));
                        queue.Enqueue(new ReleaseToolState(repository, mountedTool));
                    }

                    queue.Enqueue(CreatePoseState(
                        requestedTool.FlangeMatingPose,
                        request.IgnoreMask,
                        "Move to " + request.Tool + " pickup station",
                        RobotPoseTargetKind.Flange));
                    queue.Enqueue(new MountToolState(repository, requestedTool));
                    mountedRequestedTool = true;
                }
            }

            // For each screw: approach, contact, apply pressure, retreat.
            for (var i = 0; i < request.ScrewColors.Length; i++)
            {
                Transform tcp;
                var color = request.ScrewColors[i];
                if (!repository.TryGetScrew(color, out tcp) || tcp == null)
                {
                    throw new InvalidOperationException(
                        string.Format("Screw not found for color ({0}).", color));
                }

                var approachPosition = tcp.position + tcp.up * ApproachOffset;
                var approach = Matrix4x4.TRS(approachPosition, tcp.rotation, Vector3.one);
                var contact = Matrix4x4.TRS(tcp.position, tcp.rotation, Vector3.one);

                queue.Enqueue(CreatePoseState(
                    approach,
                    request.IgnoreMask,
                    string.Format("Approach screw {0}", i + 1),
                    RobotPoseTargetKind.ActiveToolTcp));
                queue.Enqueue(CreatePoseState(
                    contact,
                    request.IgnoreMask,
                    string.Format("Contact screw {0}", i + 1),
                    RobotPoseTargetKind.ActiveToolTcp));
                queue.Enqueue(new ApplyPressureState(
                    request.Pressure,
                    request.UseGlue,
                    request.GlueColor,
                    string.Format("Pressure screw {0}", i + 1)));
                queue.Enqueue(CreatePoseState(
                    approach,
                    request.IgnoreMask,
                    string.Format("Retreat screw {0}", i + 1),
                    RobotPoseTargetKind.ActiveToolTcp));
            }

            if (mountedRequestedTool)
            {
                queue.Enqueue(CreatePoseState(
                    requestedTool.FlangeMatingPose,
                    request.IgnoreMask,
                    "Move to " + request.Tool + " release station",
                    RobotPoseTargetKind.Flange));
                queue.Enqueue(new ReleaseToolState(repository, requestedTool));
            }

            // Home is defined as a flange pose.
            Transform home;
            if (!repository.TryGetHome(out home) || home == null)
                throw new InvalidOperationException("Robot home pose not found.");

            queue.Enqueue(CreatePoseState(
                home,
                request.IgnoreMask,
                "Return to home",
                RobotPoseTargetKind.Flange));

            return queue;
        }

        public static MoveToPoseState CreateWorldFlangeMove(
            Transform target,
            string label,
            SolutionIgnoreMask ignoreMask = SolutionIgnoreMask.All)
        {
            if (target == null) throw new ArgumentNullException(nameof(target));
            return CreatePoseState(
                target,
                ignoreMask,
                label,
                RobotPoseTargetKind.Flange);
        }

        private static MoveToPoseState CreatePoseState(
            Transform target,
            SolutionIgnoreMask ignoreMask,
            string label,
            RobotPoseTargetKind targetKind)
        {
            return CreatePoseState(
                Matrix4x4.TRS(target.position, target.rotation, Vector3.one),
                ignoreMask,
                label,
                targetKind);
        }

        private static void ValidateSolverToolIndex(
            Controller controller,
            ManipulationToolInfo tool)
        {
            if (tool.SolverToolIndex <= 0 ||
                tool.SolverToolIndex > controller.Tools.Count)
            {
                throw new InvalidOperationException(
                    string.Format(
                        "Tool {0} has solver index {1}, but Controller has {2} configured tool offsets.",
                        tool.Tool,
                        tool.SolverToolIndex,
                        controller.Tools.Count));
            }
        }

        private static MoveToPoseState CreatePoseState(
            Matrix4x4 pose,
            SolutionIgnoreMask ignoreMask,
            string label,
            RobotPoseTargetKind targetKind)
        {
            var target = new CartesianTarget(
                pose,
                Configuration.Default,
                ExtJoint.Default);

            return new MoveToPoseState(
                target,
                WorldFrame,
                ignoreMask,
                label,
                targetKind);
        }
    }
}
