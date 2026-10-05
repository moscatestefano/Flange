using Preliy.Flange;
using UnityEngine;

namespace Preliy.Flange.Orchestration
{
    /// <summary>
    /// Resta fermo per una durata fissa senza toccare il robot.
    /// </summary>
    public sealed class WaitState : IRobotState
    {
        public string Label { get; }
        public string ErrorMessage => null;

        private readonly float _duration;
        private float _elapsed;

        public WaitState(float durationSeconds, string label = "Wait")
        {
            _duration = Mathf.Max(0f, durationSeconds);
            Label = label;
        }

        public void Enter(Controller controller)
        {
            _elapsed = 0f;
        }

        public RobotStateResult Tick(Controller controller, float deltaTime)
        {
            _elapsed += deltaTime;
            return _elapsed >= _duration ? RobotStateResult.Completed : RobotStateResult.Running;
        }

        public void Exit(Controller controller, RobotStateResult result)
        {
        }
    }
}
