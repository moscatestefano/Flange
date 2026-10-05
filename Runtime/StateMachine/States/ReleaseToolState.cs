using System;
using Preliy.Flange;

namespace Preliy.Flange.Orchestration
{
    /// <summary>
    /// Stato istantaneo: sgancia il tool e lo lascia sulla stazione da cui era stato preso.
    /// </summary>
    public sealed class ReleaseToolState : IRobotState
    {
        public string Label { get; }
        public string ErrorMessage { get; private set; }

        private readonly IManipulationRepository _repository;
        private readonly ManipulationToolInfo _tool;
        private bool _done;

        public ReleaseToolState(
            IManipulationRepository repository,
            ManipulationToolInfo tool,
            string label = "Release tool")
        {
            _repository = repository ?? throw new ArgumentNullException(nameof(repository));
            _tool = tool ?? throw new ArgumentNullException(nameof(tool));
            Label = label;
        }

        public void Enter(Controller controller)
        {
            ErrorMessage = null;
            _done = false;

            try
            {
                _repository.ReleaseTool(controller, _tool);
                controller.Tool.Value = 0;
                _done = true;
            }
            catch (Exception exception)
            {
                ErrorMessage = string.Format(
                    "ReleaseToolState '{0}': {1}",
                    Label,
                    exception.Message);
            }
        }

        public RobotStateResult Tick(Controller controller, float deltaTime)
        {
            return _done ? RobotStateResult.Completed : RobotStateResult.Failed;
        }

        public void Exit(Controller controller, RobotStateResult result)
        {
        }
    }
}
