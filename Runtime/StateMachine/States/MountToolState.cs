using System;
using Preliy.Flange;

namespace Preliy.Flange.Orchestration
{
    /// <summary>
    /// Stato istantaneo: prende il tool dalla stazione e lo rende attivo nel solver.
    /// </summary>
    public sealed class MountToolState : IRobotState
    {
        public string Label { get; }
        public string ErrorMessage { get; private set; }

        private readonly IManipulationRepository _repository;
        private readonly ManipulationToolInfo _tool;
        private bool _done;

        public MountToolState(
            IManipulationRepository repository,
            ManipulationToolInfo tool,
            string label = "Mount tool")
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
                _repository.MountTool(controller, _tool);
                controller.Tool.Value = _tool.SolverToolIndex;
                _done = true;
            }
            catch (Exception exception)
            {
                ErrorMessage = string.Format(
                    "MountToolState '{0}': {1}",
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
