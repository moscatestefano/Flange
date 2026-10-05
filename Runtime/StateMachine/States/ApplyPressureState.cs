using Preliy.Flange;

namespace Preliy.Flange.Orchestration
{
    /// <summary>
    /// Mantiene il tool sul TCP della vite per la durata richiesta.
    /// Nel caso studio attuale la pressione è simulata dal tempo di attesa.
    /// </summary>
    public sealed class ApplyPressureState : IRobotState
    {
        private const float MaximumPressureDurationSeconds = 8f;

        public string Label { get; }
        public string ErrorMessage => null;

        public float Pressure { get; }
        public float Duration { get; }
        public bool UseGlue { get; }
        public GlueColor GlueColor { get; }

        private float _elapsed;

        public ApplyPressureState(
            float pressure,
            bool useGlue,
            GlueColor glueColor,
            string label = "Apply pressure")
        {
            Pressure = IsValidPressure(pressure) ? pressure : 1f;
            Duration = Pressure * MaximumPressureDurationSeconds;
            UseGlue = useGlue;
            GlueColor = glueColor;
            Label = label;
        }

        private static bool IsValidPressure(float pressure)
        {
            return !float.IsNaN(pressure) &&
                   !float.IsInfinity(pressure) &&
                   pressure >= 0f && pressure <= 1f;
        }

        public void Enter(Controller controller)
        {
            _elapsed = 0f;
        }

        public RobotStateResult Tick(Controller controller, float deltaTime)
        {
            _elapsed += deltaTime;
            return _elapsed >= Duration
                ? RobotStateResult.Completed
                : RobotStateResult.Running;
        }

        public void Exit(Controller controller, RobotStateResult result)
        {
        }
    }
}
