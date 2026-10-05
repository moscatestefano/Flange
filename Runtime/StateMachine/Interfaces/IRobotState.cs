using Preliy.Flange;

namespace Preliy.Flange.Orchestration
{
    /// <summary>Frame represented by a Cartesian target pose.</summary>
    public enum RobotPoseTargetKind
    {
        Flange,
        ActiveToolTcp
    }

    /// <summary>
    /// Esito di un singolo tick di un <see cref="IRobotState"/>.
    /// </summary>
    public enum RobotStateResult
    {
        Running,
        Completed,
        Failed
    }

    /// <summary>
    /// Step atomico e ripetibile eseguito dall'<see cref="Orchestrator"/>.
    /// Le implementazioni devono essere leggere da costruire (create in anticipo
    /// da un <see cref="TaskBuilder"/>) e non devono mai bloccare: tutto il lavoro
    /// avviene in modo incrementale dentro <see cref="Tick"/>, chiamato una volta
    /// per frame.
    /// </summary>
    public interface IRobotState
    {
        /// <summary>Etichetta leggibile, utile per UI/progress e log.</summary>
        string Label { get; }

        /// <summary>Valorizzato quando <see cref="Tick"/> ritorna <see cref="RobotStateResult.Failed"/>.</summary>
        string ErrorMessage { get; }

        /// <summary>Chiamato una sola volta, subito prima del primo <see cref="Tick"/>.</summary>
        void Enter(Controller controller);

        /// <summary>Chiamato una volta per frame dall'orchestrator finché lo stato è attivo.</summary>
        RobotStateResult Tick(Controller controller, float deltaTime);

        /// <summary>Chiamato una sola volta quando lo stato smette di essere attivo, qualunque sia l'esito.</summary>
        void Exit(Controller controller, RobotStateResult result);
    }
}
