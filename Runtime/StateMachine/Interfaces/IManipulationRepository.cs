using UnityEngine;

namespace Preliy.Flange.Orchestration
{
    /// <summary>
    /// Ponte minimo verso i repository già presenti nella scena.
    /// L'Orchestrator non conosce come tool e viti sono memorizzati.
    /// </summary>
    public interface IManipulationRepository
    {
        bool TryGetTool(ManipulationTool tool, out ManipulationToolInfo info);
        bool TryGetToolBySolverIndex(int solverToolIndex, out ManipulationToolInfo info);
        bool TryGetScrew(ScrewColor color, out Transform tcp);
        bool TryGetHome(out Transform home);

        /// <summary>Collega fisicamente il tool al robot e aggiorna lo stato applicativo.</summary>
        void MountTool(Controller controller, ManipulationToolInfo info);

        /// <summary>Scollega fisicamente il tool dal robot e lo lascia sulla sua stazione.</summary>
        void ReleaseTool(Controller controller, ManipulationToolInfo info);
    }

    /// <summary>Informazioni minime che il builder deve conoscere di un tool.</summary>
    public sealed class ManipulationToolInfo
    {
        public ManipulationTool Tool { get; }
        public int SolverToolIndex { get; }
        /// <summary>
        /// World pose of the flange/tool-changer mating frame at this tool's station.
        /// Task motion to this pose is solved as a flange target.
        /// </summary>
        public Transform FlangeMatingPose { get; }

        public ManipulationToolInfo(
            ManipulationTool tool,
            int solverToolIndex,
            Transform flangeMatingPose)
        {
            Tool = tool;
            SolverToolIndex = solverToolIndex;
            FlangeMatingPose = flangeMatingPose;
        }
    }
}
