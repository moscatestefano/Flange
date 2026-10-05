using System.Collections.Generic;
using Preliy.Flange;
using UnityEngine;

namespace Preliy.Flange.Orchestration
{
    /// <summary>
    /// Base for application-specific task builders. Implementations translate
    /// their own request model into the generic robot-state sequence.
    /// </summary>
    public abstract class TaskBuilderBase : MonoBehaviour
    {
        public abstract IEnumerable<IRobotState> Build(Controller controller);
    }
}
