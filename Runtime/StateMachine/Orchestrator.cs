using System;
using System.Collections.Generic;
using UnityEngine;
using Preliy.Flange;

namespace Preliy.Flange.Orchestration
{
    public enum OrchestratorStatus
    {
        Idle,
        Running,
        Recovering,
        Paused,
        Completed,
        Faulted
    }

    [AddComponentMenu("Robotics/Orchestrator")]
    [DisallowMultipleComponent]
    [RequireComponent(typeof(Controller))]
    public class Orchestrator : MonoBehaviour
    {
        [SerializeField] private Controller _controller;
        [SerializeField] private MonoBehaviour _repositoryComponent;

        private IManipulationRepository _repository;
        private readonly Queue<IRobotState> _queue = new Queue<IRobotState>();
        private string _recoveryOriginFault;
        private bool _resumeToRecovery;

        public OrchestratorStatus Status { get; private set; } = OrchestratorStatus.Idle;
        public IRobotState CurrentState { get; private set; }
        public Controller Controller { get { return _controller; } }
        public int PendingCount { get { return _queue.Count; } }

        public event Action<IRobotState> OnStateStarted;
        public event Action<IRobotState> OnStateCompleted;
        public event Action<IRobotState, string> OnStateFailed;
        public event Action OnQueueCompleted;

        private void Reset()
        {
            _controller = GetComponent<Controller>();
        }

        private void OnValidate()
        {
            if (_controller == null)
                _controller = GetComponent<Controller>();

            _repository = _repositoryComponent as IManipulationRepository;
        }

        private void Awake()
        {
            if (_controller == null)
                _controller = GetComponent<Controller>();

            _repository = _repositoryComponent as IManipulationRepository;
        }

        /// <summary>Submit a prebuilt sequence of generic robot states.</summary>
        public void SubmitTask(IEnumerable<IRobotState> states)
        {
            if (states == null) throw new ArgumentNullException(nameof(states));
            if (Status != OrchestratorStatus.Idle && Status != OrchestratorStatus.Completed)
            {
                Logger.Log(LogType.Warning,
                    "Task submission rejected because the orchestrator is " + Status + ".", this);
                return;
            }

            try
            {
                EnqueueRange(states);
                Run();
            }
            catch (Exception exception)
            {
                Fault(null, exception.Message);
            }
        }

        /// <summary>Build and submit a sequence while keeping builder errors inside orchestration handling.</summary>
        public void SubmitTask(Func<IEnumerable<IRobotState>> buildStates)
        {
            if (buildStates == null) throw new ArgumentNullException(nameof(buildStates));
            if (Status != OrchestratorStatus.Idle && Status != OrchestratorStatus.Completed)
            {
                Logger.Log(LogType.Warning,
                    "Task submission rejected because the orchestrator is " + Status + ".", this);
                return;
            }

            try
            {
                EnqueueRange(buildStates());
                Run();
            }
            catch (Exception exception)
            {
                Fault(null, exception.Message);
            }
        }

        public void Enqueue(IRobotState state)
        {
            if (state == null) throw new ArgumentNullException(nameof(state));
            _queue.Enqueue(state);
        }

        public void EnqueueRange(IEnumerable<IRobotState> states)
        {
            if (states == null) throw new ArgumentNullException(nameof(states));
            foreach (var state in states)
                Enqueue(state);
        }

        public void Run()
        {
            if (Status == OrchestratorStatus.Idle || Status == OrchestratorStatus.Completed)
                Status = OrchestratorStatus.Running;
        }

        public void Pause()
        {
            if (Status == OrchestratorStatus.Running)
                Status = OrchestratorStatus.Paused;
            else if (Status == OrchestratorStatus.Recovering)
            {
                _resumeToRecovery = true;
                Status = OrchestratorStatus.Paused;
            }
        }

        public void Resume()
        {
            if (Status == OrchestratorStatus.Paused)
            {
                Status = _resumeToRecovery
                    ? OrchestratorStatus.Recovering
                    : OrchestratorStatus.Running;
                _resumeToRecovery = false;
            }
        }

        public void Stop()
        {
            if (CurrentState != null)
            {
                try
                {
                    CurrentState.Exit(_controller, RobotStateResult.Failed);
                }
                catch (Exception exception)
                {
                    Logger.Log(LogType.Warning, "State cleanup failed during Stop: " + exception.Message, this);
                }
                CurrentState = null;
            }

            _queue.Clear();
            _recoveryOriginFault = null;
            _resumeToRecovery = false;
            Status = OrchestratorStatus.Idle;
        }

        private void Update()
        {
            if (Status != OrchestratorStatus.Running &&
                Status != OrchestratorStatus.Recovering)
                return;

            if (_controller == null || !_controller.IsValid.Value)
            {
                Fault(CurrentState, "Controller is missing or not valid.");
                return;
            }

            if (CurrentState == null)
            {
                if (_queue.Count == 0)
                {
                    if (Status == OrchestratorStatus.Recovering)
                    {
                        _recoveryOriginFault = null;
                        Status = OrchestratorStatus.Faulted;
                        return;
                    }

                    Status = OrchestratorStatus.Completed;
                    OnQueueCompleted?.Invoke();
                    return;
                }

                CurrentState = _queue.Dequeue();

                try
                {
                    CurrentState.Enter(_controller);
                    OnStateStarted?.Invoke(CurrentState);
                }
                catch (Exception exception)
                {
                    Fault(CurrentState, exception.Message);
                    return;
                }
            }

            RobotStateResult result;

            try
            {
                result = CurrentState.Tick(_controller, Time.deltaTime);
            }
            catch (Exception exception)
            {
                Fault(CurrentState, exception.Message);
                return;
            }

            switch (result)
            {
                case RobotStateResult.Running:
                    return;

                case RobotStateResult.Completed:
                {
                    var finished = CurrentState;

                    try
                    {
                        finished.Exit(_controller, RobotStateResult.Completed);
                    }
                    catch (Exception exception)
                    {
                        Fault(finished, exception.Message);
                        return;
                    }

                    CurrentState = null;
                    OnStateCompleted?.Invoke(finished);
                    return;
                }

                case RobotStateResult.Failed:
                    Fault(CurrentState, CurrentState.ErrorMessage);
                    return;
            }
        }

        private void Fault(IRobotState state, string message)
        {
            var wasRecovering = Status == OrchestratorStatus.Recovering;

            if (state != null)
            {
                try
                {
                    state.Exit(_controller, RobotStateResult.Failed);
                }
                catch
                {
                    // Il fault originale è più importante del fault durante Exit.
                }
            }

            CurrentState = null;
            _queue.Clear();

            var safeMessage = string.IsNullOrEmpty(message)
                ? "Unknown orchestrator fault."
                : message;

            if (wasRecovering)
            {
                safeMessage = string.Format(
                    "Home recovery failed after the original fault '{0}': {1}",
                    _recoveryOriginFault,
                    safeMessage);
                _recoveryOriginFault = null;
                Status = OrchestratorStatus.Faulted;
                Logger.Log(LogType.Error, "Orchestrator faulted: " + safeMessage, this);
                OnStateFailed?.Invoke(state, safeMessage);
                return;
            }

            Status = OrchestratorStatus.Faulted;
            TryBeginHomeRecovery(state, safeMessage);

            Logger.Log(LogType.Error, "Orchestrator faulted: " + safeMessage, this);
            OnStateFailed?.Invoke(state, safeMessage);
        }

        private void TryBeginHomeRecovery(IRobotState failedState, string originalFault)
        {
            if (failedState == null ||
                failedState.Label == "Return to home" ||
                _controller == null ||
                !_controller.IsValid.Value ||
                _repository == null)
                return;

            try
            {
                Transform home;
                if (!_repository.TryGetHome(out home) || home == null)
                    return;

                _queue.Enqueue(TaskBuilder.CreateWorldFlangeMove(
                    home,
                    "Fault recovery to home"));
                _recoveryOriginFault = originalFault;
                _resumeToRecovery = false;
                Status = OrchestratorStatus.Recovering;
            }
            catch (Exception recoverySetupException)
            {
                Logger.Log(
                    LogType.Warning,
                    "Could not schedule Home recovery: " + recoverySetupException.Message,
                    this);
            }
        }
    }
}
