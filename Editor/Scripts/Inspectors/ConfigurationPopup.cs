using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEngine;

namespace Preliy.Flange.Editor
{
    public class ConfigurationPopup : PopupWindowContent
    {
        private readonly Controller _controller;
        private readonly List<IKSolution> _solutions;
        private string[] _labels;

        public ConfigurationPopup(Controller controller, bool turn)
        {
            _controller = controller;
            var robotTarget = new CartesianTarget(_controller.GetTcpRelativeToRefFrame(), _controller.Configuration.Value, _controller.MechanicalGroup.JointState.ExtJoint);
            _solutions = controller.Solver.GetAllSolutions(robotTarget, _controller.Tool.Value, _controller.Frame.Value, turn);
            _labels = _solutions.Select(solution => solution.GetLabel()).ToArray();
        }

        public override Vector2 GetWindowSize()
        {
            return new Vector2(480, Mathf.Max(50f, _solutions.Count * (EditorGUIUtility.singleLineHeight + 4f) + 10f));
        }

        public override void OnGUI(Rect rect)
        {
            if (_controller == null)
            {
                EditorGUILayout.LabelField("Controller is unavailable.");
                return;
            }

            if (_solutions.Count == 0)
            {
                EditorGUILayout.HelpBox("No inverse-kinematic configuration is available for the current target.", MessageType.Info);
                return;
            }

            var currentIndex = _solutions.FindIndex(item => item.Configuration == _controller.Configuration.Value);
            if (currentIndex < 0) currentIndex = 0;

            EditorGUI.BeginChangeCheck();
            var selectedIndex = EditorGUILayout.Popup("Configuration", currentIndex, _labels);
            if (EditorGUI.EndChangeCheck() && selectedIndex >= 0 && selectedIndex < _solutions.Count)
            {
                _controller.Solver.TryApplySolution(_solutions[selectedIndex]);
            }
        }
    }
}
