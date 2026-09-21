using System;
using System.Collections.Generic;
using UnityEngine;

namespace Preliy.Flange
{
    [ExecuteAlways]
    [AddComponentMenu("Robotics/Transform Joint")]
    [RequireComponent(typeof(Frame))]
    [DisallowMultipleComponent]
    public class TransformJoint : MonoBehaviour, IEqualityComparer<TransformJoint>
    {
        public IProperty<float> Position => _position;
        
        public Frame Frame
        {
            get
            {
                if (_frame == null) _frame = GetComponent<Frame>();
                return _frame;
            }
        }

        public JointConfig Config
        {
            get => _config;
            set => _config = value;
        }

        public bool UseCustomAxis => _useCustomAxis;
        public Vector3 CustomAxis => _customAxis;
        public Vector3 CustomOriginOffset => _customOriginOffset;

        [SerializeField]
        private Property<float> _position = new Property<float>();

        [SerializeField]
        private JointConfig _config = JointConfig.Default;

        [SerializeField]
        private bool _useCustomAxis;
        [SerializeField]
        private Vector3 _customAxis = Vector3.up;
        [SerializeField]
        private Vector3 _customOriginOffset;

        private Frame _frame;

        private void OnEnable()
        {
            _position.Subscribe(SetValue);
        }

        private void OnDisable()
        {
            _position.Unsubscribe(SetValue);
        }

        private void OnValidate()
        {
            _position.OnValidate();
        }

        private void SetValue(float value)
        {
            if (_useCustomAxis)
            {
                var frameMatrix = HomogeneousMatrix.CreateRaw(Frame.Config);
                var origin = Matrix4x4.Translate(_customOriginOffset);
                var angleDegrees = Config.GetValidValue(value);
                var axis = _customAxis.sqrMagnitude > 1e-8f ? _customAxis.normalized : Vector3.up;
                var rotation = Matrix4x4.Rotate(Quaternion.AngleAxis(angleDegrees, axis));
                transform.SetLocalMatrix(frameMatrix * origin * rotation);
            }
            else
            {
                transform.SetLocalMatrix(HomogeneousMatrix.Create(Frame.Config, Config, value));
            }
            
#if UNITY_EDITOR
            if (!Application.isPlaying) UnityEditor.EditorUtility.SetDirty(this);
#endif
        }

        public void ConfigureCustomAxis(Vector3 axis, bool enabled = true)
        {
            ConfigureCustomAxis(axis, Vector3.zero, enabled);
        }

        public void ConfigureCustomAxis(Vector3 axis, Vector3 originOffset, bool enabled = true)
        {
            _customAxis = axis.sqrMagnitude > 1e-8f ? axis.normalized : Vector3.up;
            _customOriginOffset = originOffset;
            _useCustomAxis = enabled;
            SetValue(_position.Value);
        }

        public enum JointType
        {
            Rotation,
            Displacement
        }
        
        public bool Equals(TransformJoint x, TransformJoint y)
        {
            if (ReferenceEquals(x, y)) return true;
            if (ReferenceEquals(x, null)) return false;
            if (ReferenceEquals(y, null)) return false;
            return Equals(x.name, y.name);
        }
        
        public int GetHashCode(TransformJoint obj)
        {
            if (ReferenceEquals(obj, null)) return 0;
            return obj.name == null ? 0 : obj.name.GetHashCode();
        }
    }
}
