using System;
using UnityEngine;
using Object = UnityEngine.Object;

namespace Preliy.Flange
{
    [Serializable]
    public class CartesianTarget
    {
        public Object Object => _object;
        public Matrix4x4 Pose
        {
            get => _pose;
            set => _pose = value;
        }
        public Configuration Configuration
        {
            get => _configuration;
            set => _configuration = value;
        }
        public ExtJoint ExtJoint
        {
            get => _extJoint;
            set => _extJoint = value == null ? null : value.Clone();
        }

        [SerializeField]
        private Object _object;
        [SerializeField]
        private Matrix4x4 _pose;
        [SerializeField]
        private Configuration _configuration;
        [SerializeField]
        private ExtJoint _extJoint;

        public static CartesianTarget Default => new CartesianTarget(Matrix4x4.identity, Configuration.Default, ExtJoint.Default);

        public CartesianTarget(Matrix4x4 pose, Configuration configuration, ExtJoint extJoint, Object @object = null)
        {
            _object = @object;
            _pose = pose;
            _configuration = configuration;
            _extJoint = extJoint == null ? null : extJoint.Clone();
        }

        protected CartesianTarget(CartesianTarget other)
        {
            _object = other == null ? null : other.Object;
            _pose = other == null ? Matrix4x4.identity : other.Pose;
            _configuration = other == null ? Configuration.Default : other.Configuration;
            _extJoint = other == null || other.ExtJoint == null ? null : other.ExtJoint.Clone();
        }

        public CartesianTarget Clone()
        {
            return new CartesianTarget(this);
        }
    }
}
