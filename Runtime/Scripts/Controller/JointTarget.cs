using System;
using UnityEngine;

namespace Preliy.Flange
{
    /// <summary>
    /// <see cref="JointTarget"/> defined the position of robot and external axes
    /// </summary>
    [Serializable]
    public class JointTarget
    {
        public const int LENGTH = 13;
        
        public RobJoint RobJoint
        {
            get => _robJoint;
            set => _robJoint = value == null ? null : value.Clone();
        }
        
        public ExtJoint ExtJoint
        {
            get => _extJoint;
            set => _extJoint = value == null ? null : value.Clone();
        }

        public float[] Value
        {
            get
            {
                var array = new float[LENGTH];
                for (var i = 0; i < LENGTH; i++)
                {
                    array[i] = this[i];
                }
                return array;
            }
        }

        [SerializeField]
        private RobJoint _robJoint;
        [SerializeField]
        private ExtJoint _extJoint;
        
        public float this[int index]
        {
            get
            {
                switch (index)
                {
                    case 0: return _robJoint[0];
                    case 1: return _robJoint[1];
                    case 2: return _robJoint[2];
                    case 3: return _robJoint[3];
                    case 4: return _robJoint[4];
                    case 5: return _robJoint[5];
                    case 6: return _robJoint[6];
                    case 7: return _extJoint[0];
                    case 8: return _extJoint[1];
                    case 9: return _extJoint[2];
                    case 10: return _extJoint[3];
                    case 11: return _extJoint[4];
                    case 12: return _extJoint[5];
                    default: throw new IndexOutOfRangeException("Invalid index!");
                }
            }
            set
            {
                switch (index)
                {
                    case 0:
                        _robJoint[0] = value;
                        break;
                    case 1:
                        _robJoint[1] = value;
                        break;
                    case 2:
                        _robJoint[2] = value;
                        break;
                    case 3:
                        _robJoint[3] = value;
                        break;
                    case 4:
                        _robJoint[4] = value;
                        break;
                    case 5:
                        _robJoint[5] = value;
                        break;
                    case 6:
                        _robJoint[6] = value;
                        break;
                    case 7:
                        _extJoint[0] = value;
                        break;
                    case 8:
                        _extJoint[1] = value;
                        break;
                    case 9:
                        _extJoint[2] = value;
                        break;
                    case 10:
                        _extJoint[3] = value;
                        break;
                    case 11:
                        _extJoint[4] = value;
                        break;
                    case 12:
                        _extJoint[5] = value;
                        break;
                    default:
                        throw new IndexOutOfRangeException("Invalid index!");
                }
            }
        }

        public JointTarget(params float[] value)
        {
            if (value == null) throw new ArgumentNullException(nameof(value));
            if (value.Length > LENGTH) throw new ArgumentOutOfRangeException();

            _robJoint = RobJoint.Default;
            _extJoint = ExtJoint.Default;
            
            for (var i = 0; i < value.Length; i++)
            {
                this[i] = value[i];
            }
        }

        public JointTarget(RobJoint robJoint, ExtJoint extJoint)
        {
            _robJoint = robJoint == null ? null : robJoint.Clone();
            _extJoint = extJoint == null ? null : extJoint.Clone();
        }

        protected JointTarget(JointTarget other)
        {
            _robJoint = other == null || other.RobJoint == null ? null : other.RobJoint.Clone();
            _extJoint = other == null || other.ExtJoint == null ? null : other.ExtJoint.Clone();
        }

        public JointTarget Clone()
        {
            return new JointTarget(this);
        }

        public static JointTarget Default => 
            new JointTarget(0, 0, 0, 0, 0, 0, 0,
            Math.FLOAT_MAX, Math.FLOAT_MAX, Math.FLOAT_MAX, Math.FLOAT_MAX, Math.FLOAT_MAX, Math.FLOAT_MAX);
        
        public static JointTarget Null => 
            new JointTarget(0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0);

        public override string ToString() => $"[{_robJoint}] [{_extJoint}]";
    }
}
