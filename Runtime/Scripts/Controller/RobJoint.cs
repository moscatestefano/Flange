using System;
using UnityEngine;

namespace Preliy.Flange
{
    [Serializable]
    public class RobJoint
    {
        public const int LENGTH = 7;

        public float[] Value
        {
            get
            {
                var array = new float[LENGTH];
                for (var i = 0; i < LENGTH; i++) array[i] = this[i];
                return array;
            }
            set
            {
                if (value == null) throw new ArgumentNullException(nameof(value));
                if (value.Length > LENGTH) throw new ArgumentOutOfRangeException(nameof(value));
                for (var i = 0; i < value.Length; i++) this[i] = value[i];
            }
        }

        [SerializeField] private float _r1;
        [SerializeField] private float _r2;
        [SerializeField] private float _r3;
        [SerializeField] private float _r4;
        [SerializeField] private float _r5;
        [SerializeField] private float _r6;
        [SerializeField] private float _r7;

        public float this[int index]
        {
            get
            {
                switch (index)
                {
                    case 0: return _r1;
                    case 1: return _r2;
                    case 2: return _r3;
                    case 3: return _r4;
                    case 4: return _r5;
                    case 5: return _r6;
                    case 6: return _r7;
                    default: throw new IndexOutOfRangeException("Invalid index!");
                }
            }
            set
            {
                switch (index)
                {
                    case 0: _r1 = value; break;
                    case 1: _r2 = value; break;
                    case 2: _r3 = value; break;
                    case 3: _r4 = value; break;
                    case 4: _r5 = value; break;
                    case 5: _r6 = value; break;
                    case 6: _r7 = value; break;
                    default: throw new IndexOutOfRangeException("Invalid index!");
                }
            }
        }

        public RobJoint(params float[] value)
        {
            if (value == null) throw new ArgumentNullException(nameof(value));
            if (value.Length > LENGTH) throw new ArgumentOutOfRangeException(nameof(value));

            _r1 = 0f; _r2 = 0f; _r3 = 0f; _r4 = 0f; _r5 = 0f; _r6 = 0f; _r7 = 0f;
            for (var i = 0; i < value.Length; i++) this[i] = value[i];
        }

        public static RobJoint Default => new RobJoint(0f, 0f, 0f, 0f, 0f, 0f, 0f);

        public RobJoint Clone() { return new RobJoint(Value); }

        public override string ToString() { return string.Join(", ", Value); }
    }
}
