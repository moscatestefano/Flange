using System.Linq;
using NUnit.Framework;

namespace Preliy.Flange.Editor.Tests
{
    public class TestJointTarget
    {
        [Test]
        public void Create()
        {
            var value = new float[] { 10, 20, 30, 40, 50, 60, 70, 10, 20, 30, 40, 50, 60 };

            var jointTarget = new JointTarget(value);
            
            for (var i = 0; i < JointTarget.LENGTH; i++)
            {
                Assert.AreEqual(value[i], jointTarget[i]);
            }
        }
        
        [Test]
        public void CreateCombine()
        {
            var value = new float[] { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13 };
            var robJoint = new RobJoint(value.Take(7).ToArray());
            var extJoint = new ExtJoint(value.Skip(7).ToArray());
            
            var jointTarget = new JointTarget(robJoint, extJoint);
            
            for (var i = 0; i < value.Length; i++)
            {
                Assert.AreEqual(value[i], jointTarget[i], $"Invalid valid on index {i}!");
            }
        }

        [Test]
        public void Clone()
        {
            var value = new float[] { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13 };

            var jointTargetInit = new JointTarget(value);
            var jointTargetClone = jointTargetInit.Clone();

            jointTargetInit[0] = 0;
            
            Assert.AreNotEqual(jointTargetInit[0], jointTargetClone[0]);
        }
    }
}
