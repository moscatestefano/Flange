using UnityEngine;

namespace Preliy.Flange
{
    public static class CartesianTargetExtension
    {
        public static CartesianTarget AddOffset(this CartesianTarget cartesianTarget, Matrix4x4 offset)
        {
            var result = cartesianTarget.Clone();
            result.Pose = cartesianTarget.Pose * offset;
            return result;
        }
        
        public static CartesianTarget AddOffset(this CartesianTarget cartesianTarget, Vector3 offset)
        {
            var result = cartesianTarget.Clone();
            result.Pose = cartesianTarget.Pose * Matrix4x4.TRS(offset, Quaternion.identity, Vector3.one);
            return result;
        }
        
        public static CartesianTarget AddOffset(this CartesianTarget cartesianTarget, Quaternion offset)
        {
            var result = cartesianTarget.Clone();
            result.Pose = cartesianTarget.Pose * Matrix4x4.TRS(Vector3.zero, offset, Vector3.one);
            return result;
        }
    }
}
