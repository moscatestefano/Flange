using UnityEngine;

namespace Preliy.Flange
{
    [ExecuteAlways]
    public class LookAtTarget : MonoBehaviour
    {
        [SerializeField]
        private Transform _target;
        [SerializeField]
        private bool _classic;
        [SerializeField]
        private Vector3 _offset;
        [SerializeField]
        private Vector3 _customUp = Vector3.zero;

        
        private void LateUpdate()
        {
            if (_target == null) return;
            if (_classic)
            {
                transform.LookAt(_target);
            }
            else
            {
                Vector3 directionToTarget = _target.TransformPoint(_offset) - transform.position;
                Vector3 forwardVector =  Vector3.Cross(directionToTarget.normalized, _customUp == Vector3.zero ? _target.forward : _customUp);
                transform.rotation = Quaternion.LookRotation(directionToTarget, forwardVector);
            }
        }
    }
}
