/*
Copyright (c) 2022 FZI Forschungszentrum Informatik

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. 
*/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

/**
 * Base class for Inverse kinematics mangement.
 * Should be attached to every base of a robotic arm.
 * 
 * Author: Pascal Zwick
 * e-mail: zwick@fzi.de
 * */

namespace BurstIK
{
    public class IKSystem : MonoBehaviour
    {
        //Defines the end effector target
        public Transform Target;

        //Should the target rotation taken into account
        public bool EnableRotationalTarget = false;

        //Keeps track of all children joints
        public RobotJoint[] joints { get; private set; }

        //tracks if the ik should be enabled
        private bool ikEnabled;

        // Reloades joints and registeres itself at the IKManager
        void Start()
        {
            joints = this.GetComponentsInChildren<RobotJoint>();

            SetIKState(true);
        }

        //Just to see all joints in the editor
        private void OnValidate()
        {
            joints = this.GetComponentsInChildren<RobotJoint>();
        }

        // Unregister from IKManager when destroyed
        private void OnDestroy()
        {
            SetIKState(false);
        }

        public void SetIKState(bool state)
        {
            ikEnabled = state;
            if (ikEnabled)
                IKManager.instance.RegisterIK(this);
            else
                IKManager.instance.UnregisterIK(this);
        }

        //Gizmo drawing
        private void OnDrawGizmosSelected()
        {
            if (joints.Length > 1)
            {
                Gizmos.color = Color.green;

                Vector3 prev = joints[0].transform.position;
                for (int i = 1; i < joints.Length; ++i)
                {
                    Vector3 p = joints[i].transform.position;

                    Gizmos.DrawLine(prev, p);

                    prev = p;
                }

                Gizmos.color = Color.red;
                foreach (RobotJoint j in joints)
                {
                    if (j.GetType() == typeof(RobotHingeJoint))
                        Gizmos.DrawLine(j.transform.position, j.transform.position + 0.2f * (j.transform.rotation * j.Axis));
                    else if (j.GetType() == typeof(RobotShiftJoint))
                        Gizmos.DrawLine(j.transform.position, j.transform.position + 0.2f * (j.transform.rotation * Quaternion.Inverse(j.transform.localRotation) * j.Axis));
                }

                //Gizmos.color = Color.blue;
                //Gizmos.DrawWireSphere(solveForwardKinematics(new float[] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }), 0.1f);
            }
        }

    }
}