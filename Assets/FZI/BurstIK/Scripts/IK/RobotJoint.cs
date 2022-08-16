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
 * Abstracts a robot joint.
 * For a new joint type, the IKManager Forwards pass needs to be adapted to support the new joint type.
 * 
 * Author: Pascal Zwick
 * e-mail: zwick@fzi.de
 * */

namespace BurstIK
{
    public abstract class RobotJoint : MonoBehaviour
    {

        public enum JointType
        {
            HINGE, SHIFT
        }

        [System.Serializable]
        public enum JointAxis
        {
            X, Y, Z
        }

        //The axis the joint works on set by the user.
        public JointAxis JAxis;

        //The axis the joint works on as float3.
        public Vector3 Axis { get; private set; }

        //Defines the type of the joint. Needs to be set by subclass.
        public JointType Type { get; protected set; }

        //Defines the valid value interval of this joint
        public float2 ValidValueInterval = new float2(-10000000, 10000000);

        //Defines the maximum speed the joint can move
        public float MaxSpeed = 1;

        public virtual void Awake()
        {
            this.Axis = calculateAxis();
        }

        private void OnValidate()
        {
            this.Axis = calculateAxis();
        }

        //Creates a vector3 axis from JointAxis enum
        public virtual Vector3 calculateAxis()
        {
            return JAxis == JointAxis.X ? Vector3.right : (JAxis == JointAxis.Y ? Vector3.up : Vector3.forward);
        }

        //Applies the data coming from the IKManager IK calculation.
        public abstract void ApplyForwardKinematics(float value, float3 initialPosition, quaternion initialRotation);


        /* ########################################
         * Struct representing a joint used for calculations in the IKManager
         * ONLY PUT STRUCTS or other primary datatypes in here.
         * Especially DON'T PUT classes in here, which violate data oriented programming.
         * ########################################
         */
        public struct JointData
        {
            //Basic joint data
            public JointType type;
            public float3 axis;
            public float value;
            public float2 validValueInterval;
            public float maxSpeed;

            //init data
            public float3 localPosition;
            public quaternion localRotation;

            //Data needed for derivative calculations
            public float curGrad;
            public float3 P, dP;
            public quaternion Q, dQ;
            public float3 Target;
            public quaternion TargetRotation;
            public float rotGradWeight;
        }



    }
}