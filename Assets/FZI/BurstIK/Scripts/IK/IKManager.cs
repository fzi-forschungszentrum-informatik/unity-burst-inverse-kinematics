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

//#define IK_SOLUTION_EXPERIMENTAL

using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using Unity.Jobs;
using Unity.Collections;
using Unity.Burst;

/**
 * This is an IK implementation using Burst and Data Oriented programming.
 * It is a global management script which needs to be added ONCE to a scene.
 * 
 * Author: Pascal Zwick
 * e-mail: zwick@fzi.de
 * */

namespace BurstIK
{
    public class IKManager : MonoBehaviour
    {
        //Gradient stepsize definition
        const float GRAD_EPS = 1e-2f;
        const float GRAD_EPS_SHIFT = 1e-3f;

        //Instance for singleton usage (ONLY use one instance in the scene)
        public static IKManager instance;

        //Define Weights for Gradient descent function calculation
        public float DefaultRotationWeight = 8.0f;
        public float DefaultLocationWeight = 64.0f;
        public float DefaultPenaltyWeight = 128.0f;

        public int IterationsPerFrame = 1;


        //Keeps track of all IK Instances in the scene
        private List<IKSystem> ik_systems;


        //Buffers holding joint data for various compute jobs.
        //!!!Memory management needs to be done manually!!!
        private NativeArray<RobotJoint.JointData> buffer_joint_data;
        private NativeArray<ComputeForwardIO> data_io;

#if IK_SOLUTION_EXPERIMENTAL
    private NativeArray<ProcessData> ikProcessData;
#endif

        //Indicates, if the data buffers need to be reloaded
        private bool bufferDirty;

        //Initalizes stuff
        void Awake()
        {
            instance = this;

            ik_systems = new List<IKSystem>();
        }

        // Updates all IKs using Burst compiled multithreading
        void Update()
        {
            //Rebuild buffers if the dirty flag is set
            if (bufferDirty)
            {
                RebuildBuffers();

                bufferDirty = false;
            }

            //Time measurement stuff

            /* ########################################
             * Update buffer data with current targets
             * ########################################
             */
            //float t0 = Time.realtimeSinceStartup;

            int k = 0;
            for (int i = 0; i < ik_systems.Count; ++i)
            {
                IKSystem s = ik_systems[i];

                for (int j = 0; j < s.joints.Length; ++j)
                {
                    RobotJoint.JointData jd = buffer_joint_data[k];

                    jd.Target = s.Target.transform.position;
                    jd.TargetRotation = s.Target.transform.rotation;
                    jd.rotGradWeight = s.EnableRotationalTarget ? DefaultRotationWeight : 0;

                    buffer_joint_data[k] = jd;

                    ++k;
                }

                ComputeForwardIO io = data_io[i];
                io.localToWorldMatrix = s.joints[0].transform.parent.localToWorldMatrix;

                data_io[i] = io;
            }

            /* ########################################
             * Actually compute inverse kinematics
             * ########################################
             */

            JobHandle forwardHandle, inverseHandle;
            for (int i = 0; i < IterationsPerFrame; ++i)
            {
                //Dispatch forward kinematics jobs
#if IK_SOLUTION_EXPERIMENTAL
            ComputeForwardKinematics forwardJob = new ComputeForwardKinematics { data = buffer_joint_data, data_io = data_io, processData = ikProcessData };
#else
                ComputeForwardKinematics forwardJob = new ComputeForwardKinematics { data = buffer_joint_data, data_io = data_io };
#endif
                forwardHandle = forwardJob.Schedule(ik_systems.Count, 1);

                //Dispatch Inverse kinematics jobs
                ComputeInverseGradientDescent inverseJob = new ComputeInverseGradientDescent { data = buffer_joint_data, deltaT = Time.deltaTime, locationWeight = DefaultLocationWeight, penaltyWeight = DefaultPenaltyWeight };
                inverseHandle = inverseJob.Schedule(buffer_joint_data.Length, 4, forwardHandle);

                JobHandle.CompleteAll(ref forwardHandle, ref inverseHandle);
            }


            /* ########################################
             * Apply computed IK values
             * ########################################
             */
            k = 0;
            for (int i = 0; i < ik_systems.Count; ++i)
            {
                IKSystem s = ik_systems[i];

                for (int j = 0; j < s.joints.Length; ++j)
                {
                    RobotJoint.JointData jd = buffer_joint_data[k];

                    s.joints[j].ApplyForwardKinematics(jd.value, jd.localPosition, jd.localRotation);

                    ++k;
                }

            }

            /* ########################################
             * Debugging
             * ########################################
             */
            //Debug.Log("T: " + (Time.realtimeSinceStartup - t0) * 1000.0f + " ms");

            //for (int i = 0; i < ik_solvers.Count; ++i)
            //Debug.Log(ik_solvers[i].transform.parent.gameObject.name + " " + data_io[i].position + " " + ik_solvers[i].Target.position);

        }

        //Rebuilds data buffers to fit current IK solvers
        private void RebuildBuffers()
        {
            //Dispose current buffers if created: free memory
            if (buffer_joint_data.IsCreated)
            {
                buffer_joint_data.Dispose();
                data_io.Dispose();

#if IK_SOLUTION_EXPERIMENTAL
            ikProcessData.Dispose();
#endif
            }


            /* ########################################
             * Create new buffer instances.
             * Fill buffers with data.
             * ########################################
             */
            List<RobotJoint.JointData> l_joints = new List<RobotJoint.JointData>();
            data_io = new NativeArray<ComputeForwardIO>(ik_systems.Count, Allocator.TempJob);

            //Loop over all IK solvers
            for (int i = 0; i < ik_systems.Count; ++i)
            {
                IKSystem s = ik_systems[i];

                ComputeForwardIO io;
                io.offset = new int2(l_joints.Count, s.joints.Length);
                io.position = 0;
                io.rotation = quaternion.identity;
                io.localToWorldMatrix = s.joints[0].transform.parent.localToWorldMatrix;

                data_io[i] = io;

                //Loop over joints of current solver and store data in buffers
                for (int j = 0; j < s.joints.Length; ++j)
                {
                    RobotJoint joint = s.joints[j];
                    RobotJoint.JointData jd;
                    jd.type = joint.Type;
                    jd.axis = joint.Axis;
                    jd.value = 0f;
                    jd.validValueInterval = joint.ValidValueInterval;
                    jd.maxSpeed = joint.MaxSpeed;

                    jd.localPosition = joint.transform.localPosition;
                    jd.localRotation = joint.transform.localRotation;

                    jd.P = 0;
                    jd.dP = 0;
                    jd.Q = quaternion.identity;
                    jd.dQ = quaternion.identity;

                    jd.Target = 0;
                    jd.TargetRotation = quaternion.identity;
                    jd.rotGradWeight = 0;
                    jd.curGrad = 0;

                    l_joints.Add(jd);
                }

            }


            buffer_joint_data = new NativeArray<RobotJoint.JointData>(l_joints.ToArray(), Allocator.Persistent);

#if IK_SOLUTION_EXPERIMENTAL
        ikProcessData = new NativeArray<ProcessData>(l_joints.Count, Allocator.Persistent);
#endif

            Debug.Log("[IKManager] Rebuilded Buffer: solvers: " + ik_systems.Count + ", buffer_size=" + l_joints.Count);

        }

        //Called by IKSolver when activated
        public void RegisterIK(IKSystem solver)
        {
            ik_systems.Add(solver);

            bufferDirty = true;
        }

        //Called by IKSolver when destroyed / deactivated
        public void UnregisterIK(IKSystem solver)
        {
            ik_systems.Remove(solver);

            bufferDirty = true;
        }


        //Clean up memory when scene is destroyed
        private void OnDestroy()
        {
            if (buffer_joint_data.IsCreated)
            {
                buffer_joint_data.Dispose();
                data_io.Dispose();

#if IK_SOLUTION_EXPERIMENTAL
            ikProcessData.Dispose();
#endif
            }
        }


        /* ########################################
         *               JOB SECTION 
         * ########################################
         */

        //struct for forward kinematics calculation
        public struct ComputeForwardIO
        {
            public int2 offset;//offset, joint count
            public float4x4 localToWorldMatrix;

            public float3 position;
            public quaternion rotation;
        }

#if IK_SOLUTION_EXPERIMENTAL
    public struct ProcessData
    {
        public float3 jP;
        public quaternion jQ;
        public float3 dP;
        public quaternion dQ;
    }
#endif
        /*
         * Computes forward kinematics for one specific IKSolver.
         */
        [BurstCompile]
        public struct ComputeForwardKinematics : IJobParallelFor
        {
            [NativeDisableParallelForRestriction]
            public NativeArray<RobotJoint.JointData> data;

            public NativeArray<ComputeForwardIO> data_io;

#if IK_SOLUTION_EXPERIMENTAL
        [NativeDisableParallelForRestriction]
        public NativeArray<ProcessData> processData;
#endif

            //Called automatically by C# Job system.
            public void Execute(int index)
            {
                //Fetch data for current execution index
                ComputeForwardIO io = data_io[index];
                int2 offset = io.offset;


#if IK_SOLUTION_EXPERIMENTAL
            /* ########################################
             * Forward Pass + Derivative
             * Calculates f(W), df/dw_i, for all joints w_i in joint set W.
             * W represents all joints of current IKSolver
             * Calculation is done in 3 passes (forward -> backward -> forward)
             * ########################################
             */

            float3 cP = 0.0f;
            quaternion cQ = quaternion.identity;

            //Forward
            for (int i = 0; i < offset.y; ++i)
            {
                RobotJoint.JointData j0 = data[i + offset.x];
                ProcessData pd = processData[i + offset.x];

                //Apply joint offset and rotation
                cP += math.mul(cQ, j0.localPosition);

                pd.jP = cP;
                pd.jQ = cQ;

                if (i != offset.y - 1)
                {
                    //Shift joints don't change rotation, so only change translation. Joint rotation can differ from previous, so it also needs multiplicaton.
                    if (j0.type == RobotJoint.JointType.SHIFT)
                    {
                        cP += math.mul(cQ, j0.axis * j0.value);
                        cQ = math.mul(cQ, j0.localRotation);
                    }
                    //Hinge joints only change rotation, 
                    else if (j0.type == RobotJoint.JointType.HINGE)
                    {
                        cQ = math.mul(cQ, math.mul(j0.localRotation, quaternion.AxisAngle(j0.axis, math.radians(j0.value))));
                    }
                }

                processData[i + offset.x] = pd;
            }

            //Backward
            float3 dP = 0.0f;
            quaternion dQ = quaternion.identity;

            ProcessData pd0 = processData[offset.y - 1];
            pd0.jP = 0.0f;
            pd0.jQ = quaternion.identity;
            processData[offset.y - 1] = pd0;

            for (int i = offset.y - 2; i >= 0; --i)
            {
                RobotJoint.JointData j0 = data[i + offset.x];
                RobotJoint.JointData j1 = data[i + offset.x + 1];
                ProcessData pd = processData[i + offset.x];

                //Apply joint offset and rotation
                dP += j1.localPosition;

                pd.dP = dP;
                pd.dQ = dQ;

                quaternion lq = math.mul(j0.localRotation, quaternion.AxisAngle(j0.axis, math.radians(j0.value)));
                dP = math.mul(lq, dP);

                //Shift joints don't change rotation, so only change translation. Joint rotation can differ from previous, so it also needs multiplicaton.
                if (j0.type == RobotJoint.JointType.SHIFT)
                {
                    dQ = math.mul(j0.localRotation, dQ);
                    dP += j0.axis * j0.value;
                }
                //Hinge joints only change rotation, 
                else if (j0.type == RobotJoint.JointType.HINGE)
                {
                    dQ = math.mul(lq, dQ);
                }

                processData[i + offset.x] = pd;
            }

            //Derivative Forward
            io.position = math.mul(io.localToWorldMatrix, new float4(cP, 1)).xyz;
            io.rotation = math.mul(new quaternion(io.localToWorldMatrix), cQ);
            data_io[index] = io;

            for (int i = 0; i < offset.y - 1; ++i)
            {
                RobotJoint.JointData j0 = data[i + offset.x];
                ProcessData pd = processData[i + offset.x];

                float3 p = pd.jP;
                quaternion q = pd.jQ;

                //Shift joints don't change rotation, so only change translation. Joint rotation can differ from previous, so it also needs multiplicaton.
                if (j0.type == RobotJoint.JointType.SHIFT)
                {
                    p += math.mul(q, j0.axis * (j0.value + GRAD_EPS_SHIFT));
                    q = math.mul(q, j0.localRotation);
                }
                //Hinge joints only change rotation, 
                else if (j0.type == RobotJoint.JointType.HINGE)
                {
                    q = math.mul(q, math.mul(j0.localRotation, quaternion.AxisAngle(j0.axis, math.radians(j0.value + GRAD_EPS))));
                }

                p = p + math.mul(q, pd.dP);
                q = math.mul(q, pd.dQ);
                
                j0.dP = math.mul(io.localToWorldMatrix, new float4(p, 1)).xyz;
                j0.dQ = math.mul(new quaternion(io.localToWorldMatrix), q);
                j0.P = io.position;
                j0.Q = io.rotation;

                data[i + offset.x] = j0;
            }

#else
                float3 prev = data[offset.x].localPosition;
                quaternion rotation = quaternion.identity;

                //Init temp buffers
                NativeArray<float3> prevs = new NativeArray<float3>(offset.y - 1, Allocator.Temp);//TODO delete and use directly joint data
                NativeArray<quaternion> rotations = new NativeArray<quaternion>(offset.y - 1, Allocator.Temp);

                //First joint is always the base, so it is the reference.
                prevs[0] = data[offset.x].localPosition;
                rotations[0] = quaternion.identity;

                /* ########################################
                 * Forward Pass
                 * Calculates f(W), df/dw_i, for all joints w_i in joint set W.
                 * W represents all joints of current IKSolver
                 * ########################################
                 */
                for (int i = 0; i < offset.y - 1; ++i)
                {
                    RobotJoint.JointData j0 = data[i + offset.x];
                    RobotJoint.JointData j1 = data[i + offset.x + 1];

                    //Shift joints don't change rotation, so only change translation. Joint rotation can differe from previous, so it also needs multiplicaton.
                    if (j0.type == RobotJoint.JointType.SHIFT)
                    {
                        prev += math.mul(rotation, j0.axis * j0.value);
                        rotation = math.mul(rotation, j0.localRotation);

                        for (int j = 0; j < i; ++j)
                        {
                            prevs[j] += math.mul(rotations[j], j0.axis * j0.value);
                            rotations[j] = math.mul(rotations[j], j0.localRotation);
                        }
                        prevs[i] += math.mul(rotations[i], j0.axis * (j0.value + GRAD_EPS_SHIFT));
                        rotations[i] = math.mul(rotations[i], j0.localRotation);
                    }
                    //Hinge joints only change rotation, 
                    else if (j0.type == RobotJoint.JointType.HINGE)
                    {
                        rotation = math.mul(rotation, math.mul(j0.localRotation, quaternion.AxisAngle(j0.axis, math.radians(j0.value))));

                        for (int j = 0; j < i; ++j)
                        {
                            rotations[j] = math.mul(rotations[j], math.mul(j0.localRotation, quaternion.AxisAngle(j0.axis, math.radians(j0.value))));
                        }
                        rotations[i] = math.mul(rotations[i], math.mul(j0.localRotation, quaternion.AxisAngle(j0.axis, math.radians(j0.value + GRAD_EPS))));
                    }

                    //Apply joint offset and rotation
                    prev += math.mul(rotation, j1.localPosition);

                    for (int j = 0; j <= i; ++j)
                    {
                        prevs[j] += math.mul(rotations[j], j1.localPosition);
                    }
                    for (int j = i + 1; j < offset.y - 1; ++j)
                    {
                        prevs[j] = prev;
                        rotations[j] = rotation;
                    }
                }

                /* ########################################
                 *              OUTPUT DATA  
                 * Output end effector rotation, position in world space
                 * as well as related to derivatives df/dw_i
                 * ########################################
                 */

                float4x4 rm = io.localToWorldMatrix;
                float3 c0 = math.normalize(new float3(rm.c0.x, rm.c0.y, rm.c0.z));
                float3 c1 = math.normalize(new float3(rm.c1.x, rm.c1.y, rm.c1.z));
                float3 c2 = math.normalize(new float3(rm.c2.x, rm.c2.y, rm.c2.z));
                float3x3 rm3 = new float3x3(c0, c1, c2);

                io.position = math.mul(io.localToWorldMatrix, new float4(prev, 1)).xyz;
                io.rotation = math.mul(new quaternion(rm3), rotation);
                data_io[index] = io;

                for (int i = 0; i < offset.y - 1; ++i)
                {
                    RobotJoint.JointData j0 = data[i + offset.x];

                    j0.dP = math.mul(io.localToWorldMatrix, new float4(prevs[i], 1)).xyz;
                    j0.dQ = math.mul(new quaternion(rm3), rotations[i]);
                    j0.P = io.position;
                    j0.Q = io.rotation;

                    data[i + offset.x] = j0;
                }

                //free memory of temp buffers
                prevs.Dispose();
                rotations.Dispose();
#endif
            }
        }

        /*
         * Computes inverse kinematics using output of forward pass.
         * Does basic momentum gradient descent.
         * Computation is done for every joint.
         */
        [BurstCompile]
        public struct ComputeInverseGradientDescent : IJobParallelFor
        {
            public NativeArray<RobotJoint.JointData> data;

            //Time delta, i.e. Time.fixedDeltaTime
            public float deltaT;
            public float locationWeight;
            public float penaltyWeight;

            float rotNorm(float3 a, float3 b)
            {
                return 1 - math.dot(a, b);
            }

            float linstep(float a, float b, float x)
            {
                return math.clamp((x - a) / (b - a), 0, 1);
            }

            //Called by C# job system for every joint.
            public void Execute(int index)
            {
                //fetch joint data
                RobotJoint.JointData jd = data[index];

                //Calculate differentials
                float3x3 mQ = new float3x3(jd.Q);
                float3x3 mdQ = new float3x3(jd.dQ);
                float3x3 mTQ = new float3x3(jd.TargetRotation);

                //Calculates true penalty and gradient of penalty. Punishes values going towards edges of valid joint interval.
                float pen0 = math.exp(jd.validValueInterval.x - jd.value) + math.exp(jd.value - jd.validValueInterval.y);
                float pen1 = math.exp(jd.validValueInterval.x - jd.value - (jd.type == RobotJoint.JointType.HINGE ? GRAD_EPS : GRAD_EPS_SHIFT)) + math.exp(jd.value + (jd.type == RobotJoint.JointType.HINGE ? GRAD_EPS : GRAD_EPS_SHIFT) - jd.validValueInterval.y);

                //Calculates true distance and derivative distance to target. 
                //float d0 = locationWeight * math.lengthsq(jd.Target - jd.P) + jd.rotGradWeight * (math.lengthsq(mQ.c0 - mTQ.c0) + math.lengthsq(mQ.c1 - mTQ.c1) + math.lengthsq(mQ.c2 - mTQ.c2)) + pen0 * penaltyWeight;
                //float d1 = locationWeight * math.lengthsq(jd.Target - jd.dP) + jd.rotGradWeight * (math.lengthsq(mdQ.c0 - mTQ.c0) + math.lengthsq(mdQ.c1 - mTQ.c1) + math.lengthsq(mdQ.c2 - mTQ.c2)) + pen1 * penaltyWeight;

                float d0 = locationWeight * math.lengthsq(jd.Target - jd.P) + jd.rotGradWeight * (rotNorm(mQ.c0, mTQ.c0) + rotNorm(mQ.c1, mTQ.c1) + rotNorm(mQ.c2, mTQ.c2)) + pen0 * penaltyWeight;
                float d1 = locationWeight * math.lengthsq(jd.Target - jd.dP) + jd.rotGradWeight * (rotNorm(mdQ.c0, mTQ.c0) + rotNorm(mdQ.c1, mTQ.c1) + rotNorm(mdQ.c2, mTQ.c2)) + pen1 * penaltyWeight;

                //calculate gradient dPdv with normalization 
                float dPdv = (d1 - d0) / (jd.type == RobotJoint.JointType.HINGE ? GRAD_EPS : GRAD_EPS_SHIFT);

                //Apply momentum, i.e. moving average
                dPdv = math.lerp(jd.curGrad, dPdv, 0.2f);

                //Reweight gradient <==> Adaptive stepsize
                //float reweight = math.smoothstep(0, 0.004f, math.abs(dPdv)) * math.sign(dPdv) * math.smoothstep(0, jd.maxSpeed * deltaT * 0.25f, d0);
                float reweight = linstep(0, 0.004f, math.abs(dPdv)) * math.sign(dPdv) * linstep(0, jd.maxSpeed * deltaT * 0.25f, d0);

                //Apply value change with deltaT
                jd.value -= jd.maxSpeed * reweight * deltaT;
                jd.curGrad = dPdv;

                data[index] = jd;
            }
        }

    }
}