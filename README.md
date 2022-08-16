# Unity Burst Inverse Kinematics (BurstIK)
This project aims to provide an inverse kinematics (IK) solution for Unity using the new Burst compiler.
It uses data oriented parallel processing for multiple IK systems.
We tested the code using Unity 2020.3.12f1 with Burst 1.4.11.

### Minimum requirements
- Unity 2020
- Burst Compiler v1.4.11
- (Optional) High definition render pipeline (HDRP) v10.5.0

### Features
- Inverse kinematics for an arbitrary series of hinge and translation joints
- Parallel execution using the Burst compiler
- Support for a vast amount of robot instances (35 six axis robots take less than 0.4ms to compute)

### Limitations
- Calculation weights can only be set globally (IKManager)
- No path planning &rarr; Needs smooth movement of the IK target
- Single value joints &rarr; Translation or rotation around one axis at a time
    - **Workaround:** Use multiple single joints in a hierarchy

## Setup
There are two ways to include this package in your project.
1. Download the Unity package from the release tab **(Recommended)**
1. Clone the latest git repository version (Contains a whole Unity project)

There are two Unity packages available, one that only contains the code and one with an whole example project.
For the package with examples, your project has to fulfill all optional requirements for it to work properly.

After downloading the Unity package, follow these instruction to import BurstIK to your project: https://docs.unity3d.com/Manual/AssetPackagesImport.html
