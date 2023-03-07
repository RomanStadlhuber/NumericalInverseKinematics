# Numerical Inverse Kinematics using On-Manifold Optimization

This repository contains the Matlab code and Simulink Block Library to implement numerical inverse kinematics inside a simulink project.<br/>
Invese kinematics are computed using the [Levenberg-Marquardt Algorithm](https://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm), also referred to as Damped Least Squares, which optimizes the gobal distance between the endeffector and the goal pose on the 3D rigid motion group SE(3).

This library includes the following features:

- _Auto-differentiation_ (i.e. computing the Jacobian) for arbitrary serial open chains.
- _MC sample based initialization_, that is no initial guess needs to be provided.
- _Customization of the damping factor_ via the Simulink block mask.
- _Custom weighting factors_ for the pose error via block input ports.

**Please remark** that this solver only optimizes a kinematic constraint and therefore does not take into account any kinetic properties of the serial open chain. This is experimental code, do not use in a production envirnoment.

## Setup and Quick Start

Besides the example code, the following files are absolutely required in order for the block to work:

- `DampedLeastSquaresIK.m` - the Level 2 S-Function that implements the solver.
- `dlsik_lib.slx` - the simulink block library.
- `adjointSE3.m` - computes the SE(3) Adjoint of a transform, used to transfer local twists to the Lie-Algebra
- `spaceJacobian.m` - computes the global interpretation of the Jacobian matrix used during optimization.
- `monteCarloInitialGuess.m` - generates an approximative initial guess using MC sampling.

### Simulink

To include the custom IK block into your simulink project, clone this repository directly into the project. The `dlsik_lib.slx` file contains the block relevant Simulink block. Open this file, then drag and drop the block into your Simulink project.

The input and output connections of the block are similar to that of the [Inverse Kinematics Block provided by MathWorks](https://de.mathworks.com/help/robotics/ref/inversekinematics.html). It only differs in terms of the configuration that needs to be done using the mask interface, which will be detailed further below. The following image displays an example setup of both blocks side-by-side.

![IK Block Example Setup](simulink_example.png)

For further details on the connection setup, please refer to the [official IK example](https://de.mathworks.com/help/robotics/ug/trace-end-effector-ik-simulink.html) provided by MathWorks.

#### Block Mask Parameters

In order for the block to work, more information is required.

- A reference to the [Rigid Body Tree](https://de.mathworks.com/help/robotics/ref/rigidbodytree.html) object.
- The name of the endeffector body, the frame transform of which is used as the plant state.
- The scalar value of the damping factor that is used to formulate the constraint.
- A maximum number of allowed iterations before aborting the optimization process.
- The acceptance threshold for a solution.
- Whether or not the block should compute its own initial guess.

Below is an example of the configuration that can be made for the block.

![Configuration Mask of the IK Block](ik-block_mask.png)

Experiments with the UR10 6DOF arm have shown fast convergence and sufficient results with a damping factor of `0.125`. The robot was able to reach its goal without an initial guess.

## Matlab Examples - Understanding the Code

In order to get a better understanding of what is happening underneath the hood, some standalone example scripts are provided.

- `simpleRobotTest.m` - which implements the LM optimization on _Example 6.1_ in _"Modern Robotics"_ [(Lynch and Park, 2017)](http://hades.mech.northwestern.edu/images/7/7f/MR.pdf).
- `exampleUR10.m` and `examplePanda.m` - both concisely demonstrate quick usage by constructing a rigid body tree object, desired trajectory waypoints and invoking the solver method.

You can modify one of the examples to load your own robot and get an idea on how to setup an IK solver for it.

### Creating a Rigid Body Tree

Create a matrix contaning the DH parameters of the robot. Set the desired joint types in an additional array.

```matlab
% Denavit-Hartenberg parameters:
%           A           alpha   d       theta
dhparams = [0           pi/2    0.1273      0
            -0.312      0       0           0
            -0.5723     0       0           0
            0           pi/2    0.163941    0
            0           -pi/2   0.1157      0
            0           0       0.0922      0];

jTypes = ["revolute" "revolute" "revolute" "revolute" "revolute" "revolute"];
```

The function `getRigidBodytree` contsructs a tree object that can be used for optimization.

```matlab
robot = getRigidBodyTree(dhparams, jTypes);
```

Also, don't forget to provide the name of the endeffector such that it can be used for auto-differentiation.

```matlab
tcpName = char(robot.BodyNames(robot.NumBodies));
```

### Configuring and Invoking the Optimizer

Given a set of waypoint transformations, represented by 4 by 4 affine transform matrices (the waypoint list ultimately being of size `[4 4 n]` for a set of `n` waypoints), the following information configures the solver:

```matlab
weights = [0 0 0 1 1 1]; % use the first three for orientation and the latter for translation
initialGuess = homeConfiguration(robot);
minDistance = 1e-5;
maxIterations = 150;
```

An initial guess using selective random sampling may optionally be provided:

```matlab
initialGuess = monteCarloInitialGuess(robot, tcpName, waypoints(:,:,1));
```

The function `traceTrajectory` then generates both the trajectory traced by the endeffector using the optimized solutions as well as the joint-angle values themselves.

```matlab
[outTrajectory, outJointStates] = traceTrajectory(robot, tcpName, waypoints, maxIterations, minDistance, weights, initialGuess);
```

### Plotting the Results

To give visual feedback of how well the desired endeffector-trajectory could be traced, call the `viz` function using the output information.

```matlab
viz(robot, outTrajectory, targetPositions, outJointStates);
```

For further details, refer to the provided examples and the literature mentioned below.

## Further Resources

### Remarks

In this library, a manifold representation in SE(3) was chosen in favor of the reduced geometric form introduced in (Buss, 2009) in order to provide more generality and avoid the problem of gimbal lock that might arise when using Euler-angles. The Levenberg-Marquardt optimization algorithm was favored instead of the Newton-Rhapson Method - which is used by (Lynich and Park, 2017) - to guarantee stability in the neighborhood of local minima. Approximation and control is computed in the Lie-algebra se(3) instead of the local tangent space purely out of personal preference.

### Sources

Below is a list of recommended readings about the topics of Lie-Groups, On-Manifold Optimization and Inverse Kinematics for serial manipulators.

- [A micro Lie Theory for state estimation in robotics](https://arxiv.org/abs/1812.01537), Solà, Deray and Atchuthan - a gentle introduction into Lie-groupds, specifically aimed at roboticists.
  - There exists a [complementary online lecture](https://www.youtube.com/watch?v=nHOcoIyJj2o) from one of the authors, aiming to provide an overview to the topic.
- [A tutorial on SE(3) transformation parameterizations and on-manifold optimization](https://ingmec.ual.es/~jlblanco/papers/jlblanco2010geometry3D_techrep.pdf), Blanco Claraco - an overview to the state-of-the-art landscape of on-manifold optimization techniques.
- [Modern Robotics - Mechanics, Planning and Control](http://hades.mech.northwestern.edu/images/7/7f/MR.pdf), Lynch and Park - a group-theoretic approach to dealing with problems in robot mechanics.
- [Introduction to Inverse Kinematics with Jacobian Transpose, Pseudoinverse and Damped Least Squares methods](http://graphics.cs.cmu.edu/nsp/course/15-464/Spring11/handouts/iksurvey.pdf), Buss - an overview of the Damped Least Squares approach to inverse kinematics for serial manipulators.
- Naive Lie Theory, Stillwell - a more in-depth and general mathematical discussion of various Lie-groups (it does not mention SE(3) but the math applies there just as well).
