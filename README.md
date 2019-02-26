[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)
# Robotic Arm Pick & Place Project
---

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
---
### Forward Kinematic Analysis

### Kuka KR210 robot DH parameters.

```python
# DH Table
DH_Table = {alpha0:      0, a0:      0, d1:  0.75, q1:        q1,
            alpha1: -pi/2., a1:   0.35, d2:     0, q2: -pi/2.+q2,
            alpha2:      0, a2:   1.25, d3:     0, q3:        q3,
            alpha3: -pi/2., a3: -0.054, d4:  1.50, q4:        q4,
            alpha4:  pi/2., a4:      0, d5:     0, q5:        q5,
            alpha5: -pi/2., a5:      0, d6:     0, q6:        q6,
            alpha6:      0, a6:      0, d7: 0.303, q7:         0}
```

### Creating the individual transformation matrices about each joint:

```python
# Homogeneous transform matrix function

def TF_Mat(alpha, a, d, q):
    TF = Matrix([[            cos(q),           -sin(q),           0,             a],
                 [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                 [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                 [                 0,                 0,           0,             1]])
    return TF
```
Now we substitute the DH parameters into the transformation matrix: 

```python
   ## Substitute DH_Table
T0_1 = TF_Mat(alpha0, a0, d1, q1).subs(DH_table)
T1_2 = TF_Mat(alpha1, a1, d2, q2).subs(DH_table)
T2_3 = TF_Mat(alpha2, a2, d3, q3).subs(DH_table)
T3_4 = TF_Mat(alpha3, a3, d4, q4).subs(DH_table)
T4_5 = TF_Mat(alpha4, a4, d5, q5).subs(DH_table)
T5_6 = TF_Mat(alpha5, a5, d6, q6).subs(DH_table)
T6_EE = TF_Mat(alpha6, a6, d7, q7).subs(DH_table)

```
rotation correction URDF versus DH Convention: 180 degree rotation on the Z axis, and -90 degree on the y axis

```python
rz = Matrix([[-1, 0,0, 0],
             [0, -1, 0, 0],
             [0, 0, 1, 0],
             [0, 0, 0, 1]])
             
ry = Matrix([[0, 0, -1, 0],
             [0, 1, 0, 0],
             [1, 0, 0, 0], 
             [0, 0, 0, 1]])


R_corr = (R_z * R_y)

T_total= (T0_EE * R_corr)
```


## Inverse Kinematics Analysis


### Inverse Position

Collect the end-effector position(**Px, Py, Pz**) and orientation (**Roll, Pitch, Yaw**) from system
```python
    # Requested end-effector (EE) position
    px = req.poses[x].position.x
    py = req.poses[x].position.y
    pz = req.poses[x].position.z
      
    # Requested end-effector (EE) orientation
    (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(
        [req.poses[x].orientation.x,
         req.poses[x].orientation.y,
         req.poses[x].orientation.z,
         req.poses[x].orientation.w])
```

We substitute the obtained roll, pitch and yaw in the final rotation matrix.

```python
 # determine EE rotation matrix RPY (Roll, Pitch, Yaw)
    r = symbols('r')
    p = symbols('p')
    y = symbols('y')

    # Roll
    Rot_X = Matrix([[       1,       0,       0],
                    [       0,  cos(r), -sin(r)],
                    [       0,  sin(r),  cos(r)]])
    # Pitch
    Rot_Y = Matrix([[  cos(p),       0,  sin(p)],
                    [       0,       1,       0],
                    [ -sin(p),       0,  cos(p)]])
    # Yaw
    Rot_Z = Matrix([[  cos(y), -sin(y),       0],
                    [  sin(y),  cos(y),       0],
                    [       0,       0,       1]])

    Rot_EE = Rot_Z * Rot_Y * Rot_X
    
    # apply rotation correction 
    Rot_EE = ROT_EE * R_corr
    
    # End effector final rotation matrix
    Rot_EE = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})
```
we begin the steps to calculate **𝜃1, 𝜃2 and 𝜃3**

```python
    # n-vectors from EE rotation matrix
    nx = Rot_EE[0,2]
	  ny = Rot_EE[1,2]
	  nz = Rot_EE[2,2]
    
    # Calculate Wrest Center
    d6 = 0 # d6 value from DH_Table
	  l = 0.303	    
	  wcx = px - (d6 + l) * nx
	  wcy = py - (d6 + l) * ny
	  wcz = pz - (d6 + l) * nz
```

```python
    # Theta1 formula
    theta1 = atan2(wcy,wcx)
```
Using cosine laws SSS from trigonometry, we can calculate **𝜃2 and 𝜃3**. 

```python
    #triangle sides for theta2 and theta3
    side_a = 1.501 
	  side_b = sqrt(pow((sqrt(wcx * wcx + wcy * wcy) - 0.35), 2) + pow((wcz - 0.75), 2))
    side_c = 1.25
    
    # SSS angles
    angle_a = acos((pow(side_b, 2) + pow(side_c, 2) - pow(side_a, 2))/(2*side_b*side_c))
	  angle_b = acos((pow(side_c, 2) + pow(side_a, 2) - pow(side_b, 2))/(2*side_c*side_a))
    
    # Theta2 and Theta3 formulas
	  theta2 = pi/2 - angle_a - atan2(wcz - 0.75, sqrt(wcx * wcx + wcy * wcy) - 0.35)
	  theta3 = pi/2 - (angle_b + 0.036)
    
``` 

### Inverse Orientation

Next we need to determining the homogeneous rotaion matrix from link_3 to gripper_link. The R3_6 matrix will be used to find values of the final three joint variables **𝜃4, 𝜃5 and 𝜃6**.
```python
    # Homogeneous rotation matrix from base_link to link_3 
    R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
    
    # Evaluate the matrix using the previous calculated theta values
	  R0_3 = R0_3.evalf(subs ={q1: theta1, q2: theta2, q3: theta3})
    
    # Multiplying the transpose of R0_3 with Rot_EE gives us the final homogeneous rotation matrix
    # Note: using an inverse matrix function created a large error which was resolved using a transpose function
	  R3_6 = R0_3.transpose() * Rot_EE

```python
     # **𝜃4, 𝜃5 and 𝜃6** formulas
     theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]),R3_6[1,2]  
     theta4 = atan2(R3_6[2,2], -R3_6[0,2]) 
     theta6 = atan2(-R3_6[1,1],R3_6[1,0])
```

### Note
If errors occure when launching program try.
```
export GAZEBO_MODEL_PATH=~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/models

source ~/catkin_ws/devel/setup.bash
```
