#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <math.h>


#define MATRIX_SIZE 4

int Q_J1 = 0;
int Q_J5  = 0;

float J1_ANGLE = 0.0001;
float J2_ANGLE = -90;
float J3_ANGLE = 90;
float J4_ANGLE = 0;
float J5_ANGLE = 0.01;
float J6_ANGLE = 0;

float J1_d = 169.77;
float J2_d = 0;
float J3_d  = 0;
float J4_d  = -222.63;
float J5_d  = 0;
float J6_d  = -36.25;

float J1_a  = 64.2;
float J2_a  = 305;
float J3_a  = 0;
float J4_a  = 0;
float J5_a  = 0;
float J6_a  = 0;

/*
   Convert degrees to radians.
   params: degrees
   return: float
*/
float convToRadians(float degrees)
{
  float pi = 3.14159265;
  return (degrees * pi) / 180;
}

float convToDegrees(float radians)
{
  float pi = 3.14159265;
  return (180 * radians) / pi;
}


/*
   Generates a workframe used for matrix multiplication.
   This is used for calcuating the Rotational Matrix for the first Joint
   and the final joint.
   params: frame - A matrix to build the workframe onto.
   return: frame by reference.
*/
void generateWorkFrame(float frame[MATRIX_SIZE][MATRIX_SIZE]) {
  for (int i = 0; i < MATRIX_SIZE; i++) {
    for (int j = 0; j < MATRIX_SIZE; j++)
    {
      if (i == j)
        frame[i][j] = 1;
      else
        frame[i][j] = 0;
    }
  }
}


/*
   Calculate a single joint frame based on DH params. Only a row is used as input
   per joint.
   params: Single DH array row.
   params in array: Theta, alpha, d, a
   return: joint frame by reference
*/
void createJointFrame(float input_frame[MATRIX_SIZE], float joint_frame[MATRIX_SIZE][MATRIX_SIZE])
{
  //Initialize joint frame array
  for (int i = 0; i < MATRIX_SIZE; i++) {
    for (int j = 0; j < MATRIX_SIZE; j++)
    {
      joint_frame[i][j] = 0;
    }
  }

  // Setup the first row of the joint frame
  joint_frame[0][0] = cosf(input_frame[0]);
  joint_frame[0][1] = -sinf(input_frame[0]) * cosf(input_frame[1]);
  joint_frame[0][2] = sinf(input_frame[0]) * sinf(input_frame[1]);
  joint_frame[0][3] = input_frame[3] * cosf(input_frame[0]);

  // Setup the second row of the joint frame
  joint_frame[1][0] = sinf(input_frame[0]);
  joint_frame[1][1] = cosf(input_frame[0]) * cosf(input_frame[1]);
  joint_frame[1][2] = -1 * cosf(input_frame[0]) * sinf(input_frame[1]);
  joint_frame[1][3] = input_frame[3] * sinf(input_frame[0]);

  // Setup the third row of the joint frame
  joint_frame[2][0] = 0;
  joint_frame[2][1] = sinf(input_frame[1]);
  joint_frame[2][2] = cosf(input_frame[1]);
  joint_frame[2][3] = input_frame[2];

  // Setup the fourth row of the joint frame
  joint_frame[3][0] = 0;
  joint_frame[3][1] = 0;
  joint_frame[3][2] = 0;
  joint_frame[3][3] = 1;

}


/*
   Calculate rotational matrix for joint
   params: frame 1 and frame 2
   return: Rotation frame via reference
*/
void createRotationFrame(float f1[MATRIX_SIZE][MATRIX_SIZE], float f2[MATRIX_SIZE][MATRIX_SIZE],
                         float rot_frame[MATRIX_SIZE][MATRIX_SIZE])
{
  //Initialize rot_frame array
  for (int i = 0; i < MATRIX_SIZE; i++) {
    for (int j = 0; j < MATRIX_SIZE; j++)
    {
      rot_frame[i][j] = 0;
    }
  }

  for (int i = 0; i < MATRIX_SIZE; i++) {
    for (int j = 0; j < MATRIX_SIZE; j++) {
      for (int k = 0; k < MATRIX_SIZE; k++)
      {
        rot_frame[i][j] += f1[i][k] * f2[k][j];
      }
    }
  }
}


void multiply3x3(float f1[3][3], float f2[3][3], float rot_frame[3][3])
{
  //Initialize rot_frame array
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++)
    {
      rot_frame[i][j] = 0;
    }
  }

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++)
      {
        rot_frame[i][j] += f1[i][k] * f2[k][j];
      }
    }
  }
}

/*
   Calculate a 3x3 inverse matrix
   Calculation based on method from https://www.thecrazyprogrammer.com/2017/02/c-c-program-find-inverse-matrix.html
   params: input matrix, output matrix
   return by reference: output matrix
*/
void createInverseMatrix(float input[3][3], float output[3][3])
{
  // Find the determinant
  float determinant = 0;
  for (int i = 0; i < 3; i++)
    determinant = determinant + (input[0][i] * (input[1][(i + 1) % 3] * input[2][(i + 2) % 3] - input[1][(i + 2) % 3] * input[2][(i + 1) % 3]));

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      output[i][j] = ((input[(j + 1) % 3][(i + 1) % 3] * input[(j + 2) % 3][(i + 2) % 3]) - (input[(j + 1) % 3][(i + 2) % 3] * input[(j + 2) % 3][(i + 1) % 3]) / determinant);
    }
  }
}


void createRoll_Rotation(float z, float arr[3][3])
{
  float zr = convToRadians(z);
  
  // First row
  arr[0][0] = cos(zr);
  arr[0][1] = -sin(zr);
  arr[0][2] = 0;
  
  // Second Row
  arr[1][0] = sin(zr);
  arr[1][1] = cos(zr);
  arr[1][2] = 0;
  
  // Third Row
  arr[2][0] = 0;
  arr[2][1] = 0;
  arr[2][2] = 1;
}


void createPitch_Rotation(float y, float arr[3][3])
{
  float yr = convToRadians(y);
  
  // First row
  arr[0][0] = cos(yr);
  arr[0][1] = 0;
  arr[0][2] = sin(yr);
  
  // Second Row
  arr[1][0] = 0;
  arr[1][1] = 1;
  arr[1][2] = 0;
  
  // Third Row
  arr[2][0] = -sin(yr);
  arr[2][1] = 0;
  arr[2][2] = cos(yr);
}


void createYaw_Rotation(float x, float arr[3][3])
{
  float xr = convToRadians(x);
  
  // First row
  arr[0][0] = 1;
  arr[0][1] = 0;
  arr[0][2] = 0;
  
  // Second Row
  arr[1][0] = 0;
  arr[1][1] = cos(xr);
  arr[1][2] = -sin(xr);
  
  // Third Row
  arr[2][0] = 0;
  arr[2][1] = sin(xr);
  arr[2][2] = cos(xr);
}

/*
   Calculate forward kinematics given angles for each joint
   params: Angles for joints 1-6, Answer matrix to hold the distances acheived

   return:
   final_matrix[0] = X distance
   final_matrix[1] = Y distance
   final_matrix[2] = Z distance
   final_matrix[3] = Not used. The 1 is used for rotational matrix math but
   is not used in the final_matrix

*/
void forwardKinematics(float j1, float j2, float j3,
                       float j4, float j5, float j6,
                       float final_matrix[MATRIX_SIZE][MATRIX_SIZE])
{
  float wf[4][4] = {};
  generateWorkFrame(wf);

  // conv to radians.
  // All future calculations use radians instead of degrees.
  j1 = convToRadians(j1);
  j2 = convToRadians(j2);
  j3 = convToRadians(j3 - 90);
  j4 = convToRadians(j4);
  j5 = convToRadians(j5);
  j6 = convToRadians(j6 + 180);

  float dh1[MATRIX_SIZE] = {};
  float dh2[MATRIX_SIZE] = {};
  float dh3[MATRIX_SIZE] = {};
  float dh4[MATRIX_SIZE] = {};
  float dh5[MATRIX_SIZE] = {};
  float dh6[MATRIX_SIZE] = {};

  // Assign Theta
  dh1[0] = j1;
  dh2[0] = j2;
  dh3[0] = j3;
  dh4[0] = j4;
  dh5[0] = j5;
  dh6[0] = j6;

  // Assign Alpha
  // Each joint rotates around the Z axis. Joints need to be aligned in the DH
  // table in order to proper set up a final rotational matrix.
  dh1[1] = convToRadians(-90);
  dh2[1] = convToRadians(0);
  dh3[1] = convToRadians(90);
  dh4[1] = convToRadians(-90);
  dh5[1] = convToRadians(90);
  dh6[1] = convToRadians(0);

  // Assign d
  // Total distance of length at each joint
  dh1[2] = J1_d;
  dh2[2] = J2_d;
  dh3[2] = J3_d;
  dh4[2] = J4_d;
  dh5[2] = J5_d;
  dh6[2] = J6_d;

  // Assign a
  // Offset of each joint.
  dh1[3] = J1_a;
  dh2[3] = J2_a;
  dh3[3] = J3_a;
  dh4[3] = J4_a;
  dh5[3] = J5_a;
  dh6[3] = J6_a;

  // Create Joint frames. Each joint frame will be used to calculate Rotational Matrixes
  float jf1[MATRIX_SIZE][MATRIX_SIZE] = {};
  float jf2[MATRIX_SIZE][MATRIX_SIZE] = {};
  float jf3[MATRIX_SIZE][MATRIX_SIZE] = {};
  float jf4[MATRIX_SIZE][MATRIX_SIZE] = {};
  float jf5[MATRIX_SIZE][MATRIX_SIZE] = {};
  float jf6[MATRIX_SIZE][MATRIX_SIZE] = {};

  createJointFrame(dh1, jf1);
  createJointFrame(dh2, jf2);
  createJointFrame(dh3, jf3);
  createJointFrame(dh4, jf4);
  createJointFrame(dh5, jf5);
  createJointFrame(dh6, jf6);

// Calculate where J1 is in Quad. This is used for Inverse Kinematics
  if(jf1[1][3] < 0){
    if(jf1[0][3] < 0 )
      Q_J1 = 1;
    else
      Q_J1 = 2;
  }
  else{
    if(jf1[0][3] < 0)
      Q_J1 = 4;
    else
      Q_J1 = 3;
  }
  
// Calculate where J5 is in Quad. Used for IK
  if(jf5[1][3] < 0){
    if(jf5[0][3] < 0 )
      Q_J5 = 1;
    else
      Q_J5 = 2;
  }
  else{
    if(jf5[0][3] < 0)
      Q_J5 = 4;
    else
      Q_J5 = 3;
  }
  // Create Rotational Matrixes. Each matrix is based off of the current joint
  // and the compounded rotational matrix. The final rotational matrix is a culmination
  // of all of the rotational matrixes.
  // The joints start at the base and work its way to the wrist.
  // The final rotational matrix contains the X, Y, Z coordinates for the tip of the wrist/claw.

  float rot1[MATRIX_SIZE][MATRIX_SIZE] = {};
  float rot2[MATRIX_SIZE][MATRIX_SIZE] = {};
  float rot3[MATRIX_SIZE][MATRIX_SIZE] = {};
  float rot4[MATRIX_SIZE][MATRIX_SIZE] = {};
  float rot5[MATRIX_SIZE][MATRIX_SIZE] = {};
  float rot6[MATRIX_SIZE][MATRIX_SIZE] = {};
  float rotf[MATRIX_SIZE][MATRIX_SIZE] = {};

  createRotationFrame(jf1, wf, rot1);
  createRotationFrame(rot1, jf2, rot2);
  createRotationFrame(rot2, jf3, rot3);
  createRotationFrame(rot3, jf4, rot4);
  createRotationFrame(rot4, jf5, rot5);
  createRotationFrame(rot5, jf6, rot6);
  createRotationFrame(wf, rot6, rotf);
  

  // Set needed values to final matrix. We don't care about anything else.
  // This will send the X, Y, Z, and 1 as an answer.
  for (int i = 0; i < MATRIX_SIZE; i++)
    for (int j = 0; j < MATRIX_SIZE; j++)
      final_matrix[i][j] = rotf[i][j];
}


/*
    Calculate inverse kinematics for entire arm.
    Given a target destination, this will return angles for each joint to be in
    order for arm to reach the target.

    Calculations are split between the wrist and the arm. Yaw, Pitch, and Roll cut down
    the number of ways to reach the desired location. Supplying Yaw, Pitch, and Roll
    orients the wrist in one way and the arm joints are calculated based on them.

    params: X pos, Y pos, Z pos, Yaw, Pitch, Roll,
    return by reference: array of angles for joints 1-6.
*/
void inverseKinematics(float x_pos, float y_pos, float z_pos,
                  float yaw, float pitch, float roll,
                  float angle_array[6])
{
  // Create DH parameters for joints 1-3.
  // These parameters allow us to start with the wrist and work backwards to
  // the base.

  // Use the current joint positions to start calculations.
  float dh1[MATRIX_SIZE] = {};
  float dh2[MATRIX_SIZE] = {};
  float dh3[MATRIX_SIZE] = {};
  float dh4[MATRIX_SIZE] = {};
  float dh5[MATRIX_SIZE] = {};

  // Use current joint angle to fill first DH parameter.
  // The current angles allow us to create an inverse matrix to be connected
  // to the wrist.
  dh1[0] = convToRadians(J1_ANGLE);
  dh2[0] = convToRadians(J2_ANGLE);
  dh3[0] = convToRadians(J3_ANGLE - 90);
  dh4[0] = convToRadians(J4_ANGLE);
  dh5[0] = convToRadians(J5_ANGLE);

  // Insert joint orientation as parameters.
  dh1[1] = convToRadians(-90);
  dh2[1] = convToRadians(0);
  dh3[1] = convToRadians(90);
  dh4[1] = convToRadians(-90);
  dh5[1] = convToRadians(90);

  // Joint distances (d)
  dh1[2] = J1_d;
  dh2[2] = J2_d;
  dh3[2] = J3_d;
  dh4[2] = J4_d;
  dh5[2] = J5_d;

  // Joint offsets (a)
  dh1[3] = J1_a;
  dh2[3] = J2_a;
  dh3[3] = J3_a;
  dh4[3] = J4_a;
  dh5[3] = J5_a;

  // Create workframe for inverse matrix math.
  // This matrix will start off multiplying matrixes.
  float wf[MATRIX_SIZE][MATRIX_SIZE] = {};
  generateWorkFrame(wf);

  // Generate Joint frames 1-3 to be used to calculate inverse matrix
  float jf1[MATRIX_SIZE][MATRIX_SIZE] = {};
  float jf2[MATRIX_SIZE][MATRIX_SIZE] = {};
  float jf3[MATRIX_SIZE][MATRIX_SIZE] = {};
  float jf4[MATRIX_SIZE][MATRIX_SIZE] = {};
  float jf5[MATRIX_SIZE][MATRIX_SIZE] = {};

  createJointFrame(dh1, jf1);
  createJointFrame(dh2, jf2);
  createJointFrame(dh3, jf3);
  createJointFrame(dh4, jf4);
  createJointFrame(dh5, jf5);
  

  // Generate rotational matrixes
  float rot1[MATRIX_SIZE][MATRIX_SIZE] = {};
  float rot2[MATRIX_SIZE][MATRIX_SIZE] = {};
  float rot3[MATRIX_SIZE][MATRIX_SIZE] = {};
  float rot4[MATRIX_SIZE][MATRIX_SIZE] = {};
  float rot5[MATRIX_SIZE][MATRIX_SIZE] = {};
  
  createRotationFrame(jf1, wf, rot1);
  createRotationFrame(rot1, jf2, rot2);
  createRotationFrame(rot2, jf3, rot3);
  createRotationFrame(rot3, jf4, rot4);
  createRotationFrame(rot4, jf5, rot5);


  // Create matrix of rot3's x,y,z that will be used for making an Inverse Matrix
  float rot_base[3][3] = {};
  
  for(int i = 0; i < 3; i++)
    for(int j = 0; j < 3; j++)
      rot_base[i][j] = rot3[i][j];


  float inv[3][3] = {};
  createInverseMatrix(rot_base, inv);

  // Use forward kinematics to create rotational base to start at.

  float fwd_kin[MATRIX_SIZE][MATRIX_SIZE] = {};
  forwardKinematics(J1_ANGLE,J2_ANGLE,J3_ANGLE,J4_ANGLE,J5_ANGLE,J6_ANGLE, fwd_kin);

  // create forward kinematic base
  float fk_base[3][3] = {};
  for(int i = 0; i < 3; i++)
    for(int j = 0; j < 3; j++)
      fk_base[i][j] = fwd_kin[i][j];
      
  // Create Rotation matrixes for Yaw, pitch, roll
  float yaw_array[3][3] = {};
  float pitch_array[3][3] = {};
  float roll_array[3][3] = {};
  
  createRoll_Rotation(roll, roll_array);
  createPitch_Rotation(pitch, pitch_array);
  createYaw_Rotation(yaw, yaw_array);
  
  // Multiply Roll matrix with pitch matrix
  float r_p[3][3] = {};
  multiply3x3(pitch_array, roll_array, r_p);
  float rpy[3][3] = {};
  multiply3x3(r_p, yaw_array, rpy);
  
  // Multiply current ROT with yaw, pitch, roll matrix
  float rotation[3][3] = {};
  multiply3x3(rpy, fk_base, rotation);
  

  // multiply inverse with rotation calculated
  float base[3][3] = {};
  multiply3x3(rotation, inv, base);
  
  // Calculate joint angles

  //J1

  float x_arm_dist = sqrt(pow((rot5[0][3] + x_pos),2) + pow((rot5[1][3] + y_pos),2));
  float x_arm_pos = x_arm_dist + J1_a;
  float x_arm_neg = x_arm_dist - J1_a;

  float y_arm_dist = (rot5[2][3] + z_pos) - J1_d;
  
  float j2dist = 0;
  
  if(Q_J1 == Q_J5)
    j2dist = sqrt(pow(x_arm_neg, 2) + pow(y_arm_dist, 2));
  else
    j2dist = sqrt(pow(x_arm_pos, 2) + pow(y_arm_dist, 2));
  
  float d8 = sqrt(pow(dh3[3],2) + pow(dh4[2],2));
  
  float j1angle = atan((rot5[1][3] +y_pos)/(rot5[0][3] + x_pos));
  
  float theta1 = atan(y_arm_dist/x_arm_neg);
  float theta5 = atan(x_arm_neg/y_arm_dist);
  theta1 = convToDegrees(theta1);
  theta5 = convToDegrees(theta5);
  
  float theta6 = atan(y_arm_dist/x_arm_pos);
  theta6 = convToDegrees(theta6);
  
  float theta2_cos = acos(((pow(dh2[3],2) + pow(j2dist,2) - pow(d8,2))/(2*dh2[3] * j2dist)));
  theta2_cos = convToDegrees(theta2_cos);
  
  float theta2_atan2 = atan2(sqrt(1-((pow(dh2[3],2) + pow(j2dist,2) - pow(d8,2))/(2*dh2[3] * j2dist))), ((pow(dh2[3],2) + pow(j2dist,2) - pow(d8,2))/(2*dh2[3] * j2dist)));
  theta2_atan2 = convToDegrees(theta2_atan2);
  
  // Calculate J2
  float j2angle = 0;
  if( x_arm_neg > 0 && Q_J1 == Q_J5)
  {
    j2angle = -1 * (theta1 + theta5);
  }
  else 
  {
    if( x_arm_neg > 0 && Q_J5 != Q_J1)
      j2angle = -1 * (180 - theta6 + theta2_cos);
    else
    {
      if(x_arm_neg < 0 && Q_J1 != Q_J5)
        j2angle = -1 * (180 - theta6 + theta2_cos);
      else
        j2angle = -1 * (theta2_cos +theta5 + 90);
    }
  }

  float theta4 = 0;
  
  float j3dist = ((pow(d8,2) + pow(dh2[3],2) - pow(j2dist,2)))/(2*d8 * dh2[3]);
  float j3acos = acos(j3dist);

  float j3atan2 = atan2(j3dist, pow(j3dist,2));

  float j3angle = 180 - convToDegrees(j3atan2) + theta4;
  
  // Calculate yaw, pitch, roll angles
  // Calculate possible angles using quadratic equation
  // Then choose the angle that isn't 180
  

  float j5p = convToDegrees(atan2(sqrt(1 - pow(base[2][2],2)), base[2][2]));

  float j5n = convToDegrees(atan2((-1 * sqrt(1-pow(base[2][2],2))), base[2][2]));

  float j5angle = 0;
  if(J5_ANGLE > 0 && j5p > 0)
    j5angle = j5p;
  else
    j5angle = j5n;
  
  std::cout << "j5p:" <<j5p <<std::endl;
  std::cout <<"j5n" << j5n <<std::endl;
  
  float j4p = convToDegrees(atan2(base[1][2], base[0][2]));
  float j4n = convToDegrees(atan2((-1 * base[1][2]), (-1* base[0][2])));
  
  float j4angle = 0;
  if(j5angle > 0)
    j4angle = j4p;
  else
    j4angle = j4n;
    
  std::cout << j4p << std::endl;
  std::cout << j4n <<std::endl;
  float j6p = 0;
  if(base[2][1] < 0)
    j6p = convToDegrees(atan2(base[2][1],(-1 * base[2][0]))) + 180;
  else
    j6p = convToDegrees(atan2(base[2][1],(-1 * base[2][0]))) - 180;
  float j6n = 0;
  if(base[2][1] < 0)
    j6n = convToDegrees(atan2((-1 * base[2][1]), base[2][0])) - 180;
  else
    j6n = convToDegrees(atan2((-1 * base[2][1]),base[2][0])) + 180;
  float j6angle = 0;
  if(j5angle < 0)
    j6angle = j6n;
  else
    j6angle = j6p;
    
  std::cout << "j6p:"<<j6p <<std::endl;
  std::cout << "j6n:" << j6n <<std::endl;
    
  // Place angles in array.
  angle_array[0] = j1angle;
  angle_array[1] = j2angle;
  angle_array[2] = j3angle;
  angle_array[3] = j4angle;
  angle_array[4] = j5angle;
  angle_array[5] = j6angle;
  
  // Update Current angles
  J1_ANGLE = j1angle;
  J2_ANGLE = j2angle;
  J3_ANGLE = j3angle;
  J4_ANGLE = j4angle;
  J5_ANGLE = j5angle;
  J6_ANGLE = j6angle;
}


#endif
