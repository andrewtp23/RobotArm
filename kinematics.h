#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <math.h>

float convToRadians(float degrees)
{
	float pi = 3.14159265;
	return (degrees * pi)/180;
};

float **WORKFRAME =  {{1,0,0,0},
						            {0,1,0,0},
						            {0,0,1,0},
						            {0,0,0,1}};

						
/*
 * Calculate a single joint frame based on DH params. Only a row is used as input
 * per joint.
 * params: Single DH array row. 
 * params in array: Theta, alpha, d, a
 * return: joint frame
 */ 
 float **createJointFrame(float *input_frame)
{
  float **joint_frame;
  Serial.println(5);
  //Initialize joint frame array
  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 4; j++)
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
  //joint_frame[1][2] = -cosf(input_frame[0]) * sinf(input_frame[1]);
  joint_frame[1][3] = input_frame[3] * sinf(input_frame[0]);

  // Setup the third row of the joint frame
  joint_frame[2][0] = 0;
  joint_frame[2][1] = sinf(input_frame[1]);
  joint_frame[2][2] = cosf(input_frame[1]);
  joint_frame[2][3] = input_frame[2];

  // Setup the third row of the joint frame
  joint_frame[3][0] = 0;
  joint_frame[3][1] = 0;
  joint_frame[3][2] = 0;
  joint_frame[3][3] = 1;
  
  return joint_frame;
};

/*
 * Calculate rotational matrix for joint
 * params: frame 1 and frame 2 
 * return: Rotation frame
 */ 
float **createRotationFrame(float **f1, float **f2)
{
  float **rot_frame;

  //Initialize rot_frame array
  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 4; j++)
    {
      rot_frame[i][j] = 0;
    }
  }

  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 4; j++){
      for(int k = 0; k < 4; k++)
        {
          rot_frame[i][j] += f1[i][k] * f2[k][j];
        }
    }
  }
}
#endif

