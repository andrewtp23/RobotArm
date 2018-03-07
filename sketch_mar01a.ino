#include "kinematics.h"

bool RUN = true;

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
if(RUN)
{
  float j1 = 0.0001;
  float j2 = -90;
  float j3 = 90;
  float j4 = 0;
  float j5 = 0.001;
  float j6 = 0;
  
  // conv to radians.
  // All future calculations use radians instead of degrees.
  j1 = convToRadians(j1);
  j2 = convToRadians(j2);
  j3 = convToRadians(j3 - 90);
  j4 = convToRadians(j4);
  j5 = convToRadians(j5);
  j6 = convToRadians(j6 + 180);

  RUN = false;
  float dh1[4] = {};
  float dh2[4] = {};
  float dh3[4] = {};
  float dh4[4] = {};
  float dh5[4] = {};
  float dh6[4] = {};

  //Theta
  dh1[0] = j1;
  dh2[0] = j2;
  dh3[0] = j3;
  dh4[0] = j4;
  dh5[0] = j5;
  dh6[0] = j6;
  
  //Alpha
  // Each joint rotates around the Z axis. Joints need to be aligned in the DH
  // table in order to proper set up a final rotational matrix.
  dh1[1] = convToRadians(-90);
  dh2[1] = convToRadians(0);
  dh3[1] = convToRadians(90);
  dh4[1] = convToRadians(-90);
  dh5[1] = convToRadians(90);
  dh6[1] = convToRadians(0);

  //d
  // Total distance of length at each joint
  dh1[2] = 169.77;
  dh2[2] = 0;
  dh3[2] = 0;
  dh4[2] = -222.63;
  dh5[2] = 0;
  dh6[2] = -36.25;

  //a
  // Offset of each joint.
  dh1[3] = 64.2;
  dh2[3] = 305;
  dh3[3] = 0;
  dh4[3] = 0;
  dh5[3] = 0;
  dh6[3] = 0;

  float **jf1 = createJointFrame(dh1);
  float **jf2 = createJointFrame(dh2);
  float **r1 = createRotationFrame(jf1, WORKFRAME);

  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 4; j++){
      Serial.println( r1[i][j]);
    }
  }
  /*
  float **jf3 = createJointFrame(dh3);
  float **jf4 = createJointFrame(dh4);
  float **jf5 = createJointFrame(dh5);
  float **jf6 = createJointFrame(dh6);

  for(int i = 0; i < 4; i++)
    Serial.println(dh1[i], 10);

  for(int i = 0; i < 4; i++)
  {
    for(int j = 0; j < 4; j++)
    {
      Serial.print((float)jf1[i][j], 6);
      Serial.print(" ");
    }
    Serial.println();
  }*/
 
}
}

