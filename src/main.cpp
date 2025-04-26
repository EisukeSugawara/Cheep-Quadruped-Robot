#include <Arduino.h>
#include <ArduinoEigenDense.h>

using namespace Eigen;

// 各リンクの長さを定義
const float length_coxa = 30.0;
const float length_femur = 60.0;
const float length_tibia = 80.0;

// 足先座標を格納する構造体
struct FootPos {
  float x;
  float y;
  float z;
};

// 座標変換を行う関数
FootPos coordTf(float x, float y, float z) {

  Matrix3f rotateMatrix;
  rotateMatrix << 0, 1, 0,
                  -1, 0,0,
                  0, 0, 1;

  Vector3f vectorBody;
  vectorBody << x,
                y,
                z;

  Vector3f translationVector;
  translationVector << -50,
                        50,
                        0;

  Vector3f result = rotateMatrix * vectorBody + translationVector;

  FootPos foot_pos_leg = {result(0), result(1), result(2)};

  return foot_pos_leg;
}

// 関節角度を格納する構造体
struct JointAngle {
  float coxa;
  float femur;
  float tibia;
};

// 逆運動学を行う関数
JointAngle IK(FootPos foot_pos_leg) {

  JointAngle angle;

  float r = sqrt(foot_pos_leg.x*foot_pos_leg.x + foot_pos_leg.y*foot_pos_leg.y);
  float s = sqrt((r - length_coxa)*(r - length_coxa) + foot_pos_leg.z*foot_pos_leg.z);
  float theta_ABC = acos((length_femur*length_femur + length_tibia*length_tibia - s*s) / (2*length_femur*length_tibia));
  float theta_alpha = atan2(-foot_pos_leg.z, r - length_coxa);
  float theta_BAC = acos((length_femur*length_femur + s*s - length_tibia*length_tibia) / (2*length_femur*s));

  angle.coxa = atan2(foot_pos_leg.y, foot_pos_leg.x) * 180 / PI;  // theta_coxa
  angle.femur = (theta_alpha - theta_BAC) * 180.0 / PI;           // theta_femur
  angle.tibia = (PI - theta_ABC) * 180.0 / PI;                    // theta_tibia

  return angle;
};

void setup(){
  Serial.begin(115200);

  delay(5000);

  FootPos foot_pos_leg = coordTf(50, 140, -80);
  
  Serial.printf("foot_pos_leg.x: %f\n", foot_pos_leg.x);
  Serial.printf("foot_pos_leg.y: %f\n", foot_pos_leg.y);
  Serial.printf("foot_pos_leg.z: %f\n", foot_pos_leg.z);
  Serial.printf("\n");

  JointAngle angle_0 = IK(foot_pos_leg);

  Serial.printf("leg_0.coxa: %f\n", angle_0.coxa);
  Serial.printf("leg_0.femur: %f\n", angle_0.femur);
  Serial.printf("leg_0.tibia: %f\n", angle_0.tibia);

}

void loop() {}