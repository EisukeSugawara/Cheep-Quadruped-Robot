#include <Arduino.h>

// 時間変数を定義
unsigned long time1 = 0;
unsigned long time2 = 0;
unsigned long time3 = 0;

// 各リンクの長さを定義
const float l_coxa = 30.0;
const float l_femur = 60.0;
const float l_tibia = 80.0;

const float body_width = 100.0; // ボディの幅
const float body_length = 100.0; // ボディの長さ

// ボディ座標系の初期位置を定義
const float x_origin_body = 50.0;
const float y_origin_body = 140.0;
const float z_origin_body = -80.0;

// 定数：ラジアンから度への変換
const float RAD2DEG = 180.0 / PI;

// ラジアンを度に変換するヘルパー関数
inline float rad2deg(float rad) {
  return rad * RAD2DEG;
}

// 同次変換後の座標をまとめる構造体
struct Coordinates {
  float x;
  float y;
  float z;
};

// 同次変換の関数
Coordinates homogeneousTransformation(float x, float y, float z) {
  Coordinates transformed;
  transformed.x = y - body_length / 2; // rotateMatrix の第1行と translationVector の x 要素
  transformed.y = -x + body_width / 2; // rotateMatrix の第2行と translationVector の y 要素
  transformed.z = z;                   // rotateMatrix の第3行と translationVector の z 要素
  return transformed;
}

// 逆運動学の結果をまとめる構造体
struct JointAngles {
  float theta_coxa;
  float theta_femur;
  float theta_tibia;
};

// 逆運動学の計算関数
JointAngles inverseKinematics(float x, float y, float z) {
  JointAngles angle;

  float r = sqrt(x * x + y * y);
  float s = sqrt((r - l_coxa) * (r - l_coxa) + z * z);
  
  float theta_ABC = acos((l_femur * l_femur + l_tibia * l_tibia - s * s) / (2 * l_femur * l_tibia));
  float theta_alpha = atan2(-z, r - l_coxa);
  float theta_BAC = acos((l_femur * l_femur + s * s - l_tibia * l_tibia) / (2 * l_femur * s));

  angle.theta_coxa  = rad2deg(atan2(y, x)) + 45.0;
  angle.theta_femur = rad2deg(theta_alpha - theta_BAC);
  angle.theta_tibia = rad2deg(PI - theta_ABC) - 90.0; 
  
  return angle;
}


void setup(){
  Serial.begin(115200);

  delay(3000);

  time1 = micros();

  Coordinates transformed = homogeneousTransformation(x_origin_body, y_origin_body, z_origin_body);

  time2 = micros();

  JointAngles angle = inverseKinematics(transformed.x, transformed.y, transformed.z);
  
  time3 = micros();

  Serial.printf("x: %f, y: %f, z: %f\n", transformed.x, transformed.y, transformed.z);
  Serial.printf("theta_coxa: %f\n", angle.theta_coxa);
  Serial.printf("theta_femur: %f\n", angle.theta_femur);
  Serial.printf("theta_tibia: %f\n", angle.theta_tibia);
  Serial.print("\n");  

  Serial.printf("HT_time: %d us\n", time2- time1);
  Serial.printf("IK_time: %d us\n", time3 - time2);
  Serial.print("\n");
}

void loop() {}