#pragma once
#include <cmath>

#define DEG2RAD(x) ((x) * 0.017453293f)//pi/180
#define RAD2DEG(x) ((x) * 57.295779513f)//180/pi

typedef struct{
  float x,y,t;
  void print();
}Pos;

void Pos::print(){
  Serial.print("\nPos: ");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.println(t);
}

// 角度を正規化（-pi ~ pi）
float normalizeAngle(float theta) {
    return theta - 2 * M_PI * floorf((theta + M_PI) / (2 * M_PI));
}

typedef struct{
  float d, t;
} Wall;
