//Author by Nguyen Nho Phong Vu
//Date version: 11/8/2024 version 1
//Date version: 17/8/2024 version 2
#include <stdlib.h>
#include <math.h>

extern int bienTest;
//=================================================
//------ Bien toan cuc de dieu khien ------
//=================================================
extern int _0hRO, _4hRO, _8hRO, _9hRO, _speed0h, _speed3h, _speed6h, _speed9h;
extern int _0h, _4h, _8h, _9h, _giaToc2, _giaToc4, _giaToc8, _giaToc10;
//==============KhaiBaoTam=====================
extern int _robotChange, IMURobotRun;
extern int omega;
extern int TARGET_SPEED;
extern int curentSpeed;
extern int increment; // Bi?n gia tang d? di?u ch?nh t?c d? tang d?n
extern int decrement; // Bi?n gi?m d? di?u ch?nh t?c d? gi?m d?n
//===============================================
extern int initial_angle;
extern int current_angle;
extern int deviation_angle;
//==============KhaiCacBienTinhToan==============
extern float Vx, Vy, A, B, C, D;
extern float V0hx, V0hy, V3hx, V3hy, V6hx, V6hy, V9hx, V9hy;
extern float speed0h, speed3h, speed6h, speed9h, angle0h, angle3h, angle6h, angle9h;
//===============================================
extern int IMU_china;
//extern int16_t IMU;
#ifndef PID_H
#define PID_H

typedef struct {
    float integral;
    float previous_error;
} PIDController;

// Khai báo hàm PID_Compute v?i các tham s? PID tr?c ti?p
float PID_Compute(float Kp, float Ki, float Kd, float minValue, float maxValue, 
                  PIDController *pid, float setpoint, float measured_value);

#endif // PID_H
int robotAngle(void);
int parabola_acceleration(int current_speed, int target_speed, int t);
int parabola_deceleration(int current_speed, int target_speed, int t);
void tinhToanGoc(float gocXoay, float V, float omega);
void robotStop(int doCung);
void runStraight(int goc, int tocDoChay);
void runSnake(int goc, int tocDoChay, float tocDoXoay);
void runGrab();
void stop_rotation();
void runAngle(int goc, int tocDoChay, int tocDoXoay);