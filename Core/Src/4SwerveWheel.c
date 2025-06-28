#include "4SwerveWheel.h"
#include "ala_Config.h"
#define pi 3.1415
int bienTest = 0;
//=================================================
//------ Bien toan cuc de dieu khien ------
//=================================================
int part = 0;
int _0hRO = 0, _3hRO = 0, _6hRO = 0, _9hRO = 0, _speed0h = 0, _speed3h = 0, _speed6h = 0, _speed9h = 0;
int _0h = 0, _3h = 0, _6h = 0, _9h = 0, _giaToc2 = 0, _giaToc4 = 0, _giaToc8 = 0, _giaToc10 = 0;
//==============KhaiBaoTam=====================
int chenhLechTocDo = 0;
int _robotChange = 1, IMURobotRun = 0;
int omega = 0;
int TARGET_SPEED = 0;
int currentSpeed = 0;
#define MAX_SPEED 200          // T?c d? t?i da
#define ACCELERATION_LIMIT 500 // Gi?i h?n gia t?c (tùy ch?nh)
#define DECELERATION_LIMIT 20000 // Gi?i h?n gi?m t?c (tùy ch?nh)
int increment = 0; // Bi?n gia tang d? di?u ch?nh t?c d? tang d?n
int decrement = 0; // Bi?n gi?m d? di?u ch?nh t?c d? gi?m d?n
//===============================================
int initial_angle = 0;
int current_angle = 0;
int deviation_angle = 0;
//==============KhaiCacBienTinhToan==============
#define R 0.71
#define W 1
#define L 1
float Vx, Vy, A, B, C, D;
float V0hx, V0hy, V3hx, V3hy, V6hx, V6hy, V9hx, V9hy;
float speed0h, speed3h, speed6h, speed9h, angle0h, angle3h, angle6h, angle9h;
//===============================================

// Ð?nh nghia bi?n PIDController
PIDController pidRunStraight;
PIDController pidBamThanhTrai;
PIDController pidBamThanhTraiLui;
PIDController pidGiamTocLzTruoc;

extern int16_t IMU;

int robotAngle(void)
{
	//Goc IMU duoc tinh nhu sau: cung chieu kim dong ho IMU > 0, nguoc chieu IMU < 0
	return -IMU;
}
// Tinh toan PID
float PID_Compute(float Kp, float Ki, float Kd, float minValue, float maxValue, 
                  PIDController *pid, float setpoint, float measured_value) {
    float error = setpoint - measured_value;

    // Tính toán ph?n tích phân
    pid->integral += error;

    // Tính toán ph?n d?o hàm (n?u c?n)
    float derivative = error - pid->previous_error;
    pid->previous_error = error;

    // Tính toán d?u ra PID
    float output = (Kp * error) + (Ki * pid->integral) + (Kd * derivative);

    // Gi?i h?n output trong kho?ng minValue và maxValue
    if (output > maxValue) {
        output = maxValue;
    } else if (output < minValue) {
        output = minValue;
    }

    return output;
}

//====Ham tra ve banh co toc do lon nhat
int findMaxValue() {
    int max_value = _0h;
    int result = 2;

    if (_3h > max_value) {
        max_value = _3h;
        result = 4;
    }

    if (_6h > max_value) {
        max_value = _6h;
        result = 8;
    }

    if (_9h > max_value) {
        max_value = _9h;
        result = 10;
    }

    return result;
}
int gioiHanTocDo(int speed){
	if(speed > 250) speed = 250;
	else if(speed <= 3) {
		if(TARGET_SPEED == 2) speed = 2;
		else speed = 3;
	}
	return speed;
}
int update_speed(int target_speed) {
	static char p = 0;
	if(currentSpeed < target_speed) {
		if(p++ %2 == 0) {
			if(currentSpeed < 50) currentSpeed+=2;
			else currentSpeed+=3;
			if(currentSpeed >= target_speed) currentSpeed = target_speed;
		}
	}
	else if(currentSpeed > target_speed) {
		if(p++ %2 == 0){
			if(currentSpeed > 150)	currentSpeed-=3;
			else currentSpeed-=3;
			if(currentSpeed <= target_speed) currentSpeed = target_speed;
		}
	}
	return currentSpeed;
}
//=======================End cac ham gia toc============================
void tinhToanGoc(float gocXoay, float V, float omega){
	_robotChange = 1;
	Vx = V * cos(gocXoay * pi / 1800);
	Vy = V * sin(gocXoay * pi / 1800);
	A = Vx - (omega*L)/2;
	B = Vx + (omega*L)/2;
	C = Vy - (omega*W)/2;
	D = Vy + (omega*L)/2;
	
	//=============Tinh toan goc 2h=============
	V0hx = A; //B - A
	V0hy = D; //C - D
	speed0h = sqrt(pow(A,2) + pow(D,2));
	angle0h = atan2(D,A) * 1800 / pi;
	//angle0h = atan2(C,B);
	//===========End tinh toan goc 2h===========
	
	//=============Tinh toan goc 3h=============
	V3hx = A;
	V3hy = C;
	speed3h = sqrt(pow(A,2) + pow(C,2));
	angle3h = atan2(C,A) * 1800 / pi;
	//angle3h = atan2(C,A);
	//===========End tinh toan goc 3h===========
	
	//=============Tinh toan goc 6h=============
	V6hx = B; //A - B
	V6hy = C; //D - C
	speed6h = sqrt(pow(B,2) + pow(C,2));
	angle6h = atan2(C,B) * 1800 / pi;
	//angle6h = atan2(D,A);
	//===========End tinh toan goc 6h===========
	
	//=============Tinh toan goc 9h=============
	V9hx = B;
	V9hy = D;
	speed9h = sqrt(pow(B,2) + pow(D,2));
	angle9h = atan2(D,B) * 1800 / pi;
	//angle9h = atan2(D,B);
	//===========End tinh toan goc 9h===========
	
	_0hRO = angle0h;
	_3hRO = angle3h;
	_6hRO = angle6h;
	_9hRO = angle9h;
	
	_0h = speed0h;
	_3h = speed3h;
	_6h = speed6h;
	_9h = speed9h;
}
void robotStop(int doCung){
	int tocdo = update_speed(2);
	TARGET_SPEED = 2;
	_0h = tocdo;
	_3h = tocdo;
	_6h = tocdo;
	_9h = tocdo;

}
int findSection(float angle) {
    // Chu?n hóa góc v? kho?ng 0 d?n 360 d?
    float normalizedAngle = fmod(fmod(angle, 3600) + 3600, 3600);

    // Xác d?nh ph?n c?a hình tròn
    if ((normalizedAngle > 3375 && normalizedAngle <= 3600) || (normalizedAngle >= 0 && normalizedAngle <= 225)) {
        return 1; // Ph?n 1
    } else if (normalizedAngle <= 675) {
        return 2; // Ph?n 2
    } else if (normalizedAngle <= 1125) {
        return 3; // Ph?n 3
    } else if (normalizedAngle <= 1575) {
        return 4; // Ph?n 4
    } else if (normalizedAngle <= 2025) {
        return 5; // Ph?n 5
    } else if (normalizedAngle <= 2475) {
        return 6; // Ph?n 6
    } else if (normalizedAngle <= 2925) {
        return 7; // Ph?n 7
    } else {
        return 8; // Ph?n 8
    }
}
void robotRun(int dir,int tocdo_target){ // Co IMU
	
	//TARGET_SPEED = tocdo;
	int tocdo = update_speed(tocdo_target);
	if(_robotChange){
		osDelay(50);
		if(_robotChange){
			IMURobotRun = robotAngle();
			_robotChange = 0;
		}
	}
	part = findSection(dir);
	
	chenhLechTocDo = PID_Compute(0.6, 0, 0.4, -250, 250, &pidRunStraight, IMURobotRun, robotAngle());
	if(part == 1){ //7
		_0h = tocdo + chenhLechTocDo;
		_3h = tocdo + chenhLechTocDo;
		_6h = tocdo - chenhLechTocDo;
		_9h = tocdo - chenhLechTocDo;
	}
	else if(part == 2){ //2
		_0h = tocdo;
		_3h = tocdo + chenhLechTocDo;
		_6h = tocdo;
		_9h = tocdo - chenhLechTocDo;
	}
	else if(part == 3){ //3
		_0h = tocdo - chenhLechTocDo;
		_3h = tocdo + chenhLechTocDo;
		_6h = tocdo + chenhLechTocDo;
		_9h = tocdo - chenhLechTocDo;
	}
	else if(part == 4){ //4
		_0h = tocdo - chenhLechTocDo;
		_3h = tocdo;
		_6h = tocdo + chenhLechTocDo;
		_9h = tocdo;
	}
	else if(part == 5){ //5
		_0h = tocdo - chenhLechTocDo;
		_3h = tocdo - chenhLechTocDo;
		_6h = tocdo + chenhLechTocDo;
		_9h = tocdo + chenhLechTocDo;
	}
	else if(part == 6){ //6
		_0h = tocdo;
		_3h = tocdo - chenhLechTocDo;
		_6h = tocdo;
		_9h = tocdo + chenhLechTocDo;
	}
	else if(part == 7){ //7
		_0h = tocdo + chenhLechTocDo;
		_3h = tocdo - chenhLechTocDo;
		_6h = tocdo - chenhLechTocDo;
		_9h = tocdo + chenhLechTocDo;
	}
	else{
		_0h = tocdo + chenhLechTocDo;
		_3h = tocdo;
		_6h = tocdo - chenhLechTocDo;
		_9h = tocdo;
	}
	
	// Gioi han lai toc do
	if(_0h >= 254)  _0h = 254;
	else if(_0h <= 3) _0h = 3;
	
	if(_3h >= 254)  _3h = 254;
	else if(_3h <= 3) _3h = 3;
	
	if(_6h >= 254)  _6h = 254;
	else if(_6h <= 3) _6h = 3;
	
	if(_9h >= 254)  _9h = 254;
	else if(_9h <= 3) _9h = 3;
	///////////////////////////
	_0hRO = dir;
	_3hRO = dir;
	_6hRO = dir;
	_9hRO = dir;
	//Neu khong xai gia toc
	
}
void runSnake(int goc, int tocDoChay, float heSoCong){
	_robotChange = 1;
	if(tocDoChay > 165) tocDoChay = 165;
	int tocdo = update_speed(tocDoChay);
	tinhToanGoc(goc, tocdo, tocdo/heSoCong);
}
void robot_rotate(int tocDo){
	_robotChange = 1;
	tinhToanGoc(0, 0, tocDo);
	int tuyetDoiTocDo = abs(tocDo);
	_0h = tuyetDoiTocDo;
	_3h = tuyetDoiTocDo;
	_6h = tuyetDoiTocDo;
	_9h = tuyetDoiTocDo;
}
void runGrab(){

}

void stop_rotation() {
	// Reset giá tr? d? l?ch v? 0
	deviation_angle = 0;
}

void runAngle(int goc, int tocDoChay, int tocDoXoay){
//	if(_robotChange){
//		IMURobotRun = robotAngle();
//		_robotChange = 0;
//	}
	// Tính toán d? l?ch
	deviation_angle = (goc - robotAngle()) / 10;
	
	// Ði?u ch?nh d? l?ch d? n?m trong kho?ng -180 d?n 180 d?
	if (deviation_angle > 180) {
			deviation_angle -= 360;
	} else if (deviation_angle < -180) {
			deviation_angle += 360;
	}
	
	runSnake(deviation_angle, tocDoChay, tocDoXoay);
}