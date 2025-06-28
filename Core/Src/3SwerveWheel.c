#include "main.h"
#include "ala_Config.h"
#include "3SwerveWheel.h"
#define pi 3.1415
int bienTest = 0;
int dirStopRobot = 0;
//=================================================
//------ Bien toan cuc de dieu khien ------
//=================================================
int part = 0;
int _0hRO = 0, _4hRO = 0, _8hRO = 0, _speed0h = 0, _speed4h = 0, _speed8h = 0;
int _0h = 0, _4h = 0, _8h = 0, _giaToc0 = 0, _giaToc4 = 0, _giaToc8 = 0;
//==============KhaiBaoTam=====================
int _robotChange = 1, IMURobotRun = 0;
int chenhLechTocDo = 0;
int omega = 0;
int TARGET_SPEED = 0;
int currentSpeed = 0;
//===============================================
int initial_angle = 0;
int current_angle = 0;
int deviation_angle = 0;
//==============KhaiCacBienTinhToan==============
#define R 0.71
#define W 1
#define L 1
float V0hx, V0hy, V4hx, V4hy, V8hx, V8hy;
float speed0h, speed4h, speed8h, angle0h, angle4h, angle8h;
//===============================================

// Ð?nh nghia bi?n PIDController
PIDController pidRunStraight;
PIDController pidBamThanhTrai;
PIDController pidBamThanhTraiLui;
PIDController pidGiamTocLzTruoc;

extern int16_t IMU;

int16_t robotAngle(void)
{
	//Goc IMU duoc tinh nhu sau: cung chieu kim dong ho IMU > 0, nguoc chieu IMU < 0
	//return -IMU;
	return -IMU;
}
//int16_t robotAngle_TQ(void)
//{
//	//Goc IMU duoc tinh nhu sau: cung chieu kim dong ho IMU > 0, nguoc chieu IMU < 0
//	return -IMU_china;
//}

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
		if(p++ %3 == 0) {
			if(currentSpeed < 50) currentSpeed+=1;
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
	V0hx = V * cos(gocXoay * pi / 1800);
	V0hy = (V * sin(gocXoay * pi / 1800)) + omega;
	V4hx = V * cos((gocXoay - 1200) * pi / 1800);
	V4hy = (V * sin((gocXoay - 1200) * pi / 1800)) + omega;
	V8hx = V * cos((gocXoay - 2400) * pi / 1800);
	V8hy = (V * sin((gocXoay - 2400) * pi / 1800)) + omega;
	
	//=============Tinh toan goc 0h=============
	speed0h = sqrt(pow(V0hx,2) + pow(V0hy,2));
	angle0h = atan2(V0hy,V0hx) * 1800 / pi;
	//=============Tinh toan goc 4h=============
	speed4h = sqrt(pow(V4hx,2) + pow(V4hy,2));
	angle4h = atan2(V4hy,V4hx) * 1800 / pi;
	//=============Tinh toan goc 8h=============
	speed8h = sqrt(pow(V8hx,2) + pow(V8hy,2));
	angle8h = atan2(V8hy,V8hx) * 1800 / pi;
	
	_0hRO = angle0h;
	_4hRO = angle4h;
	_8hRO = angle8h;
	
	_0h = gioiHanTocDo(speed0h);
	_4h = gioiHanTocDo(speed4h);
	_8h = gioiHanTocDo(speed8h);
}
//void robotStop(int doCung){
//	int tocdo = update_speed(2);
//	TARGET_SPEED = 2;
//	_0h = tocdo;
//	_4h = tocdo;
//	_8h = tocdo;
//}
void robotStop(int doCung){
	_robotChange = 1;
	int tocdo = update_speed(2);
	TARGET_SPEED = 2;
	_0h = tocdo;
	_4h = tocdo;
	_8h = tocdo;
	part = findSection(dirStopRobot);
	chenhLechTocDo = -PID_Compute(0.6, 0, 0.4, -250, 250, &pidRunStraight, IMURobotRun, robotAngle());
	
	if(part == 1){
		_0h = tocdo;
		_4h = tocdo - chenhLechTocDo;
		_8h = tocdo + chenhLechTocDo;
	}
	else if(part == 2){
		_0h = tocdo + chenhLechTocDo;
		_4h = tocdo - chenhLechTocDo;
		_8h = tocdo - chenhLechTocDo;
	}
	else if(part == 3){
		_0h = tocdo;
		_4h = tocdo + chenhLechTocDo;
		_8h = tocdo - chenhLechTocDo;
	}
	else if(part == 4){
		_0h = tocdo - chenhLechTocDo;
		_4h = tocdo + chenhLechTocDo;
		_8h = tocdo + chenhLechTocDo;
	}
	if(tocdo == 2){
		if(!RED_BLUE){
			_0hRO = 0;
			_4hRO = 0;
			_8hRO = 0;
		}
		if(doCung == 2){
			_0h = 2;
			_4h = 2;
			_8h = 2;
		}
		else {
			_0h = 0;
			_4h = 0;
			_8h = 0;
		}
	}
}
int findSection(float angle) {
    // Chuan hóa góc v tu 0 do 3600 do
    float normalizedAngle = fmod(fmod(angle, 3600) + 3600, 3600);

    // Xác dinh cac phan cua hình tròn
    if ((normalizedAngle > 3150 && normalizedAngle < 3600) || (normalizedAngle >= 0 && normalizedAngle <= 450)) {
        return 1; // Phan 1
    } else if (normalizedAngle <= 1350) {
        return 2; // Phan 2
    } else if (normalizedAngle <= 2250) {
        return 3; // Phan 3
    } else {
        return 4; // Phan 4
    }
}
void robotRun(int dir,int tocdo_target){ // Co IMU
	//TARGET_SPEED = tocdo;
	int tocdo = update_speed(tocdo_target);
	int buTruDir = 0;
	//Xem truoc do robot co thay doi goc hay khong
	if(_robotChange){
		osDelay(50);
		if(_robotChange){
			IMURobotRun = robotAngle();
			_robotChange = 0;
		}
	}
	//Gan bien part bang truong hop nao tu bien dir
	part = findSection(dir);

	chenhLechTocDo = -PID_Compute(0.6, 0, 0.4, -250, 250, &pidRunStraight, IMURobotRun, robotAngle());
	buTruDir = IMURobotRun - robotAngle();
	//buTruDir = 0;
	
	if(part == 1){
		_0h = tocdo;
		_4h = tocdo - chenhLechTocDo;
		_8h = tocdo + chenhLechTocDo;
	}
	else if(part == 2){
		_0h = tocdo + chenhLechTocDo;
		_4h = tocdo - chenhLechTocDo;
		_8h = tocdo - chenhLechTocDo;
	}
	else if(part == 3){
		_0h = tocdo;
		_4h = tocdo + chenhLechTocDo;
		_8h = tocdo - chenhLechTocDo;
	}
	else if(part == 4){
		_0h = tocdo - chenhLechTocDo;
		_4h = tocdo + chenhLechTocDo;
		_8h = tocdo + chenhLechTocDo;
	}
	
	// Gioi han lai toc do
	if(_0h >= 254)  _0h = 254;
	else if(_0h <= 3) _0h = 3;
	
	if(_4h >= 254)  _4h = 254;
	else if(_4h <= 3) _4h = 3;
	
	if(_8h >= 254)  _8h = 254;
	else if(_8h <= 3) _8h = 3;
	
	_0hRO = dir - buTruDir;
	_4hRO = dir - 1200 - buTruDir;
	_8hRO = dir + 1200 - buTruDir;
	dirStopRobot = dir;
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
	_4h = tuyetDoiTocDo;
	_8h = tuyetDoiTocDo;
}
void runGrab(){

}

void runAngle(int goc, int tocDoChay, int tocDoXoay){
//	if(_robotChange){
//		IMURobotRun = robotAngle();
//		_robotChange = 0;
//	}
	// Tính toán d? l?ch
	deviation_angle = -(goc - robotAngle())%3600;
	
//	if (deviation_angle > 1800) {
//			deviation_angle -= 3600;
//	} else if (deviation_angle < -1800) {
//			deviation_angle += 3600;
//	}
	
	runSnake(deviation_angle, tocDoChay, tocDoXoay);
}