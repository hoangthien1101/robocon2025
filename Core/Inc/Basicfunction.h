#include "gamepad_bluetooth.h"
#include "stm32h7xx_hal.h"
#include "ala_Config.h"
#include "cmsis_os.h"
#include "4SwerveWheel.h"
#include "math.h"
int doiGocXoay = 0, gocBuTruXoay = 0;
int bienDem_Encoder = 0;
int bienNapBanh = 0;
int bit_giuNong = 0;
int Lucbantest = 0;
int BanTren = 0;
int BanDuoi = 0;
int speedXoayNang = 0;
int Bit_lazer = 0;
int speedXoay = 0;
int doigoc = 0;
int tocCham = 0;
extern int LucBan, LucBanXeo, tongLucBan, KC, congLuc, RC_1, RC_2;
extern int increment; // Bi?n gia tang d? di?u ch?nh t?c d? tang d?n
extern int decrement; // Bi?n gi?m d? di?u ch?nh t?c d? gi?m d?n
extern uint8_t RX_UART1[2], RX_UART2[10], RX_UART3[1], RX_UART4[10], RX_UART5[6];
//Khai bao cac bien do line
int errorLinePhai = 0, errorLineTrai = 0, lastGocMX = 0;
int speed = 0;
int targetMamXoay = 0, errorBienTro = 0, errorGocXoay = 0, targetGocChuyen = 0, gocXoay = 0;
float speedRobotLine = 0, erRobotLine = 0, lastErRobotLine = 0, KdRobotLine = 0, KiRobotLine = 0, KpRobotLine = 0;
//************End****************//
//Khai bao cac goc bien tro
#define minMX 1915
int gocRe = minMX + 395, _gocMX = minMX + 262, gocBan2Diem = minMX + 405, gocChuyenGan = minMX + 292, gocChuyenXa = minMX + 232, gocPen = minMX + 45, gocBan3Diem = minMX + 232 ;
//************End****************//
extern char bienTruyenNhan;
extern unsigned char robot_Data [8];
extern uint16_t adc_data[11];
uint32_t _ADC[11],_ADC_SUM[11];
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern volatile int32_t num_over_t1, num_over_t2, num_over_t3, num_over_t4, num_over_t5, num_over_t8, num_over_t10,num_over_t_Ex_Can1,num_over_t_Ex_Can2;
uint32_t lazer_Truoc = 0, lazer_Trai = 0, lazer_Phai = 0;
//===================Khai bao cac bien cho bo banh xe======================
int bit_stop = 0, thoatVongLap = 0;
int testXoa = 0, voHieuHoaDiThang = 1, demCheckHamDieuTocDC = 0;
int old_currentSpeed = 0, bienTangGiaToc = 0;
char bitResetBanhXe = 1, bitTest = 0;
int gocQuayBanh = 0;
int speedGocQuay0h = 0, erGocQuay0h = 0, lasterGocQuay0h = 0, KdGocQuay0h = 0, KiGocQuay0h = 0, KpGocQuay0h = 0;
int speedGocQuay3h = 0, erGocQuay3h = 0, lastErGocQuay3h = 0, KdGocQuay3h = 0, KiGocQuay3h = 0, KpGocQuay3h = 0;
int speedGocQuay6h = 0, erGocQuay6h = 0, lastErGocQuay6h = 0, KdGocQuay6h = 0, KiGocQuay6h = 0, KpGocQuay6h = 0;
int speedGocQuay9h = 0, erGocQuay9h = 0, lastErGocQuay9h = 0, KdGocQuay9h = 0, KiGocQuay9h = 0, KpGocQuay9h = 0;
int TH = 0, check1Lan = 0, gocBanDau = 0;
int erGocquay = 0, heSoQuayGoc = 0;

int bienBreak = 0;
extern float speed0h, speed3h, speed6h, speed9h, angle0h, angle3h, angle6h, angle9h;
//===================End Khai bao cac bien cho bo banh xe======================
//===================Khai bao cac bien cho la ban======================
int16_t accumulated_yaw = 0.0f;
int16_t previous_yaw = 0.0f;
int8_t first_run_yaw = 1;
//extern int IMU_china;
extern int16_t IMU, UWB, GocUWB;
//===================End Khai bao cac bien cho la ban======================
//Khai bao cac bit chuong trinh
uint8_t bit_veHome = 1, bit_banBanh = 0, bit_Napbanh = 0, bit_nangNong = 0, bit_NongBan = 0, bit_thu = 0, bit_gocBan = 0, ena_vitme = 0;
uint8_t bitNgatBanh0h = 0, bitNgatBanh3h = 0, bitNgatBanh6h = 0;
//Ket thuc khai bao cac bit chuong trinh
int angle = 0;       // Góc t? 0 d?n 360 d? (ki?u int)
int magnitude = 0;   // Ð? dài du?ng chéo (t?c d?) t? tâm (ki?u int)
volatile int16_t encoder_count_T1 = 0, encoder_count_T2 = 0, encoder_count_T3 = 0, encoder_count_T4 = 0, encoder_count_T5 = 0, encoder_count_T8 = 0; // Gia tri CNT hien tai (co the am)
volatile int16_t prev_encoder_count_T1 = 0, prev_encoder_count_T2 = 0, prev_encoder_count_T3 = 0, prev_encoder_count_T4 = 0, prev_encoder_count_T5 = 0, prev_encoder_count_T8 = 0; // Gia tri CNT truoc do
volatile int32_t encoder_value_T1 = 0, encoder_value_T2 = 0, encoder_value_T3 = 0, encoder_value_T4 = 0, encoder_value_T5 = 0, encoder_value_T8 = 0; // Bien 32-bit luu gia tri thuc

//=====Khai bao bien toan cuc
int lazer_KhoangCach = 0, bienTroNongBan = 0, lastUWB = 0, currentUWB = 0, countUWB = 0, lastGocNongBan = -1, lazerTrai = 0, lazerPhai = 0;
void putchar1(unsigned char ch)
{
	HAL_UART_Transmit(&huart1, (uint8_t*)&ch,1,0x01FF);
	//UART1->DR = (ch & (uint16_t)0x01FF); 
}
//void resetIMU_TQ()
//{
//	uint8_t unblockIMU[5] = {0xFF, 0XAA, 0X69, 0X88, 0XB5};
//	uint8_t resetDataIMU[5] = {0xFF, 0xAA, 0x01, 0x04, 0x00};
//	uint8_t saveDataIMU[5] = {0xFF, 0XAA, 0X00, 0X00, 0X00};
//	HAL_UART_Transmit(&huart5, unblockIMU,5,0x01FF);
//	HAL_Delay(2);
//	HAL_UART_Transmit(&huart5, resetDataIMU,5,0x01FF);
//	HAL_Delay(2);
//	HAL_UART_Transmit(&huart5, saveDataIMU,5,0x01FF);
//	first_run_yaw = 1;
//}
////Ham update doc la ban Trung Quoc
//void update_accumulated_yaw(int16_t current_yaw) {
//    int16_t delta_yaw;

//    // Neu la lan chay dau tien, thiet lap gia tri ban dau cho previous_yaw
//    if (first_run_yaw) {
//        previous_yaw = current_yaw;
//        first_run_yaw = 0;
//    }

//    // Tính toán s? thay d?i c?a góc yaw
//    delta_yaw = current_yaw - previous_yaw;

//    // Dieu chinh delta_yaw khi vuot qua pham vi -1800 den 1800
//    if (delta_yaw > 1800) {
//        delta_yaw -= 3600;
//    } else if (delta_yaw < -1800) {
//        delta_yaw += 3600;
//    }

//    // Cong don gia tri yaw
//    accumulated_yaw += delta_yaw;

//    // Cap nhat gia tri truoc do
//    previous_yaw = current_yaw;
//		
//		IMU_china = accumulated_yaw;
//}

void putchar3(unsigned char ch)
{
	HAL_UART_Transmit(&huart3, (uint8_t*)&ch,1,0x01FF);
	//UART3->DR = (ch & (uint16_t)0x01FF); 
}
void putchar5(unsigned char ch)
{
	HAL_UART_Transmit(&huart5, (uint8_t*)&ch,1,0x01FF);
	//UART5->DR = (ch & (uint16_t)0x01FF); 
}
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void ADCValue_Control()
{
	//_ADC[8] = adc_data[8];
	static int count,i;
	if(count++ <11)
	{
		for(i = 0; i < 11; i++)
		{
			_ADC_SUM[i] += adc_data[i];
		}
	}
	else 
	{
		for(i = 0; i < 11; i++)
		{
			_ADC[i] = _ADC_SUM[i]/11;
			_ADC_SUM[i] = 0;

		}
		count = 0;
	}
}
//Ham trung gian de quy doi encoder theo ti le banh rang, ghi vao thanh ghi khong ghi duoc so am

int32_t quyDoiEncoder(int32_t encoder){
	int32_t encoderQuyDoi = encoder/11.52;
	return encoderQuyDoi;
}
int32_t TungKhongQuyDoi(int32_t encoder){
	int32_t quydoi = encoder/210.2744;
	return quydoi;
}
//int32_t TungQuyDoi(int32_t encoder){
//	int32_t quydoi = (encoder*3600)/2615;
//	return quydoi;
//}
//int32_t QuanQuyDoi(int32_t encoder){
//	int32_t quydoi = encoder/1;
//	return quydoi;
//}
void update_encoder_value() {
    // Doc gia tri hien tai cua encoder
  encoder_count_T1 = TIM1->CNT;
	encoder_count_T2 = TIM2->CNT;
	encoder_count_T3 = TIM3->CNT;
	encoder_count_T4 = TIM4->CNT;
	encoder_count_T5 = TIM5->CNT;
	encoder_count_T8 = TIM8->CNT;

    // Tinh toan su thay doi
  int16_t delta_T1 = encoder_count_T1 - prev_encoder_count_T1;
	int16_t delta_T2 = encoder_count_T2 - prev_encoder_count_T2;
	int16_t delta_T3 = encoder_count_T3 - prev_encoder_count_T3;
	int16_t delta_T4 = encoder_count_T4 - prev_encoder_count_T4;
	int16_t delta_T5 = encoder_count_T5 - prev_encoder_count_T5;
	int16_t delta_T8 = encoder_count_T8 - prev_encoder_count_T8;

    // Kiem tra tran hoac nguoc tran va dieu chinh delta
    if (delta_T1 > 32767) {
        // Truong hop dem nguoc qua 0 (underflow)
        delta_T1 -= 65536;
    } else if (delta_T1 < -32767) {
        // Truong hop dem tien qua 65535 (overflow)
        delta_T1 += 65536;
    }		
    // Kiem tra tran hoac nguoc tran va dieu chinh delta
    if (delta_T2 > 32767) {
        // Truong hop dem nguoc qua 0 (underflow)
        delta_T2 -= 65536;
    } else if (delta_T2 < -32767) {
        // Truong hop dem tien qua 65535 (overflow)
        delta_T2 += 65536;
    }
    // Kiem tra tran hoac nguoc tran va dieu chinh delta
    if (delta_T3 > 32767) {
        // Truong hop dem nguoc qua 0 (underflow)
        delta_T3 -= 65536;
    } else if (delta_T3 < -32767) {
        // Truong hop dem tien qua 65535 (overflow)
        delta_T3 += 65536;
    }
    // Kiem tra tran hoac nguoc tran va dieu chinh delta
    if (delta_T4 > 32767) {
        // Truong hop dem nguoc qua 0 (underflow)
        delta_T4 -= 65536;
    } else if (delta_T4 < -32767) {
        // Truong hop dem tien qua 65535 (overflow)
        delta_T4 += 65536;
    }
    // Kiem tra tran hoac nguoc tran va dieu chinh delta
    if (delta_T5 > 32767) {
        // Truong hop dem nguoc qua 0 (underflow)
        delta_T5 -= 65536;
    } else if (delta_T5 < -32767) {
        // Truong hop dem tien qua 65535 (overflow)
        delta_T5 += 65536;
    }
    // Kiem tra tran hoac nguoc tran va dieu chinh delta
    if (delta_T8 > 32767) {
        // Truong hop dem nguoc qua 0 (underflow)
        delta_T8 -= 65536;
    } else if (delta_T8 < -32767) {
        // Truong hop dem tien qua 65535 (overflow)
        delta_T8 += 65536;
    }
    // Cong don gia tri vao encoder_value
    encoder_value_T1 += delta_T1;
		encoder_value_T2 += delta_T2;
		encoder_value_T3 += delta_T3;
		encoder_value_T4 += delta_T4;
		encoder_value_T5 += delta_T5;
		encoder_value_T8 += delta_T8;
    // Cap nhat gia tri encoder truoc do
    prev_encoder_count_T1 = encoder_count_T1;
		prev_encoder_count_T2 = encoder_count_T2;
		prev_encoder_count_T3 = encoder_count_T3;
		prev_encoder_count_T4 = encoder_count_T4;
		prev_encoder_count_T5 = encoder_count_T5;
		prev_encoder_count_T8 = encoder_count_T8;
}

//so xung 1 vong / 3600 = ti le
int32_t en_0hRo() //TIMER 4
{
	int32_t encoderT4;
	encoderT4 = encoder_value_T4;
	return -quyDoiEncoder(encoderT4);
}
int32_t en_Chay4h() //TIMER 1
{
	int32_t encoderT1;
	encoderT1 = encoder_value_T1;
	return abs(quyDoiEncoder(encoderT1)/10);
}
int32_t en_4hRo() //TIMER 2
{
	int32_t encoderT2;
	encoderT2 = encoder_value_T2;
	return -quyDoiEncoder(encoderT2);
}
int32_t en_nangXoay() //TIMER 8
{
	int32_t encoderT8;
	encoderT8 = encoder_value_T8;
	return -TungKhongQuyDoi(encoderT8); 
}
int32_t en_9hRo() { //TIMER 3
	int32_t encoderT3;
	encoderT3 = encoder_value_T3;
	return -quyDoiEncoder(encoderT3);
}
int32_t en_8hRo() { //TIMER 5
	int32_t encoderT5;
	encoderT5 = encoder_value_T5;
	return -quyDoiEncoder(encoderT5);
}
int32_t en_can1() { //TIMER 4
	int32_t encoderT4;
	encoderT4 = num_over_t_Ex_Can1;
	return -quyDoiEncoder(encoderT4);
}
int32_t en_can2() { //TIMER 4
	int32_t encoderT4;
	encoderT4 = num_over_t_Ex_Can2;
	return -quyDoiEncoder(encoderT4);
}
//========================================================================================================================================================
void ResetEn_Chay(void) 
{
		num_over_t10=0;
}
void Reseten_0hRo(void) // TIMER4
{
	TIM5->CNT = 0; 
	//TIM1->CNT = 0; //2709
	//num_over_t1 = 0;
	encoder_value_T4 = 0;
	prev_encoder_count_T4 = 0;
}
void ResetEn_4hChay(void) // TIMER1
{
	TIM1->CNT = 0; //-2709
	//num_over_t2 = 0;
	encoder_value_T1 = 0;
	prev_encoder_count_T1 = 0;
}
void Reseten_4hRo(void) // TIMER2
{
	TIM2->CNT = 0; //2709
	//num_over_t3 = 0;
	encoder_value_T2 = 0;
	prev_encoder_count_T2 = 0;
}
void Reseten_nangXoay(void) // TIMER8
{
	TIM8->CNT = 0; //2709
	//num_over_t4 = 0;
	encoder_value_T8 = 0;
	prev_encoder_count_T8 = 0;
}
void ResetEn_9hRo(void) // TIMER3
{
	TIM3->CNT = 0; //2709
	//num_over_t5 = 0;
	encoder_value_T3 = 0;
	prev_encoder_count_T3 = 0;
}
void Reseten_8hRo(void) // TIMER5
{
	TIM4->CNT = 0; //2709
	//num_over_t8 = 0;
	encoder_value_T5 = 0;
	prev_encoder_count_T5 = 0;
}
void ResetEn_mamXoay(void) // CAN1 - Chuyen Thanh Encoder Ex
{
	 num_over_t_Ex_Can1 = 0;
}
void ResetEn_Can2_Ex(void) // CAN2 - Chuyen Thanh Encoder Ex
{
   num_over_t_Ex_Can2 = 0;
}
//======================================================================================================================================================
void ResetEncoder_FULL(void)
{
	Reseten_4hRo();
	Reseten_8hRo();
	Reseten_nangXoay();
	Reseten_0hRo();
	ResetEn_9hRo();
	ResetEn_4hChay();
	ResetEn_mamXoay();
	ResetEn_Can2_Ex();
	
}
void ResetEncoder_chay(void)
{
//	Reseten_9hRo();
//	ResetEn_8hChay();
}
void ResetEncoder_Home(void)
{
	TIM3->CNT = 0; //2709
	num_over_t3 = 0;
	
	TIM4->CNT = 0; //2709
	num_over_t4 = 0;
	
	TIM5->CNT = 0; //2709
	num_over_t5 = 0;
	
	// 3hRO
	encoder_value_T3 = 0; 
	prev_encoder_count_T3 = 0;
	// 6hRO
	encoder_value_T4 = 0;
	prev_encoder_count_T4 = 0;
	// 0hRO
	encoder_value_T5 = 0;
	prev_encoder_count_T5 = 0;
}
void ResetIMU(void)
{
	run_read_gyro_uart4();
}
void motor0hRo(int pwm){	
	if(pwm > 230) pwm = 230;
	if(pwm > 0) { mor_0hRO_next;}
	else 		 mor_0hRO_back;  
	mor_0hRO = abs(pwm);
}
void motor4hRo(int pwm){
	if(pwm > 230) pwm = 230;
	if(pwm > 0) { mor_4hRo_next;}
	else 		 mor_4hRo_back;  
	mor_4hRo = abs(pwm);
}
void motor8hRo(int pwm){
	if(pwm > 230) pwm = 230;
	if(pwm > 0) { mor_8hRo_next;}
	else 		 mor_8hRo_back;  
	mor_8hRo = abs(pwm);
}
void motor9hRo(int pwm){
	if(pwm > 230) pwm = 230;
	if(pwm > 0) { mor_9hRo_next;}
	else 		 mor_9hRo_back;  
	mor_9hRo = abs(pwm);
}

void quayGoc0h(int goc){
	erGocQuay0h = -(goc - en_0hRo());
	KpGocQuay0h = erGocQuay0h * 1.2;
	KiGocQuay0h = (KiGocQuay0h + erGocQuay0h) * 0.1; //0.01
	KdGocQuay0h = (erGocQuay0h - lasterGocQuay0h) * 0.5; //0.1
	speedGocQuay0h = abs(KpGocQuay0h + KiGocQuay0h + KdGocQuay0h);
//	speedGocQuay = abs(1.3 * erGocQuay);
	if (speedGocQuay0h > 230) {
//		if(abs(erGocQuay3h) < 200) speedGocQuay3h = 50;
//		else speedGocQuay3h = 254;
		speedGocQuay0h = 230;
	}
	else if (speedGocQuay0h < 4) {
		if(abs(erGocQuay0h) == 0) speedGocQuay0h = 2;
		else speedGocQuay0h = 4;
	}
	if(erGocQuay0h > 0) motor0hRo(-speedGocQuay0h);
	else motor0hRo(speedGocQuay0h);
	lasterGocQuay0h = erGocQuay0h;
}
void quayGoc4h(int goc){
	erGocQuay3h = -(goc - en_4hRo());
	KpGocQuay3h = erGocQuay3h * 1.4;
	KiGocQuay3h = (KiGocQuay3h + erGocQuay3h) * 0.1; //0.01
	KdGocQuay3h = (erGocQuay3h - lastErGocQuay3h) * 0.5; //0.1
	speedGocQuay3h = abs(KpGocQuay3h + KiGocQuay3h + KdGocQuay3h);
//	speedGocQuay = abs(1.3 * erGocQuay);
	if (speedGocQuay3h > 250) {
//		if(abs(erGocQuay3h) < 200) speedGocQuay3h = 50;
//		else speedGocQuay3h = 254;
		speedGocQuay3h = 250;
	}
	else if (speedGocQuay3h < 4) {
		if(abs(erGocQuay3h) == 0) speedGocQuay3h = 2;
		else speedGocQuay3h = 4;
	}
	if(erGocQuay3h > 0) motor4hRo(-speedGocQuay3h);
	else motor4hRo(speedGocQuay3h);
	lastErGocQuay3h = erGocQuay3h;
}
void quayGoc8h(int goc){
	erGocQuay6h = -(goc - en_8hRo());
	KpGocQuay6h = erGocQuay6h * 1.4;
	KiGocQuay6h = (KiGocQuay6h + erGocQuay6h) * 0.1; //0.01
	KdGocQuay6h = (erGocQuay6h - lastErGocQuay6h) * 0.5; //0.1
	speedGocQuay6h = abs(KpGocQuay6h + KiGocQuay6h + KdGocQuay6h);
//	speedGocQuay = abs(1.3 * erGocQuay);
	if (speedGocQuay6h > 250) {
//		if(abs(erGocQuay3h) < 200) speedGocQuay3h = 50;
//		else speedGocQuay3h = 254;
		speedGocQuay6h = 250;
	}
	else if (speedGocQuay6h < 4) {
		if(abs(erGocQuay6h) == 0) speedGocQuay6h = 2;
		else speedGocQuay6h = 4;
	}
	if(erGocQuay6h > 0) motor8hRo(-speedGocQuay6h);
	else motor8hRo(speedGocQuay6h);
	lastErGocQuay6h = erGocQuay6h;
}
void quayGoc9h(int goc){
	erGocQuay9h = -(goc - en_9hRo());
	KpGocQuay9h = erGocQuay9h * 1.4;
	KiGocQuay9h = (KiGocQuay9h + erGocQuay9h) * 0.1; //0.01
	KdGocQuay9h = (erGocQuay9h - lastErGocQuay9h) * 0.5; //0.1
	speedGocQuay9h = abs(KpGocQuay9h + KiGocQuay9h + KdGocQuay9h);
//	speedGocQuay = abs(1.3 * erGocQuay);
	if (speedGocQuay9h > 250) {
//		if(abs(erGocQuay3h) < 200) speedGocQuay3h = 50;
//		else speedGocQuay3h = 254;
		speedGocQuay9h = 250;
	}
	else if (speedGocQuay9h < 4) {
		if(abs(erGocQuay9h) == 0) speedGocQuay9h = 2;
		else speedGocQuay9h = 4;
	}
	if(erGocQuay9h > 0) motor9hRo(-speedGocQuay9h);
	else motor9hRo(speedGocQuay9h);
	lastErGocQuay9h = erGocQuay9h;
}
float modulo(float dividend, float divisor) { //ham chia co du va gia tri luon > 0
    return dividend - (divisor * (int)(dividend / divisor));
}
float signum(float x) { //Ham tra ve gia tri am, duong hoac 0
    if (x > 0.0) return 1.0;
    else if (x < 0.0) return -1.0;
    else return 0.0;
}
int closestAngle(int gocHienTai, int gocTuongLai){
	// Xem khoang cach giua 2 goc
	int dir = modulo(gocTuongLai, 3600) - modulo(gocHienTai, 3600);
	// Chuyen goc tu -360 -> 360 thanh -180 -> 180
	if (abs(dir) > 1800){
		dir = -(signum(dir) * 3600) + dir;
	}
	return dir;
}
//================================Chay dao chieu dong co
void setDirection0h(int setpoint)
{
		float currentAngle = en_0hRo();
		// find closest angle to setpoint
		float setpointAngle = closestAngle(currentAngle, setpoint);
		// find closest angle to setpoint + 1800
		float setpointAngleFlipped = closestAngle(currentAngle, setpoint + 1800);
		// if the closest angle to setpoint is shorter
		if (abs(setpointAngle) <= abs(setpointAngleFlipped))
		{
			// unflip the motor direction use the setpoint
			mor_0h_next;
			quayGoc0h(currentAngle + setpointAngle);
		}
		// if the closest angle to setpoint + 180 is shorter
		else
		{
			// flip the motor direction and use the setpoint + 180
			mor_0h_back;
			quayGoc0h(currentAngle + setpointAngleFlipped);
		}
}
void setDirection4h(int setpoint)
{
		float currentAngle = en_4hRo();
		// find closest angle to setpoint
		float setpointAngle = closestAngle(currentAngle, setpoint);
		// find closest angle to setpoint + 1800
		float setpointAngleFlipped = closestAngle(currentAngle, setpoint + 1800);
		// if the closest angle to setpoint is shorter
		if (abs(setpointAngle) <= abs(setpointAngleFlipped))
		{
			// unflip the motor direction use the setpoint
			mor_4h_next;
			quayGoc4h(currentAngle + setpointAngle);
		}
		// if the closest angle to setpoint + 180 is shorter
		else
		{
			// flip the motor direction and use the setpoint + 180
			mor_4h_back;
			quayGoc4h(currentAngle + setpointAngleFlipped);
		}
}
void setDirection8h(int setpoint)
{
		float currentAngle = en_8hRo();
		// find closest angle to setpoint
		float setpointAngle = closestAngle(currentAngle, setpoint);
		// find closest angle to setpoint + 1800
		float setpointAngleFlipped = closestAngle(currentAngle, setpoint + 1800);
		// if the closest angle to setpoint is shorter
		if (abs(setpointAngle) <= abs(setpointAngleFlipped))
		{
			// unflip the motor direction use the setpoint
			mor_8h_next;
			quayGoc8h(currentAngle + setpointAngle);
		}
		// if the closest angle to setpoint + 180 is shorter
		else
		{
			// flip the motor direction and use the setpoint + 180
			mor_8h_back;
			quayGoc8h(currentAngle + setpointAngleFlipped);
		}
}
void setDirection9h(int setpoint)
{
		float currentAngle = en_9hRo();
		// find closest angle to setpoint
		float setpointAngle = closestAngle(currentAngle, setpoint);
		// find closest angle to setpoint + 1800
		float setpointAngleFlipped = closestAngle(currentAngle, setpoint + 1800);
		// if the closest angle to setpoint is shorter
		if (abs(setpointAngle) <= abs(setpointAngleFlipped))
		{
			// unflip the motor direction use the setpoint
			mor_9h_next;
			quayGoc9h(currentAngle + setpointAngle);
		}
		// if the closest angle to setpoint + 180 is shorter
		else
		{
			// flip the motor direction and use the setpoint + 180
			mor_9h_back;
			quayGoc9h(currentAngle + setpointAngleFlipped);
		}
}
void quayGoc3Banh(int goc0h, int goc4h, int goc8h){
	setDirection0h(goc0h);
	setDirection4h(goc4h);
	setDirection8h(goc8h);
}
void chay3Banh(int speed0h, int speed4h, int speed8h){
	mor_0h = speed0h;
	mor_4h = speed4h;
	mor_8h = speed8h;
}
//==========================================================Cac ham cho co cau========================================

void nhanBanh(){
	if(!Home_napBanh){
		DC_Ban0h_back;
		DC_Ban3h_back;
		DC_Ban6h_back;
		DC_Ban0h = 40;
		DC_Ban3h = 40;
		DC_Ban6h = 40;
//		while(1) {
//			if(CB_nhanBong_Tren == 1) break;
//			osDelay(1);
//		}
		while(1) {
			gamePad();
			if(CB_nhanBong_Duoi == 1) break;
			if(DOWN) break;
			osDelay(1);
		}
		DC_Ban0h = 0;
		DC_Ban3h = 0;
		DC_Ban6h = 0;
	}else{
		vehomeNapBanh();
		DC_Ban0h_back;
		DC_Ban3h_back;
		DC_Ban6h_back;
		DC_Ban0h = 40;
		DC_Ban3h = 40;
		DC_Ban6h = 40;
//		while(1) {
//			if(CB_nhanBong_Tren == 1) break;
//			osDelay(1);
//		}
		while(1) {
			gamePad();
			if(CB_nhanBong_Duoi == 1) break;
			if(DOWN) break;
			osDelay(1);
		}
		DC_Ban0h = 0;
		DC_Ban3h = 0;
		DC_Ban6h = 0;
	}

}
void nhanBanhPen(){
	if(!Home_napBanh){
		DC_Ban0h_back;
		DC_Ban3h_back;
		DC_Ban6h_back;
		DC_Ban0h = 40;
		DC_Ban3h = 40;
		DC_Ban6h = 40;

		while(1) {
			gamePad();
			if(CB_nhanBong_Duoi == 1) break;
			if(DOWN) break;
			osDelay(1);
		}
		osDelay(400);
		while(1) {
			DC_Ban0h_next;
			DC_Ban3h_next;
			DC_Ban6h_next;
			DC_Ban0h = 124 + Lucbantest;
			DC_Ban3h = 224 + Lucbantest;
			DC_Ban6h = 224 + Lucbantest;
			if(SELECT){
					osDelay(10);
					while(SELECT) osDelay(1);
					Lucbantest -= 2;
			}
			if(START){
				osDelay(10);
				while(START) osDelay(1);
				Lucbantest += 2;
			}
			if(SQUARE) break;
			if(DOWN) break;
			osDelay(1);
		}
	}else{
		vehomeNapBanh();
		DC_Ban0h_back;
		DC_Ban3h_back;
		DC_Ban6h_back;
		DC_Ban0h = 40;
		DC_Ban3h = 40;
		DC_Ban6h = 40;
		while(1) {
			gamePad();
			if(CB_nhanBong_Duoi == 1) break;
			if(DOWN) break;
			osDelay(1);
		}
		osDelay(300);
		while(1){
			DC_Ban0h_next;
			DC_Ban3h_next;
			DC_Ban6h_next;
			DC_Ban0h = 124 + Lucbantest;
			DC_Ban3h = 224 + Lucbantest;
			DC_Ban6h = 224 + Lucbantest;
			if(SELECT){
					osDelay(10);
					while(SELECT) osDelay(1);
					Lucbantest -= 2;
			}
			if(START){
				osDelay(10);
				while(START) osDelay(1);
				Lucbantest += 2;
			}
			if(SQUARE) break;
			if(DOWN) break;
			osDelay(1);
		}
	}
}
//void vehomeNapBanh(){
//		bit_Napbanh = 0;
//		DC_Nap_next;
//		DC_Nap = 250;
//		osDelay(200);
//		while(1){	
//				if(!Home_napBanh)
//				{
//					DC_Nap = 0;
//					break;
//				}
//				osDelay(1);
//			}
//			bienNapBanh = 1;
//			osDelay(500);
//			
//			while(1){
//				if(bienNapBanh == 1){
//					DC_Nap_back;
//					DC_Nap = 40;
//					if(!Home_napBanh)
//					{
//						bienNapBanh = 2;
//						DC_Nap = 0;
//						break;
//					}
//				}
//				osDelay(1);
//			}
//}

void vehomeNapBanh(){
	  int bienChongNhieuNap = 0;
		bit_Napbanh = 0;
		DC_Nap_next;
		DC_Nap = 250;
		osDelay(450);
		while(1){
				if(!Home_napBanh) bienChongNhieuNap++; 
				if(bienChongNhieuNap > 10)
				{
					DC_Nap = 0;
					bienChongNhieuNap = 0;
					break;
				}
				if(DOWN) break;
				osDelay(1);
			}
}
//void nhanBanh(){
//	if(!Home_napBanh)
//	{
//		DC_Ban0h_back;
//		DC_Ban3h_back;
//		DC_Ban6h_back;
//		DC_Ban0h = 35;
//		DC_Ban3h = 35;
//		DC_Ban6h = 35;
//		while(1) {
//			gamePadCoCau();
//			gamePadpen();
//			gamePad();
//			if(CB_nhanBong_Duoi == 0) break;
//			if(DOWN) break;
//			osDelay(1);
//		}
//		DC_Ban0h = 0;
//		DC_Ban3h = 0;
//		DC_Ban6h = 0;
//	}
//	else{
//		vehomeNapBanh();
//		DC_Ban0h_back;
//		DC_Ban3h_back;
//		DC_Ban6h_back;
//		DC_Ban0h = 35;
//		DC_Ban3h = 35;
//		DC_Ban6h = 35;
//		while(1) {
//			gamePadCoCau();
//			gamePad();
//			gamePadpen();
//			if(CB_nhanBong_Duoi == 0) break;
//			if(DOWN) break;
//			osDelay(1);
//		}
//		DC_Ban0h = 0;
//		DC_Ban3h = 0;
//		DC_Ban6h = 0;
//	}
//}

void banBanh(int tocDoBanTren, int tocDoBanDuoi){
	if(CB_nhanBong_Duoi == 1){
		int p;
		int minsp = 20;
		bit_banBanh = 1;
		robotStop(0);
		DC_Ban0h_next;
		DC_Ban3h_next;
		DC_Ban6h_next;
		DC_Ban0h = tocDoBanTren;
		DC_Ban3h = tocDoBanDuoi;
		DC_Ban6h = tocDoBanDuoi;
		osDelay(250);
		bit_Napbanh = 1;
		while(1) {
			if(!CB_nhanBong_Tren) break;
			if(DOWN) break;
			osDelay(1);
		}
		while(1) {
			if(CB_nhanBong_Tren) break;
			if(DOWN) break;
			osDelay(1);
		}

		osDelay(200);
		while(1){
			if(p++ %4 == 0)
			{
				tocDoBanTren-- ;
				tocDoBanDuoi--;
				if(tocDoBanTren <= minsp) tocDoBanTren = minsp;
				if(tocDoBanTren <= minsp) tocDoBanDuoi = minsp;
				DC_Ban0h = tocDoBanTren;
				DC_Ban3h = tocDoBanDuoi;
				DC_Ban6h = tocDoBanDuoi;
			}
			if(tocDoBanDuoi <=30) 
			{
				DC_Ban0h = 0;
				DC_Ban3h = 0;
				DC_Ban6h = 0;
				break;
			}
			osDelay(1);
		}
		DC_Ban0h = 0;
		DC_Ban3h = 0;
		DC_Ban6h = 0;
		bit_banBanh = 0;
	}
}
void banBanhPen(){
		bit_banBanh = 1;
		bit_Napbanh = 1;
		while(1) {
			if(!CB_nhanBong_Tren) break;
			if(DOWN) break;
			osDelay(1);
		}
		while(1) {
			if(CB_nhanBong_Tren) break;
			if(DOWN) break;
			osDelay(1);
		}
		DC_Ban0h = 0;
		DC_Ban3h = 0;
		DC_Ban6h = 0;
		bit_banBanh = 0;
}
//hamcuaQuan
//void banBanh(int tocDoBanTren, int tocDoBanDuoi){
//	int p;
//	int minsp = 20;
//	bit_banBanh = 1;
//	robotStop(0);
//	DC_Ban0h_next;
//	DC_Ban3h_next;
//	DC_Ban6h_next;
//	DC_Ban0h = tocDoBanTren;
//	DC_Ban3h = tocDoBanDuoi;
//	DC_Ban6h = tocDoBanDuoi;
//	osDelay(1200);
//	bit_Napbanh = 1;
//	while(1) {
//		//mam_Xoay = 2;
//		if(DOWN) break;
//		if(!CB_nhanBong_Duoi) break;
//		osDelay(1);
//	}
//	while(1) {
//		//mam_Xoay = 2;
//		if(DOWN) break;
//		if(CB_nhanBong_Duoi) break;
//		osDelay(1);
//	}
//	osDelay(200);
//	while(1)
//	{
//		if(p++ %7 == 0)
//		{
//			tocDoBanTren-- ;
//			tocDoBanDuoi--;
//			if(tocDoBanTren <= minsp) tocDoBanTren = minsp;
//			if(tocDoBanTren <= minsp) tocDoBanDuoi = minsp;
//			DC_Ban0h = tocDoBanTren;
//			DC_Ban3h = tocDoBanDuoi;
//			DC_Ban6h = tocDoBanDuoi;
//		}
//		if(tocDoBanDuoi <=30) 
//		{
//			DC_Ban0h = 0;
//			DC_Ban3h = 0;
//			DC_Ban6h = 0;
//			break;
//		}
//		osDelay(1);
//	}
//	DC_Ban0h = 0;
//	DC_Ban3h = 0;
//	DC_Ban6h = 0;
//	bit_banBanh = 0;
//}
void xoayRobot(){
//	gocXoay = robotAngle() + GocUWB*3.75;
	int butru;
	if(GocUWB < 0) butru = 20;
	else butru = -10;
	while (1){
		errorGocXoay = (0 - GocUWB -butru);
		speedXoay = abs(errorGocXoay) * 0.2;
		if (speedXoay >= 30) {
			speedXoay = 30;
		}else if (speedXoay <= 4) {
			speedXoay = 4; 
		}				
		if(errorGocXoay > 0) {
			robot_rotate(speedXoay);
		}else {
			robot_rotate(-speedXoay);
		}
		if(abs(errorGocXoay) < 3) break;
		if(DOWN) break;
		osDelay(1);
	}
	robotStop(2);
	while(1){
		if(DOWN) break;
		osDelay(1);
	}
}

void setNangH(int speed){
	if(speed >= 0){
		xoayNang_next;
		if(CTHT_Tren) xoayNang=0;
//		else if(speed < 50 && speed > 0) xoayNang = 50;
		else xoayNang = abs(speed);
	}
	else {
		xoayNang_back;
		if(CTHT_Duoi) xoayNang=0;
		else xoayNang = abs(speed);
	}
}

void xoayGocBanBanh(int gocBan, int rulo0h, int rulo3h6h){
	if(CB_nhanBong_Duoi == 1){
		bit_gocBan = 1;
		robotStop(0);
	 _gocMX = gocBan;
	 while(1) {
		 if(abs(bienTroNongBan - _gocMX) < 5) break;
		 if(DOWN) break;
		 gamePad();
		 osDelay(1);
	 }
	 banBanh(rulo0h,rulo3h6h);
	 bit_gocBan = 0;
	}else{
		nhanBanh();
		osDelay(100);
		bit_gocBan = 1;
		robotStop(0);
	 _gocMX = gocBan;
	 while(1) {
		 if(abs(bienTroNongBan - _gocMX) < 5) break;
		 if(DOWN) break;
		 gamePad();
		 osDelay(1);
	 }
	 banBanh(rulo0h,rulo3h6h);
	 bit_gocBan = 0;
	}
}
void xoayGocReBanh(int gocBan, int rulo0h, int rulo3h6h){
	if(CB_nhanBong_Duoi == 1){
		bit_gocBan = 1;
		robotStop(0);
	 _gocMX = gocBan;
	 while(1) {
		 if(abs(bienTroNongBan - _gocMX) < 5) break;
		 if(DOWN) break;
		 gamePad();
		 osDelay(1);
	 }
	 banBanh(rulo0h,rulo3h6h);
	}else{
		nhanBanh();
		bit_gocBan = 1;
		robotStop(0);
	 _gocMX = gocBan;
	 while(1) {
		 if(abs(bienTroNongBan - _gocMX) < 5) break;
		 if(DOWN) break;
		 gamePad();
		 osDelay(1);
	 }
	 banBanh(rulo0h,rulo3h6h);
	}
}

void giuNangCoDinh(int setPoint){
	static char p = 0;
//	int setPoint = 1000;
	int er = 0;
	int kp = 0;
	int tocDoNang = 0;
	er =  setPoint - en_nangXoay();
	kp = er*9;
	if(abs(er) <= 1) tocDoNang = 0;
	else {
		if(kp > 0) {
			tocDoNang = kp;
			if(kp > 100) tocDoNang = 100;
//			else if(er < 40) er = 40;
		}
		else {
			tocDoNang = kp;
			if(kp < -30) tocDoNang = -30;
		}
	}
	setNangH(tocDoNang);
}

void haNongXuongH(){
	bit_giuNong = 0;
	int bienChongNhieuCTHTTren = 0;
	bit_nangNong = 0;
	static char p = 0;
	int maxSpeedNong = 200;
	int minSpeedNong = 5;
	int bienChongNhieuCTHT = 0;
	while(1){
		if(p++ %2 == 0) {
			if(speedXoayNang < 100) speedXoayNang++;
			else speedXoayNang+=2;
			if(speedXoayNang >= maxSpeedNong) speedXoayNang = maxSpeedNong;
		}
		setNangH(-speedXoayNang);
		if(en_nangXoay() < 1000) break;
		osDelay(1);
	}
	while(1){
		if(p++ %2 == 0) {
			if(speedXoayNang > 100) speedXoayNang--;
			else speedXoayNang--;
			if(speedXoayNang <= minSpeedNong) speedXoayNang = minSpeedNong;
		}
		setNangH(-speedXoayNang);
		if(CTHT_Duoi) bienChongNhieuCTHT++; 
		if(bienChongNhieuCTHT > 10) break;
		osDelay(1);
	}
	setNangH(0);
	Reseten_nangXoay();
}

void nangNongLenH(){
	bit_nangNong = 1;
	static char p = 0;
	int maxSpeedNong = 200;
	int minSpeedNong = 60;
	int bienChongNhieuCTHT = 0;
	while(1){
		gamePad();
		if( p++%4 == 0) {
			bienDem_Encoder += 1;
			if(speedXoayNang < 60) speedXoayNang = 60;
			else 
			{
				speedXoayNang+=5;
			}
			if(speedXoayNang >= maxSpeedNong) speedXoayNang = maxSpeedNong;
		}
		setNangH(speedXoayNang);
		if(bienDem_Encoder > 10) break;
		osDelay(1);
	}
	while(1){
		gamePad();
		if(p++ %3 == 0) {
			speedXoayNang-=3;
			if(speedXoayNang <= minSpeedNong) speedXoayNang = minSpeedNong;
		}
		setNangH(speedXoayNang);
		if(CTHT_Tren) bienChongNhieuCTHT++; 
		if(bienChongNhieuCTHT > 10) break;
		osDelay(1);
	}
	setNangH(40);
	osDelay(100);
	bit_giuNong = 1;
}

///////// HAM VE HOME ==================================================================

void veHome(int buTru0h, int buTru4h, int buTru8h){
	int pwmGoHome = 30;
	int pwmBackHome = 10;
	
	motor0hRo(-pwmGoHome);
	motor4hRo(-pwmGoHome);
	motor8hRo(-pwmGoHome);

	int bien1 = 0, bien2 = 0, bien3 = 0;
	while(1){
		if(!CB_0h) motor0hRo(2), bien1 = 1;
		if(!CB_4h) motor4hRo(2), bien2 = 1;
		if(!CB_8h) motor8hRo(2), bien3 = 1;
		if(bien1 && bien3 && bien2) break;
		osDelay(1);
	}
	
	motor0hRo(-10);
	motor4hRo(-10);
	motor8hRo(-10);
	
	osDelay(50);
	Reseten_0hRo();
	Reseten_4hRo();
	Reseten_8hRo();
	
	bien1 = 0, bien2 = 0, bien3 = 0;
	while(1){
		int xungEn = -100;
		if(en_0hRo() <= xungEn) motor0hRo(2), bien1 = 1;
		if(en_4hRo() <= xungEn) motor4hRo(2), bien2 = 1;
		if(en_8hRo() <= xungEn) motor8hRo(2), bien3 = 1;
		if(bien1 && bien3 && bien2 ) break;
		osDelay(1);
	}
	motor0hRo(pwmBackHome);
	motor4hRo(pwmBackHome);
	motor8hRo(pwmBackHome);
	
	bien1 = 0, bien2 = 0, bien3 = 0;
	while(1){
		if(!CB_0h) motor0hRo(2), bien1 = 1;
		if(!CB_4h) motor4hRo(2), bien2 = 1;
		if(!CB_8h) motor8hRo(2), bien3 = 1;
		if(bien1 && bien3 && bien2 ) break;
		osDelay(1);
	}
	osDelay(50);
	ResetEncoder_FULL();
	
	bien1 = 0, bien2 = 0, bien3 = 0;
	int themGoc8h = buTru8h;
	int themGoc4h = buTru4h;
	int	themGoc0h = buTru0h;
	
	while(1){
		quayGoc0h(themGoc0h);
		quayGoc4h(themGoc4h);
		quayGoc8h(themGoc8h);
		if(abs(en_0hRo() - themGoc0h) <= 1) motor0hRo(2), bien1 = 1;
		if(abs(en_4hRo() - themGoc4h) <= 1) motor4hRo(2), bien2 = 1;
		if(abs(en_8hRo() - themGoc8h) <= 1) motor8hRo(2), bien3 = 1;
		if(bien1 && bien3 && bien2) break;
		osDelay(1);
	}
//	motor0hRo(2);
//	motor4hRo(2);
//	motor8hRo(2);
	osDelay(50);
	Reseten_0hRo();
	Reseten_4hRo();
	Reseten_8hRo();
	_0h = 0;
	_4h = 0;
	_8h = 0;
	bit_veHome = 0;
}

// Hàm tính toán góc và d? dài du?ng chéo
void calculateJoystickOutput(int x, int y) {
    // Tính toán góc t? tr?c Y và theo chi?u kim d?ng h?
    int raw_angle = (int)(atan2(x, y) * (1800.0 / 3.14)); // Nhân 10 d? tang d? chính xác
    if (raw_angle < 0) {
        raw_angle += 3600; // Chuy?n d?i t? kho?ng -1800-1800 thành 0-3600
    }
    angle = raw_angle; // Ðua v? kho?ng 0-360 d?

    // Tính toán d? dài du?ng chéo (magnitude)
    magnitude = (int)sqrt(x * x + y * y); // Magnitude s? là giá tr? nguyên
}


void setmamXoay(int speedMX) {
	if(speedMX > 0){
		xoayNang_next;
	}
	else xoayNang_back;
	
	if(abs(speedMX) > 120) speedMX = 120;
	xoayNang = abs(speedMX);
}
void controlMotor(targetMamXoay) {
	ena_vitme = 0;
	  errorBienTro = -targetMamXoay + bienTroNongBan;
	 if(abs(errorBienTro) > 0) {
			speed = abs(errorBienTro) * 2.8;
			if (speed > 255) {
				speed = 255;
			}else if (speed <= 10) {
				speed = 10; 
			}				
			if (errorBienTro > 0) {
				setmamXoay(-speed);
			}else {
				setmamXoay(speed);
			}
		}
	 else {
		  //ena_vitme = 1;
			setmamXoay(0);
	}
}
void controlMotorPen(targetMamXoay) {
	  errorBienTro = -targetMamXoay + bienTroNongBan;
	 if(abs(errorBienTro) > 0) {
			speed = abs(errorBienTro) * 2.5;
			if (speed > 250) {
				speed = 250;
			}else if (speed <= 8) {
				speed = 8; 
			}				
			if (errorBienTro > 0) {
				setmamXoay(-speed);
			}else {
				setmamXoay(speed);
			}
		}
	 else {
		  ena_vitme = 1;
			setmamXoay(0);
	}
}
void controlMotorPID(int targetMamXoay) {
    // Khai báo h? s? PID
    float Kp = 2;
    float Ki = 0.001;

    // Static d? gi? giá tr? gi?a các l?n g?i hàm
    static int integral = 0;

    // Tính sai s?
    int errorBienTro = targetMamXoay - bienTroNongBan;

    // C?ng d?n sai s? d? tính tích phân
    integral += errorBienTro;

    // Tính d?u ra PID
    int output = Kp * errorBienTro + Ki * integral;

    // Gi?i h?n t?c d?
    int speed = abs(output);
    if (speed > 120) {
        speed = 120;
    } else if (speed < 1) {
        speed = 0;
    }

    // Ði?u khi?n d?ng co
    if (abs(errorBienTro) > 3) {
        if (output > 0) {
            setmamXoay(speed);
        } else {
            setmamXoay(-speed);
        }
    } else {
        setmamXoay(0);
        integral = 0; // Reset tích phân khi g?n d?t m?c tiêu
    }
}

void gamePad() {
	int leftTT = (abs((int8_t)LJOY_LR) > 12) || (abs((int8_t)LJOY_UD) > 12);
	if(R1 && leftTT){
		if(doiGocXoay) gocBuTruXoay = robotAngle();
		//calculateJoystickOutput((int8_t)LJOY_LR, (int8_t)LJOY_UD);
		runAngle(-angle + gocBuTruXoay - 450, magnitude*2.5, -2.5);
		doiGocXoay = 0;
	}
	else if (L1 && leftTT){
		if(doiGocXoay) gocBuTruXoay = robotAngle();
		//calculateJoystickOutput((int8_t)LJOY_LR, (int8_t)LJOY_UD);
		runAngle(-angle + gocBuTruXoay - 450, magnitude*2.5, 2.5);
		doiGocXoay = 0;
	}
	else {
		doiGocXoay = 1;
    int tocdo = 50; // Default speed
    int stop = 0;
		int isRunning = 1; //-450 Fixed angle adjustment
    // Determine speed based on L1 and L2 inputs
		int direction = 0; // Default direction
		int tocDoJoy = 0;
		int tocTB = 50, tocNhanh = 250;
		if(RED_BLUE){
			tocCham = 15;
		}else{
			tocCham = 8;
		}
    if (L1) tocdo = tocNhanh;
    else if (L2) tocdo = tocCham;
		else tocdo = tocTB;
		//Khi nang nong thi khong cho chay nhanh
		if(bit_nangNong) tocdo = tocCham;
    // Calculate direction and run behavior
    
    if (UP && !DOWN && !RIGHT && !LEFT) {
        direction = 0 - doigoc;
    } else if (!UP && DOWN && !RIGHT && !LEFT) {
        direction = 1800 - doigoc;
     } else if (!UP && !DOWN && RIGHT && !LEFT) {
        direction = 900 - doigoc;
    } else if (!UP && !DOWN && !RIGHT && LEFT) {
        direction = -900 - doigoc;
    } else if (UP && !DOWN && RIGHT && !LEFT) {
        direction = 450 - doigoc;
    } else if (UP && !DOWN && !RIGHT && LEFT) {
        direction = -450 - doigoc;
    } else if (!UP && DOWN && RIGHT && !LEFT) {
        direction = 1350 - doigoc;
    } else if (!UP && DOWN && !RIGHT && LEFT) {
        direction = -1350 - doigoc;
    } else {
        isRunning = 0;
    }
		
		// Handle joystick input
		if ((int8_t)RJOY_LR >= 50 && isRunning) {
				runSnake(direction, tocdo, 2.5);
		} else if ((int8_t)RJOY_LR <= -50 && isRunning) {
				runSnake(direction, tocdo, -2.5);
		} else if (isRunning) {
				robotRun(direction, tocdo);
		} else if ((abs((int8_t)LJOY_LR) > 10) || (abs((int8_t)LJOY_UD) > 10)) {
				if(bit_nangNong) tocDoJoy = tocCham;
				else	tocDoJoy = magnitude*2;
				if(L2) tocDoJoy = tocCham;
			
				if ((int8_t)RJOY_LR >= 50) {
						runSnake(angle - doigoc, tocDoJoy, 2.5);
				} else if ((int8_t)RJOY_LR <= -50) {
						runSnake(angle - doigoc, tocDoJoy, -2.5);
				}
				else {
					robotRun(angle - doigoc, tocDoJoy);
				}
		}
		else {
				if ((int8_t)RJOY_LR >= 50) {
						robot_rotate(tocdo);
				} else if ((int8_t)RJOY_LR <= -50) {
						robot_rotate(-tocdo);
				} else {
						robotStop(0);
				}
		}
	}
}

//int calculate(int K) {
////    int A[] = {180, 199, 210, 250, 256, 295, 313, 386};
////    int B[] = {136, 138, 139, 148, 149, 161, 162, 186};
//		int A[] = {180, 199, 210, 250, 256, 295, 313, 386, 420, 450, 500};
//    int B[] = {136, 138, 139, 148, 149, 161, 162, 186, 195, 205, 220};
//		int n = sizeof(A) / sizeof(A[0]);
//		
//    for (int i = 0; i < n - 1; i++) {
//        if (K >= A[i] && K <= A[i+1]) {
//            return B[i] + (K - A[i]) * (B[i+1] - B[i]) / (A[i+1] - A[i]);
//        }
//    }
//    return 255;
//}

int calculate(int K) {
    int A[] = {180, 199, 210, 250, 256, 295, 313, 386, 420, 450, 500};
    int B[] = {136, 138, 139, 150, 152, 163, 165, 186, 193, 198, 212};
    int n = sizeof(A) / sizeof(A[0]);
    
    if (K < A[0]) {
        return 255;  
    }
    
    if (K > A[n-1]) {
        int x0 = A[n-3], y0 = B[n-3];
        int x1 = A[n-2], y1 = B[n-2];
        int x2 = A[n-1], y2 = B[n-1];
        
        return (K-x1)*(K-x2)/((x0-x1)*(x0-x2))*y0 + 
               (K-x0)*(K-x2)/((x1-x0)*(x1-x2))*y1 + 
               (K-x0)*(K-x1)/((x2-x0)*(x2-x1))*y2;
    }
    
    for (int i = 0; i < n - 1; i++) {
        if (K >= A[i] && K <= A[i+1]) {
            return B[i] + (K - A[i]) * (B[i+1] - B[i]) / (A[i+1] - A[i]);
        }
    }
    return 255;
}
int clampPWM(int value) {
    if (value < 100) return 100;   
    if (value > 250) return 250;   
    return value;
}
void chayXoayBenPhai(){
	gocBuTruXoay = robotAngle();
	while(1){
		// So thu 1 la huong chay, 2 la toc do chay, 3 la toc do xoay va huong cang lon thi xoay cang cham
		// Tat ca cac huong deu bi nghich
		runAngle(-850 + gocBuTruXoay + doigoc, 100, 3.5);
		if(abs(robotAngle() + 900 - gocBuTruXoay) < 2) break;
		if(DOWN) break;
		osDelay(1);
	}
	ResetEn_4hChay();
	while(1){
		robotRun(0 - doigoc, 150);
		if(en_Chay4h() > 500) break;
		if(DOWN) break;
		osDelay(1);
	}
	while(1){
		robotRun(0 - doigoc, 150);
		if(!CB_Quang) break;
		if(DOWN) break;
		osDelay(1);
	}
	robotStop(0);
	ResetEn_4hChay();
	while(1){
		robotRun(-1350 - doigoc, 150);
		if(en_Chay4h() > 1650) break;
		if(DOWN) break;
		osDelay(1);
	}
	robotStop(0);
}
void gamePadCoCau(){
		if(UWB == 0){
			Den_Do_on;
		}
		else{
			Den_Do_off;
		} 
		
		if(!R1 && PS){
			int goc;
			int p = 0;
			if(UWB < 180 || UWB >= 500  || calculate(UWB) == 255){
				DC_Ban0h = 0;
				DC_Ban3h = 0;
				DC_Ban6h = 0;
				Den_Do_on;
			}else{
				KC = UWB;
				if(UWB < 320){
					goc = gocChuyenGan;
				}
				else if(UWB >= 320){
					goc = gocChuyenXa;
				}				
				else if(UWB >= 380){
					goc = gocChuyenXa - 50;
				}
				_gocMX = goc;
				while(1){
					//p += 2;
					if(abs(bienTroNongBan - _gocMX) <= 3) break;
					//if(p >= 2000) break;
					if(DOWN) break;
					osDelay(1);
				}
				if(UWB < 320){
					LucBan = clampPWM((calculate(UWB) - 3) + Lucbantest);
				}else{
					LucBan = clampPWM((calculate(UWB) - 15) + Lucbantest);
				}
				xoayGocBanBanh(goc,(LucBan - 20) + Lucbantest,(LucBan + 25) + Lucbantest);
				//banBanh((LucBan - 20) + Lucbantest,(LucBan + 25) + Lucbantest);
				Lucbantest = 0;	
				//_gocMX = (gocChuyenGan + gocChuyenXa) / 2;
				}
			}else if(R1 && PS){
				bit_gocBan = 1;
				LucBan = 150  + Lucbantest;
				xoayGocBanBanh(gocChuyenGan, LucBan - 20, LucBan + 25);
				Lucbantest = 0;
			}

		if(SELECT){
			osDelay(10);
			while(SELECT) osDelay(1);
			Lucbantest-=2;
		}
		
		if(START){
			osDelay(10);
			while(START) osDelay(1);
			Lucbantest+=2;
		}
		
//		if(O){
//			xoayNang_next;
//			xoayNang = 50;
//		}else if(SQUARE){
//			xoayNang_back;
//			xoayNang = 50;
//		}
		if(R2){
				osDelay(50);
				Bit_lazer =~ Bit_lazer;
				if(Bit_lazer){
					Den_On;
					Den = 250;
				}
				else {
					Den_Off;
					Den = 0;
				}
				while(R2)	osDelay(1);
		}

		if(O){
			int p = 0;
			while(1){
				p += 2;
				RC_1 = 1800;
				RC_2 = 800;
				gamePad();
				if(p >= 900) break;
				osDelay(1);
			}
				RC_1 = 800;
				RC_2 = 1800;	
		}
		
		if(SQUARE){
			bit_gocBan = 1;
			LucBan = 166  + Lucbantest;
			xoayGocBanBanh(gocBan3Diem, LucBan, LucBan);
			Lucbantest = 0;
			//_gocMX = gocChuyenGan;
		}
		
		if(X){
			bit_gocBan = 1;
			int goc = gocBan2Diem ;
			LucBan = 138 + Lucbantest;
			xoayGocBanBanh(goc,LucBan,LucBan);
			Lucbantest = 0;	
			Den_Off;
			Den = 0;
		}
		
		if(TRIANGLE){
			//bit_gocBan = 1;
			int goc = gocRe;
			xoayGocReBanh(goc,3,140);
			Den_On;
			Den = 250;
			_gocMX = gocBan2Diem;
		}		
		if((int8_t)RJOY_UD <= -100 && CB_nhanBong_Tren == 0){
			nhanBanh();
		}
}

void gamePadpen(){
		if(SQUARE){
			banBanhPen();
		}
		if(R2){
				osDelay(50);
				Bit_lazer =~ Bit_lazer;
				if(Bit_lazer){
					Den_On;
					Den = 250;
				}
				else {
					Den_Off;
					Den = 0;
				}
				while(R2)	osDelay(1);
		}
//		if(SELECT){
//			osDelay(10);
//			while(SELECT) osDelay(1);
//			Lucbantest-=2;
//		}
//		
//		if(START){
//			osDelay(10);
//			while(START) osDelay(1);
//			Lucbantest+=2;
//		}
}
