#include <core_cm7.h>
#include "i2c_lcd.h"
#include "ala_Config.h"

extern UART_HandleTypeDef huart1;
extern int16_t IMU, UWB, GocUWB;
extern uint16_t adc_data[11];
extern char GP_BTN [10];
extern char MANG_GAME[8];
extern int part, chenhLechTocDo;
extern uint8_t bit_banBanh;
//extern int speed_start;
extern int bientest321,bienDem_Encoder;
extern int LucBan, KC, speedXoay;
extern int LucBanXeo;
char buff[33];
extern int Lucbantest, errorBienTro;

//======================================================================
void lcdViewer( int mode)
{
		if(RED_BLUE) lcd_printStr(17,0,"DAU");
		else lcd_printStr(17,0,"PEN");
	if (mode == 0) //Test tat ca cac thong so
	{
		lcd_printStr(0,0,"CB:");
		lcd_printBit(CB_nhanBong_Tren);
		lcd_printBit(CB_nhanBong_Duoi);
		lcd_printBit(Home_napBanh);
		lcd_printBit(CB_Quang);
//		lcd_printBit(CTHT_Duoi);
		lcd_printStr(0,1,"LB:");
		lcd_printInt(3,1,robotAngle());
		lcd_printStr(0,2,"TG:");
		lcd_printInt(3,2,(int8_t)GP_BTN[3]);
		lcd_printStr(0,3,"CB048:");
		lcd_printBit(CB_0h);
		lcd_printBit(CB_4h);
		lcd_printBit(CB_8h);
		lcd_printStr(7,1,"0h:");
		lcd_printInt(10,1,en_0hRo());
		lcd_printStr(7,2,"3h:");
		lcd_printInt(10,2,en_4hRo());
		lcd_printStr(14,1,"6h:");
		lcd_printInt(17,1,en_8hRo());
		lcd_printStr(12,3,"KhC:");
		lcd_printInt(16,3,UWB);
		lcd_printStr(10,0,"0H:");
		lcd_printInt(13,0,en_Chay4h());
	}
	
	
	else if (mode == 1) //Test encoder
	{
				lcd_printStr(0,0,"M1 ");
				lcd_printStr(5,0,"BTRO:");
				lcd_printInt(10,0,bienTroNongBan);
				lcd_printStr(0,2,"LUC:");
				lcd_printInt(4,2,LucBan);
				lcd_printInt(12,2,Lucbantest);
				lcd_printStr(0,3,"KhC:");
				lcd_printInt(4,3,UWB);
				lcd_printStr(12,3,"KC:");
				lcd_printInt(16,3,KC);
				lcd_printInt(0,1,DC_Ban0h);
				lcd_printStr(5,1,"bbBanh");
				lcd_printInt(12,1,bit_banBanh);
				lcd_printStr(14,1,"enaVM");
				lcd_printInt(19,1,ena_vitme);
	}
	else if(mode == 2){
		//OI2
    lcd_printInt(1,0,CB_1);
		lcd_printInt(1,1,CB_2);
		lcd_printInt(1,2,Home_napBanh);
		lcd_printInt(1,3,CB_4);
//		lcd_printInt(3,0,CB_5);
//		lcd_printInt(3,1,CB_Nhan);
		lcd_printInt(3,2,CB_nhanBong_Duoi);
		lcd_printInt(3,0,CB_nhanBong_Tren);
		// IO-3
//		lcd_printInt(7,0,CTHT_Duoi);
//		lcd_printInt(7,1,CTHT_Tren);
//		lcd_printInt(7,2,CTHT_);
//		lcd_printInt(7,3,CTHT);
//		lcd_printInt(11,0,CB_9h);
//		lcd_printInt(2,1,CB_1);

	}
		else if(mode == 3){ // Test doc mau celo
		lcd_printStr(0,0,"M3");
		
		lcd_printStr(0,1,"dev:");
		lcd_printInt(6,1, deviation_angle);
		
//		lcd_printStr(0,2,"error:");
//		lcd_printInt(6,2,error);
//		
//		lcd_printStr(0,3,"speed:");
//		lcd_printInt(6,3,speed);

//    lcd_printStr(10,1,"CB_9h:");
//		lcd_printInt(17,1,_9h);
	}
	else if(mode == 4){ //Test nut tay game
		lcd_printStr(0,0,"M4 test UART");
		lcd_printInt(0,1,RX_UART5[0]);
		lcd_printInt(0,2,RX_UART5[1]);
		lcd_printInt(5,2,RX_UART5[2]);
		lcd_printInt(0,3,RX_UART5[3]);
		lcd_printInt(5,3,RX_UART5[4]);
		lcd_printInt(10,2,UWB);
		lcd_printInt(10,3,GocUWB);
	}
	
	else if(mode == 5){ //Test encoder
		lcd_printStr(0,0,"M5 Test Bo Banh Xe");
		
		lcd_printStr(0,1,"CB0369:");
		lcd_printBit(CB_0h);
		lcd_printBit(CB_4h);
		lcd_printBit(CB_8h);
		lcd_printBit(CB_9h);
		lcd_printBit(CTHT_Duoi);
		lcd_printBit(CTHT_Tren);
		lcd_printBit(CTHT);

		
//		lcd_printStr(0,2,"0h:");
//		lcd_printInt(4,2,en_Chay4h());
//		
//		lcd_printStr(0,3,"3h:");
//		lcd_printInt(4,3,en_4hRo());
//		
//		lcd_printStr(10,2,"6h:");
//		lcd_printInt(14,2,en_8hRo());
//		
//		lcd_printStr(10,3,"9h:");
//		lcd_printInt(14,3,en_9hRo());
//		lcd_printInt(17,3,en_nangXoay());
	}
	
	else if(mode == 6) { // test cam bien Input
		lcd_printStr(0,0,"M6 TEST CB");
		lcd_printInt(0,1,CB_nhanBong_Tren);
		lcd_printInt(0,2,CB_nhanBong_Duoi);
		lcd_printInt(0,3,(int8_t)GP_BTN[2]);
		lcd_printInt(5,0,(int8_t)GP_BTN[3]);
		
		lcd_printInt(5,1,(int8_t)GP_BTN[4]);
		lcd_printInt(5,2,(int8_t)GP_BTN[5]);
		lcd_printInt(5,3,GP_BTN[6]);
		lcd_printInt(10,0,GP_BTN[7]);
		lcd_printInt(12,1,robotAngle());

	}
	else if(mode == 7) { // test adc
		lcd_printStr(0,0,"M7 TEST ADC");
		
		lcd_printInt(0,1,adc_data[0]);	
		lcd_printInt(5,1,adc_data[1]);
		lcd_printInt(10,1,adc_data[2]);
		
		lcd_printInt(0,2,adc_data[3]);	
		lcd_printInt(5,2,adc_data[4]);
		lcd_printInt(10,2,adc_data[5]);
		lcd_printInt(15,2,adc_data[6]);
		
		lcd_printInt(0,3,adc_data[7]);	
		lcd_printInt(5,3,adc_data[8]);
		lcd_printInt(10,3,adc_data[9]);
		lcd_printInt(15,3,adc_data[10]);
	}
	else {
		lcd_printStr(0,0,"M8");
		lcd_printStr(0,1,"KC");
		lcd_printInt(10,1,UWB);
		lcd_printStr(0,2,"KCLZ");
		lcd_printInt(10,2,lazer_KhoangCach);
	}
}


