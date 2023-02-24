#include "include.h"
#include "trans_hex.h"
#include "adc.h"

int main(void)
{	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable ,ENABLE);//½ûÓÃJTAG ÆôÓÃ SWD
	delay_init();
	////->

	mpu__init();                  //leg
	adc_init();                   //knee
	trans_hex_init(2, 115200);    //foot
	
	trans_others_init(1, 115200); //uart2PC
	trans_bt_init(3, 115200);     //uart2bluetooth
	
	////<-
	maincycle_ms_init(50);
	while(1);
}

void Maincycle_Handler()
{
	////->
	
    static uint16_t knee_ = 0;
	static int foot_1 = 0;
	static int foot_2 = 0;
	static int foot_3 = 0;
	static int foot_4 = 0;
	static int foot_5 = 0;
	static int foot_6 = 0;
	static int foot_7 = 0;
	static int foot_8 = 0;
	static int foot_9 = 0;
	static int foot_10 = 0;
	static int foot_11 = 0;
	static int foot_12 = 0;
	static int foot_13 = 0;
	static int foot_14 = 0;
	static int foot_15 = 0;
	static int foot_16 = 0;
	
	static float leg = 0;         //leg
	static float knee = 0;        //knee
	static float foot_front = 0;  //foot
	static float foot_middle = 0; //foot
	static float foot_back = 0;   //foot
	
	////<-
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	////->
	
	mpu_getdata();
	adc_getdata(&knee_);
    trans_hex_R(2, &foot_1, 
	               &foot_2,
				   &foot_3,
				   &foot_4,
				   &foot_5,
				   &foot_6,
				   &foot_7,
				   &foot_8,
				   &foot_9,
				   &foot_10,
				   &foot_11,
				   &foot_12,
				   &foot_13,
				   &foot_14,
				   &foot_15,
				   &foot_16 );
				   
	leg = roll + 90;                           //leg
	knee = 4096 - knee_;                       //knee
	foot_front = foot_3 + foot_5;              //foot
	foot_middle = foot_9 + foot_11 + foot_13;  //foot
	foot_back = foot_15;                       //foot
	
	trans_bt_T(3, 0, 0, 0, 
			   leg, knee, foot_front, foot_middle, foot_back,
			   0, 0, 0, 0, 0,
			   0, 0, 0, 0, 0,
			   0, 0, 0, 0, 0,
			   0, 0, 0, 0, 0,
			   0, 0, 0, 0, 0 );
	
//  printf("%3d %3d %3d ", (int)yaw, (int)roll, (int)pitch);    
//	printf("%5d ", knee_);
//	printf("%5d ", foot_1);
//	printf("%5d ", foot_2);
//	printf("%5d ", foot_3);
//	printf("%5d ", foot_4);
//	printf("%5d ", foot_5);
//	printf("%5d ", foot_6);
//	printf("%5d ", foot_7);
//	printf("%5d ", foot_8);
//	printf("%5d ", foot_9);
//	printf("%5d ", foot_10);
//	printf("%5d ", foot_11);
//	printf("%5d ", foot_12);
//	printf("%5d ", foot_13);
//	printf("%5d ", foot_14);
//	printf("%5d ", foot_15);
//	printf("%5d ", foot_16);
     
	printf("%5d %5d %5d %5d %5d", (int)leg, (int)knee, (int)foot_front, (int)foot_middle, (int)foot_back);
	printf("\r\n");
	
	////<-
}
