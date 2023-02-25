#include "include.h"
#include "trans_hex.h"
#include "adc.h"

int main(void)
{	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable ,ENABLE);//½ûÓÃJTAG ÆôÓÃ SWD
	delay_init();
	//////////////////////////////////////////////////->

	mpu__init();                  //leg
	adc_init();                   //knee
	trans_hex_init(2, 115200);    //foot
	
	trans_others_init(1, 115200); //uart2PC
	trans_bt_init(3, 115200);     //uart2bluetooth
	
	//////////////////////////////////////////////////<-
	maincycle_ms_init(50);
	while(1);
}

void Maincycle_Handler()
{
	//////////////////////////////////////////////////->
	//----- base
	static int cnt = 0;
	int i = 0;
	
	//----- state
	static enum{
	    NONESTATE1,
		RESCAL,
        NONESTATE2,
	    PHASEDIV
	}state = NONESTATE1;
	
	static char button_NONESTATE1_to_RESCAL = 0;
	static char button_RESCAL_to_NONESTATE2 = 0;
	static char button_NONESTATE2_to_PHASEDIV = 0;
	
	//----- pid
    // knee
    static uint16_t knee_sensor = 0;
	static float knee_convert = 0;
	static float knee_average_filter[5] = {0, 0, 0, 0, 0};
	static float knee_big = 0;
	static float knee = 0;
	static float knee_past = 0;
	static float knee_delta = 0;
	// leg
	static float leg_convert = 0;
	static float leg_average_filter[5] = {0, 0, 0, 0, 0};
	static float leg_big = 0;
	static float leg = 0;
	static float leg_past = 0;
	static float leg_delta = 0;
	static char  leg_NEGATIVE_OR_POSITIVE = 0;
	static char  leg_delta_NEGATIVE_OR_POSITIVE = 0;
	// foot
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
	static float foot_front_big = 0;
	static float foot_middle_big = 0;
	static float foot_back_big = 0;
	static float foot_front = 0;
	static float foot_middle = 0;
	static float foot_back = 0;
	static float foot_front_past = 0;
	static float foot_middle_past = 0;
	static float foot_back_past = 0;
	static float foot_front_delta = 0;
	static float foot_middle_delta = 0;
	static float foot_back_delta = 0;
	static float foot_front_SMALL_OR_BIG = 0;
	static float foot_middle_SMALL_OR_BIG = 0;
	static float foot_back_SMALL_OR_BIG = 0;
	
	//----- others
	static float knee_min = 1000000000;
	static float knee_max = -1000000000;
	static float leg_min = 1000000000;
	static float leg_max = -1000000000;
	static float foot_front_min = 1000000000;
	static float foot_front_max = -1000000000;
	static float foot_middle_min = 1000000000;
	static float foot_middle_max = -1000000000;
	static float foot_back_min = 1000000000;
	static float foot_back_max = -1000000000;
	
	static float threshold_foot_SMALL_OR_BIG = 0;
	static float threshold_leg_NEGATIVE_OR_POSITIVE = 0;
	static float threshold_leg_delta_NEGATIVE_OR_POSITIVE = 0;
	
	static enum{
		GP0,
	    GP1,
		GP2,
		GP3,
		GP4,
		GP5
	}GP_state = GP0;
	static int to_GP0_cnt = 0;
	static int to_GP1_cnt = 0;
	static int to_GP2_cnt = 0;
	static int to_GP3_cnt = 0;
	static int to_GP4_cnt = 0;
	static int to_GP5_cnt = 0;
	static int to_GPnot0_cnt = 0;
	static float state_change_delay = 0;
	
	//////////////////////////////////////////////////<-
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	//////////////////////////////////////////////////->
	cnt = (cnt == 99) ? 0 : cnt + 1;
	
	//----- R_data
	trans_bt_R(3, &button_NONESTATE1_to_RESCAL,
	              &button_RESCAL_to_NONESTATE2,
				  &button_NONESTATE2_to_PHASEDIV,
				  &threshold_foot_SMALL_OR_BIG, 
	              &threshold_leg_NEGATIVE_OR_POSITIVE, 
	              &threshold_leg_delta_NEGATIVE_OR_POSITIVE, 
	              &state_change_delay, 0,
				  0, 0, 0, 0, 0,
				  0, 0, 0, 0, 0,
				  0, 0, 0, 0, 0,
				  0, 0, 0, 0, 0,
				  0, 0, 0, 0, 0);
	
    //----- sensors
	//knee
	adc_getdata(&knee_sensor);
    knee_convert = knee_sensor;
	knee_big = 0;
	for(i = 0; i < 5; i++)
	    knee_big += knee_average_filter[i] / 5;
	for(i = 5 - 1; i > 0; i--)
	    knee_average_filter[i] = knee_average_filter[i - 1];
    knee_average_filter[0] = knee_convert;	
	//leg
	mpu_getdata();
	leg_convert = roll + 90;	
	leg_big = 0;
	for(i = 0; i < 5; i++)
	    leg_big += leg_average_filter[i] / 5;
	for(i = 5 - 1; i > 0; i--)
	    leg_average_filter[i] = leg_average_filter[i - 1];	
    leg_average_filter[0] = leg_convert;	
    //foot
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
	foot_front_big = foot_3 + foot_5;              
	foot_middle_big = foot_9 + foot_11 + foot_13;
	foot_back_big = foot_15;
	
	//----- state
	if(state == NONESTATE1)
	{
	    if(button_NONESTATE1_to_RESCAL != 0)
			state = RESCAL;
	}
	
	else if(state == RESCAL)
	{
	    if(button_RESCAL_to_NONESTATE2 != 0)
			state = NONESTATE2;
		
		if( knee_big < knee_min)
			knee_min = knee_big;
	    if( knee_big > knee_max)
			knee_max = knee_big;
		
		if( leg_big < leg_min)
			leg_min = leg_big;
	    if( leg_big > leg_max)
			leg_max = leg_big;
		
		if( foot_front_big < foot_front_min)
			foot_front_min = foot_front_big;
	    if( foot_front_big > foot_front_max)
			foot_front_max = foot_front_big;
		
		if( foot_middle_big < foot_middle_min)
			foot_middle_min = foot_middle_big;
	    if( foot_middle_big > foot_middle_max)
			foot_middle_max = foot_middle_big;
		
		if( foot_back_big < foot_back_min)
			foot_back_min = foot_back_big;
	    if( foot_back_big > foot_back_max)
			foot_back_max = foot_back_big;
	}
	
	else if(state == NONESTATE2)
	{
	    if(button_NONESTATE2_to_PHASEDIV != 0)
			state = PHASEDIV;
		
	}
	
	else if(state == PHASEDIV)
	{
		//num process
	    knee        = (knee_big        - knee_min       ) / (knee_max        - knee_min       );
		leg         = (leg_big         - leg_min        ) / (leg_max         - leg_min        ) * 2 - 1;
		foot_front  = (foot_front_big  - foot_front_min ) / (foot_front_max  - foot_front_min );
		foot_middle = (foot_middle_big - foot_middle_min) / (foot_middle_max - foot_middle_min);
		foot_back   = (foot_back_big   - foot_back_min  ) / (foot_back_max   - foot_back_min  );
		
		if     ( knee < 0) knee = 0;
	    else if( knee > 1) knee = 1;
		
		if     ( leg < -1) leg = -1;
	    else if( leg > 1 ) leg = 1;
		
		if     ( foot_front < 0) foot_front = 0;
	    else if( foot_front > 1) foot_front = 1;
		
		if     ( foot_middle < 0) foot_middle = 0;
	    else if( foot_middle > 1) foot_middle = 1;
		
		if     ( foot_back < 0) foot_back = 0;
	    else if( foot_back > 1) foot_back = 1;
		
		knee_delta        = knee        - knee_past;
		leg_delta         = leg         - leg_past;
		foot_front_delta  = foot_front  - foot_front_past;
		foot_middle_delta = foot_middle - foot_middle_past;
		foot_back_delta   = foot_back   - foot_back_past;
		
		knee_past         = knee;
		leg_past          = leg;
		foot_front_past   = foot_front;
		foot_middle_past  = foot_middle;
		foot_back_past    = foot_back;
		
	    leg_NEGATIVE_OR_POSITIVE       = (leg         >= threshold_leg_NEGATIVE_OR_POSITIVE)? 1 : 0;
	    leg_delta_NEGATIVE_OR_POSITIVE = (leg_delta   >= threshold_leg_delta_NEGATIVE_OR_POSITIVE)? 1 : 0;
	    foot_front_SMALL_OR_BIG        = (foot_front  >= threshold_foot_SMALL_OR_BIG)? 1 : 0;
	    foot_middle_SMALL_OR_BIG       = (foot_middle >= threshold_foot_SMALL_OR_BIG)? 1 : 0;
	    foot_back_SMALL_OR_BIG         = (foot_back   >= threshold_foot_SMALL_OR_BIG)? 1 : 0;
		
		//GP state
		if(     foot_front_SMALL_OR_BIG  == 1 || 
                foot_middle_SMALL_OR_BIG == 1 ||			
			    foot_back_SMALL_OR_BIG   == 1 )
	    {
			to_GP0_cnt += 1;
			
			if     (foot_front_SMALL_OR_BIG == 1 && foot_back_SMALL_OR_BIG == 0)
				to_GP1_cnt += 1;
			else if(foot_front_SMALL_OR_BIG == 1 && foot_back_SMALL_OR_BIG == 1)
				to_GP2_cnt += 1;
			else if(foot_front_SMALL_OR_BIG == 0 && foot_back_SMALL_OR_BIG == 1)
				to_GP3_cnt += 1;
	    }
		else
		{
			to_GPnot0_cnt += 1;
			
			if(     leg_NEGATIVE_OR_POSITIVE == 0 && leg_delta_NEGATIVE_OR_POSITIVE == 1)
				to_GP4_cnt += 1;
			else if(leg_NEGATIVE_OR_POSITIVE == 1 && leg_delta_NEGATIVE_OR_POSITIVE == 1)
				to_GP5_cnt += 1;		
		}

		
		if     (to_GP0_cnt == 40)
			    GP_state = GP0, to_GP0_cnt = 0, to_GP1_cnt = 0, to_GP2_cnt = 0, to_GP3_cnt = 0, to_GP4_cnt = 0, to_GP5_cnt = 0, to_GPnot0_cnt = 0;
		
		if     (GP_state == GP0)
		{
			if(to_GPnot0_cnt == 3)
				GP_state = GP4, to_GP0_cnt = 0, to_GP1_cnt = 0, to_GP2_cnt = 0, to_GP3_cnt = 0, to_GP4_cnt = 0, to_GP5_cnt = 0, to_GPnot0_cnt = 0;		
		}
		else
		{
			if     (to_GP1_cnt >= 1)
				GP_state = GP1, to_GP0_cnt = 0, to_GP1_cnt = 0, to_GP2_cnt = 0, to_GP3_cnt = 0, to_GP4_cnt = 0, to_GP5_cnt = 0, to_GPnot0_cnt = 0;
			
			else if(to_GP2_cnt >= (int)state_change_delay) 
				GP_state = GP2, to_GP0_cnt = 0, to_GP1_cnt =  0, to_GP2_cnt = 0, to_GP3_cnt = 0, to_GP4_cnt = 0, to_GP5_cnt = 0, to_GPnot0_cnt = 0;
			
			else if(to_GP3_cnt >= 1)
				GP_state = GP3, to_GP0_cnt = 0, to_GP1_cnt = 0, to_GP2_cnt = 0, to_GP3_cnt = 0, to_GP4_cnt = 0, to_GP5_cnt = 0, to_GPnot0_cnt = 0;
			
			else if(to_GP4_cnt >= (int)state_change_delay)
				GP_state = GP4, to_GP0_cnt = 0, to_GP1_cnt = 0, to_GP2_cnt = 0, to_GP3_cnt = 0, to_GP4_cnt = 0, to_GP5_cnt = 0, to_GPnot0_cnt = 0;
			
			else if(to_GP5_cnt >= (int)state_change_delay)
				GP_state = GP5, to_GP0_cnt = 0, to_GP1_cnt = 0, to_GP2_cnt = 0, to_GP3_cnt = 0, to_GP4_cnt = 0, to_GP5_cnt = 0, to_GPnot0_cnt = 0;		
		}	
	}
		
	//----- T_data
	
	if(cnt % 2 == 0)
	    trans_bt_T(3, state, 
	               leg_NEGATIVE_OR_POSITIVE, 
	               leg_delta_NEGATIVE_OR_POSITIVE, 
	               foot_front_SMALL_OR_BIG,
                   foot_middle_SMALL_OR_BIG,
                   foot_back_SMALL_OR_BIG,
	               GP_state, 0,
			       knee_big,        knee_min,        knee_max,        knee,        knee_delta,
			       leg_big,         leg_min,         leg_max,         leg,         leg_delta,
			       foot_front_big,  foot_front_min,  foot_front_max,  foot_front,  foot_front_delta,
			       foot_middle_big, foot_middle_min, foot_middle_max, foot_middle, foot_middle_delta,
			       foot_back_big,   foot_back_min,   foot_back_max,   foot_back,   foot_back_delta );
	
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
     
//	printf("%5d %5d %5d %5d %5d", (int)leg, (int)knee, (int)foot_front, (int)foot_middle, (int)foot_back);
//	printf("\r\n");
	
	//////////////////////////////////////////////////<-
}
