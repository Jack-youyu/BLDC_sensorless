#include "HAL_device.h"
#include "HAL_conf.h"
#include "stdio.h"
#include "delay.h"
#include "oled.h"
#include "bmp.h"
#include "parameter.h"

#define LED1_ON()  GPIO_ResetBits(GPIOA,GPIO_Pin_11)	// PA11
#define LED1_OFF()  GPIO_SetBits(GPIOA,GPIO_Pin_11)	// PA11
#define LED1_TOGGLE()  (GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_11))?(GPIO_ResetBits(GPIOA,GPIO_Pin_11)):(GPIO_SetBits(GPIOA,GPIO_Pin_11))	// PA11

#define LED2_ON()  GPIO_ResetBits(GPIOA,GPIO_Pin_12)	// PA12
#define LED2_OFF()  GPIO_SetBits(GPIOA,GPIO_Pin_12)	// PA12
#define LED2_TOGGLE()  (GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_12))?(GPIO_ResetBits(GPIOA,GPIO_Pin_12)):(GPIO_SetBits(GPIOA,GPIO_Pin_12))	// PA12

#define PB13_OUT0 GPIOB->BRR|=0x00002000
#define PB14_OUT0 GPIOB->BRR|=0x00004000
#define PB15_OUT0 GPIOB->BRR|=0x00008000

#define PB13_OUT1 GPIOB->BSRR|=0x00002000
#define PB14_OUT1 GPIOB->BSRR|=0x00004000
#define PB15_OUT1 GPIOB->BSRR|=0x00008000



 #define s1 1 //step1
 #define s2 2 //step2
 #define s3 3 //step3
 #define s4 4 //step4
 #define s5 5 //step5
 #define s6 6 //step6

uint16_t VSP_adc=0,VSP_duty,speed_cnt[10]={0},speed;
unsigned int target_duty,tar_cnt;
static uint8_t time_over=0;

unsigned char pole_pair;
unsigned int cnt_Ldidt;
unsigned int cnt_30deg;

unsigned int zero_point,zero_point_rising,zero_point_falling;

unsigned int zd1,zd2,zd3;
unsigned int zp1,zp2,zp3;

unsigned char flag_Ldidt,flag_30deg,flag_zcd;
unsigned char flag_check_stop;
unsigned char sensorless_state;
unsigned char flag_one_roll,flag_one_step;

unsigned char flag_motor_run,flag_motor_stop;
unsigned int roll_cnt,elec_period_cnt,Tar_cnt;
unsigned char safe_ramp_up;
unsigned int rpm,rpm1,rpm2,rpm3,rpm4,rpm5,rpm6,rpm7,rpm8;
unsigned int target_rpm;
unsigned int rpmTL1,rpmTL2,rpmTH1,rpmTH2,rpmT1,rpmT2,rpmT3,rpmT4;
unsigned int rpm_max,rpm_min;
unsigned char angle_Ldidt,angle_Ldidt_L,angle_Ldidt_H;
unsigned char angle_30deg,angle_30deg_L,angle_30deg_H;
unsigned char zpc_compensation;
unsigned char flag_do_remove_vibration;

unsigned int startup_duty_org;
unsigned char startup_steptime_org;

unsigned char angle_Ldidt_L_org,angle_Ldidt_H_org;
unsigned char angle_30deg_L_org,angle_30deg_H_org;
unsigned char zp_compensation_org;
unsigned char adjustrpm_wait3_org;
unsigned char rough_adj_org;

unsigned char zpc_rising,zpc_falling,zpc_falling_max;

volatile unsigned int current_adc,ca1,ca2,ca3;
volatile unsigned int current_offset;
unsigned int full_current_adc,over_current_adc;
unsigned char flag_over_current,flag_full_current;
//startup
unsigned int startup_alignment_duty;
unsigned int startup_alignment_time;
unsigned int startup_duty;
unsigned int startup_steptime;
unsigned int startup_speed;

//function prototype
void Periph_Init(void);
void RCC_Config(void);
void GPIO_Config(void);
void AD_Init(void);
uint16_t do_ADC_speed_command(void);

void BLDC_TIM1_Config(void);
void BLDC_TIM2_Config(void);
void reset_TIM2(void);
void TIM2_isr(void);
void NVIC_Config(void);
void BLDC_Start(void);
void detect_speed_command(void);
void adjust_speed(void);
void set_drive_duty(unsigned int duty);
void reset_SysTick(void);
void parameter_Init(void);

void start_zero_crossing_detection(void);
void detect_zero_crossing(void);
unsigned int get_avg(unsigned int value);

void drive_a_3P_BLDC_motor_square_sensorless(void);
void start_motor(void);
void stop_motor(void);
void UVW_off(void);
void motor_start(void);
void sensorless_startup_cw_step(void);
void sensorless_startup_ccw_step(void);
void wait_Ldidt(void);
void wait_30deg(void);

void check_current(void);
void adjust_Ldidt_30deg_zpc(void);

void UH_on(void);
void UL_on(void);
void VH_on(void);
void VL_on(void);
void WH_on(void);
void WL_on(void);

void UH_off(void);
void UL_off(void);
void VH_off(void);
void VL_off(void);
void WH_off(void);
void WL_off(void);

void commutate_UV(void);
void commutate_UW(void);	
void commutate_VW(void);
void commutate_VU(void);
void commutate_WU(void);
void commutate_WV(void);

void test_startup_6step(void);
void charge_gate_driver_capacitor(void);
void test_bemf_during_stop(void);
void test_bemf_circuit(void);
void test_speed_command(void);
/********************************************************************************************************
**������Ϣ ��main(void)
**�������� ��
**������� ����
**������� ����
********************************************************************************************************/
int main(void)
{
	Periph_Init();
	OLED_Init();
	LED1_ON();
	LED2_OFF();
	//test_startup_6step();
	//motor_start();
	flag_motor_run=FALSE;//motor not run
	flag_motor_stop=TRUE;//motor stop
	drive_a_3P_BLDC_motor_square_sensorless();
    while(1) {            //����ѭ��

		//detect_speed_command();

//		OLED_ShowString(8,0,"BLDC_AD_TEST",16);

//		OLED_ShowNum(20,16,VSP_adc,4,16);
//		OLED_ShowNum(20,32,time_over,2,16);
//		OLED_ShowNum(20,48,target_duty,3,16);
//		OLED_ShowNum(68,32,speed,5,16);
//		OLED_ShowNum(68,48,TIM2->CCR1,5,16);
		
		//test_bemf_during_stop();
		//test_bemf_circuit();
		//test_speed_command();
//		OLED_Refresh();
//		delay_ms(1000);
    }
}

/********************************************************************************************************
**������Ϣ ��Periph_Init(void)
**�������� ��Periph��ʼ��
**������� ����
**������� ����
********************************************************************************************************/
void Periph_Init(void)
{
	
	/*ʹ������ʱ��*/
	RCC_Config();
	
	/*GPIO����*/
	GPIO_Config();
	
	parameter_Init();
	
	/*ADC1����*/
	AD_Init();
	
	/*TIM����*/
	BLDC_TIM1_Config();
	BLDC_TIM2_Config();
	
	
	/*�ж����ȼ�����*/
	NVIC_Config();
}

/********************************************************************************************************
**������Ϣ ��RCC_Configuration
**�������� ������ʱ��ʹ��
**������� ����
**������� ����
********************************************************************************************************/
void RCC_Config(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB,ENABLE);//ʹ��GPIOA,GPIOBʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1|RCC_APB2Periph_ADC1,ENABLE);//ʹ��TIM1ʱ�ӣ�ADC1ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2|RCC_APB1Periph_TIM3,ENABLE);//ʹ��TIM2ʱ��,TIM3ʱ��
}
/********************************************************************************************************
**������Ϣ ��GPIO_Configuration
**�������� ������ʱ��ʹ��
**������� ����
**������� ����
********************************************************************************************************/
void GPIO_Config(void)
{
	
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	/*LED_GPIO����*/
	GPIO_InitStructure.GPIO_Pin  =  GPIO_Pin_11|GPIO_Pin_12;//LED
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/*TIM1_GPIO����*/
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;//TIM1��� UH VH WH
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;//TIM1��� UL VL WL
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/*OVP and speed adjust detect GPIO����*/
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1;// PB0|PB1
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/*bemf_GPIO  I_sum GPIO����*/
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;//to detect the threshold of zero-crossing point and DC_bus current for current limitation and protection
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;//as analog input for ADC1
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/*GPIOA Pin8,9,10 TIM1_CH1 CH2 CH3�������*/
	//GPIOA->AFRL=0x00000222;//Pin0,1,2 AF-2
	GPIOA->AFRH=0x00000222;//Pin8,9,10 AF-2
	
}
/**********************************************************************
* Description    : parameter_init
* Input          : None
* Output         : None
* Return         : None
* Attention      : 
**********************************************************************/
void parameter_Init(void)
{
	pole_pair=Pole_No/2;
	
	//startup
	startup_alignment_duty=Startup_Alignment_Duty;
	startup_alignment_time=Startup_Alignment_Time;
	
	startup_duty=Startup_Duty;
	startup_steptime=Startup_StepTime;
	
	startup_speed=Startup_Speed;
	
	
	
	
	
}
/**********************************************************************
* Description    : SysTick_configuration
* Input          : None
* Output         : None
* Return         : None
* Attention      : SysTick is a 24-bit down-counter
**********************************************************************/
void SysTick_Init(void)
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);//systickclock=HCLK
	SysTick->LOAD=0x1000000-1; //interrupt period is (2^24)/48=349525.3us =0.35s
	
	reset_SysTick();
	
	SysTick->CTRL=SysTick_CTRL_ENABLE_Msk;//ENABLE=1: enable SysTick, start counting

}
void reset_SysTick(void)
{
	SysTick->VAL=0;//reset counter and immediately reload counter
}
void enable_interrupt_for_tick_clock(void)
{
	SysTick->CTRL|=0x00000001;
}
void disable_interrupt_for_tick_clock(void)
{
	SysTick->CTRL&=0xFFFFFFFE;
}

void SysTick_Handler(void)
{
	

}
/**********************************************************************
* Description    : AD_configuration
* Input          : None
* Output         : None
* Return         : None
* Attention      : None
**********************************************************************/
void AD_Init(void)
{
	ADC_InitTypeDef ADC_InitTypeDef;
	
	ADC_DeInit(ADC1);
	
	ADC_InitTypeDef.ADC_Mode=ADC_Mode_Single;//single sampling
	
	ADC_InitTypeDef.ADC_Resolution=ADC_Resolution_12b;//Resolution 12bit
	ADC_InitTypeDef.ADC_PRESCARE=ADC_PCLK2_PRESCARE_2;//HCLK/2/2=12Mhz
	ADC1->ADCFG|=0x00000400;//select sampling time 7.5cycle
	
	ADC_Init(ADC1,&ADC_InitTypeDef);
	ADC1->ADCR&=0xfffff7ff;//Data align method:right align
	ADC_Cmd(ADC1,ENABLE);

}

uint32_t do_ADC_bemf_U(void)
{
	ADC1->ADCHS=0x0001;////select ADC channel 0
	ADC1->ADCR|=0x00000100;//ADST=1: start A/D conversion
	while(!(ADC1->ADSTA&0x00000001));//wait ADIF=1: wait conversion ok
	ADC1->ADSTA|=0x00000001;//write 1 to clear ADIF
	
	return (ADC1->ADDATA&0x0000FFFF);
}

uint32_t do_ADC_bemf_V(void)
{
	ADC1->ADCHS=0x0002;////select ADC channel 1
	ADC1->ADCR|=0x00000100;//ADST=1: start A/D conversion
	while(!(ADC1->ADSTA&0x00000001));//wait ADIF=1: wait conversion ok
	ADC1->ADSTA|=0x00000001;//write 1 to clear ADIF
	
	return (ADC1->ADDATA&0x0000FFFF);
}

uint32_t do_ADC_bemf_W(void)
{
	ADC1->ADCHS=0x0004;////select ADC channel 2
	ADC1->ADCR|=0x00000100;//ADST=1: start A/D conversion
	while(!(ADC1->ADSTA&0x00000001));//wait ADIF=1: wait conversion ok
	ADC1->ADSTA|=0x00000001;//write 1 to clear ADIF
	
	return (ADC1->ADDATA&0x0000FFFF);
}

uint32_t do_ADC_I_sum(void)
{
	ADC1->ADCHS=0x0008;////select ADC channel 3
	ADC1->ADCR|=0x00000100;//ADST=1: start A/D conversion
	while(!(ADC1->ADSTA&0x00000001));//wait ADIF=1: wait conversion ok
	ADC1->ADSTA|=0x00000001;//write 1 to clear ADIF
	
	return (ADC1->ADDATA&0x0000FFFF);
}

uint32_t do_ADC_DCbus(void)
{
	ADC1->ADCHS=0x0100;//select ADC channel 8
	ADC1->ADCR|=0x00000100;//ADST=1: start A/D conversion
	while(!(ADC1->ADSTA&0x00000001));//wait ADIF=1: wait conversion ok
	ADC1->ADSTA|=0x00000001;//write 1 to clear ADIF
	
	return (ADC1->ADDATA&0x0000FFFF);
}
/**********************************************************************
* Description    : Do AD conventer
* Input          : None
* Output         : None
* Return         : AD conventer data
* Attention      : None
**********************************************************************/
uint16_t do_ADC_speed_command(void)
{
	ADC1->ADCHS=0x0200;//select ADC channel 9
	ADC1->ADCR|=0x00000100;
	while(!(ADC1->ADSTA&0x00000001));
	ADC1->ADSTA|=0x00000001;//write 1 to clear ADIF
	return (ADC1->ADDATA&0x0000FFFF);
}
/**********************************************************************
* Description    : Detect analog voltage speed command
* Input          : None
* Output         : None
* Return         : None
* Attention      : None
**********************************************************************/
void detect_speed_command(void)
{
	//uint16_t VSP_duty;
	VSP_adc=do_ADC_speed_command();//get current target voltage
	VSP_duty=(VSP_adc*100)/4096;//target duty ����
	if(VSP_duty>5)//minimun startup duty
	{
		target_duty=VSP_duty;
	}
	else
	{
		target_duty=0;
	}
}
/**********************************************************************
* Description    : Adjust speed
* Input          : None
* Output         : None
* Return         : None
* Attention      : None
**********************************************************************/
void adjust_speed(void)
{
	
	unsigned int temp,i;
//	speed_cnt[0]=speed_cnt[1];
//	speed_cnt[1]=speed_cnt[2];
//	speed_cnt[2]=speed_cnt[3];
//	speed_cnt[3]=speed_cnt[4];
//	speed_cnt[4]=speed_cnt[5];
//	speed_cnt[5]=speed_cnt[6];
//	speed_cnt[6]=speed_cnt[7];
//	speed_cnt[7]=speed_cnt[8];
//	speed_cnt[8]=speed_cnt[9];
//	speed_cnt[9]=TIM2->CCR1+(65535*time_over);
//	for(i=0;i<10;i++)
//	{
//		speed+=speed_cnt[i];
//	}
//	speed/=10;
	
	
	if(flag_open_loop)//open loop drivering 
	{
		temp=100-((((TIM1->CCR1+1)*100)/(TIM1_Arr+1)));
		if(temp!=target_duty)
		{
			if(temp<target_duty)
			{
				if(TIM1->CCR1>4)
				{
					TIM1->CCR1-=4;
					TIM1->CCR2-=4;
					TIM1->CCR3-=4;
				}
				else
				{
					TIM1->CCR1=1;
					TIM1->CCR2=1;
					TIM1->CCR3=1;
				}
			}
			else
			{
				if(TIM1->CCR1<(TIM1_Arr+1)-4)
				{
					TIM1->CCR1+=4;
					TIM1->CCR2+=4;
					TIM1->CCR3+=4;
				}

			}
		
		}
		else
		{
			LED2_ON();
		}
		return ;
	}
	
	
}
/**********************************************************************
* Description    : Set driver duty
* Input          : driver duty
* Output         : None
* Return         : None
* Attention      : duty:0 to 100
**********************************************************************/
void set_drive_duty(unsigned int duty)
{
	unsigned int temp;
	temp=((100-duty)*(TIM1_Arr+1))/100;
	TIM1->CCR1=temp;
	TIM1->CCR2=temp;
	TIM1->CCR3=temp;

}
/**********************************************************************
* Description    : BLDC_TIM1_Configuration
* Input          : None
* Output         : None
* Return         : None
* Attention      : None
**********************************************************************/
void BLDC_TIM1_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef	TIM_OCStructure;
	TIM_BDTRInitTypeDef TIM_BDTRStructure;
	
	TIM_DeInit(TIM1);//default all parameter
	
	/*�� ����TIM1������*/
	TIM_TimeBaseStructure.TIM_Period=TIM1_Arr;
	TIM_TimeBaseStructure.TIM_Prescaler=0;//prescale 0+1
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_CenterAligned2;//center aligned mode2������Ƚϱ�־ֻ�����ϼ���ʱ����
	TIM_TimeBaseStructure.TIM_ClockDivision=0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
	
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);//��ʼ����ʱ��
	
	/*�� ���ñȽ����ͨ��*/
	TIM_OCStructure.TIM_OCMode=TIM_OCMode_PWM2;//����Ƚ�ģʽ-PWMģʽ2
	TIM_OCStructure.TIM_OCIdleState=TIM_OCIdleState_Set;//OCx���е�ƽΪ��
	TIM_OCStructure.TIM_OCNIdleState=TIM_OCNIdleState_Reset;//OCNx���е�ƽΪ��
	TIM_OCStructure.TIM_OCPolarity=TIM_OCPolarity_High;//OCx������� �ߵ�ƽ��Ч
	TIM_OCStructure.TIM_OCNPolarity=TIM_OCNPolarity_High;//OCNx������� �ߵ�ƽ��Ч
	TIM_OCStructure.TIM_OutputState=TIM_OutputState_Disable;//OCx����ر�
	TIM_OCStructure.TIM_OutputNState=TIM_OutputNState_Disable;//OCNx����ر�
	TIM_OCStructure.TIM_Pulse=0;//U phase duty
	
	TIM_OC1Init(TIM1,&TIM_OCStructure);//��ʼ������Ƚ�ͨ��1 UH
	
	TIM_OCStructure.TIM_Pulse=0;//V phase duty
	TIM_OC2Init(TIM1,&TIM_OCStructure);//��ʼ������Ƚ�ͨ��2 VH
	
	TIM_OCStructure.TIM_Pulse=0;//W phase duty
	TIM_OC3Init(TIM1,&TIM_OCStructure);//��ʼ������Ƚ�ͨ��3 WH
	
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);//ʹ�ܲ���ȽϼĴ���1Ԥװ��
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);//ʹ�ܲ���ȽϼĴ���2Ԥװ��
	TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Enable);//ʹ�ܲ���ȽϼĴ���3Ԥװ��
	
	/*�� ����ɲ�����������ƼĴ���*/	
	TIM_BDTRStructure.TIM_AutomaticOutput=TIM_AutomaticOutput_Disable;//�Զ����ʹ�ܹر�
	TIM_BDTRStructure.TIM_Break=TIM_Break_Disable;//��ֹɲ������
	TIM_BDTRStructure.TIM_BreakPolarity=TIM_BreakPolarity_Low;//ɲ�����뼫�� �͵�ƽ��Ч
	TIM_BDTRStructure.TIM_DeadTime=10;//����ʱ�� 10us
	TIM_BDTRStructure.TIM_LOCKLevel=TIM_LOCKLevel_OFF;//�����ر�
	TIM_BDTRStructure.TIM_OSSIState=TIM_OSSIState_Enable;//ʹ�ܿ���ģʽ��(MOE=0)��ʱ��������ʱOCx/OCxN�����
	TIM_BDTRStructure.TIM_OSSRState=TIM_OSSRState_Enable;//ʹ����תģʽ��(MOE=1)��ʱ��������ʱOCx/OCxN�����
	
	TIM_BDTRConfig(TIM1,&TIM_BDTRStructure);//��ʼ��ɲ�����������ƼĴ���
	
	/*�� ����TIM1���ⲿ����ͬ��������Դ*/
	
	//TIM_SelectInputTrigger(TIM1, TIM_TS_ITR1); //���봥��Դѡ��TIM2 TIM1->SMCR TSλ:001 
	
	
	/*�� ʹ��TIM1��PWM���*/
	TIM_CtrlPWMOutputs(TIM1,ENABLE);//MOE=1��ʹ��PWM���

	TIM_Cmd(TIM1,ENABLE);//CEN=1��ʹ��TIM1
}
void enable_intterupt_for_adjust_rpm(void)
{
	TIM1->DIER|=0x00000001;//UIE=1: enable update interrupt
}
void disable_interrupt_for_adjust_rpm(void)
{
	TIM1->DIER&=0xFFFFFFFE;//UIE=0: disable update interrupt
}
void enable_interrupt_for_TIM1_break(void)
{
	TIM1->DIER|=0x000000070;//BIE=1: enable break interrupt
}
void disable_interrupt_for_TIM1_break(void)
{
	TIM1->DIER&=0xFFFFFF8F;//BIE=0: disable break interrupt
}
/**********************************************************************
* Description    : BLDC_TIM2_Configuration 
* Input          : None
* Output         : None
* Return         : None
* Attention      : TIM2 as a 32-bit up-counter/down-counter
**********************************************************************/
void BLDC_TIM2_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	TIM_DeInit(TIM2);//default all parameter
	
	/*�� ����TIM2������*/
	TIM_TimeBaseStructure.TIM_Period=0xFFFFFFFF;//ARR	
	TIM_TimeBaseStructure.TIM_Prescaler=0;//prescale 1
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//���ϼ���
	TIM_TimeBaseStructure.TIM_ClockDivision=0;//����ʱ�ӷָ�0
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;//�ظ�����0
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);//��ʼ��������2
	
	TIM2->CR1&=0xFFF6;//URS=0 select update request source UDIS=0 enabl UEV
	
	reset_TIM2();
	/*�� ��TIM2*/
	TIM2->CR1|=0x0001;//ʹ��TIM2������

}

void reset_TIM2(void)
{
	TIM2->EGR|=0x0001;//UG=1 generate a update event to reset TIM2
	while(!(TIM2->SR&0x0001));//wait UIF set by H/W
	TIM2->SR&=0x0000;//clear UIF
}

void enable_interrupt_for_Ldidt_30deg_zcd(void)
{
	TIM2->DIER|=0x0001;//UIE=1����TIM2�����ж�
}
void disable_interrupt_for_Ldidt_30deg_zcd(void)
{
	TIM2->DIER&=0xFFFE;//UIE=0,�ر�TIM2�����ж�
}


/**********************************************************************
* Description    : NVIC_config
* Input          : None
* Output         : None
* Return         : None
* Attention      : None
**********************************************************************/
void NVIC_Config(void)
{
	//for Ldidt delay, 30deg delay and zero-crossing detection interval
	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn,1); //(0: highest; 3: lowest)
	
	//for rpm adjusting and OCP_by_PWM_break using TIM1 break input
	NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
	NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn,3); //(0: highest; 3: lowest)
	
}

/**********************************************************************
* Description    : BLDC_TRG_COM interrupt request handle function
* Input          : None
* Output         : None
* Return         : None
* Attention      : None
**********************************************************************/
void TIM1_BRK_UP_TRG_COM_IRQHandler()
{
	unsigned int t;
	tar_cnt++;
	t=1;
	if(tar_cnt>(t*20))//pwm interrupt period 50us(frequence 20Khz)==>20 interrupts means 1ms
	{
			adjust_speed();
	}
	TIM1->SR&=0x0000;//���жϱ�־λ
	
}

/**********************************************************************
* Description    : TIM2 interrupt handle function
* Input          : None
* Output         : None
* Return         : None
* Attention      : None
**********************************************************************/
void TIM2_IRQHandler()
{
	TIM2_isr();
	
	if(TIM2->SR&0x0001)//�����ж�
	{
		TIM2->SR&=0xFFFE;//���жϱ�־λ
	}
	
}
void TIM2_isr(void)
{
	if(flag_zcd)
	{
		detect_zero_crossing();
	}
	else if(flag_Ldidt)//wait_Ldidt��������ʼ��������
	{
		start_zero_crossing_detection();
	}
	else if(flag_30deg)//wait_30deg���������̻���
	{
		switch(sensorless_state)
		{
			case s1:
				commutate_UV();//UV(s1)
				break;
			case s2:
				commutate_UW();//UW(s2)
				break;
			case s3:
				commutate_VW();//VW(s3)
				break;
			case s4:
				commutate_VU();//VU(s4)
				break;
			case s5:
				commutate_WU();//WU(s5)
				break;
			case s6:
				commutate_WV();//WV(s6)
				break;
		}
		
		wait_Ldidt();
		
		flag_check_stop^=1;//toggle this flag every step
		
		
	}
}

void start_zero_crossing_detection(void)
{
	TIM2->ARR=(48*(Tzcd*10))/10;//ÿ��Tzcd���жϣ�����������(TIM2 clock source 48MHz)
	reset_TIM2();//����TIM2
	
	flag_Ldidt=FALSE;
	flag_30deg=FALSE;
	flag_zcd=TRUE;//
	
	if((sensorless_state==s1)||(sensorless_state==s3)||(sensorless_state==s5))
	{
		if(flag_clockwise)//��falling���ʱ
		{
			zd1=zero_point*2;zd2=zd1;zd3=zd1;
		}
		else//��rising���ʱ
		{
			zd1=0;zd2=0;zd3=0;
		}
	}
	else//sensorless_state==s2||s4||s6
	{
		if(flag_clockwise)//��rising���ʱ
		{
			zd1=0;zd2=0;zd3=0;
		}
		else//��falling���ʱ
		{
			zd1=zero_point*2;zd2=zd1;zd3=zd1;
		}
	}
	
}
void detect_zero_crossing(void)
{
	unsigned int temp;
	switch(sensorless_state)
	{
		case s1://UV
			temp=do_ADC_bemf_W();
			temp=get_avg(temp);
			if(flag_clockwise)//detect zero-crossing of W's falling back-EMF
			{
				if(temp<zero_point_falling)
				{
					if(roll_cnt<4)
					{
						if(do_ADC_bemf_W()<zero_point_falling)return;//
					}
					sensorless_state=s2;//next state
					wait_30deg();//30 degree delay begining
					
				}
			}
			else//detect zero-crossing of W's rising back-EMF
			{
				if(temp>zero_point_rising)
				{
					if(roll_cnt<4)
					{
						if(do_ADC_bemf_W()<zero_point_rising)return;
					}
					sensorless_state=s6;//next state
					wait_30deg();//30 degree delay begining
					
				}
			}
			break;
			
		case s2://UW
			temp=do_ADC_bemf_V();temp=get_avg(temp);
			if(flag_clockwise)//detect zero-crossing of V's rising back-EMF
			{
				if(temp>zero_point_rising)
				{
					if(roll_cnt<4)
					{
						if(do_ADC_bemf_V()<zero_point_rising)return;
					}
					sensorless_state=s3;//next state
					wait_30deg();//30 degree delay begining
				}
			}
			else//detect zero-crossing of V's falling back-EMF
			{
				if(temp<zero_point_falling)
				{
					if(roll_cnt<4)
					{
					if(do_ADC_bemf_V()>zero_point_falling)return;
					}
				sensorless_state=s1;//change to next state
				wait_30deg();//30 degree delay begining
				}
			}
			break;
			
		case s3://VW
			temp=do_ADC_bemf_U();temp=get_avg(temp);
			if(flag_clockwise)//detect zero-crossing of U's falling back-EMF
			{
				if(temp<zero_point_falling)
				{
					if(roll_cnt<4)
					{
						if(do_ADC_bemf_U()>zero_point_falling)return;
					}
					sensorless_state=s4;//change to next state
					wait_30deg();
				}
			}
			else//detect zero-crossing of U's rising back-EMF
			{
				if(temp>zero_point_rising)
				{
					if(roll_cnt<4)
					{
						if(do_ADC_bemf_U()<zero_point_rising)return;
					}
					sensorless_state=s2;//change to next state
					wait_30deg();//30 degree delay begining
				}
			}
			break;
			
		case s4://VU
			temp=do_ADC_bemf_W(); get_avg(temp);
			if(flag_clockwise)//detect zero-crossing of W's rising back-EMF
			{
				if(temp>zero_point_rising)
				{
					if(roll_cnt<4)
					{
						if(do_ADC_bemf_W()<zero_point_rising)return;
					}
					sensorless_state=s5;//change to next state
					wait_30deg();//30 degree delay begining
				}
			}
			else//detect zero-crossing of W's falling back-EMF
			{
				if(temp<zero_point_falling)
				{
					if(roll_cnt<4)
					{
						if(do_ADC_bemf_W()>zero_point_falling)return;
					}
					sensorless_state=s3;//change to next state
					wait_30deg();//30 degree delay begining
				}
			}
			break;
		
		case s5://WU
			temp=do_ADC_bemf_V();temp=get_avg(temp);
			if(flag_clockwise)//detect zero-crossing of V's falling back��EMF
			{
				if(temp<zero_point_falling)
				{
					if(roll_cnt<4)
					{
						if(do_ADC_bemf_V()>zero_point_falling)return;
					}
					sensorless_state=s6;//change to next state
					wait_30deg();//30 degree delay begining
				}
			}
			else//detect zero-crossing of V's rising back-EMF
			{
				if(temp>zero_point_rising)
				{
					if(roll_cnt<4)
					{
						if(do_ADC_bemf_V()<zero_point_rising)return;
					}
					sensorless_state=s4;//change to next state
					wait_30deg();//30 degree delay begining
				}
			}
			break;
		
		case s6://WV
			temp=do_ADC_bemf_U();temp=get_avg(temp);
			if(flag_clockwise)//detect zero-crossing of U's rising back-EMF
			{
				if(temp>zero_point_rising)
				{
					if(roll_cnt<4)
					{
						if(do_ADC_bemf_U()<zero_point_rising)return;
					}
					sensorless_state=s1;//change to next state
					wait_30deg();//30 degree delay begining
				}
			}
			else //detect zero-crossing of U's falling back-EMF
			{
				if(temp<zero_point_falling)
				{
					if(roll_cnt<4)
					{
						if(do_ADC_bemf_U()>zero_point_falling)return;
					}
					sensorless_state=s5;//change to next state
					wait_30deg();//30 degree delay begining
				}
			}
			break;
			
	}
	
}
unsigned int get_avg(unsigned int value)
{
	value=(value+zd1+zd2+zd3)/4;
	zd3=zd2;zd2=zd1;zd1=value;
	return value;
}
void wait_30deg(void)
{
	TIM2->ARR=cnt_30deg;
	reset_TIM2();
	
	flag_Ldidt=FALSE;
	flag_30deg=TRUE;
	flag_zcd=FALSE;
}
void drive_a_3P_BLDC_motor_square_sensorless(void)
{
	while(1)
	{
	//�ٶ�ָ���ȡ
	//detect_speed_command();
	
	//if motor is not running
	if(!flag_motor_run)
	{
		//start motor
		start_motor();
	}
	//if motor is already running
	if(flag_motor_run)
	{
		//1. check if over current
		if(flag_over_current)
		{
			stop_motor();
			//ocp_cnt++;
			
		}
		//2. update Ldidt,30deg,zpc
		adjust_Ldidt_30deg_zpc();
		
	}
	
	
	
	
	}

}
void start_motor(void)
{
	//initialize variable
	roll_cnt=0;
	Tar_cnt=0;
	//run_time_cnt=0;
	//adjustrpm_wait_cnt=0;
	//zpc_cnt1=0; zpc_cnt2=0;
	//ocp_debounce_cnt=0;

	//OTP_debounce_cntH=0;
	//OTP_debounce_cntL=0;
	//flag_over_temperature_H=FALSE;
	//flag_over_temperature_L=FALSE;
	
	//old_tick_cnt=0;
	elec_period_cnt=0x1000000; //(! don't forget)	

	ca1=0; ca2=0; ca3=0;
	rpm=0; rpm1=0; rpm2=0; rpm3=0; rpm4=0; rpm5=0; rpm6=0; rpm7=0; rpm8=0;

	flag_over_current=FALSE;
	flag_full_current=FALSE;



	
	//brake before startup
	UVW_off();
	
	enable_interrupt_for_tick_clock();
	
	motor_start();
	
	enable_intterupt_for_adjust_rpm();
	
	flag_motor_run=TRUE;//run
}
void stop_motor(void)
{
	if(!flag_motor_run)
	{
		return;
	}
	disable_interrupt_for_Ldidt_30deg_zcd();//��ֹ�������κ�30�Ȼ����ж�
	disable_interrupt_for_adjust_rpm();//��ֹת�ٵ����ж�
	UVW_off();//UVWȫ��
	flag_motor_run=FALSE;//ͣת
	
}
void motor_start(void)
{
	unsigned int temp;
//precharge
	
	charge_gate_driver_capacitor();
//alignment (��ǰ,�ȶ�λ)
	UVW_off();
	
	set_drive_duty(startup_alignment_duty);
	
	if(flag_clockwise)
	{
		WH_on();VL_on();//WV(s6) �𲽵�һ����ǰһ��
		//WH_on();UL_on();//WU(s5) �𲽵�һ����ǰ����
	}
	else
	{
		UH_on();VL_on();//UW(s1)

	}
	delay_ms(startup_alignment_time);
	UVW_off();
	delay_ms(10);
	
//start from open-loop
	set_drive_duty(startup_duty);
	if(flag_clockwise)
	{
		sensorless_startup_cw_step();
		sensorless_state=s6;
		zero_point=do_ADC_bemf_W()/2;//�𶯵������WV(s6)
		//UVW_off();
	}
	else
	{
		sensorless_startup_ccw_step();
		sensorless_state=s1;
		
		//set initial value of zero-point
		zero_point=do_ADC_bemf_U()/2; //�𶯵������UV(s1)
	}
	
//enter closed-loop
	//initialize'cnt_Ldidt'&'cnt_30deg'
	temp=(48000000*5)/(pole_pair*startup_speed); //TIM2��30��ʱ���������Ĵ���
	cnt_Ldidt=temp/2;
	cnt_30deg=temp;
	
	//initialize'zero_point_rising'&'zero_point_falling'
	zero_point_rising=zero_point;
	zero_point_falling=zero_point;
	zp1=zero_point; zp2=zp1; zp3=zp1;
	
	wait_Ldidt();
	enable_interrupt_for_Ldidt_30deg_zcd();

	OLED_ShowNum(20,16,zero_point,4,16);
	OLED_Refresh();
	//while(1);
}
void sensorless_startup_cw_step(void)
{
	//startup1cw
	//commutate_UV();delay_ms(startup_steptime);//UV(s1)
	UH_on();VL_on();delay_ms(startup_steptime);//UV(s1)
	commutate_UW();delay_ms(startup_steptime);//UW(s2)
	commutate_VW();delay_ms(startup_steptime);//VW(s3)
	commutate_VU();delay_ms(startup_steptime);//VU(s4)
	commutate_WU();delay_ms(startup_steptime);//WU(s5)
	commutate_WV();//WV(s6)
	
}

void sensorless_startup_ccw_step(void)
{
	//startup6ccw
	commutate_WV();delay_ms(startup_steptime);//WV(s6)
	//WH_on();VL_on();delay_ms(startup_steptime);//WV(s6)
	commutate_WU();delay_ms(startup_steptime);//WU(s5)
	commutate_VU();delay_ms(startup_steptime);//VU(S4)
	commutate_VW();delay_ms(startup_steptime);//VW(s3)
	commutate_UW();delay_ms(startup_steptime);//UW(s2)
	commutate_UV();//UV(s1)
}

void adjust_startup_duty(void)
{
	

}
void wait_Ldidt(void)
{
	TIM2->ARR=cnt_Ldidt;
	reset_TIM2();
	
	flag_Ldidt=TRUE;
	flag_30deg=FALSE;
	flag_zcd=FALSE;
	
	if(roll_cnt>8)
	{
		check_current();//here, check current
		if(sensorless_state==s2)
		{
			//VSP_adc=do_ADC_Vsp();
		}
	}
	flag_one_step=TRUE;
		
}

void check_current(void)
{
	current_adc=do_ADC_I_sum();//detect current
	
	current_adc=(current_adc+ca1)/2;
	ca1=current_adc;
	
	if(current_adc>over_current_adc)
	{
		UVW_off();
		flag_over_current=TRUE;
	}
}
void adjust_Ldidt_30deg_zpc(void)
{
	//if safe ramp up 
	if(safe_ramp_up)
	{
		if(rpm<(rpmTL2*14/16))//��rough tuning�׶�
		{
			if(roll_cnt>16)//�ܿ��𲽽׶Σ�����ת16Ȧ��ſ�ʼʵʩ
			{
				angle_Ldidt_H=angle_Ldidt_H_org+8;
			}
		}
		else if(rpm>rpmTL2)//��fine tuning�׶�
		{
			if(angle_Ldidt_H>angle_Ldidt_H_org)
			{
				delay_ms(100);//ÿ���ӳ�100ms
				angle_Ldidt_H--;
			}
		}
	}
	//����ת�٣���̬����'angle_Ldidt'��'angle_30deg'
	if(rpm<rpm_min)
	{
		angle_Ldidt=angle_Ldidt_L;
		angle_30deg=angle_30deg_L;
	}
	else if(rpm>rpm_max)
	{
		angle_Ldidt=angle_Ldidt_H;
		angle_30deg=angle_30deg_H;
	}
	else
	{
		angle_Ldidt=angle_Ldidt_L+((int)(rpm-rpm_min)*(int)(angle_Ldidt_H-angle_Ldidt_L))/(int)(rpm_max-rpm_min);
		angle_30deg=angle_30deg_L+((int)(rpm-rpm_min)*(int)(angle_30deg_H-angle_30deg_L))/(int)(rpm_max-rpm_min);
	}
	//�������ڼ䣬�ָ�zpc_rising������ֵ
	if(roll_cnt>16)
	{
		if(!flag_do_remove_vibration)
		{
			zpc_rising=zpc_compensation;
		}
	}
}

void UVW_off(void)
{
	UH_off();
 	UL_off();
 	VH_off();
	VL_off();
	WH_off();
	WL_off();
	
}

void UH_on(void)
{
	TIM1->CCER|=0x00000001;//UH_ON
}
void UL_on(void)
{
	PB13_OUT1;
}
void VH_on(void)
{
	TIM1->CCER|=0x00000010;//VH_ON
}
void VL_on(void)
{
	PB14_OUT1;
}
void WH_on(void)
{
	TIM1->CCER|=0x00000100;//WH_ON
}
void WL_on(void)
{
	PB15_OUT1;
}


void UH_off(void)
{
	TIM1->CCER&=0xFFFFFFFE;
}
void UL_off(void)
{
	PB13_OUT0;
}
void VH_off(void)
{
	TIM1->CCER&=0xFFFFFFEF;
}
void VL_off(void)
{
	PB14_OUT0;
}
void WH_off(void)
{
	TIM1->CCER&=0xFFFFFEFF;
}
void WL_off(void)
{
	PB15_OUT0;
}

void commutate_UV(void)	
{
	if(flag_clockwise)
	{
		UH_on();WH_off();
	}
	else
	{
		VL_on();WL_off();
	}
}
void commutate_UW(void)
{
	if(flag_clockwise)
	{
		WL_on();VL_off();
	}
	else
	{
		UH_on();VH_off();
	}
}
void commutate_VW(void)
{
	if(flag_clockwise)
	{
		VH_on();UH_off();
	}
	else
	{
		WL_on();UL_off();
	}
}	
void commutate_VU(void)
{
	if(flag_clockwise)
	{
		UL_on();WL_off();
	}
	else
	{
		VH_on();WH_off();
	}
}	
void commutate_WU(void)
{
	if(flag_clockwise)
	{
		WH_on();VH_off();
	}
	else
	{
		UL_on();VL_off();
	}
}	
void commutate_WV(void)
{
	if(flag_clockwise)
	{
		VL_on();UL_off();
	}
	else
	{
		WH_on();UH_off();
	}
}	
void test_startup_6step(void)
{
	unsigned int step_time;
	step_time=15;
	set_drive_duty(10);
	delay_ms(1000);
	
	charge_gate_driver_capacitor();
	
	//alignment
	//UVW_off();
	WH_on();VL_on();//s6
	delay_ms(startup_alignment_time);
	
	while(1)
	{
		/*
		UVW_off();
		UH_on();VL_on();//s1
		delay_ms(step_time);
		
		UVW_off();
		UH_on();WL_on();//s2
		delay_ms(step_time);
		
		UVW_off();
		VH_on();WL_on();//s3
		delay_ms(step_time);
		
		UVW_off();
		VH_on();UL_on();//s4
		delay_ms(step_time);
		
		UVW_off();
		WH_on();UL_on();//s5
		delay_ms(step_time);
		
		UVW_off();
		WH_on();VL_on();//s6
		delay_ms(step_time);
		*/
	commutate_UV();delay_ms(step_time);//UV(s1)
	commutate_UW();delay_ms(step_time);//UW(s2)
	commutate_VW();delay_ms(step_time);//VW(s3)
	commutate_VU();delay_ms(step_time);//VU(s4)
	commutate_WU();delay_ms(step_time);//WU(s5)
	commutate_WV();delay_ms(step_time);//WV(s6)
	}
}
//==============================================================================
void charge_gate_driver_capacitor(void)
{
   //#if defined _MM32SPIN160C || defined _MM32SPIN360C
	//!!! let gate-driver's charge-pump capacitor pre-charged softly
	#define NOP1    __nop();
	#define NOP2    NOP1 NOP1
	#define NOP5    NOP2 NOP2 NOP1
	#define NOP10   NOP5 NOP5
	unsigned int i;
	for(i=0;i<35;i++){
		UL_on(); NOP10; UL_off(); delay_10us(10);
		VL_on(); NOP10; VL_off(); delay_10us(10);
		WL_on(); NOP10; WL_off(); delay_10us(10);
	}
   //#else
	//UL_on(); VL_on(); WL_on(); delay_ms(1); UVW_off(); delay_10us(1);
   //#endif
}
//==============================================================================
void test_bemf_during_stop(void)
{
	unsigned int tmp1,tmp2,tmp3;

   //#if with_Gate_Driver
	charge_gate_driver_capacitor();	//��ʹUVWΪ0
   //#endif

	tmp1=do_ADC_bemf_U();
	tmp2=do_ADC_bemf_V();
	tmp3=do_ADC_bemf_W();

	OLED_ShowNum(20,16,tmp1,4,16);
	OLED_ShowNum(20,32,tmp2,4,16);
	OLED_ShowNum(20,48,tmp3,4,16);
	OLED_Refresh();
	while(1);
}

//==============================================================================
void test_bemf_circuit(void)
{
	unsigned int tmp1,tmp2,tmp3;	

	//let gate-driver's charge-pump capacitor pre-charged
	UL_on(); VL_on(); WL_on(); delay_ms(1); UVW_off(); delay_ms(3);

	set_drive_duty(100); //��100%

	UH_on(); delay_ms(10); tmp1=do_ADC_bemf_U(); //1ms�����
//	UH_off(); delay_ms(10);

//	VH_on(); delay_ms(10); tmp2=do_ADC_bemf_V(); //1ms�����
//	VH_off(); delay_ms(10);

//	WH_on(); delay_ms(10); tmp3=do_ADC_bemf_W(); //1ms�����
//	WH_off(); delay_ms(10);

//	set_drive_duty(0); //��0%
	while(1)
	{
	delay_ms(10); tmp1=do_ADC_bemf_U(); //1ms�����
	OLED_ShowNum(20,16,tmp1,4,16);
	OLED_ShowNum(20,32,tmp2,4,16);
	OLED_ShowNum(20,48,tmp3,4,16);
	OLED_Refresh();
	//while(1);
	}
}
void test_speed_command(void)
{
	while(1)
	{
		delay_ms(100);
		VSP_adc=do_ADC_speed_command();//get current target voltage
		VSP_duty=(VSP_adc*100)/4096;//target duty ����
		OLED_ShowNum(20,16,VSP_duty,4,16);
		OLED_Refresh();
	}
}


