//------------------------------------------------------------------------------
// System Configuration
//------------------------------------------------------------------------------

//motor power
#define Motor_DCV        24  //unit: V

//voltage divider //(94.0/1.5) for 310V; (47.0/4.3) for 48V; (47.0/9.1) for 24V; (47.0/20.0) for 12V; (56.0/10.0) for 24V/MotorDK
#define bemf_Divider     (56.0/10.0) //56K,10.0K (DCbus max.=33V)
#define DCbus_Divider    (100.0/10.0) //100K,10K	

//------------------------------------------------------------------------------
// Motor Parameters
//------------------------------------------------------------------------------

#define PWM_Freq 20 //unit KHz
#define TIM1_Arr ((SystemCoreClock/(PWM_Freq*2000))-1) //(=1199 when the Systemclock=48MHz and PWM_Freq=20)
#define PWM_Duty 100-((((TIM1->CCR1+1)*100)/(TIM1_Arr+1)))

#define flag_open_loop 1
#define flag_clockwise 1

#define Tzcd 10 //unit us 每隔Tzcd进中断做一次过零点侦测

//*** general

	//rotor's pole number
	#define Pole_No 12


 	//width of 'L*di/dt' just after commutating (1~30)
	#define Ldidt_Angle_L 3 //unit: degree (at low speed)
	#define Ldidt_Angle_H 10 //unit: degree (at high speed)
	
	//lead angle before next commutating (0~29)
	#define Lead_Angle_L 0  //unit: degree (at low speed)
	#define Lead_Angle_H 15 //unit: degree (at high speed)

	//zero-point compensation (0~32, 0: no compensation)
	#define ZP_Compensation 10
	
	//*** startup

 	//startup duty: for motor to start stably
	#define Startup_Duty 10.0 //unit: %
	#define Startup_Duty2 Startup_Duty //unit: % //(if against the wind)

	//startup step time: for motor to start stably
	#define Startup_StepTime 20 //unit: ms (1~100)

	//startup speed: for motor to start smoothly
	#define Startup_Speed 1200 //30~2400

	//startup threshold: lower than this threshold, the motor is supposed to keep still before startup
	#define Startup_Threshold 64 //1~255 (back-emf's ADC value) [255: suppose motor kept still]

	//do brake before startup
	#define Startup_Brake 0	//1=Yes; 0=No

	//soft startup
	#define Startup_Soft 0 //1=Yes; 0=No

	//startup alignment
	#define Startup_Alignment 0 //1=Yes; 0=No
	#define Startup_Alignment_Duty 10 //unit: %
	#define Startup_Alignment_Time 100 //unit: ms
	
	//startup short step
	#define Startup_Short_Step 0 //1=Yes; 0=No

	//startup opposite step & opposite time
	#define Startup_Opposite_Step 1 //1=Yes; 0=No
	#define Startup_Opposite_Time 4 //unit: ms
	
	

