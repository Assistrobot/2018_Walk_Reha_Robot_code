// 소속 : 서울과학기술대 휴머노이드로봇 연구2실
// 이름 : 김명주
// 직책 : 석사과정
// 제목 : MyeongJuNRC - 국립재활원 프로젝트

#include "DSP28x_Project.h"                  // Device Headerfile and Examples Include File
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "string.h"

// pragma CODE_SECTION -> Flash에서 RAM으로 함수를 옮기기 위한 작업 .
#pragma CODE_SECTION(Encoder_define,"ramfuncs")
#pragma CODE_SECTION(FSLP_Value_define,"ramfuncs")
#pragma CODE_SECTION(Uart_transmit,"ramfuncs")
#pragma CODE_SECTION(UART_Put_String,"ramfuncs")
#pragma CODE_SECTION(Motor_transmit,"ramfuncs")
#pragma CODE_SECTION(Motor_Put_String,"ramfuncs")
#pragma CODE_SECTION(Motor_Put_Char,"ramfuncs")
#pragma CODE_SECTION(BT_transmit,"ramfuncs")


extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;

// CPU timer0 선언
interrupt void cpu_timer0_isr(void);
float32 Cpu_Clk;
float32 Timer_Prd;

Uint16 i = 0, M_i = 0, E_i = 0;

#define SYSCLK      150E6   /* 150MHz */
#define TBCLK       150E6   /* 150MHz */
#define PWMCARRIER  20E3    /* 20kHz */

//오른발의 변수 선언, R : Right , RU : Right Upper , LU : Left Upper , RL : Right Lower , LL : Left Lower
float position_R_RU_C, position_R_LL_C, position_R_RL_C, position_R_LU_C; // 오른쪽 발의 각 Position
float Force_R_RU_C, Force_R_LU_C ,Force_R_RL_C ,Force_R_LL_C, Force_R, Force_R_C; // 오른쪽 발의 각 Force
Uint16 position_R_RU, position_R_LL, position_R_RL, position_R_LU;
Uint16 Force_R_RU, Force_R_LU ,Force_R_RL ,Force_R_LL , FSLP_flag=0; // 오른쪽 발의 Force
float COP_R_Buff[10], Force_R_Buff[10]; // Moving Average를 위한 Buffer
float COP_R, Torque_R;
float COP_R_mva =0, Force_R_mva =0;

//왼쪽발의 변수 선언, L : Left , RU : Right Upper , LU : Left Upper , RL : Right Lower , LL : Left Lower
float position_L_RU_C, position_L_LL_C, position_L_RL_C, position_L_LU_C; // 왼쪽 발의 각 Position
float Force_L_RU_C, Force_L_LU_C ,Force_L_RL_C ,Force_L_LL_C, Force_L, Force_L_C; // 왼쪽 발의 각 Force
Uint16 position_L_RU, position_L_LL, position_L_RL, position_L_LU;
Uint16 Force_L_RU, Force_L_LU ,Force_L_RL ,Force_L_LL;
float COP_L_Buff[10], Force_L_Buff[10];
float COP_L, Torque_L;
float COP_L_mva =0, Force_L_mva =0;

// Encoder 변수 선언 , 함수 선언
Uint32 Encoder[10],Encoder_sum=0,Encoder_cnt=0,E_90_cnt=0;
double Encoder_deg_new=0;
double Encoder_deg_old=0;
double Encoder_deg=0;
Uint32 Encoder_revcnt =0;
double Encoder_vel=0;
double E_vel_deg_new=0;
double E_vel_deg_old=0;
double EV_Buff[30];
double EV_mva=0;


void Encoder_define();



// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@정준이의 변수선언@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
float motor_duty_by_torque=0;
int break_timer=0;
float break_duty=0;
double move_dis=0;
int a=0;
unsigned int mode_num=0;
unsigned int leg_num=0;
unsigned int start_bit=0;
unsigned int end_bit=0;
float Motor_Pwm=0;
unsigned int Init_finish=0;
double target_dis=0;
char target_dis_1=0;
char target_dis_2=0;
double training_timer=0;


double target_time=0;
double target_hour=0;
double target_min=0;
double target_sec=0;
double time_now=0;

unsigned int Type_sel=0;

double Init_distance=0;


double target_gain=0;
double ratio_gain=0;
unsigned int break_on_bit=0;
unsigned int pause_bit=0;
double Init_deg=0;
double velocity=0;
double under_velocity=0;
double Total_torque=0;


int  time_now_hour=0;
int  time_now_min=0;
int  time_now_min_10=0;
int time_now_min_1=0;
int move_distance_4=0;
int  move_distance_3=0;
int  move_distance_2=0;
int  move_distance_1=0;
int Robot_Init_bit=0;
unsigned int em_break=0;

double smooth_rise=0;



double       a0 =      0.7086  ;
double       a1 =     0.04348 ;
double       b1 =     0.04126 ;
double       a2 =    -0.03393 ;
double       b2 =    -0.01856 ;
double       a3 =     0.01754  ;
double       b3 =   0.0006721  ;
double       a4 =    -0.01032  ;
double       b4 =    0.003381 ;
double       a5 =    0.004178  ;
double       b5 =   -0.003621  ;
double       a6 =   -0.001026  ;
double       b6 =    0.002338  ;
double       a7 =  -0.0004222  ;
double       b7 =  -0.0008692  ;
double       a8 =   0.0005095  ;
double       b8 =   0.0002923  ;
double       w =     0.03604  ;
double APM_assist=0;

// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@




//정준이의 함수------------------
void Robot_Initalize(void);
void cleaer_variable(void);
//------------------------











// PWM Duty 변수 선언, 함수 선언
float32 PwmCarrier;
float32 PwmDuty , BrakeDuty=0;
void InitEPwm1Module(void);
void InitEPwm2Module(void);

// 통신 설정을 위한 함수
void scia_echoback_init(void);
void scia_fifo_init(void);
void scib_echoback_init(void);
void scib_fifo_init(void);
void scic_echoback_init(void);
void scic_fifo_init(void);

// App 전송
void BT_transmit();
void BT_Put_String(char *BT_string);

// Data 전송을 위한 함수
void Uart_transmit();
void UART_Put_String(char *Uart_string);

// Motor를 LCD 없이 동작시키기 위한 통신
void Motor_transmit();
void Motor_Put_String(char *Motor_string);
void Motor_Put_Char(unsigned char Uart_string);

// 통신 변수 선언.
Uint16 TimerCount = 0, MotorCount = 0, M_Comm_flag = 0, Brake_test_count=0,TimerCount_2=0;
char UT1[100] , MT1[50], BT1[50];
unsigned char Motor_char[15], Motor_char_s[10];
Uint16 Mt_cnt = 0, m = 0, c = 0;

interrupt void sciaRxFifoIsr(void);
char RxBuff[16];
char Receivedbuff;


// FSLP의 Calibration에 관한 모든게 정의되어있는 함수.
void FSLP_Value_define();

// Brake duty test를 위해 임시로 만든 함수.
void Brake_duty_test();

// Main 함수 시작

void main(void)



{


// Step 1. Disable Global Interrupt
   DINT;

// Step 2. 시스템 컨트롤 초기화:
   InitSysCtrl();

// FLASH 영역의 함수를 빠른 속도를 위해 RAM으로 구동시키기 위해 선언한 함수
   MemCopy(&RamfuncsLoadStart, & RamfuncsLoadEnd, & RamfuncsRunStart);
   InitFlash();

// Step 3. 인터럽트 초기화:
   InitPieCtrl();
   IER = 0x0000;
   IFR = 0x0000;
   InitPieVectTable();

// GPIO Pin을 밑의 기능을 사용하기 위해 재배치
   InitSciaGpio();
   InitScibGpio();
   InitScicGpio();
   InitAdc();
   InitEPwm1Gpio();

// Vector table을 내가 사용하기 위한 기능으로 배치
   EALLOW;
   PieVectTable.TINT0 = &cpu_timer0_isr;
   PieVectTable.SCIRXINTA = &sciaRxFifoIsr;
   SysCtrlRegs.HISPCP.bit.HSPCLK = 1;

   GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0;
   GpioCtrlRegs.GPAMUX2.bit.GPIO17 =0;
   GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0;
   GpioCtrlRegs.GPBMUX2.bit.GPIO48 = 0;
   GpioCtrlRegs.GPBMUX2.bit.GPIO51 = 0;
   GpioCtrlRegs.GPBMUX2.bit.GPIO52 = 0;
   GpioCtrlRegs.GPBMUX1.bit.GPIO37 = 0;
   GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 0;
   GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0;
   GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 0;
   GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0;
   GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0;
   GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0;
   GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0;
   GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0;

   GpioCtrlRegs.GPCMUX2.bit.GPIO86 = 0;
   GpioCtrlRegs.GPCMUX2.bit.GPIO85 = 0;
   GpioCtrlRegs.GPCMUX2.bit.GPIO84 = 0;
   GpioCtrlRegs.GPCMUX2.bit.GPIO83 = 0;
   GpioCtrlRegs.GPCMUX2.bit.GPIO82 = 0;
   GpioCtrlRegs.GPCMUX2.bit.GPIO81 = 0;
   GpioCtrlRegs.GPCMUX2.bit.GPIO80 = 0;
   GpioCtrlRegs.GPBMUX1.bit.GPIO47 = 0;
   GpioCtrlRegs.GPBMUX1.bit.GPIO46 = 0;
   GpioCtrlRegs.GPBMUX1.bit.GPIO45 = 0;
   GpioCtrlRegs.GPBMUX1.bit.GPIO44 = 0;
   GpioCtrlRegs.GPBMUX1.bit.GPIO43 = 0;
   GpioCtrlRegs.GPBMUX1.bit.GPIO42 = 0;
   GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 0;
   GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 0;
   GpioCtrlRegs.GPBMUX1.bit.GPIO38 = 0;

   GpioCtrlRegs.GPADIR.bit.GPIO15 = 0;
   GpioCtrlRegs.GPADIR.bit.GPIO17 = 0;
   GpioCtrlRegs.GPADIR.bit.GPIO5 = 0;
   GpioCtrlRegs.GPBDIR.bit.GPIO48 = 1;
   GpioCtrlRegs.GPBDIR.bit.GPIO51 = 1;
   GpioCtrlRegs.GPBDIR.bit.GPIO52 = 1;
   GpioCtrlRegs.GPBDIR.bit.GPIO37 = 0;
   GpioCtrlRegs.GPADIR.bit.GPIO25 = 0;
   GpioCtrlRegs.GPADIR.bit.GPIO27 = 0;
   GpioCtrlRegs.GPADIR.bit.GPIO12 = 0;
   GpioCtrlRegs.GPADIR.bit.GPIO14 = 0;
   GpioCtrlRegs.GPBDIR.bit.GPIO32 = 0;
   GpioCtrlRegs.GPADIR.bit.GPIO7 = 0;
   GpioCtrlRegs.GPADIR.bit.GPIO9 = 0;
   GpioCtrlRegs.GPADIR.bit.GPIO11 = 0;

   GpioCtrlRegs.GPCDIR.bit.GPIO86 = 1;
   GpioCtrlRegs.GPCDIR.bit.GPIO85 = 1;
   GpioCtrlRegs.GPCDIR.bit.GPIO84 = 1;
   GpioCtrlRegs.GPCDIR.bit.GPIO83 = 1;
   GpioCtrlRegs.GPCDIR.bit.GPIO82 = 1;
   GpioCtrlRegs.GPCDIR.bit.GPIO81 = 1;
   GpioCtrlRegs.GPCDIR.bit.GPIO80 = 1;
   GpioCtrlRegs.GPBDIR.bit.GPIO47 = 1;
   GpioCtrlRegs.GPBDIR.bit.GPIO46 = 1;
   GpioCtrlRegs.GPBDIR.bit.GPIO45 = 1;
   GpioCtrlRegs.GPBDIR.bit.GPIO44 = 1;
   GpioCtrlRegs.GPBDIR.bit.GPIO43 = 1;
   GpioCtrlRegs.GPBDIR.bit.GPIO42 = 1;
   GpioCtrlRegs.GPBDIR.bit.GPIO41 = 1;
   GpioCtrlRegs.GPBDIR.bit.GPIO40 = 1;
   GpioCtrlRegs.GPBDIR.bit.GPIO38 = 1;

// PWM 1B를 사용하기 위해 GPIO1을 Pull-up 시킴
   GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;
   GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;
// PWM 1B를 사용하기 위해 MUX Pin 배치
   GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;
   GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;

   EDIS;
// PWM 초기화 함수
    InitEPwm1Module();
    InitEPwm2Module();

// CPU Timer 초기화
   InitCpuTimers();
   Cpu_Clk = 150;          // 현재 시스템 클럭을 설정 (MHz 단위)
   Timer_Prd = 5000;      // 타이머 주기 설정 (usec 단위) // 200 Hz -> 5000
   ConfigCpuTimer(&CpuTimer0, Cpu_Clk, Timer_Prd);

// CPU Timer0 시작
   StartCpuTimer0();

// CPU Timer0 인터럽트 활성화
   PieCtrlRegs.PIEIER1.bit.INTx7 = 1;         // PIE 인터럽트(TINT0) 활성화
   PieCtrlRegs.PIEIER9.bit.INTx1 = 1;		  // SCIRXB
   IER =  IER | M_INT1 | M_INT9;              // CPU 인터럽트(INT1), SCIRXB  활성화

// 통신함수 초기화
   scia_fifo_init();      // Initialize the SCI FIFO
   scia_echoback_init();  // Initalize SCI for echoback
   scib_fifo_init();      // Initialize the SCI FIFO
   scib_echoback_init();  // Initalize SCI for echoback
   scic_fifo_init();      // Initialize the SCI FIFO
   scic_echoback_init();  // Initalize SCI for echoback

// ADC 설정
   AdcRegs.ADCTRL3.bit.ADCCLKPS = 3;          // ADCCLK = HSPCLK/(ADCCLKPS*2)/(CPS+1)
   AdcRegs.ADCTRL1.bit.CPS = 1;               // ADCCLK = 75MHz/(3*2)/(1+1) = 6.25MHz
   AdcRegs.ADCTRL1.bit.ACQ_PS = 3;            // 샘플/홀드 사이클 = ACQ_PS + 1 = 4 (ADCCLK기준)
   AdcRegs.ADCTRL1.bit.SEQ_CASC = 1;          // 시퀀스 모드 설정: 직렬 시퀀스 모드 (0:병렬 모드, 1:직렬 모드)
   AdcRegs.ADCMAXCONV.bit.MAX_CONV1 = 8;      // ADC 채널수 설정: 1개(=MAX_CONV+1)채널을 ADC

   AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0;       // ADC 순서 설정: 첫번째로 ADCINA2 채널을 ADC
   AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 1;
   AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 2;
   AdcRegs.ADCCHSELSEQ1.bit.CONV03 = 3;
   AdcRegs.ADCCHSELSEQ2.bit.CONV04 = 4;
   AdcRegs.ADCCHSELSEQ2.bit.CONV05 = 5;
   AdcRegs.ADCCHSELSEQ2.bit.CONV06 = 6;
   AdcRegs.ADCCHSELSEQ2.bit.CONV07 = 7;
   AdcRegs.ADCTRL2.bit.EPWM_SOCB_SEQ = 1;     // ePWM_SOCB로 ADC 시퀀스 시동
   AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;      // ADC 시퀀스 완료시 인터럽트 발생 설정

   //ePWM_SOCB 이벤트 트리거 설정
   EPwm3Regs.ETSEL.bit.SOCBEN = 1;            // SOCB 이벤트 트리거 Enable
   EPwm3Regs.ETSEL.bit.SOCBSEL = 2;           // SCCB 트리거 조건 : 카운터 주기 일치 시
   EPwm3Regs.ETPS.bit.SOCBPRD = 1;            // SOCB 이벤트 분주 설정 : 트리거 조건 한번 마다
   EPwm3Regs.TBCTL.bit.CTRMODE = 0;           // 카운트 모드 설정: Up-conut 모드
   EPwm3Regs.TBCTL.bit.HSPCLKDIV = 1;         // TBCLK = [SYSCLKOUT / ((HSPCLKDIV*2) * 2^(CLKDIV))]
   EPwm3Regs.TBCTL.bit.CLKDIV = 1;            // TBCLK = [150MHz / (2*2)] = 37.5MHz
   EPwm3Regs.TBPRD = 1874;                    // TB주기= (TBPRD+1)/TBCLK = 1875/37.5MHz = 50us(20KHz)
   EPwm3Regs.TBCTR = 0x0000;                  // TB 카운터 초기화


   for( i = 0; i<10 ; i++)
      {
         COP_R_Buff[i]=0; Force_R_Buff[i]=0; EV_Buff[i]=0;
      }

   for( i = 0; i<30 ; i++)
      {
         EV_Buff[i]=0;
      }

   for(i=0;i<16;i++){ RxBuff[i] = 0; }

   EINT;   // Enable Global interrupt INTM
   ERTM;   // Enable Global realtime interrupt DBGM


// IDLE loop. Just sit and loop forever :
//--------------------------------------------------------------------------------------------
   for(;;)
   {
      if(MotorCount == 100)
      {
         MotorCount = 0; Motor_transmit();
      }

   }

}
// 메인함수 끝.







//정준함수-----------------------------------------------------------------------------------------
void Robot_Initalize(void)
{
	if(leg_num==1)//환측다리=오른발이면
	{
		break_duty=0.83;	//브레이크 OFF
		Motor_Pwm=0.55;	//모터 최저속도

		if(Encoder_deg_new>=345&&Encoder_deg_new<=350)
		{
			break_on_bit=1;
			//새로추가
			break_duty=0;
			Motor_Pwm=0;
			Robot_Init_bit=1;
			//
		}
	}
	   else if(leg_num==2)//왼발이면
	   {
		   break_duty=0.83;
		   Motor_Pwm=0.55;
		   if(Encoder_deg_new>=165&&Encoder_deg_new<=170)//
		   {
				break_on_bit=1;
				//새로추가
				break_duty=0;
				Motor_Pwm=0;
				Robot_Init_bit=1;
				//
		   }


/*		   if((break_timer<100)&&(break_on_bit==1))//2초동안 1차함수그래프로 브레이크 듀티, 모터pwm 상승
		   {
			   break_timer++;
			   break_duty=0.0083*(100-break_timer);
			   Motor_Pwm=0.005*(100-break_timer);
		  	}
		   if(break_timer>=100)
		   {
			   break_timer=0;
			   break_duty=0;
			   Motor_Pwm=0;
			   Init_finish=1;
			   move_dis=0;

		   }
*/


	   }
}

void cleaer_variable(void)
{
	   leg_num=0;//다리설정
	   start_bit=0;//시작비트
	   training_timer=0;//타이머0
	   target_sec=0;//목표초
	   move_dis=0;//이동거리
	   target_time=0;//목표시간
	   time_now=0;//보행시간
	   pause_bit=0;//일시정지비트
	   target_gain=0;//목표게인
	   end_bit=0;//종료비트
	   break_timer=0;//
	   break_on_bit=0;
	   ratio_gain=0;
	   mode_num=0;
	   Type_sel=0;
	   target_dis=0;
	   E_vel_deg_new=0;
	   move_distance_1=0;
	   move_distance_2=0;
	   move_distance_3=0;
	   move_distance_4=0;
	   time_now_hour=0;
	   time_now_min_10=0;
	   time_now_min=0;
	   Encoder_revcnt=0;
	   velocity=0;
	   EV_mva=0;
	   RxBuff[0]=0;
	   RxBuff[1]=0;
	   RxBuff[2]=0;
	   RxBuff[3]=0;
	   RxBuff[4]=0;
	   RxBuff[5]=0;
	   RxBuff[6]=0;
	   RxBuff[8]=0;
	   RxBuff[7]=0;
	   RxBuff[9]=0;
	   RxBuff[10]=0;
	   RxBuff[11]=0;



	   for( i = 0; i<30 ; i++)
	   {
	   		EV_Buff[i]=0;
	   	}
}































void InitEPwm1Module(void)
{
    /* Setup TBCLK */
    EPwm1Regs.TBPRD = (150E6/20E3)-1;    /* Set Timer Period */
    EPwm1Regs.TBCTR = 0;                        /* Clear Counter */

    /* Set Compare values */
    EPwm1Regs.CMPA.half.CMPA = ((EPwm1Regs.TBPRD+1)>>1);    /* Set Compare A value to 50% */
    EPwm1Regs.CMPB = ((EPwm1Regs.TBPRD+1)>>1);
    /* Setup counter mode */
    EPwm1Regs.TBCTL.bit.CTRMODE = 0;        /* Count Up (Asymmetric) */
    EPwm1Regs.TBPHS.half.TBPHS = 0;         /* Phase is 0 */
    EPwm1Regs.TBCTL.bit.PHSEN = 0;          /* Disable phase loading */
    EPwm1Regs.TBCTL.bit.PRDLD = 0;          /* Period Register is loaded from its shadow when CNTR=Zero */
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;      /* Clock ratio to SYSCLKOUT */
    EPwm1Regs.TBCTL.bit.CLKDIV = 0;         /* TBCLK = SYSCLK / (HSPCLKDIV * CLKDIV) */

    /* Setup shadowing */
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = 0;     /* Enable Shadowing */
    EPwm1Regs.CMPCTL.bit.LOADAMODE = 0;     /* Load on CNTR=Zero */

    /* Set actions */
    EPwm1Regs.AQCTLA.bit.ZRO = 2;           /* Set EPWM1A on CNTR=Zero */
    EPwm1Regs.AQCTLA.bit.CAU = 1;           /* Clear EPWM1A on event A, up count */
    EPwm1Regs.AQCTLB.bit.ZRO = 2;
    EPwm1Regs.AQCTLB.bit.CBU = 1;


    /* Set Interrupts */
    EPwm1Regs.ETSEL.bit.INTSEL = 1;         /* Select INT on CNTR=Zero */
    EPwm1Regs.ETSEL.bit.INTEN = 1;          /* Enable INT */
    EPwm1Regs.ETPS.bit.INTPRD = 1;          /* Generate INT on 1st event */
}


void InitEPwm2Module(void)
{
    /* Setup Counter Mode and Clock */
    EPwm2Regs.TBCTL.bit.CTRMODE = 0;        /* Count Up (Asymmetric) */
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0;      /* TBCLK = SYSCLKOUT / (HSPCLKDIV * CLKDIV) = 150MHz */
    EPwm2Regs.TBCTL.bit.CLKDIV = 0;

    /* Setup Phase */
    EPwm2Regs.TBPHS.half.TBPHS = 0;         /* Phase is 0 */
    EPwm2Regs.TBCTL.bit.PHSEN = 0;          /* Disable phase loading */

    /* Setup Period (Carrier Frequency) */
    EPwm2Regs.TBPRD = (150E6/20E3)-1; /* Set Timer Period, (150MHz/20KHz)-1 = 7,499 (0x1D4B) */
    EPwm2Regs.TBCTR = 0;                        /* Clear Counter */

    /* Set Compare Value */
    //EPwm2Regs.CMPA.half.CMPA = (Uint16)((EPwm2Regs.TBPRD + 1) * PWM_DUTY_RATIO_C);    /* Set Compare A Value to 20% */
    EPwm2Regs.CMPB = (Uint16)((EPwm2Regs.TBPRD + 1) * 0.3);         /* Set Compare B Value to 20% */

    /* Setup shadowing */
    EPwm2Regs.TBCTL.bit.PRDLD = 0;          /* Period Register is loaded from its shadow when CNTR=Zero */
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = 0;     /* Compare A Register is loaded from its shadow when CNTR=Zero */
    EPwm2Regs.CMPCTL.bit.LOADAMODE = 0;
    //EPwm2Regs.CMPCTL.bit.SHDWBMODE = 0;       /* Compare B Register is loaded from its shadow when CNTR=Zero */
    //EPwm2Regs.CMPCTL.bit.LOADBMODE = 0;

    /* Set actions */
    EPwm2Regs.AQCTLB.bit.ZRO = 2;       /* Set EPWM4A on CNTR=Zero */
    EPwm2Regs.AQCTLB.bit.CBU = 1;       /* Clear EPWM4A on CNTR=CMPA, Up-Count */
    //EPwm1Regs.AQCTLB.bit.ZRO = 2;     /* Set EPWM4B on CNTR=Zero */
    //EPwm1Regs.AQCTLB.bit.CBU = 1;     /* Clear EPWM4B on CNTR=CMPB, Up-Count */

    /* Set Interrupts */
    EPwm2Regs.ETSEL.bit.INTSEL = 1;     /* Select INT on CNTR=Zero */
    EPwm2Regs.ETPS.bit.INTPRD = 1;      /* Generate INT on 1st event */
//  EPwm2Regs.ETSEL.bit.INTEN = 1;      /* Enable INT */
}

void scia_echoback_init()
{
   SciaRegs.SCICTL1.bit.SWRESET = 0;
   SciaRegs.SCICCR.bit.SCICHAR = 7;   // 1 stop bit, No loopback, No parity, 8 char bits,
   SciaRegs.SCICTL1.bit.RXENA = 1;    // SCI 송신기능 Enable
   SciaRegs.SCICTL1.bit.TXENA = 1;    // async mode, idle-line protocol
   SciaRegs.SCICTL2.all =0x0003;
   SciaRegs.SCICTL2.bit.TXINTENA =1;
   SciaRegs.SCICTL2.bit.RXBKINTENA =1;

   #if (CPU_FRQ_150MHZ)
         SciaRegs.SCIHBAUD    =0x0000;  // 115200 baud @LSPCLK = 37.5MHz.
         SciaRegs.SCILBAUD    =0x0028;
   #endif
   #if (CPU_FRQ_100MHZ)
         SciaRegs.SCIHBAUD    =0x0001;  // 9600 baud @LSPCLK = 20MHz.
         SciaRegs.SCILBAUD    =0x0044;
   #endif
   SciaRegs.SCICTL1.bit.SWRESET = 1;
   //SciaRegs.SCIHBAUD    =0x0000;  // 38400 baud @LSPCLK = 37.5MHz.
   //SciaRegs.SCILBAUD    =0x0079;
   //SciaRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
}

void scia_fifo_init()
{
    SciaRegs.SCIFFTX.all=0xE040;
    SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;
    SciaRegs.SCIFFCT.all=0x0;
    SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;
    SciaRegs.SCIFFRX.bit.RXFFIENA = 1;
    SciaRegs.SCIFFRX.bit.RXFFIL = 1;
}

void scib_echoback_init()
{
   ScibRegs.SCICTL1.bit.SWRESET = 0;
   ScibRegs.SCICCR.bit.SCICHAR = 7;   // 1 stop bit, No loopback, No parity, 8 char bits,
   ScibRegs.SCICTL1.bit.RXENA = 1;    // SCI 송신기능 Enable
   ScibRegs.SCICTL1.bit.TXENA = 1;    // async mode, idle-line protocol
   ScibRegs.SCICTL2.all =0x0003;
   ScibRegs.SCICTL2.bit.TXINTENA =1;
   ScibRegs.SCICTL2.bit.RXBKINTENA =1;

   #if (CPU_FRQ_150MHZ)
         ScibRegs.SCIHBAUD    =0x0000;  // 115200 baud @LSPCLK = 37.5MHz.
         ScibRegs.SCILBAUD    =0x0028;
   #endif
   #if (CPU_FRQ_100MHZ)
         ScibRegs.SCIHBAUD    =0x0001;  // 9600 baud @LSPCLK = 20MHz.
         ScibRegs.SCILBAUD    =0x0044;
   #endif
   ScibRegs.SCICTL1.bit.SWRESET = 1;
   //ScibRegs.SCIHBAUD    =0x0000;  // 38400 baud @LSPCLK = 37.5MHz.
   //ScibRegs.SCILBAUD    =0x0079;
   //ScibRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
}

void scib_fifo_init()
{
    ScibRegs.SCIFFTX.all=0xE040;
    ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;
    ScibRegs.SCIFFCT.all=0x0;
    ScibRegs.SCIFFRX.bit.RXFFINTCLR = 1;
    ScibRegs.SCIFFRX.bit.RXFFIENA = 1;
    ScibRegs.SCIFFRX.bit.RXFFIL = 1;
}


void scic_echoback_init()
{
   ScicRegs.SCICTL1.bit.SWRESET = 0;
   ScicRegs.SCICCR.bit.SCICHAR = 7;

   ScicRegs.SCICTL1.bit.RXENA = 1;
   ScicRegs.SCICTL1.bit.TXENA = 1;

   ScicRegs.SCICTL2.all =0x0003;
   ScicRegs.SCICTL2.bit.TXINTENA =1;
   ScicRegs.SCICTL2.bit.RXBKINTENA =1;


   #if (CPU_FRQ_150MHZ)
         ScicRegs.SCIHBAUD    =0x0001;  // 9600 baud @LSPCLK = 37.5MHz.
         ScicRegs.SCILBAUD    =0x00E7;
   #endif
   #if (CPU_FRQ_100MHZ)
         ScicRegs.SCIHBAUD    =0x0001;  // 9600 baud @LSPCLK = 20MHz.
         ScicRegs.SCILBAUD    =0x0044;
   #endif
   ScicRegs.SCICTL1.bit.SWRESET = 1;
   //ScibRegs.SCIHBAUD    =0x0000;  // 38400 baud @LSPCLK = 37.5MHz.
   //ScibRegs.SCILBAUD    =0x0079;
   //ScibRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
}

void scic_fifo_init()
{
   ScicRegs.SCIFFTX.all=0xE040;
   ScicRegs.SCIFFRX.bit.RXFIFORESET = 1;
   ScicRegs.SCIFFCT.all=0x0;
   ScicRegs.SCIFFRX.bit.RXFFINTCLR = 1;
   ScicRegs.SCIFFRX.bit.RXFFIENA = 1;
   ScicRegs.SCIFFRX.bit.RXFFIL = 16;
}


void BT_Put_String(char *BT_string)
{
   while(*BT_string != 0)
   {
      SciaRegs.SCITXBUF = *BT_string++;
      while (SciaRegs.SCIFFTX.bit.TXFFST != 0) {}
   }
}


void UART_Put_String(char *Uart_string)
{
   while(*Uart_string != 0)
   {
      ScibRegs.SCITXBUF = *Uart_string++;
      while (ScibRegs.SCIFFTX.bit.TXFFST != 0) {}
   }
}

void Motor_Put_Char(unsigned char Uart_string)
{

      ScicRegs.SCITXBUF = Uart_string;
      while (ScicRegs.SCIFFTX.bit.TXFFST != 0) {}

}

void Motor_Put_String(char *Motor_string)
{
   while(*Motor_string != 0)
   {
      ScicRegs.SCITXBUF = *Motor_string++;
      while (ScicRegs.SCIFFTX.bit.TXFFST != 0) {}
   }

}


void BT_transmit()
{

	sprintf(BT1,"!s%d.%dt%d%d%dd%d.%d%d?\n\0",(int)velocity,(int)under_velocity,time_now_hour,time_now_min_10,time_now_min_1,  move_distance_1, move_distance_2,move_distance_3);

	if(Type_sel==2)//시간설정
	{
		if(time_now>target_sec)
		{
			sprintf(BT1,"!e?");
		}
	}
	else if(Type_sel==1)//거리설정
	{
		if(move_dis>target_dis)
		{
			sprintf(BT1,"!e?");
		}
	}

	BT_Put_String(BT1);

}

void Uart_transmit()
{
	//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@테스트시 넣는 코드@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@//
	//sprintf(UT1,"%d,%d,%d,%d\n\0",(int)(COP_R_mva*10),(int)(Torque_R*10),(int)(COP_L_mva*10),(int)(Torque_L*10));
    // MATLAB Graph
     //sprintf(UT1,"%d,%d,%d,%d\n\0",(int)(COP_R_mva*10),(int)(Torque_R*10),(int)(COP_L_mva*10),(int)(Torque_L*10));
	sprintf(UT1,"%d,%ld,%ld\n\0",(int)(Motor_Pwm*10000),(long)(Encoder_deg_new*100),(long)(EV_mva*10000));
	//sprintf(UT1,"%ld,%d,%d,%d,%d\n\0",(long)(EV_mva*10000),(int)(Total_torque*100),(int)(E_vel_deg_new*10),(int)(Torque_L*100),(int)(Torque_R*100));
    // UART_Put_String(UT1);
	 //sprintf(UT1,"!s%d.%dt%d%d%dd%d.%d%d?\n\0",(int)velocity,(int)under_velocity,time_now_hour,time_now_min_10,time_now_min_1,  move_distance_1, move_distance_2,move_distance_3);


  //  sprintf(UT1,"!s%d.%dt%d%d%dd%d.%d%d?\n\0",(int)velocity,(int)under_velocity,time_now_hour,time_now_min_10,time_now_min_1,  move_distance_1, move_distance_2,move_distance_3);
  //  UART_Put_String(UT1);
    // sprintf(UT1,"deg:%3d distance:%d m cnt:%d\n\0",(int)(Encoder_deg_new),(int)(move_dis*0.001),(int)Encoder_revcnt);
    //  UART_Put_String(UT1);
//	 sprintf(UT1,"%d,%d,%d\n\0",(int)(E_vel_deg_new*10),(int)(Encoder_deg_new*10),(int)(Encoder_vel*10));
//	 UART_Put_String(UT1);
    // Bluetooth Comm

   /*sprintf(UT1,"!A%4dC%3dF%3d?",(int)(Encoder_deg_new*10),(int)(COP_R_mva),(int)(Force_R_mva*9.8));
     UART_Put_String(UT1);
     for(c=0; c < 50 ; c++)
     {
         UT1[c]=0;
     }*/


    // Realtime Matlab Dislpaly

   //   sprintf(UT1,"%3.0f%3.0f%3.0f%3.0f!",COP_ab,COP_cd,Force_L,Force_R);





	//sprintf(UT1,"%ld,%d,%d,%d,%d\n\0",(long)(EV_mva*10000),(int)(Total_torque*100),(int)(E_vel_deg_new*10),(int)(Torque_L*100),(int)(Torque_R*100));

	//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@//




//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@재활치료시 넣는 코드@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@//
 /*	sprintf(UT1,"!s%d.%dt%d%d%dd%d.%d%d?\n\0",(int)velocity,(int)under_velocity,time_now_hour,time_now_min_10,time_now_min_1,  move_distance_1, move_distance_2,move_distance_3);

 	if(Type_sel==2)//시간설정
 	{
 		if(time_now>target_sec)
 		{
 			sprintf(UT1,"!e?");
 		}
 	}
 	else if(Type_sel==1)//거리설정
 	{
 		if(move_dis>target_dis)
 		{
 			sprintf(UT1,"!e?");
 		}
 	}
*/
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@//
 	UART_Put_String(UT1);




}

// LCD 없이 모터제어를 위한 통신프로토콜
void Motor_transmit()
{
   Motor_char_s[0] = 0x3A;
   Motor_char_s[1] = 0x1A;
   Motor_char_s[2] = 0x52;
   Motor_char_s[3] = 0x02;
   Motor_char_s[4] = 0x33;
   Motor_char_s[6] = 0xA1;
   Motor_char_s[8] = 0x0D;
   Motor_char_s[9] = 0x0A;

      if ( Mt_cnt == 0)
      {
          for( m=0; m < 10; m++)
          {
              if ( m == 5 || m == 7)
                 { Motor_Put_Char(0x00); }
              else
              {
                  sprintf(MT1,"%c",Motor_char_s[m]);  Motor_Put_String(MT1);
              }
          }
          Mt_cnt ++;
      }
}


void FSLP_Value_define() // FLSP에 관한 모든 값들을 다 정의함. 현재 Calibration이 된채로 사용하면 됨.
{
   if(FSLP_flag==0)
   {
     GpioDataRegs.GPBDAT.all= 0x00008000; GpioDataRegs.GPCDAT.all= 0x7F0000;
     FSLP_flag=1;
     AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;
   }

   else if(FSLP_flag==1)
   {
     Force_R_LU=AdcRegs.ADCRESULT7>>4;
     Force_R_RU=AdcRegs.ADCRESULT6>>4;
     Force_R_LL=AdcRegs.ADCRESULT2>>4;
     Force_R_RL=AdcRegs.ADCRESULT3>>4;

     Force_L_LU=AdcRegs.ADCRESULT5>>4;
     Force_L_RU=AdcRegs.ADCRESULT4>>4;
     Force_L_LL=AdcRegs.ADCRESULT1>>4;
     Force_L_RL=AdcRegs.ADCRESULT0>>4;

     if(Force_R_LU > 3800)
     { Force_R_LU = 3800; }
     else if(Force_R_LU <= 100)
     { Force_R_LU =0; }
     if(Force_R_RU > 3800)
     { Force_R_RU = 3800; }
     else if( Force_R_RU <= 100 )
     { Force_R_RU =0; }
     if(Force_R_LL > 3800)
     { Force_R_LL = 3800; }
     else if (Force_R_LL <=100)
     { Force_R_LL =0; }
     if(Force_R_RL > 3800)
     { Force_R_RL = 3800; }
     else if( Force_R_RL <= 100)
     { Force_R_RL=0; }

     if(Force_L_LU > 3800)
     { Force_L_LU=3800; }
     else if(Force_L_LU <= 100)
     { Force_L_LU=0; }
     if(Force_L_RU > 3800)
     { Force_L_RU=3800; }
     else if( Force_L_RU <= 100)
     { Force_L_RU=0; }
     if(Force_L_LL > 3800)
     { Force_L_LL=3800; }
     else if( Force_L_LL <= 100)
     { Force_L_LL=0; }
     if(Force_L_RL > 3800)
     { Force_L_RL=3800; }
     else if( Force_L_RL <= 100)
     { Force_L_RL=0; }

      Force_R = (Force_R_RU+Force_R_LU+Force_R_RL+Force_R_LL);
      Force_L = (Force_L_RU+Force_L_LU+Force_L_RL+Force_L_LL);

      if (4.152*exp(0.0001956*Force_R) < 4.5 )
      {
         Force_R_C = 0;
      }
      else
      {
         //Force_R_C = 0.004926*Force_R+2.956;
    	  Force_R_C = 0.007389*Force_R+2.956;
      }

      if (4.152*exp(0.0001956*Force_L) < 4.5 )
      {
          Force_L_C = 0;
      }
      else
      {
          //Force_L_C = 0.004926*Force_L+2.956;
    	  Force_L_C = 0.007389*Force_L+2.956;
      }

     FSLP_flag=2;
    }

   else if(FSLP_flag==2)
   {
     GpioDataRegs.GPBDAT.all= 0x00007F40; GpioDataRegs.GPCDAT.all= 0x000000;
     FSLP_flag=3;
     AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;
   }

     else if(FSLP_flag==3)
   {
      position_R_LU = AdcRegs.ADCRESULT7>>4;
      position_R_RU = AdcRegs.ADCRESULT6>>4;
      position_R_LL = AdcRegs.ADCRESULT2>>4;
      position_R_RL = AdcRegs.ADCRESULT3>>4;

      position_L_LU = AdcRegs.ADCRESULT5>>4;
      position_L_RU = AdcRegs.ADCRESULT4>>4;
      position_L_LL = AdcRegs.ADCRESULT1>>4;
      position_L_RL = AdcRegs.ADCRESULT0>>4;

      // 106 / (4095 - 700)
     if( position_R_LU == 4095 )
       { position_R_LU_C=0; }
     else
       { position_R_LU_C = (-0.03093 * (position_R_LU-700) +105) + 120; }

      if( position_R_RU == 4095 )
       { position_R_RU_C=0; }
     else
       { position_R_RU_C = (-0.03093 * (position_R_RU-700) +105) + 120; }

     // Lower 700 ~ 4000

     if( position_R_LL == 4095 )
      { position_R_LL_C=0; }
     else
      { position_R_LL_C = (0.03093 * (position_R_LL-700)) - 35; }

     if( position_R_RL == 4095 )
      { position_R_RL_C=0; }
     else
      { position_R_RL_C = (0.03093 * (position_R_RL-700)) - 35; }

     // 106 / (4095 - 700)
     if( position_L_LU == 4095 )
       { position_L_LU_C=0; }
     else
       { position_L_LU_C = (-0.03093 * (position_L_LU-700) +105) + 120; }

     if( position_L_RU == 4095 )
       { position_L_RU_C=0; }
      else
       { position_L_RU_C = (-0.03093 * (position_L_RU-700) +105) + 120; }

     // Lower 700 ~ 4000

     if( position_L_LL == 4095 )
       { position_L_LL_C=0; }
     else
       { position_L_LL_C = (0.03093 * (position_L_LL-700)) - 35; }

     if( position_L_RL == 4095 )
       { position_L_RL_C=0; }
     else
       { position_L_RL_C = (0.03093 * (position_L_RL-700)) - 35; }


     COP_R = (Force_R_RU*position_R_RU_C + Force_R_LU*position_R_LU_C + Force_R_LL*position_R_LL_C + Force_R_RL*position_R_RL_C)/ Force_R ;
     COP_L = (Force_L_RU*position_L_RU_C + Force_L_LU*position_L_LU_C + Force_L_LL*position_L_LL_C + Force_L_RL*position_L_RL_C)/ Force_L ;

     if( M_i == 0 )
     { COP_R_Buff[M_i] = COP_R ; Force_R_Buff[M_i] = Force_R_C;
       COP_L_Buff[M_i] = COP_L ; Force_L_Buff[M_i] = Force_L_C; M_i++; }
     else if( M_i == 1 )
     { COP_R_Buff[M_i] = COP_R ; Force_R_Buff[M_i] = Force_R_C;
       COP_L_Buff[M_i] = COP_L ; Force_L_Buff[M_i] = Force_L_C; M_i++; }
     else if( M_i == 2 )
     { COP_R_Buff[M_i] = COP_R ; Force_R_Buff[M_i] = Force_R_C;
       COP_L_Buff[M_i] = COP_L ; Force_L_Buff[M_i] = Force_L_C; M_i++; }
     else if( M_i == 3 )
     { COP_R_Buff[M_i] = COP_R ; Force_R_Buff[M_i] = Force_R_C;
       COP_L_Buff[M_i] = COP_L ; Force_L_Buff[M_i] = Force_L_C; M_i++; }
     else if( M_i == 4 )
     { COP_R_Buff[M_i] = COP_R ; Force_R_Buff[M_i] = Force_R_C;
       COP_L_Buff[M_i] = COP_L ; Force_L_Buff[M_i] = Force_L_C; M_i++; }
     else if( M_i == 5 )
     { COP_R_Buff[M_i] = COP_R ; Force_R_Buff[M_i] = Force_R_C;
       COP_L_Buff[M_i] = COP_L ; Force_L_Buff[M_i] = Force_L_C; M_i++; }
     else if( M_i == 6 )
     { COP_R_Buff[M_i] = COP_R ; Force_R_Buff[M_i] = Force_R_C;
       COP_L_Buff[M_i] = COP_L ; Force_L_Buff[M_i] = Force_L_C; M_i++; }
     else if( M_i == 7 )
     { COP_R_Buff[M_i] = COP_R ; Force_R_Buff[M_i] = Force_R_C;
       COP_L_Buff[M_i] = COP_L ; Force_L_Buff[M_i] = Force_L_C; M_i++; }
     else if( M_i == 8 )
     { COP_R_Buff[M_i] = COP_R ; Force_R_Buff[M_i] = Force_R_C;
       COP_L_Buff[M_i] = COP_L ; Force_L_Buff[M_i] = Force_L_C; M_i++; }
     else if( M_i == 9 )
     { COP_R_Buff[M_i] = COP_R ; Force_R_Buff[M_i] = Force_R_C;
       COP_L_Buff[M_i] = COP_L ; Force_L_Buff[M_i] = Force_L_C; M_i=0; }

     COP_R_mva= 0.1*(COP_R_Buff[0]+COP_R_Buff[1]+COP_R_Buff[2]+COP_R_Buff[3]+COP_R_Buff[4]+COP_R_Buff[5]+COP_R_Buff[6]+COP_R_Buff[7]+COP_R_Buff[8]+COP_R_Buff[9]);
     Force_R_mva = 0.1*(Force_R_Buff[0]+Force_R_Buff[1]+Force_R_Buff[2]+Force_R_Buff[3]+Force_R_Buff[4]+Force_R_Buff[5]+Force_R_Buff[6]+Force_R_Buff[7]+Force_R_Buff[8]+Force_R_Buff[9]);

     COP_L_mva= 0.1*(COP_L_Buff[0]+COP_L_Buff[1]+COP_L_Buff[2]+COP_L_Buff[3]+COP_L_Buff[4]+COP_L_Buff[5]+COP_L_Buff[6]+COP_L_Buff[7]+COP_L_Buff[8]+COP_L_Buff[9]);
     Force_L_mva = 0.1*(Force_L_Buff[0]+Force_L_Buff[1]+Force_L_Buff[2]+Force_L_Buff[3]+Force_L_Buff[4]+Force_L_Buff[5]+Force_L_Buff[6]+Force_L_Buff[7]+Force_L_Buff[8]+Force_L_Buff[9]);
     // Torque 값은 Force와 COP로 정의 함.
     Torque_R = Force_R_mva * COP_R_mva * 9.81 *0.001 ; // N*m = kg * mm * 9.81 * 0.001
     Torque_L = Force_L_mva * COP_L_mva * 9.81 *0.001 ; // N*m = kg * mm * 9.81 * 0.001
     Total_torque=(Torque_R+Torque_L)*0.5;
     FSLP_flag=0;
   }

}

void Encoder_define()
{

   Encoder_deg_old = Encoder_deg_new; // 이전 Encoder값을 저장
   E_vel_deg_old = E_vel_deg_new;

   // Encoder Digital Input 값 받기
   Encoder[0] = GpioDataRegs.GPADAT.bit.GPIO5;
   Encoder[1] = GpioDataRegs.GPBDAT.bit.GPIO37;
   Encoder[2] = GpioDataRegs.GPADAT.bit.GPIO25;
   Encoder[3] = GpioDataRegs.GPADAT.bit.GPIO27;
   Encoder[4] = GpioDataRegs.GPADAT.bit.GPIO12;
   Encoder[5] = GpioDataRegs.GPADAT.bit.GPIO14;
   Encoder[6] = GpioDataRegs.GPBDAT.bit.GPIO32;
   Encoder[7] = GpioDataRegs.GPADAT.bit.GPIO7;
   Encoder[8] = GpioDataRegs.GPADAT.bit.GPIO9;
   Encoder[9] = GpioDataRegs.GPADAT.bit.GPIO11;

   Encoder_sum =0;
   for ( Encoder_cnt = 0; Encoder_cnt <10; Encoder_cnt ++)
   {
      Encoder_sum += Encoder[Encoder_cnt] << Encoder_cnt; // Encoder_sum 은 0-1024 Pulse까지의 수를 Count해줌.
   }


   Encoder_deg_new = 360 - (double)Encoder_sum * 0.3515625 ; // Encoder값 갱신. 1024 Pulse를 0 - 360 deg로 바꿔줌.
   //Encoder_deg_new =360-Encoder_deg_new;

   if(Init_finish==1)
   {
	   if( Encoder_deg_old - Encoder_deg_new  >= 200 ) // 각속도 구할 때 갑자기 100도이상 차이나면 360 -> 0 도로 된것을 알아내는 조건
	   {
	      Encoder_revcnt ++; // 회전수 체크
	   }

	   E_vel_deg_new = Encoder_revcnt * 360 + Encoder_deg_new;
	   move_dis=0.000001*E_vel_deg_new*880/360;//총 회전각도*각거리?
	   Encoder_vel = (E_vel_deg_new-E_vel_deg_old)*100; // Angular velocity dt=0.005s

	    if( E_i == 0 )
		{ EV_Buff[E_i] = Encoder_vel ; E_i++; }
		else if( E_i == 1 )
		{ EV_Buff[E_i] = Encoder_vel ; E_i++; }
		else if( E_i == 2 )
		{ EV_Buff[E_i] = Encoder_vel ; E_i++; }
		else if( E_i == 3 )
		{ EV_Buff[E_i] = Encoder_vel ; E_i++; }
		else if( E_i == 4 )
		{ EV_Buff[E_i] = Encoder_vel ; E_i++; }
		else if( E_i == 5 )
		{ EV_Buff[E_i] = Encoder_vel ; E_i++; }
		else if( E_i == 6 )
		{ EV_Buff[E_i] = Encoder_vel ; E_i++; }
		else if( E_i == 7 )
		{ EV_Buff[E_i] = Encoder_vel ; E_i++; }
		else if( E_i == 8 )
		{ EV_Buff[E_i] = Encoder_vel ; E_i++; }
		else if( E_i == 9 )
		{ EV_Buff[E_i] = Encoder_vel ; E_i++; }
		else if( E_i == 10 )
		{ EV_Buff[E_i] = Encoder_vel ; E_i++; }
		else if( E_i == 11 )
		{ EV_Buff[E_i] = Encoder_vel ; E_i++; }
		else if( E_i == 12 )
		{ EV_Buff[E_i] = Encoder_vel ; E_i++; }
		else if( E_i == 13 )
		{ EV_Buff[E_i] = Encoder_vel ; E_i++; }
		else if( E_i == 14 )
		{ EV_Buff[E_i] = Encoder_vel ; E_i++; }
		else if( E_i == 15 )
		{ EV_Buff[E_i] = Encoder_vel ; E_i++; }
		else if( E_i == 16 )
		{ EV_Buff[E_i] = Encoder_vel ; E_i++; }
		else if( E_i == 17 )
		{ EV_Buff[E_i] = Encoder_vel ; E_i++; }
		else if( E_i == 18 )
		{ EV_Buff[E_i] = Encoder_vel ; E_i++; }
		else if( E_i == 19 )
		{ EV_Buff[E_i] = Encoder_vel ; E_i++; }
		else if( E_i == 20 )
		{ EV_Buff[E_i] = Encoder_vel ; E_i++; }
		else if( E_i == 21 )
		{ EV_Buff[E_i] = Encoder_vel ; E_i++; }
		else if( E_i == 22 )
		{ EV_Buff[E_i] = Encoder_vel ; E_i++; }
		else if( E_i == 23 )
		{ EV_Buff[E_i] = Encoder_vel ; E_i++; }
		else if( E_i == 24 )
		{ EV_Buff[E_i] = Encoder_vel ; E_i++; }
		else if( E_i == 25 )
		{ EV_Buff[E_i] = Encoder_vel ; E_i++; }
		else if( E_i == 26 )
		{ EV_Buff[E_i] = Encoder_vel ; E_i++; }
		else if( E_i == 27 )
		{ EV_Buff[E_i] = Encoder_vel ; E_i++; }
		else if( E_i == 28 )
		{ EV_Buff[E_i] = Encoder_vel ; E_i++; }
		else if( E_i == 29 )
		{ EV_Buff[E_i] = Encoder_vel ; E_i=0; }

	    EV_mva= 0.03333333*(EV_Buff[0]+EV_Buff[1]+EV_Buff[2]+EV_Buff[3]+EV_Buff[4]+EV_Buff[5]+EV_Buff[6]+EV_Buff[7]+EV_Buff[8]+EV_Buff[9]+EV_Buff[10]+EV_Buff[11]+EV_Buff[12]+EV_Buff[13]+EV_Buff[14]+EV_Buff[15]+EV_Buff[16]+EV_Buff[17]+EV_Buff[18]+EV_Buff[19]+EV_Buff[20]+EV_Buff[21]+EV_Buff[22]+EV_Buff[23]+EV_Buff[24]+EV_Buff[25]+EV_Buff[26]+EV_Buff[27]+EV_Buff[28]+EV_Buff[29]);



   }
   // vel_LFP = tau/(tau + ts) * pre_vel_LFP + tau/(tau + ts) * (x - pre_x);

 /*  if(init_finish==0)
   {
	   if(Encoder_deg_new>=350&&Encoder_deg_new<=360)
	   {
		   break_on_bit=1;
	   }
	   else if(Encoder_deg_new>=170&&Encoder_deg_new<=180)
	   {
		   break_on_bit=2;
	   }
	   else
	   {
		   break_on_bit=0;
	   }
	}
*/
}


interrupt void sciaRxFifoIsr(void)
{

	Receivedbuff=SciaRegs.SCIRXBUF.bit.RXDT;

	if(a==0)
	{
		if(Receivedbuff=='!')
		{
			 RxBuff[a]=Receivedbuff;
			 a++;
		}
		else
		{
			RxBuff[6]=0;
			a=0;
		}
	}
	else if(a==1)
	{
			RxBuff[a]=Receivedbuff;
			a++;
	}
	else if(a==2)
	{
		RxBuff[a]=Receivedbuff;
		a++;
	}
	else if(a==3)
	{
		RxBuff[a]=Receivedbuff;
		a++;
	}
	else if(a==4)
	{
		RxBuff[a]=Receivedbuff;
		a++;
	}
	else if(a==5)
	{
		RxBuff[a]=Receivedbuff;
		a++;
	}

	if(Receivedbuff=='?')
	{
		RxBuff[a]=Receivedbuff;
		a=0;

	}

	if(RxBuff[0]=='!' && RxBuff[1]=='S' && RxBuff[2]=='?' )
	{
		if(Init_finish==1)
		{
			if(leg_num!=0&&mode_num!=0)
			{
						if(Type_sel==1||Type_sel==2)
						{
							start_bit=1; RxBuff[6] =0;
						}
						else
						{
							start_bit=0; RxBuff[6] =0;
						}
			}
		}

		else
		{
			start_bit=0; RxBuff[6] =0;
		}

	}

	else if(RxBuff[0]=='!' && RxBuff[1]=='E' && RxBuff[2]=='?')
	{
		end_bit=1; RxBuff[6] =0;
		sprintf(UT1,"!e?");
		UART_Put_String(UT1);
	}
	else if(RxBuff[0]=='!' && RxBuff[1]=='P' && RxBuff[2]=='1'&& RxBuff[3]=='?')
	{
		pause_bit=1; RxBuff[6] =0;
	}
	else if(RxBuff[0]=='!' && RxBuff[1]=='P' && RxBuff[2]=='2'&& RxBuff[3]=='?')
	{
		pause_bit=0; RxBuff[6] =0;
	}
	else if(RxBuff[0]=='!' && RxBuff[1]=='M' && RxBuff[2]=='1'&& RxBuff[3]=='?')
	{
		mode_num=1; RxBuff[6] =0;
	}
	else if(RxBuff[0]=='!' && RxBuff[1]=='M' && RxBuff[2]=='2'&& RxBuff[3]=='?')
	{
		target_gain=0;
		mode_num=2;
		RxBuff[6] =0;
	}
	else if(RxBuff[0]=='!' && RxBuff[1]=='M' && RxBuff[2]=='3'&& RxBuff[3]=='?')
	{
		mode_num=3; RxBuff[6] =0;
	}

	else if(RxBuff[0]=='!' && RxBuff[1]=='H' && RxBuff[2]=='R'&& RxBuff[3]=='?')
	{
		leg_num=1; RxBuff[6] =0;
	}
	else if(RxBuff[0]=='!' && RxBuff[1]=='H' && RxBuff[2]=='L'&& RxBuff[3]=='?')
	{
		leg_num=2; RxBuff[6] =0;
	}


	else if(RxBuff[0]=='!' && RxBuff[1]=='B' && RxBuff[2]=='R' && RxBuff[3]=='?')
	{
		em_break=1;

		RxBuff[6] =0;
	}

	else if(RxBuff[0]=='!' && RxBuff[1]=='D' && RxBuff[5]=='?')
	{
		target_dis=atof(&RxBuff[2]);
		target_dis=target_dis*0.01;

		Type_sel=1;
		RxBuff[6] =0;
	}
	else if(RxBuff[0]=='!' && RxBuff[1]=='T' && RxBuff[5]=='?')
	{
		target_time=atof(&RxBuff[2]);
		target_time=target_time/100;

		target_hour=(int)target_time;
		target_min=(int)(target_time*100+0.5)-target_hour*100;
		target_sec=target_hour*3600+target_min*60;

		Type_sel=2;
		RxBuff[6] =0;
	}
	else if(RxBuff[0]=='!' && RxBuff[1]=='G' && RxBuff[4]=='?')
	{
		if(mode_num!=2)
		{
			target_gain=atof(&RxBuff[2]);

			RxBuff[6] =0;
		}
		else
		{
			RxBuff[6] =0;
		}
	}
	else if(RxBuff[0]=='!' && RxBuff[1]=='R' && RxBuff[4]=='?')
	{
		if(mode_num==3)
		{
			ratio_gain=atof(&RxBuff[2]);
			ratio_gain=ratio_gain*0.1;
			RxBuff[6] =0;
		}
		else
		{
			RxBuff[6] =0;
		}
	}
	else if(RxBuff[0]=='!' && RxBuff[1]=='K' && RxBuff[4]=='?')
		{
				ratio_gain=atof(&RxBuff[2]);
				ratio_gain=ratio_gain*0.1;
				RxBuff[6] =0;
		}
	else {RxBuff[6]=0;}


	SciaRegs.SCIFFRX.bit.RXFFOVRCLR = 1;			// Clear Overflow flag
	SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;			// Clear Interrupt flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;			// Acknowledge interrupt to PIE


}











interrupt void cpu_timer0_isr(void) // cpu timer 현재 제어주파수 100Hz
{

   FSLP_Value_define(); // FSLP로 부터 얻을수 있는 모든 값들을 다루는 함수
  /*테스트용 설정*

   Init_finish=1;
   start_bit=1;
   mode_num=3;
   테스트용 설정*/

   TimerCount++; //40 -> 2
   TimerCount_2++;

   if(Mt_cnt==0)
   {
		   MotorCount++;
   }

   if(TimerCount_2 == 2) //속도, 토크값 컴에서확인
   {
	   TimerCount_2=0;
	   Encoder_define();
	   if(start_bit==1&&end_bit==0)
	   {
		   Uart_transmit();
	   }
   }


   if(TimerCount == 20) // MATLAB 2 -> 100Hz Bluetooth 40 -> 5Hz
   {
	   TimerCount=0;  // PC로 데이터 전송하기 위한 함수
	   if(start_bit==1&&end_bit==0)
	   {
		 //  Uart_transmit();

		   BT_transmit();
	   }
   }



   if(Init_finish==0)//초기화안되어있으면
   {
	   if(leg_num==1||leg_num==2)
	   {
		   Robot_Initalize();
		   if(Robot_Init_bit==1)
		   {
			   Init_finish=1;
			   Robot_Init_bit=0;
		   }
	   }
   }
   else if(Init_finish==1)// 초기화 되어있으면
   {
	   if(end_bit==0)//정지버튼 안눌림?
	   {
		   if(start_bit==1)//시작버튼 들어옴?
		   {
			   if(pause_bit==1)//일시정지?
			   {
				   Robot_Initalize();
				   if(Robot_Init_bit==1)
				   {
					   break_duty=0;
					   Motor_Pwm=0;

				   }
			   }
			   else if(pause_bit==0)
			   {
				   Robot_Init_bit=0;
				   training_timer++;
				   if(training_timer==200)// 훈련시간 확인 알려주는것
				   {
					   training_timer=0;
					   time_now++;
				   }

				   if(mode_num==1)//모드1설정
				   {
					   if((break_timer<400)&&(start_bit==1))//2초동안 1차함수그래프로 브레이크 듀티 상승
					   {
						   break_timer++;
						   break_duty=0.002075*break_timer;
					   }
					   if(break_timer>=400)
					   {
						   break_timer=401;
					   }


					   Motor_Pwm=0.52+target_gain*0.048;

					   if(Type_sel==2)//시간설정
					   {
						   if(time_now>target_sec)
						   {
							   end_bit=1;
							   sprintf(BT1,"!e?");
							   	BT_Put_String(BT1);
						   }
					   }
					   else if(Type_sel==1)//거리설정
					   {
						   if(move_dis>target_dis)
						   {
							   end_bit=1;
							   sprintf(BT1,"!e?");
							   	BT_Put_String(BT1);
						   }
					   }
				   }
				   else if(mode_num==2)//모드 2설정
				   {

					   if((break_timer<400)&&(start_bit==1))//2초동안 1차함수그래프로 브레이크 듀티 상승
					   {
						   break_timer++;
						   break_duty=0.002075*break_timer;
					   }
					   if(break_timer>=400)
					   {
						   break_timer=401;
					   }
					   if(Encoder_deg_new>=0&&Encoder_deg_new<10)
					   {
						   smooth_rise=0;
					   }
					   else if(Encoder_deg_new>=21&&Encoder_deg_new<90)
					   {
						   smooth_rise=1;
					   }
					   else if(Encoder_deg_new>=100&&Encoder_deg_new<190)
					   {
						   smooth_rise=0;
					   }
					   else if(Encoder_deg_new>=201&&Encoder_deg_new<270)
					   {
						   smooth_rise=1;
					   }
					   else if(Encoder_deg_new>=280&&Encoder_deg_new<=360)
					   {
						   smooth_rise=0;
					   }
					   else
					   {
						   smooth_rise=0.5*(1-cos(16*Encoder_deg_new-160));
					   }

					   Motor_Pwm=EV_mva*40*smooth_rise;

				   if(Type_sel==2)
				   {
					   if(time_now>target_sec)
					   {
						   end_bit=1;
						   sprintf(BT1,"!e?");
						   	BT_Put_String(BT1);
					   }
				   }
				   else if(Type_sel==1)
				   {
					   if(move_dis>target_dis)
					   {
						   end_bit=1;
						   sprintf(BT1,"!e?");
						   	BT_Put_String(BT1);
					   }

				   }

			   }
			   else if(mode_num==3)//모드 3설정
			   {
				   if((break_timer<400)&&(start_bit==1))//2초동안 1차함수그래프로 브레이크 듀티 상승
				   {
					   break_timer++;
					   break_duty=0.002075*break_timer;
				   }
				   if(break_timer>=400)
				   {
					   break_timer=401;
				   }
				   if(Encoder_deg_new>=0&&Encoder_deg_new<10)
				   {
					   smooth_rise=0;
				   }
				   else if(Encoder_deg_new>=21&&Encoder_deg_new<90)
				   {
					   smooth_rise=1;
				   }
				   else if(Encoder_deg_new>=100&&Encoder_deg_new<190)
				   {
					   smooth_rise=0;
				   }
				   else if(Encoder_deg_new>=201&&Encoder_deg_new<270)
				   {
					   smooth_rise=1;
				   }
				   else if(Encoder_deg_new>=280&&Encoder_deg_new<=360)
				   {
					   smooth_rise=0;
				   }
				   else
				   {
					   smooth_rise=0.5*(1-cos(16*Encoder_deg_new-160));
				   }

				   //Motor_Pwm=0.52+target_gain*0.0004*(ratio_gain*Torque_R+(1-ratio_gain)*Torque_L+EV_mva*0.04);
				   APM_assist=a0 + a1*cos(Encoder_deg_new*w) + b1*sin(Encoder_deg_new*w) + a2*cos(2*Encoder_deg_new*w) + b2*sin(2*Encoder_deg_new*w) + a3*cos(3*Encoder_deg_new*w) + b3*sin(3*Encoder_deg_new*w) + a4*cos(4*Encoder_deg_new*w) + b4*sin(4*Encoder_deg_new*w) + a5*cos(5*Encoder_deg_new*w) + b5*sin(5*Encoder_deg_new*w) + a6*cos(6*Encoder_deg_new*w) + b6*sin(6*Encoder_deg_new*w) + a7*cos(7*Encoder_deg_new*w) + b7*sin(7*Encoder_deg_new*w) + a8*cos(8*Encoder_deg_new*w) + b8*sin(8*Encoder_deg_new*w);


				   //Motor_Pwm=APM_assist+EV_mva*40*smooth_rise;
				   Motor_Pwm=APM_assist;


				   if(Type_sel==2)
				   {
					   if(time_now>target_sec)
					   {
						   end_bit=1;
						   sprintf(BT1,"!e?");
						   	BT_Put_String(BT1);
					   }
				   }
				   else if(Type_sel==1)
				   {
					   if(move_dis>target_dis)
					   {
						   end_bit=1;
						   sprintf(BT1,"!e?");
						   	BT_Put_String(BT1);
					   }
				   }
			   }

			   time_now_min=time_now/60;
			   	   time_now_min=time_now_min%60;
			   	    time_now_min_10=time_now_min/10;
			   	    time_now_min_1=time_now_min-time_now_min_10*10;
			   	    time_now_hour=time_now/3600;


			   	    time_now_hour=time_now_hour%60;
			   	    move_distance_4=move_dis*1000;
			   	 	move_distance_4=move_distance_4%1;
			   	    move_distance_3=move_dis*1000/10;
			   	    move_distance_3=move_distance_3%10;
			   	    move_distance_2=move_dis*1000/100;
			   	    move_distance_2=move_distance_2%100;
			   	    move_distance_1=move_dis;
			   	    move_distance_1=move_distance_1%1000;
			   	    velocity=EV_mva*0.0088;
			   	    under_velocity=velocity*100-((int)velocity)*100;

			   }

		   }
	   }

	   else if(end_bit==1)//정지버튼 눌림?
	   {

		   if(Init_finish==1)
		   {
			   Robot_Initalize();
			   if(Robot_Init_bit==1)
			   {
				   Init_finish=0;
				  // Robot_Init_bit=0;
				   cleaer_variable();
				   Robot_Init_bit=0;
			   }

		   }
		/*   else if(Init_finish==0)
		   {


	   	   }*/
	   }
   }
///////////////////////////////////////////////////////////////


   if(start_bit==1&&Encoder_vel<-10)
   {
	   break_duty=0;
   }
   else if(start_bit==1&&Encoder_vel>-10)
   {
	   break_duty=0.83;
   }
   if(Motor_Pwm>=1)
   {
	   Motor_Pwm=1;
   }
   else if(Motor_Pwm<=0)
     {
  	   Motor_Pwm=0;
     }
   if(break_duty>=1)
   {
	   break_duty=1;
   }
   else if(break_duty<=0)
   {
	   break_duty=0;
   }


   if(em_break==1)
   {
	   break_duty=0.83;
   }
   //break_duty=0.83;
 //  Motor_Pwm=0.8;
   // Brake의 Duty를 조절하는 함수. Brake Duty는 0.0 ~ 1.0 사이어야 함.

   EPwm1Regs.TBPRD = (150E6 / 20E3) - 1;
   EPwm1Regs.CMPB = EPwm1Regs.TBPRD *break_duty;
   // Motor의 Duty를 조절하는 함수. Motor Duty는 0.0 ~ 1.0 사이어야 함.

   EPwm2Regs.TBPRD = (150E6 / 20E3) - 1;
   EPwm2Regs.CMPB = EPwm2Regs.TBPRD * Motor_Pwm;

   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE
}



//============================================================================================
