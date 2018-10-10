// 소속 : 서울과학기술대 휴머노이드로봇 연구2실
// 이름 : 김정준
// 날짜 : 2018-10-10
// 용도 : 모드별 알고리즘 실험데이터
#include "DSP28x_Project.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "string.h"
#include "variable.h"
// pragma CODE_SECTION -> Flash에서 RAM으로 함수를 옮기기 위한 작업 .
#pragma CODE_SECTION(Encoder_define,"ramfuncs")
#pragma CODE_SECTION(Uart_transmit,"ramfuncs")
#pragma CODE_SECTION(UART_Put_String,"ramfuncs")
#pragma CODE_SECTION(Motor_transmit,"ramfuncs")
#pragma CODE_SECTION(Motor_Put_String,"ramfuncs")
#pragma CODE_SECTION(Motor_Put_Char,"ramfuncs")
#pragma CODE_SECTION(BT_transmit,"ramfuncs")
#pragma CODE_SECTION(TrainAbnormalPerson,"ramfuncs")
#pragma CODE_SECTION(check_gait_score,"ramfuncs")

// CPU timer0 선언
interrupt void cpu_timer0_isr(void);

// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

//정준이의 함수------------------
int Robot_Initialize();
void clear_variable();
void MetabolizeRehabilitationRobot();
int ConnectBluetoot();
void OutputPWM();
void TrainAbnormalPerson();
void UpdateInformation();
int IsStart();
void BeNormal();
int IsPause();
int Type_Check_fun();
void Reword_inflection_point();
void Start_breaking();
void Reg_setting_fun();
//void Initialize_main_fun();
void Encoder_position_renew();
void Encoder_value_calculation();
void Gait_score_calculation();
void check_gait_score();
void break_time();
//------------------------

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
interrupt void sciaRxFifoIsr(void);

// @@@@@@@@@@@@@@@@@@@@@@@Main 함수 @@@@@@@@@@@@@@@@@@@@@@@
void main(void) {
	// Step 1. Disable Global Interrupt
	DINT;

	// Step 2. 시스템 컨트롤 초기화:
	InitSysCtrl();

	// FLASH 영역의 함수를 빠른 속도를 위해 RAM으로 구동시키기 위해 선언한 함수
	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
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
	Reg_setting_fun();

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
	IER = IER | M_INT1 | M_INT9;              // CPU 인터럽트(INT1), SCIRXB  활성화

	// 통신함수 초기화
	scia_fifo_init();      // Initialize the SCI FIFO
	scia_echoback_init();  // Initalize SCI for echoback
	scib_fifo_init();      // Initialize the SCI FIFO
	scib_echoback_init();  // Initalize SCI for echoback
	scic_fifo_init();      // Initialize the SCI FIFO
	scic_echoback_init();  // Initalize SCI for echoback

	//ePWM_SOCB 이벤트 트리거 설정
	EPwm3Regs.ETSEL.bit.SOCBEN = 1;            // SOCB 이벤트 트리거 Enable
	EPwm3Regs.ETSEL.bit.SOCBSEL = 2;           // SCCB 트리거 조건 : 카운터 주기 일치 시
	EPwm3Regs.ETPS.bit.SOCBPRD = 1;            // SOCB 이벤트 분주 설정 : 트리거 조건 한번 마다
	EPwm3Regs.TBCTL.bit.CTRMODE = 0;           // 카운트 모드 설정: Up-conut 모드
	EPwm3Regs.TBCTL.bit.HSPCLKDIV = 1; // TBCLK = [SYSCLKOUT / ((HSPCLKDIV*2) * 2^(CLKDIV))]
	EPwm3Regs.TBCTL.bit.CLKDIV = 1;        // TBCLK = [150MHz / (2*2)] = 37.5MHz
	EPwm3Regs.TBPRD = 1874; // TB주기= (TBPRD+1)/TBCLK = 1875/37.5MHz = 50us(20KHz)
	EPwm3Regs.TBCTR = 0x0000;                  // TB 카운터 초기화

	//버퍼 비우기
	for (i = 0; i < 10; i++)
		EV_Buff[i] = 0;

	for (i = 0; i < 30; i++)
		EV_Buff[i] = 0;

	for (i = 0; i < 16; i++)
		RxBuff[i] = 0;

	for (i = 0; i < 800; i++) {
		gait_abnormal_socre[i] = 0;
		gait_normal_socre[i] = 0;
	}

	EINT;
	// Enable Global interrupt INTM
	ERTM;
	// Enable Global realtime interrupt DBGM

// IDLE loop. Just sit and loop forever :
//--------------------------------------------------------------------------------------------
	for (;;) {
		if (MotorCount == 100) {
			MotorCount = 0;
			Motor_transmit();
		}
	}
}

// 메인함수 끝.
/*
 void Initialize_main_fun() {

 }
 */
//정준함수-----------------------------------------------------------------------------------------
void Reg_setting_fun() {
	EALLOW;
	PieVectTable.TINT0 = &cpu_timer0_isr;
	PieVectTable.SCIRXINTA = &sciaRxFifoIsr;
	SysCtrlRegs.HISPCP.bit.HSPCLK = 1;

	GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0;
	GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0;
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
}

int Robot_Initialize() {
	//환측다리=오른발이면
	if (leg_num == 1) {
		if (!init_bit) {
			break_duty = 0.63;	//브레이크 OFF
			Motor_Pwm = 0.55;	//모터 최저속도

			if (Encoder_deg_new >= 345 && Encoder_deg_new <= 350) {
				break_duty = 0;
				Motor_Pwm = 0;
				init_bit = 1;
				if (pause_finish == 0 && pause_bit)
					pause_finish = 1;
				return 1;
			}
			return 0;
		} else
			return 1;
	}
	//왼발이면
	else if (leg_num == 2) {
		if (!init_bit) {
			break_duty = 0.63;
			Motor_Pwm = 0.55;
			if (Encoder_deg_new >= 165 && Encoder_deg_new <= 170) {
				break_duty = 0;
				Motor_Pwm = 0;
				init_bit = 1;
				if (pause_finish == 0 && pause_bit)
					pause_finish = 1;
				return 1;
			}
			return 0;
		} else
			return 1;
	}
	return 0;
}

void clear_variable() {
	leg_num = 0;	//다리설정
	start_bit = 0;	//시작비트
	training_timer = 0;	//타이머0
	target_sec = 0;	//목표초
	move_dis = 0;	//이동거리
	target_time = 0;	//목표시간
	time_now = 0;	//보행시간
	pause_bit = 0;	//일시정지비트
	target_gain = 0;	//목표게인
	end_bit = 0;	//종료비트
	break_timer = 0;	//
	ratio_gain = 0;
	mode_num = 0;
	Type_sel = 0;
	target_dis = 0;
	E_vel_deg_new = 0;
	Encoder_revcnt = 0;
	velocity = 0;
	EV_mva = 0;

	smooth_rise = 0;
	CPM_assist = 0;
	gait_score=0;
	mean_abno_gait_speed=0;
	mean_no_gait_speed=0;
	normal_gait_size=0;
	abnormal_gait_size=0;
	foot_shift_bit=0;
	gait_bit=0;
	ex_gait_degree=180;
	start_record_bit=0;
	Score_speed=0;
	Target_speed=0;
	Add_score=0;
	slow_start_timer=0;
	break_time_now=0;
	Train_num=0;
	Train_target=0;
	time_now_min_10=0;
	time_now_min_1=0;
	time_now_sec_10=0;
	time_now_sec_1=0;
	move_distance_1000=0;
	move_distance_100=0;
	move_distance_10=0;
	move_distance_1=0;


	for (i = 0; i < 16; i++) RxBuff[i] = 0;
	for (i = 0; i < 30; i++) EV_Buff[i] = 0;
	for (i = 0; i < 800; i++) {
		gait_abnormal_socre[i] = 0;
		gait_normal_socre[i] = 0;
	}
}

void InitEPwm1Module(void) {
	/* Setup TBCLK */
	EPwm1Regs.TBPRD = (150E6 / 20E3) - 1; /* Set Timer Period */
	EPwm1Regs.TBCTR = 0; /* Clear Counter */

	/* Set Compare values */
	EPwm1Regs.CMPA.half.CMPA = ((EPwm1Regs.TBPRD + 1) >> 1); /* Set Compare A value to 50% */
	EPwm1Regs.CMPB = ((EPwm1Regs.TBPRD + 1) >> 1);
	/* Setup counter mode */
	EPwm1Regs.TBCTL.bit.CTRMODE = 0; /* Count Up (Asymmetric) */
	EPwm1Regs.TBPHS.half.TBPHS = 0; /* Phase is 0 */
	EPwm1Regs.TBCTL.bit.PHSEN = 0; /* Disable phase loading */
	EPwm1Regs.TBCTL.bit.PRDLD = 0; /* Period Register is loaded from its shadow when CNTR=Zero */
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0; /* Clock ratio to SYSCLKOUT */
	EPwm1Regs.TBCTL.bit.CLKDIV = 0; /* TBCLK = SYSCLK / (HSPCLKDIV * CLKDIV) */

	/* Setup shadowing */
	EPwm1Regs.CMPCTL.bit.SHDWAMODE = 0; /* Enable Shadowing */
	EPwm1Regs.CMPCTL.bit.LOADAMODE = 0; /* Load on CNTR=Zero */

	/* Set actions */
	EPwm1Regs.AQCTLA.bit.ZRO = 2; /* Set EPWM1A on CNTR=Zero */
	EPwm1Regs.AQCTLA.bit.CAU = 1; /* Clear EPWM1A on event A, up count */
	EPwm1Regs.AQCTLB.bit.ZRO = 2;
	EPwm1Regs.AQCTLB.bit.CBU = 1;

	/* Set Interrupts */
	EPwm1Regs.ETSEL.bit.INTSEL = 1; /* Select INT on CNTR=Zero */
	EPwm1Regs.ETSEL.bit.INTEN = 1; /* Enable INT */
	EPwm1Regs.ETPS.bit.INTPRD = 1; /* Generate INT on 1st event */
}

void InitEPwm2Module(void) {
	/* Setup Counter Mode and Clock */
	EPwm2Regs.TBCTL.bit.CTRMODE = 0; /* Count Up (Asymmetric) */
	EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0; /* TBCLK = SYSCLKOUT / (HSPCLKDIV * CLKDIV) = 150MHz */
	EPwm2Regs.TBCTL.bit.CLKDIV = 0;

	/* Setup Phase */
	EPwm2Regs.TBPHS.half.TBPHS = 0; /* Phase is 0 */
	EPwm2Regs.TBCTL.bit.PHSEN = 0; /* Disable phase loading */

	/* Setup Period (Carrier Frequency) */
	EPwm2Regs.TBPRD = (150E6 / 20E3) - 1; /* Set Timer Period, (150MHz/20KHz)-1 = 7,499 (0x1D4B) */
	EPwm2Regs.TBCTR = 0; /* Clear Counter */

	/* Set Compare Value */
	//EPwm2Regs.CMPA.half.CMPA = (Uint16)((EPwm2Regs.TBPRD + 1) * PWM_DUTY_RATIO_C);    /* Set Compare A Value to 20% */
	EPwm2Regs.CMPB = (Uint16) ((EPwm2Regs.TBPRD + 1) * 0.3); /* Set Compare B Value to 20% */

	/* Setup shadowing */
	EPwm2Regs.TBCTL.bit.PRDLD = 0; /* Period Register is loaded from its shadow when CNTR=Zero */
	EPwm2Regs.CMPCTL.bit.SHDWAMODE = 0; /* Compare A Register is loaded from its shadow when CNTR=Zero */
	EPwm2Regs.CMPCTL.bit.LOADAMODE = 0;
	//EPwm2Regs.CMPCTL.bit.SHDWBMODE = 0;       /* Compare B Register is loaded from its shadow when CNTR=Zero */
	//EPwm2Regs.CMPCTL.bit.LOADBMODE = 0;

	/* Set actions */
	EPwm2Regs.AQCTLB.bit.ZRO = 2; /* Set EPWM4A on CNTR=Zero */
	EPwm2Regs.AQCTLB.bit.CBU = 1; /* Clear EPWM4A on CNTR=CMPA, Up-Count */
	//EPwm1Regs.AQCTLB.bit.ZRO = 2;     /* Set EPWM4B on CNTR=Zero */
	//EPwm1Regs.AQCTLB.bit.CBU = 1;     /* Clear EPWM4B on CNTR=CMPB, Up-Count */

	/* Set Interrupts */
	EPwm2Regs.ETSEL.bit.INTSEL = 1; /* Select INT on CNTR=Zero */
	EPwm2Regs.ETPS.bit.INTPRD = 1; /* Generate INT on 1st event */
//  EPwm2Regs.ETSEL.bit.INTEN = 1;      /* Enable INT */
}

void scia_echoback_init() {
	SciaRegs.SCICTL1.bit.SWRESET = 0;
	SciaRegs.SCICCR.bit.SCICHAR = 7; // 1 stop bit, No loopback, No parity, 8 char bits,
	SciaRegs.SCICTL1.bit.RXENA = 1;    // SCI 송신기능 Enable
	SciaRegs.SCICTL1.bit.TXENA = 1;    // async mode, idle-line protocol
	SciaRegs.SCICTL2.all = 0x0003;
	SciaRegs.SCICTL2.bit.TXINTENA = 1;
	SciaRegs.SCICTL2.bit.RXBKINTENA = 1;

#if (CPU_FRQ_150MHZ)
	SciaRegs.SCIHBAUD = 0x0000;  // 115200 baud @LSPCLK = 37.5MHz.
	SciaRegs.SCILBAUD = 0x0028;
#endif
#if (CPU_FRQ_100MHZ)
	SciaRegs.SCIHBAUD =0x0001;  // 9600 baud @LSPCLK = 20MHz.
	SciaRegs.SCILBAUD =0x0044;
#endif
	SciaRegs.SCICTL1.bit.SWRESET = 1;
	//SciaRegs.SCIHBAUD    =0x0000;  // 38400 baud @LSPCLK = 37.5MHz.
	//SciaRegs.SCILBAUD    =0x0079;
	//SciaRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
}

void scia_fifo_init() {
	SciaRegs.SCIFFTX.all = 0xE040;
	SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;
	SciaRegs.SCIFFCT.all = 0x0;
	SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;
	SciaRegs.SCIFFRX.bit.RXFFIENA = 1;
	SciaRegs.SCIFFRX.bit.RXFFIL = 1;
}

void scib_echoback_init() {
	ScibRegs.SCICTL1.bit.SWRESET = 0;
	ScibRegs.SCICCR.bit.SCICHAR = 7; // 1 stop bit, No loopback, No parity, 8 char bits,
	ScibRegs.SCICTL1.bit.RXENA = 1;    // SCI 송신기능 Enable
	ScibRegs.SCICTL1.bit.TXENA = 1;    // async mode, idle-line protocol
	ScibRegs.SCICTL2.all = 0x0003;
	ScibRegs.SCICTL2.bit.TXINTENA = 1;
	ScibRegs.SCICTL2.bit.RXBKINTENA = 1;

#if (CPU_FRQ_150MHZ)
	ScibRegs.SCIHBAUD = 0x0000;  // 115200 baud @LSPCLK = 37.5MHz.
	ScibRegs.SCILBAUD = 0x0028;
#endif
#if (CPU_FRQ_100MHZ)
	ScibRegs.SCIHBAUD =0x0001;  // 9600 baud @LSPCLK = 20MHz.
	ScibRegs.SCILBAUD =0x0044;
#endif
	ScibRegs.SCICTL1.bit.SWRESET = 1;
	//ScibRegs.SCIHBAUD    =0x0000;  // 38400 baud @LSPCLK = 37.5MHz.
	//ScibRegs.SCILBAUD    =0x0079;
	//ScibRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
}

void scib_fifo_init() {
	ScibRegs.SCIFFTX.all = 0xE040;
	ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;
	ScibRegs.SCIFFCT.all = 0x0;
	ScibRegs.SCIFFRX.bit.RXFFINTCLR = 1;
	ScibRegs.SCIFFRX.bit.RXFFIENA = 1;
	ScibRegs.SCIFFRX.bit.RXFFIL = 1;
}

void scic_echoback_init() {
	ScicRegs.SCICTL1.bit.SWRESET = 0;
	ScicRegs.SCICCR.bit.SCICHAR = 7;

	ScicRegs.SCICTL1.bit.RXENA = 1;
	ScicRegs.SCICTL1.bit.TXENA = 1;

	ScicRegs.SCICTL2.all = 0x0003;
	ScicRegs.SCICTL2.bit.TXINTENA = 1;
	ScicRegs.SCICTL2.bit.RXBKINTENA = 1;

#if (CPU_FRQ_150MHZ)
	ScicRegs.SCIHBAUD = 0x0001;  // 9600 baud @LSPCLK = 37.5MHz.
	ScicRegs.SCILBAUD = 0x00E7;
#endif
#if (CPU_FRQ_100MHZ)
	ScicRegs.SCIHBAUD =0x0001;  // 9600 baud @LSPCLK = 20MHz.
	ScicRegs.SCILBAUD =0x0044;
#endif
	ScicRegs.SCICTL1.bit.SWRESET = 1;
	//ScibRegs.SCIHBAUD    =0x0000;  // 38400 baud @LSPCLK = 37.5MHz.
	//ScibRegs.SCILBAUD    =0x0079;
	//ScibRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
}

void scic_fifo_init() {
	ScicRegs.SCIFFTX.all = 0xE040;
	ScicRegs.SCIFFRX.bit.RXFIFORESET = 1;
	ScicRegs.SCIFFCT.all = 0x0;
	ScicRegs.SCIFFRX.bit.RXFFINTCLR = 1;
	ScicRegs.SCIFFRX.bit.RXFFIENA = 1;
	ScicRegs.SCIFFRX.bit.RXFFIL = 16;
}

void BT_Put_String(char *BT_string) {
	while (*BT_string != 0) {
		SciaRegs.SCITXBUF = *BT_string++;
		while (SciaRegs.SCIFFTX.bit.TXFFST != 0) {
		}
	}
}

void UART_Put_String(char *Uart_string) {
	while (*Uart_string != 0) {
		ScibRegs.SCITXBUF = *Uart_string++;
		while (ScibRegs.SCIFFTX.bit.TXFFST != 0) {
		}
	}
}

void Motor_Put_Char(unsigned char Uart_string) {

	ScicRegs.SCITXBUF = Uart_string;
	while (ScicRegs.SCIFFTX.bit.TXFFST != 0) {
	}

}

void Motor_Put_String(char *Motor_string) {
	while (*Motor_string != 0) {
		ScicRegs.SCITXBUF = *Motor_string++;
		while (ScicRegs.SCIFFTX.bit.TXFFST != 0) {
		}
	}

}

void BT_transmit() {
	sprintf(BT1, "!s%d.%dt%d%d%d%dd%d%d%d%d?\n\0",
			(int) velocity, (int) under_velocity,
			time_now_min_10, time_now_min_1, time_now_sec_10, time_now_sec_1,
			move_distance_1000, move_distance_100, move_distance_10,move_distance_1 );



	//시간설정
	if (Type_sel == 2) {
		if (time_now > target_sec)
			sprintf(BT1, "!e?");
	}
	//거리설정
	else if (Type_sel) {
		if (move_dis > target_dis)
			sprintf(BT1, "!e?");
	}

	BT_Put_String(BT1);
}

void Uart_transmit() {
	//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@테스트시 넣는 코드@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@//
	sprintf(UT1, "%ld,%ld,%ld,%ld`\n\0", (long) (Motor_Pwm * 10000), (long) (Encoder_deg_new * 100), (long) (EV_mva * 10000),(long) (Add_score*100 ));
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@재활치료시 넣는 코드@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@//
	/*
	 *
	sprintf(BT1, "!s%d.%dt%d%d%d%dd%d%d%d%df%d%d%dc%d%d%d?\n\0",
			(int) velocity, (int) under_velocity,
			time_now_min_10, time_now_min_1, time_now_sec_10, time_now_sec_1,
			move_distance_1000, move_distance_100, move_distance_10,move_distance_1,
			face_score_100,face_score_10,face_score_1,
			Gait_score_100, Gait_score_10, Gait_score_1 );


	 */
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@//
	UART_Put_String(UT1);
}

// LCD 없이 모터제어를 위한 통신프로토콜
void Motor_transmit() {
	Motor_char_s[0] = 0x3A;
	Motor_char_s[1] = 0x1A;
	Motor_char_s[2] = 0x52;
	Motor_char_s[3] = 0x02;
	Motor_char_s[4] = 0x33;
	Motor_char_s[6] = 0xA1;
	Motor_char_s[8] = 0x0D;
	Motor_char_s[9] = 0x0A;

	if (Mt_cnt == 0) {
		for (m = 0; m < 10; m++) {
			if (m == 5 || m == 7)
				Motor_Put_Char(0x00);
			else {
				sprintf(MT1, "%c", Motor_char_s[m]);
				Motor_Put_String(MT1);
			}
		}
		Mt_cnt++;
	}
}

void Encoder_position_renew() {
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
}
void Encoder_value_calculation() {
	Encoder_sum = 0;

	for (Encoder_cnt = 0; Encoder_cnt < 10; Encoder_cnt++) {
		Encoder_sum += Encoder[Encoder_cnt] << Encoder_cnt; // Encoder_sum 은 0-1024 Pulse까지의 수를 Count해줌.
	}

	Encoder_deg_new = 360 - (double) Encoder_sum * 0.3515625; // Encoder값 갱신. 1024 Pulse를 0 - 360 deg로 바꿔줌.
	//Encoder_deg_new =360-Encoder_deg_new;
	if (Encoder_deg_old - Encoder_deg_new >= 200) // 각속도 구할 때 갑자기 100도이상 차이나면 360 -> 0 도로 된것을 알아내는 조건
		Encoder_revcnt++; // 회전수 체크

	E_vel_deg_new = Encoder_revcnt * 360 + Encoder_deg_new;
	move_dis = 0.001 * E_vel_deg_new * 880 / 360; //총 회전각도*각거리?
	Encoder_vel = (E_vel_deg_new - E_vel_deg_old) * 100; // Angular velocity dt=0.005s

}

void Moving_avg_degree() {
	if (E_i == 0) {
		EV_Buff[E_i] = Encoder_vel;
		E_i++;
	} else if (E_i == 1) {
		EV_Buff[E_i] = Encoder_vel;
		E_i++;
	} else if (E_i == 2) {
		EV_Buff[E_i] = Encoder_vel;
		E_i++;
	} else if (E_i == 3) {
		EV_Buff[E_i] = Encoder_vel;
		E_i++;
	} else if (E_i == 4) {
		EV_Buff[E_i] = Encoder_vel;
		E_i++;
	} else if (E_i == 5) {
		EV_Buff[E_i] = Encoder_vel;
		E_i++;
	} else if (E_i == 6) {
		EV_Buff[E_i] = Encoder_vel;
		E_i++;
	} else if (E_i == 7) {
		EV_Buff[E_i] = Encoder_vel;
		E_i++;
	} else if (E_i == 8) {
		EV_Buff[E_i] = Encoder_vel;
		E_i++;
	} else if (E_i == 9) {
		EV_Buff[E_i] = Encoder_vel;
		E_i++;
	} else if (E_i == 10) {
		EV_Buff[E_i] = Encoder_vel;
		E_i++;
	} else if (E_i == 11) {
		EV_Buff[E_i] = Encoder_vel;
		E_i++;
	} else if (E_i == 12) {
		EV_Buff[E_i] = Encoder_vel;
		E_i++;
	} else if (E_i == 13) {
		EV_Buff[E_i] = Encoder_vel;
		E_i++;
	} else if (E_i == 14) {
		EV_Buff[E_i] = Encoder_vel;
		E_i++;
	} else if (E_i == 15) {
		EV_Buff[E_i] = Encoder_vel;
		E_i++;
	} else if (E_i == 16) {
		EV_Buff[E_i] = Encoder_vel;
		E_i++;
	} else if (E_i == 17) {
		EV_Buff[E_i] = Encoder_vel;
		E_i++;
	} else if (E_i == 18) {
		EV_Buff[E_i] = Encoder_vel;
		E_i++;
	} else if (E_i == 19) {
		EV_Buff[E_i] = Encoder_vel;
		E_i++;
	} else if (E_i == 20) {
		EV_Buff[E_i] = Encoder_vel;
		E_i++;
	} else if (E_i == 21) {
		EV_Buff[E_i] = Encoder_vel;
		E_i++;
	} else if (E_i == 22) {
		EV_Buff[E_i] = Encoder_vel;
		E_i++;
	} else if (E_i == 23) {
		EV_Buff[E_i] = Encoder_vel;
		E_i++;
	} else if (E_i == 24) {
		EV_Buff[E_i] = Encoder_vel;
		E_i++;
	} else if (E_i == 25) {
		EV_Buff[E_i] = Encoder_vel;
		E_i++;
	} else if (E_i == 26) {
		EV_Buff[E_i] = Encoder_vel;
		E_i++;
	} else if (E_i == 27) {
		EV_Buff[E_i] = Encoder_vel;
		E_i++;
	} else if (E_i == 28) {
		EV_Buff[E_i] = Encoder_vel;
		E_i++;
	} else if (E_i == 29) {
		EV_Buff[E_i] = Encoder_vel;
		E_i = 0;
	}

	EV_mva = 0.03333333
			* (EV_Buff[0] + EV_Buff[1] + EV_Buff[2] + EV_Buff[3] + EV_Buff[4]
					+ EV_Buff[5] + EV_Buff[6] + EV_Buff[7] + EV_Buff[8]
					+ EV_Buff[9] + EV_Buff[10] + EV_Buff[11] + EV_Buff[12]
					+ EV_Buff[13] + EV_Buff[14] + EV_Buff[15] + EV_Buff[16]
					+ EV_Buff[17] + EV_Buff[18] + EV_Buff[19] + EV_Buff[20]
					+ EV_Buff[21] + EV_Buff[22] + EV_Buff[23] + EV_Buff[24]
					+ EV_Buff[25] + EV_Buff[26] + EV_Buff[27] + EV_Buff[28]
					+ EV_Buff[29]);
	EA_mva = (EV_mva - EV_mva_old) * 100;

}

void Encoder_define() {
	Encoder_deg_old = Encoder_deg_new; // 이전 Encoder값을 저장
	E_vel_deg_old = E_vel_deg_new;
	EV_mva_old = EV_mva;

	// Encoder Digital Input 값 받기
	Encoder_position_renew();
	Encoder_value_calculation();
	Moving_avg_degree();

}

interrupt void sciaRxFifoIsr(void) {

	Receivedbuff = SciaRegs.SCIRXBUF.bit.RXDT;

	if (a == 0)
	{
		if (Receivedbuff == '!')
		{
			RxBuff[a] = Receivedbuff;
			a++;
		}
		else
		{
			RxBuff[6] = 0;
			a = 0;
		}
	}
	else if (a == 1)
	{
		RxBuff[a] = Receivedbuff;
		a++;
	}
	else if (a == 2)
	{
		RxBuff[a] = Receivedbuff;
		a++;
	}
	else if (a == 3)
	{
		RxBuff[a] = Receivedbuff;
		a++;
	}
	else if (a == 4)
	{
		RxBuff[a] = Receivedbuff;
		a++;
	}
	else if (a == 5)
	{
		RxBuff[a] = Receivedbuff;
		a++;
	}
	else if (a == 6)
	{
		RxBuff[a] = Receivedbuff;
		a++;
	}

	if (Receivedbuff == '?') {
		RxBuff[a] = Receivedbuff;
		a = 0;
	}

	if (RxBuff[0] == '!' && RxBuff[1] == 'S' && RxBuff[2] == '?') { //////////////////////////////>>>>>>>>>>>>?>?????????????????????
		if (leg_num != 0 && mode_num != 0) {
			if (Type_sel == 1 || Type_sel == 2) {
				start_bit = 1;
				RxBuff[6] = 0;
			} else {
				start_bit = 0;
				RxBuff[6] = 0;
			}
		}

		else {
			start_bit = 0;
			RxBuff[6] = 0;
		}
	}

	else if (RxBuff[0] == '!' && RxBuff[1] == 'E' && RxBuff[2] == '?') {
		end_bit = 1;
		RxBuff[6] = 0;
		sprintf(UT1, "!e?");
		UART_Put_String(UT1);
	}
	else if (RxBuff[0] == '!' && RxBuff[1] == 'P' && RxBuff[2] == '1'
			&& RxBuff[3] == '?') {
		pause_bit = 1;
		RxBuff[6] = 0;
	}
	else if (RxBuff[0] == '!' && RxBuff[1] == 'P' && RxBuff[2] == '2'
			&& RxBuff[3] == '?') {
		pause_bit = 0;
		pause_finish = 0;
		RxBuff[6] = 0;
		pause_finish = 0;
	}
	else if (RxBuff[0] == '!' && RxBuff[1] == 'M' && RxBuff[2] == '1'
			&& RxBuff[3] == '?') {
		mode_num = 1;
		RxBuff[6] = 0;
	}
	else if (RxBuff[0] == '!' && RxBuff[1] == 'M' && RxBuff[2] == '2'
			&& RxBuff[3] == '?') {
		target_gain = 0;
		mode_num = 2;
		RxBuff[6] = 0;
	}
	else if (RxBuff[0] == '!' && RxBuff[1] == 'M' && RxBuff[2] == '3'
			&& RxBuff[3] == '?') {
		mode_num = 3;
		RxBuff[6] = 0;
	}
	else if (RxBuff[0] == '!' && RxBuff[1] == 'H' && RxBuff[2] == 'R'
		&& RxBuff[3] == '?') {
		leg_num = 1;
		ex_gait_degree = 180;
		RxBuff[6] = 0;
	}
	else if (RxBuff[0] == '!' && RxBuff[1] == 'H' && RxBuff[2] == 'L'
			&& RxBuff[3] == '?') {
		leg_num = 2;
		ex_gait_degree = 0;
		RxBuff[6] = 0;
	}
	else if (RxBuff[0] == '!' && RxBuff[1] == 'D' && RxBuff[6] == '?') {
		target_dis = atof(&RxBuff[2]);
		target_dis = target_dis * 0.01;

		Type_sel = 1;
		RxBuff[6] = 0;
	}
	else if (RxBuff[0] == '!' && RxBuff[1] == 'T' && RxBuff[5] == '?') {
		target_time = atof(&RxBuff[2]);
		Train_target =(int)target_time % 10;
		Train_target++;
		target_time = (int)target_time / 10;
		target_time = (int)target_time * 60;

		Type_sel = 2;
		RxBuff[6] = 0;
	}
	else if (RxBuff[0] == '!' && RxBuff[1] == 'G' && RxBuff[4] == '?') {
		if (mode_num ==1) {
			target_gain = atof(&RxBuff[2]);
			RxBuff[6] = 0;
		}
		else RxBuff[6] = 0;
	} else if (RxBuff[0] == '!' && RxBuff[1] == 'R' && RxBuff[4] == '?') {
		if (mode_num == 2) {
			ratio_gain = atof(&RxBuff[2]);
			ratio_gain = ratio_gain * 0.1;
			RxBuff[6] = 0;
		}
		else RxBuff[6] = 0;
	}
	else if (RxBuff[0] == '!' && RxBuff[1] == 'K' && RxBuff[4] == '?') {
		if (mode_num == 2) {
			ratio_gain = atof(&RxBuff[2]);
			ratio_gain = ratio_gain * 0.1;
			RxBuff[6] = 0;
		}
		else RxBuff[6] = 0;
	}
	else RxBuff[6] = 0;

	SciaRegs.SCIFFRX.bit.RXFFOVRCLR = 1;			// Clear Overflow flag
	SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;			// Clear Interrupt flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;		// Acknowledge interrupt to PIE

}

void MetabolizeRehabilitationRobot() {

	++TimerCount; //40 -> 2
	++TimerCount_2;
	if (!Mt_cnt)
		++MotorCount;

	//속도, 토크값 컴에서확인
	if (TimerCount_2 == 2) {
		TimerCount_2 = 0;
		Encoder_define();
		if (start_bit && !end_bit) {
			Uart_transmit();
			//BT_transmit(); //테스트시 넣음
		}
	}
	// MATLAB 2 -> 100Hz Bluetooth 40 -> 5Hz
	if (TimerCount == 20) {
		TimerCount = 0;
		if (start_bit && (!end_bit))
			BT_transmit();
	}

}

int ConnectBluetooth() {
	if (leg_num == 1 || leg_num == 2)
		return 1;
	else
		return 0;
}

void OutputPWM() {
	if (start_bit && Encoder_vel < -10)
		break_duty = 0;

	if (Motor_Pwm >= 1)
		Motor_Pwm = 1;
	else if (Motor_Pwm <= 0)
		Motor_Pwm = 0;

	if (break_duty >= 1)
		break_duty = 1;
	else if (break_duty <= 0)
		break_duty = 0;

// Brake의 Duty를 조절하는 함수. Brake Duty는 0.0 ~ 1.0 사이어야 함.
	EPwm1Regs.TBPRD = (150E6 / 20E3) - 1;
	EPwm1Regs.CMPB = EPwm1Regs.TBPRD * break_duty;

// Motor의 Duty를 조절하는 함수. Motor Duty는 0.0 ~ 1.0 사이어야 함.
	EPwm2Regs.TBPRD = (150E6 / 20E3) - 1;
	EPwm2Regs.CMPB = EPwm2Regs.TBPRD * Motor_Pwm;

	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE
}

int IsStart() {
	return start_bit;
}

int IsPause() {
	if (pause_bit) {
		if (!pause_finish) {
			if (init_bit == 1)
				init_bit = 0;
			Robot_Initialize();
			return 1;
		} else
			return 1;
	}
	return 0;
}

void IncreaseTime() {
	++training_timer;
	// 훈련시간 확인 알려주는것
	if (training_timer == 200) {
		training_timer = 0;
		++time_now;
	}
}

void TrainAbnormalPerson() {
	break_duty = 0.83;
	switch (mode_num) {
	case 1:
		Start_breaking();
		CPM_assist = a0 + a1 * cos(Encoder_deg_new * w)
				+ b1 * sin(Encoder_deg_new * w)
				+ a2 * cos(2 * Encoder_deg_new * w)
				+ b2 * sin(2 * Encoder_deg_new * w)
				+ a3 * cos(3 * Encoder_deg_new * w)
				+ b3 * sin(3 * Encoder_deg_new * w)
				+ a4 * cos(4 * Encoder_deg_new * w)
				+ b4 * sin(4 * Encoder_deg_new * w)
				+ a5 * cos(5 * Encoder_deg_new * w)
				+ b5 * sin(5 * Encoder_deg_new * w)
				+ a6 * cos(6 * Encoder_deg_new * w)
				+ b6 * sin(6 * Encoder_deg_new * w)
				+ a7 * cos(7 * Encoder_deg_new * w)
				+ b7 * sin(7 * Encoder_deg_new * w)
				+ a8 * cos(8 * Encoder_deg_new * w)
				+ b8 * sin(8 * Encoder_deg_new * w);
		if (foot_shift_bit)
			Abnormal_assist_gain = 1;
		if (!foot_shift_bit)
			Abnormal_assist_gain = 1 + ratio_gain;
		Motor_Pwm = (1 + target_gain)*Abnormal_assist_gain * (CPM_assist - 0.5985) + 0.5985;

		break;
	case 2:
		Start_breaking();
		Reword_inflection_point();
		Motor_Pwm = EV_mva * 40 * smooth_rise;

		break;
	case 3:
		Start_breaking();
		CPM_assist = a0 + a1 * cos(Encoder_deg_new * w)
				+ b1 * sin(Encoder_deg_new * w)
				+ a2 * cos(2 * Encoder_deg_new * w)
				+ b2 * sin(2 * Encoder_deg_new * w)
				+ a3 * cos(3 * Encoder_deg_new * w)
				+ b3 * sin(3 * Encoder_deg_new * w)
				+ a4 * cos(4 * Encoder_deg_new * w)
				+ b4 * sin(4 * Encoder_deg_new * w)
				+ a5 * cos(5 * Encoder_deg_new * w)
				+ b5 * sin(5 * Encoder_deg_new * w)
				+ a6 * cos(6 * Encoder_deg_new * w)
				+ b6 * sin(6 * Encoder_deg_new * w)
				+ a7 * cos(7 * Encoder_deg_new * w)
				+ b7 * sin(7 * Encoder_deg_new * w)
				+ a8 * cos(8 * Encoder_deg_new * w)
				+ b8 * sin(8 * Encoder_deg_new * w);

		acc_term = b1 * w * cos(w * Encoder_deg_new)
				+ 2 * b2 * w * cos(2 * w * Encoder_deg_new)
				+ 3 * b3 * w * cos(3 * w * Encoder_deg_new)
				+ 4 * b4 * w * cos(4 * w * Encoder_deg_new)
				+ 5 * b5 * w * cos(5 * w * Encoder_deg_new)
				+ 6 * b6 * w * cos(6 * w * Encoder_deg_new)
				+ 7 * b7 * w * cos(7 * w * Encoder_deg_new)
				+ 8 * b8 * w * cos(8 * w * Encoder_deg_new)
				- a1 * w * sin(w * Encoder_deg_new)
				- 2 * a2 * w * sin(2 * w * Encoder_deg_new)
				- 3 * a3 * w * sin(3 * w * Encoder_deg_new)
				- 4 * a4 * w * sin(4 * w * Encoder_deg_new)
				- 5 * a5 * w * sin(5 * w * Encoder_deg_new)
				- 6 * a6 * w * sin(6 * w * Encoder_deg_new)
				- 7 * a7 * w * sin(7 * w * Encoder_deg_new)
				- 8 * a8 * w * sin(8 * w * Encoder_deg_new);

		//0.0008 0.0001
		//Motor_Pwm = (1 + target_gain) * (CPM_assist - 0.5985) + 0.5985+ vel_gain * (EV_mva - 1600 * (CPM_assist - 0.5985))	+ acc_gain * (EA_mva - 10000*acc_term);
		vel_acc_gain = (1 + vel_gain * (EV_mva - 1600 * (CPM_assist - 0.5985))
				+ acc_gain * (EA_mva - 10000 * acc_term));

		if (vel_acc_gain < 1)
			vel_acc_gain = 1;

		if (foot_shift_bit)
			Abnormal_assist_gain = 1;
		if (!foot_shift_bit)
			Abnormal_assist_gain = 1 + ratio_gain;

		Motor_Pwm = vel_acc_gain * Abnormal_assist_gain * (CPM_assist - 0.5985)
				+ 0.5985;

		break;
	}

}

void UpdateInformation() {
//시간변수 업데이트
	time_now_sec_1 = time_now % 60;
	time_now_sec_1 = time_now_sec_1 % 10;

	time_now_sec_10 = time_now % 60;
	time_now_sec_10 = time_now_sec_10 /10;

	time_now_min_1 = time_now / 60;
	time_now_min_1 = time_now_min_1 % 60;
	time_now_min_1 = time_now_min_1 % 10;

	time_now_min_10 = time_now / 60;
	time_now_min_10 = time_now_min_10 % 60;
	time_now_min_10 = time_now_min_10 / 10;

//거리변수 업데이트
	move_distance_1000 = move_distance_100 = move_distance_10 = move_distance_1 = move_dis;
	move_distance_1000 = move_distance_1000 / 1000;
	move_distance_100 = move_distance_100 % 1000;
	move_distance_100 = move_distance_100 / 100;
	move_distance_10 = move_distance_10 % 100;
	move_distance_10 = move_distance_10 /10;
	move_distance_1 = move_distance_1 % 10;
//각속도-->보행속도
	velocity = EV_mva * 0.0088;
	under_velocity = velocity * 100 - ((int) velocity) * 100;
}

int IsEnd() {
	return end_bit;
}

void BeNormal() {
	if (init_bit)
		init_bit = 0;
	Robot_Initialize();
	if (init_bit)
		clear_variable();
}

int Type_Check_fun() {
	if (Type_sel == 1) {
		if (move_dis > target_dis) {
			end_bit = 1;
			sprintf(BT1, "!e?");
			BT_Put_String(BT1);
			return 1;
		}
	}

	else if (Type_sel == 2) {
		if (time_now >= target_time) {
			if((Train_target-Train_num)==1){
				end_bit = 1;
				sprintf(BT1, "!e?");
				BT_Put_String(BT1);
			}
			else {
				break_time();
				return 1;
			}
		}
	}
	return 0;

}

void break_time(){
	++break_timer;
	// 휴식시간 확인 알려주는것
	if (break_timer == 200) {
		break_timer = 0;
		++break_time_now;
	}
	if (break_time_now>=60){
		time_now=0;
		++Train_num;
	}
}



void Reword_inflection_point() {
	if (Encoder_deg_new >= 0 && Encoder_deg_new < 10)
		smooth_rise = 0;
	else if (Encoder_deg_new >= 21 && Encoder_deg_new < 90)
		smooth_rise = 1;
	else if (Encoder_deg_new >= 100 && Encoder_deg_new < 190)
		smooth_rise = 0;
	else if (Encoder_deg_new >= 201 && Encoder_deg_new < 270)
		smooth_rise = 1;
	else if (Encoder_deg_new >= 280 && Encoder_deg_new <= 360)
		smooth_rise = 0;
	else
		smooth_rise = 0.5 * (1 - cos(16 * Encoder_deg_new - 160));

}
void Start_breaking() {
	if ((slow_start_timer < 400) && (start_bit == 1))   //2초동안 1차함수그래프로 브레이크 듀티 상승
			{
		slow_start_timer++;
		break_duty = 0.002075 * slow_start_timer;
	}
	if (slow_start_timer >= 400) {
		slow_start_timer = 401;
	}
}

void Gait_score_calculation() {
	//보행 점수용 각도로 변
	Gait_score_degree = Encoder_deg_new - 120;
	if (Gait_score_degree <= 0)
		Gait_score_degree = Gait_score_degree + 360;
	if (Gait_score_degree >= 180)
		Gait_score_degree = Gait_score_degree - 180;

	//초기 1주기 버리고 시작
	if (!start_record_bit) {
		if ((Gait_score_degree - ex_gait_degree) < -150)
			start_record_bit = 1;
	}

	// 180도 위상차가 나면
	else if (start_record_bit) {
		if ((Gait_score_degree - ex_gait_degree) < -150) {
			foot_shift_bit = !foot_shift_bit;
			gait_bit = 0;

			//주기끝나면 버퍼 비우기, 계산
			if (foot_shift_bit)
				check_gait_score();
		}
		//환측 stance 상황
		if (foot_shift_bit) {
			gait_normal_socre[gait_bit] = EV_mva * 0.0088;
			++gait_bit;
			normal_gait_size = gait_bit;
		}

		//건측 stance 상황
		if (!foot_shift_bit) {
			gait_abnormal_socre[gait_bit] = EV_mva * 0.0088;
			++gait_bit;
			abnormal_gait_size = gait_bit;
		}
	}

	ex_gait_degree = Gait_score_degree;
}

void check_gait_score() {
	//좌측, 우측 속도비교
	for (i = 0; i < normal_gait_size; i++)
		sum_no_gait_speed += abs(gait_normal_socre[i]);
	mean_no_gait_speed = sum_no_gait_speed / normal_gait_size;
	mean_no_gait_speed = roundf(mean_no_gait_speed * 100) / 100;

	for (i = 0; i < abnormal_gait_size; i++)
		sum_abno_gait_speed += abs(gait_abnormal_socre[i]);
	mean_abno_gait_speed = sum_abno_gait_speed / abnormal_gait_size;
	mean_abno_gait_speed = roundf(mean_abno_gait_speed * 100) / 100;

	gait_score = (110 - abs((mean_abno_gait_speed - mean_no_gait_speed)*100));
	if (gait_score < 30)
		gait_score = 0;
	if (gait_score >= 95)
		gait_score = 100;

	/*//목표속도 비교
	Score_speed=100-10*(Target_speed-velocity);
	if (Score_speed<0)
		Score_speed=0;
	if (Score_speed>100)
		Score_speed=100;

	//얼굴점수*/



	//최종점수 계산
	Add_score=gait_score;


	//버퍼비우기
	for (i = 0; i < 800; i++) {
		gait_abnormal_socre[i] = 0;
		gait_normal_socre[i] = 0;
	}
	sum_no_gait_speed=0;
	sum_abno_gait_speed=0;
	abnormal_gait_size = 0;
	normal_gait_size = 0;

}
interrupt void cpu_timer0_isr(void) // cpu timer 현재 제어주파수 100Hz
{
	MetabolizeRehabilitationRobot();

	if (!ConnectBluetooth())
		goto RETURN;

	if (!Robot_Initialize())
		goto RETURN;

	if (!IsStart())
		goto RETURN;

	if (IsPause())
		goto RETURN;

	if (Type_Check_fun())
		goto RETURN;
	IncreaseTime();
	TrainAbnormalPerson();
	Gait_score_calculation();

	if (IsEnd())
		BeNormal();
	UpdateInformation();
	RETURN: OutputPWM();
}

//============================================================================================
