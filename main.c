// �Ҽ� : ������б���� �޸ӳ��̵�κ� ����2��
// �̸� : ������
// ��¥ : 2018-07-24
// �뵵 : ��庰 �˰��� ���赥����
#include "DSP28x_Project.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "string.h"
// pragma CODE_SECTION -> Flash���� RAM���� �Լ��� �ű�� ���� �۾� .
#pragma CODE_SECTION(Encoder_define,"ramfuncs")
#pragma CODE_SECTION(Uart_transmit,"ramfuncs")
#pragma CODE_SECTION(UART_Put_String,"ramfuncs")
#pragma CODE_SECTION(Motor_transmit,"ramfuncs")
#pragma CODE_SECTION(Motor_Put_String,"ramfuncs")
#pragma CODE_SECTION(Motor_Put_Char,"ramfuncs")
#pragma CODE_SECTION(BT_transmit,"ramfuncs")
#pragma CODE_SECTION(TrainAbnormalPerson,"ramfuncs")

extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;

// CPU timer0 ����
interrupt void cpu_timer0_isr(void);
float32 Cpu_Clk;
float32 Timer_Prd;
Uint16 i = 0, M_i = 0, E_i = 0;

#define SYSCLK      150E6   /* 150MHz */
#define TBCLK       150E6   /* 150MHz */
#define PWMCARRIER  20E3    /* 20kHz */

// Encoder ���� ���� , �Լ� ����
Uint32 Encoder[10], Encoder_sum = 0, Encoder_cnt = 0, E_90_cnt = 0;
double Encoder_deg_new = 0;
double Encoder_deg_old = 0;
double Encoder_deg = 0;
Uint32 Encoder_revcnt = 0;
double Encoder_vel = 0;
double E_vel_deg_new = 0;
double E_vel_deg_old = 0;
double EV_Buff[30];
double EV_mva = 0;
double EV_mva_old = 0;
void Encoder_define();

// @@@@@@@�������� ��������@@@@@@@@
int break_timer = 0;
float break_duty = 0;
double move_dis = 0;
int a = 0;
unsigned int mode_num = 0;
unsigned int leg_num = 0;
unsigned int start_bit = 0;
unsigned int end_bit = 0;
float Motor_Pwm = 0;

double target_dis = 0;
char target_dis_1 = 0;
char target_dis_2 = 0;
double training_timer = 0;

double target_time = 0;
double target_hour = 0;
double target_min = 0;
double target_sec = 0;
double time_now = 0;

unsigned int Type_sel = 0;
double target_gain = 0;
double ratio_gain = 0;
unsigned int pause_bit = 0;
double velocity = 0;
double under_velocity = 0;

int time_now_hour = 0;
int time_now_min = 0;
int time_now_min_10 = 0;
int time_now_min_1 = 0;
int move_distance_4 = 0;
int move_distance_3 = 0;
int move_distance_2 = 0;
int move_distance_1 = 0;

double smooth_rise = 0;

double a0 = 0.7086;
double a1 = 0.04348;
double b1 = 0.04126;
double a2 = -0.03393;
double b2 = -0.01856;
double a3 = 0.01754;
double b3 = 0.0006721;
double a4 = -0.01032;
double b4 = 0.003381;
double a5 = 0.004178;
double b5 = -0.003621;
double a6 = -0.001026;
double b6 = 0.002338;
double a7 = -0.0004222;
double b7 = -0.0008692;
double a8 = 0.0005095;
double b8 = 0.0002923;
double w = 0.03604;
double CPM_assist = 0;
int pause_finish = 0;
int init_bit = 0;
double vel_gain = 0.05;//0.008
double acc_gain = 0.0001;//0.002
double EA_mva = 0;
double acc_term=0;
double vel_acc_gain=0;
double Gait_score_degree=0;
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

//�������� �Լ�------------------
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
void Type_Check_fun();
void Reword_inflection_point();
void Start_breaking();
void Reg_setting_fun();
//void Initialize_main_fun();
void Encoder_position_renew();
void Encoder_value_calculation();
void Gait_score_calculation();
//------------------------

// PWM Duty ���� ����, �Լ� ����
float32 PwmCarrier;
float32 PwmDuty, BrakeDuty = 0;
void InitEPwm1Module(void);
void InitEPwm2Module(void);

// ��� ������ ���� �Լ�
void scia_echoback_init(void);
void scia_fifo_init(void);
void scib_echoback_init(void);
void scib_fifo_init(void);
void scic_echoback_init(void);
void scic_fifo_init(void);

// App ����
void BT_transmit();
void BT_Put_String(char *BT_string);

// Data ������ ���� �Լ�
void Uart_transmit();
void UART_Put_String(char *Uart_string);

// Motor�� LCD ���� ���۽�Ű�� ���� ���
void Motor_transmit();
void Motor_Put_String(char *Motor_string);
void Motor_Put_Char(unsigned char Uart_string);

// ��� ���� ����.
Uint16 TimerCount = 0, MotorCount = 0, TimerCount_2 = 0;
char UT1[100], MT1[50], BT1[50];
unsigned char Motor_char[15], Motor_char_s[10];
Uint16 Mt_cnt = 0, m = 0, c = 0;

interrupt void sciaRxFifoIsr(void);
char RxBuff[16];
char Receivedbuff;

// @@@@@@@@@@@@@@@@@@@@@@@Main �Լ� @@@@@@@@@@@@@@@@@@@@@@@
void main(void) {
	// Step 1. Disable Global Interrupt
	DINT;

	// Step 2. �ý��� ��Ʈ�� �ʱ�ȭ:
	InitSysCtrl();

	// FLASH ������ �Լ��� ���� �ӵ��� ���� RAM���� ������Ű�� ���� ������ �Լ�
	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
	InitFlash();

	// Step 3. ���ͷ�Ʈ �ʱ�ȭ:
	InitPieCtrl();
	IER = 0x0000;
	IFR = 0x0000;
	InitPieVectTable();

	// GPIO Pin�� ���� ����� ����ϱ� ���� ���ġ
	InitSciaGpio();
	InitScibGpio();
	InitScicGpio();
	InitAdc();
	InitEPwm1Gpio();

	// Vector table�� ���� ����ϱ� ���� ������� ��ġ
	Reg_setting_fun();

	// PWM �ʱ�ȭ �Լ�
	InitEPwm1Module();
	InitEPwm2Module();

	// CPU Timer �ʱ�ȭ
	InitCpuTimers();
	Cpu_Clk = 150;          // ���� �ý��� Ŭ���� ���� (MHz ����)
	Timer_Prd = 5000;      // Ÿ�̸� �ֱ� ���� (usec ����) // 200 Hz -> 5000
	ConfigCpuTimer(&CpuTimer0, Cpu_Clk, Timer_Prd);

	// CPU Timer0 ����
	StartCpuTimer0();

	// CPU Timer0 ���ͷ�Ʈ Ȱ��ȭ
	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;         // PIE ���ͷ�Ʈ(TINT0) Ȱ��ȭ
	PieCtrlRegs.PIEIER9.bit.INTx1 = 1;		  // SCIRXB
	IER = IER | M_INT1 | M_INT9;              // CPU ���ͷ�Ʈ(INT1), SCIRXB  Ȱ��ȭ

	// ����Լ� �ʱ�ȭ
	scia_fifo_init();      // Initialize the SCI FIFO
	scia_echoback_init();  // Initalize SCI for echoback
	scib_fifo_init();      // Initialize the SCI FIFO
	scib_echoback_init();  // Initalize SCI for echoback
	scic_fifo_init();      // Initialize the SCI FIFO
	scic_echoback_init();  // Initalize SCI for echoback

	// ADC ����
	/*
	 AdcRegs.ADCTRL3.bit.ADCCLKPS = 3;    // ADCCLK = HSPCLK/(ADCCLKPS*2)/(CPS+1)
	 AdcRegs.ADCTRL1.bit.CPS = 1;         // ADCCLK = 75MHz/(3*2)/(1+1) = 6.25MHz
	 AdcRegs.ADCTRL1.bit.ACQ_PS = 3;     // ����/Ȧ�� ����Ŭ = ACQ_PS + 1 = 4 (ADCCLK����)
	 AdcRegs.ADCTRL1.bit.SEQ_CASC = 1; // ������ ��� ����: ���� ������ ��� (0:���� ���, 1:���� ���)
	 AdcRegs.ADCMAXCONV.bit.MAX_CONV1 = 8;  // ADC ä�μ� ����: 1��(=MAX_CONV+1)ä���� ADC

	 AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0;      // ADC ���� ����: ù��°�� ADCINA2 ä���� ADC
	 AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 1;
	 AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 2;
	 AdcRegs.ADCCHSELSEQ1.bit.CONV03 = 3;
	 AdcRegs.ADCCHSELSEQ2.bit.CONV04 = 4;
	 AdcRegs.ADCCHSELSEQ2.bit.CONV05 = 5;
	 AdcRegs.ADCCHSELSEQ2.bit.CONV06 = 6;
	 AdcRegs.ADCCHSELSEQ2.bit.CONV07 = 7;
	 AdcRegs.ADCTRL2.bit.EPWM_SOCB_SEQ = 1;     // ePWM_SOCB�� ADC ������ �õ�
	 AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;      // ADC ������ �Ϸ�� ���ͷ�Ʈ �߻� ����
	 */
	//ePWM_SOCB �̺�Ʈ Ʈ���� ����
	EPwm3Regs.ETSEL.bit.SOCBEN = 1;            // SOCB �̺�Ʈ Ʈ���� Enable
	EPwm3Regs.ETSEL.bit.SOCBSEL = 2;           // SCCB Ʈ���� ���� : ī���� �ֱ� ��ġ ��
	EPwm3Regs.ETPS.bit.SOCBPRD = 1;            // SOCB �̺�Ʈ ���� ���� : Ʈ���� ���� �ѹ� ����
	EPwm3Regs.TBCTL.bit.CTRMODE = 0;           // ī��Ʈ ��� ����: Up-conut ���
	EPwm3Regs.TBCTL.bit.HSPCLKDIV = 1; // TBCLK = [SYSCLKOUT / ((HSPCLKDIV*2) * 2^(CLKDIV))]
	EPwm3Regs.TBCTL.bit.CLKDIV = 1;        // TBCLK = [150MHz / (2*2)] = 37.5MHz
	EPwm3Regs.TBPRD = 1874; // TB�ֱ�= (TBPRD+1)/TBCLK = 1875/37.5MHz = 50us(20KHz)
	EPwm3Regs.TBCTR = 0x0000;                  // TB ī���� �ʱ�ȭ

	//���� ����
	for (i = 0; i < 10; i++)
		EV_Buff[i] = 0;

	for (i = 0; i < 30; i++)
		EV_Buff[i] = 0;

	for (i = 0; i < 16; i++)
		RxBuff[i] = 0;

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

// �����Լ� ��.
/*
 void Initialize_main_fun() {

 }
 */
//�����Լ�-----------------------------------------------------------------------------------------
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

	// PWM 1B�� ����ϱ� ���� GPIO1�� Pull-up ��Ŵ
	GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;
	GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;

	// PWM 1B�� ����ϱ� ���� MUX Pin ��ġ
	GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;
	GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;

	EDIS;
}

int Robot_Initialize() {
	//ȯ���ٸ�=�������̸�
	if (leg_num == 1) {
		if (!init_bit) {
			break_duty = 0.63;	//�극��ũ OFF
			Motor_Pwm = 0.55;	//���� �����ӵ�

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
	//�޹��̸�
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
	leg_num = 0;	//�ٸ�����
	start_bit = 0;	//���ۺ�Ʈ
	training_timer = 0;	//Ÿ�̸�0
	target_sec = 0;	//��ǥ��
	move_dis = 0;	//�̵��Ÿ�
	target_time = 0;	//��ǥ�ð�
	time_now = 0;	//����ð�
	pause_bit = 0;	//�Ͻ�������Ʈ
	target_gain = 0;	//��ǥ����
	end_bit = 0;	//�����Ʈ
	break_timer = 0;	//
	ratio_gain = 0;
	mode_num = 0;
	Type_sel = 0;
	target_dis = 0;
	E_vel_deg_new = 0;
	move_distance_1 = 0;
	move_distance_2 = 0;
	move_distance_3 = 0;
	move_distance_4 = 0;
	time_now_hour = 0;
	time_now_min_10 = 0;
	time_now_min = 0;
	Encoder_revcnt = 0;
	velocity = 0;
	EV_mva = 0;
	RxBuff[0] = 0;
	RxBuff[1] = 0;
	RxBuff[2] = 0;
	RxBuff[3] = 0;
	RxBuff[4] = 0;
	RxBuff[5] = 0;
	RxBuff[6] = 0;
	RxBuff[8] = 0;
	RxBuff[7] = 0;
	RxBuff[9] = 0;
	RxBuff[10] = 0;
	RxBuff[11] = 0;
	smooth_rise = 0;
	CPM_assist = 0;

	for (i = 0; i < 30; i++) {
		EV_Buff[i] = 0;
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
	SciaRegs.SCICTL1.bit.RXENA = 1;    // SCI �۽ű�� Enable
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
	ScibRegs.SCICTL1.bit.RXENA = 1;    // SCI �۽ű�� Enable
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

	sprintf(BT1, "!s%d.%dt%d%d%dd%d.%d%d?\n\0", (int) velocity,
			(int) under_velocity, time_now_hour, time_now_min_10,
			time_now_min_1, move_distance_1, move_distance_2, move_distance_3);
	//�ð�����
	if (Type_sel == 2) {
		if (time_now > target_sec)
			sprintf(BT1, "!e?");
	}
	//�Ÿ�����
	else if (Type_sel) {
		if (move_dis > target_dis)
			sprintf(BT1, "!e?");
	}

	BT_Put_String(BT1);
}

void Uart_transmit() {
	//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@�׽�Ʈ�� �ִ� �ڵ�@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@//
	sprintf(UT1, "%ld,%ld,%ld\n\0", (long) (Motor_Pwm * 10000),	(long) (Encoder_deg_new * 100), (long) (EV_mva * 10000));
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@��Ȱġ��� �ִ� �ڵ�@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@//
	/*	sprintf(UT1,"!s%d.%dt%d%d%dd%d.%d%d?\n\0",(int)velocity,(int)under_velocity,time_now_hour,time_now_min_10,time_now_min_1,  move_distance_1, move_distance_2,move_distance_3);

	 if(Type_sel==2)//�ð�����
	 {
	 if(time_now>target_sec)
	 {
	 sprintf(UT1,"!e?");
	 }
	 }
	 else if(Type_sel==1)//�Ÿ�����
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

// LCD ���� ������� ���� �����������
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
		Encoder_sum += Encoder[Encoder_cnt] << Encoder_cnt; // Encoder_sum �� 0-1024 Pulse������ ���� Count����.
	}

	Encoder_deg_new = 360 - (double) Encoder_sum * 0.3515625; // Encoder�� ����. 1024 Pulse�� 0 - 360 deg�� �ٲ���.
	//Encoder_deg_new =360-Encoder_deg_new;
	if (Encoder_deg_old - Encoder_deg_new >= 200) // ���ӵ� ���� �� ���ڱ� 100���̻� ���̳��� 360 -> 0 ���� �Ȱ��� �˾Ƴ��� ����
		Encoder_revcnt++; // ȸ���� üũ

	E_vel_deg_new = Encoder_revcnt * 360 + Encoder_deg_new;
	move_dis = 0.000001 * E_vel_deg_new * 880 / 360; //�� ȸ������*���Ÿ�?
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
	Encoder_deg_old = Encoder_deg_new; // ���� Encoder���� ����
	E_vel_deg_old = E_vel_deg_new;
	EV_mva_old = EV_mva;

	// Encoder Digital Input �� �ޱ�
	Encoder_position_renew();
	Encoder_value_calculation();
	Moving_avg_degree();

}

interrupt void sciaRxFifoIsr(void) {

	Receivedbuff = SciaRegs.SCIRXBUF.bit.RXDT;

	if (a == 0) {
		if (Receivedbuff == '!') {
			RxBuff[a] = Receivedbuff;
			a++;
		} else {
			RxBuff[6] = 0;
			a = 0;
		}
	} else if (a == 1) {
		RxBuff[a] = Receivedbuff;
		a++;
	} else if (a == 2) {
		RxBuff[a] = Receivedbuff;
		a++;
	} else if (a == 3) {
		RxBuff[a] = Receivedbuff;
		a++;
	} else if (a == 4) {
		RxBuff[a] = Receivedbuff;
		a++;
	} else if (a == 5) {
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
	} else if (RxBuff[0] == '!' && RxBuff[1] == 'P' && RxBuff[2] == '1'
			&& RxBuff[3] == '?') {
		pause_bit = 1;
		RxBuff[6] = 0;
	} else if (RxBuff[0] == '!' && RxBuff[1] == 'P' && RxBuff[2] == '2'
			&& RxBuff[3] == '?') {
		pause_bit = 0;
		pause_finish = 0;
		RxBuff[6] = 0;
		pause_finish = 0;
	} else if (RxBuff[0] == '!' && RxBuff[1] == 'M' && RxBuff[2] == '1'
			&& RxBuff[3] == '?') {
		mode_num = 1;
		RxBuff[6] = 0;
	} else if (RxBuff[0] == '!' && RxBuff[1] == 'M' && RxBuff[2] == '2'
			&& RxBuff[3] == '?') {
		target_gain = 0;
		mode_num = 2;
		RxBuff[6] = 0;
	} else if (RxBuff[0] == '!' && RxBuff[1] == 'M' && RxBuff[2] == '3'
			&& RxBuff[3] == '?') {
		mode_num = 3;
		RxBuff[6] = 0;
	}

	else if (RxBuff[0] == '!' && RxBuff[1] == 'H' && RxBuff[2] == 'R'
			&& RxBuff[3] == '?') {
		leg_num = 1;
		RxBuff[6] = 0;
	} else if (RxBuff[0] == '!' && RxBuff[1] == 'H' && RxBuff[2] == 'L'
			&& RxBuff[3] == '?') {
		leg_num = 2;
		RxBuff[6] = 0;
	}

	else if (RxBuff[0] == '!' && RxBuff[1] == 'D' && RxBuff[5] == '?') {
		target_dis = atof(&RxBuff[2]);
		target_dis = target_dis * 0.01;

		Type_sel = 1;
		RxBuff[6] = 0;
	} else if (RxBuff[0] == '!' && RxBuff[1] == 'T' && RxBuff[5] == '?') {
		target_time = atof(&RxBuff[2]);
		target_time = target_time / 100;

		target_hour = (int) target_time;
		target_min = (int) (target_time * 100 + 0.5) - target_hour * 100;
		target_sec = target_hour * 3600 + target_min * 60;

		Type_sel = 2;
		RxBuff[6] = 0;
	} else if (RxBuff[0] == '!' && RxBuff[1] == 'G' && RxBuff[4] == '?') {
		if (mode_num != 2) {
			target_gain = atof(&RxBuff[2]);

			RxBuff[6] = 0;
		} else {
			RxBuff[6] = 0;
		}
	} else if (RxBuff[0] == '!' && RxBuff[1] == 'R' && RxBuff[4] == '?') {
		if (mode_num == 3) {
			ratio_gain = atof(&RxBuff[2]);
			ratio_gain = ratio_gain * 0.1;
			RxBuff[6] = 0;
		} else {
			RxBuff[6] = 0;
		}
	} else if (RxBuff[0] == '!' && RxBuff[1] == 'K' && RxBuff[4] == '?') {
		ratio_gain = atof(&RxBuff[2]);
		ratio_gain = ratio_gain * 0.1;
		RxBuff[6] = 0;
	} else {
		RxBuff[6] = 0;
	}

	SciaRegs.SCIFFRX.bit.RXFFOVRCLR = 1;			// Clear Overflow flag
	SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;			// Clear Interrupt flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;		// Acknowledge interrupt to PIE

}

void MetabolizeRehabilitationRobot() {

	++TimerCount; //40 -> 2
	++TimerCount_2;
	if (!Mt_cnt)
		++MotorCount;

	//�ӵ�, ��ũ�� �Ŀ���Ȯ��
	if (TimerCount_2 == 2) {
		TimerCount_2 = 0;
		Encoder_define();
		if (start_bit && !end_bit) {
			Uart_transmit();
			BT_transmit(); //�׽�Ʈ�� ����
		}
	}
	/*
	 // MATLAB 2 -> 100Hz Bluetooth 40 -> 5Hz
	 if (TimerCount == 20) {
	 // PC�� ������ �����ϱ� ���� �Լ�
	 TimerCount = 0;
	 if (start_bit && (!end_bit)) {
	 //BT_transmit();
	 }
	 }
	 */
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

// Brake�� Duty�� �����ϴ� �Լ�. Brake Duty�� 0.0 ~ 1.0 ���̾�� ��.
	EPwm1Regs.TBPRD = (150E6 / 20E3) - 1;
	EPwm1Regs.CMPB = EPwm1Regs.TBPRD * break_duty;

// Motor�� Duty�� �����ϴ� �Լ�. Motor Duty�� 0.0 ~ 1.0 ���̾�� ��.
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
	// �Ʒýð� Ȯ�� �˷��ִ°�
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
		Motor_Pwm = (1 + target_gain) * (CPM_assist - 0.5985) + 0.5985;
		Type_Check_fun();
		break;
	case 2:
		Start_breaking();
		Reword_inflection_point();
		Motor_Pwm = EV_mva * 40 * smooth_rise;
		Type_Check_fun();
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
		vel_acc_gain=(1+vel_gain * (EV_mva - 1600 * (CPM_assist - 0.5985))	+ acc_gain * (EA_mva - 10000*acc_term));
		if(vel_acc_gain<1)
			vel_acc_gain=1;
		Motor_Pwm = vel_acc_gain*(1 + target_gain) * (CPM_assist - 0.5985) + 0.5985 ;
		Type_Check_fun();
		break;
	}

}

void UpdateInformation() {
//�ð����� ������Ʈ
	time_now_min = time_now / 60;
	time_now_min = time_now_min % 60;
	time_now_min_10 = time_now_min / 10;
	time_now_min_1 = time_now_min - time_now_min_10 * 10;
	time_now_hour = time_now / 3600;
	time_now_hour = time_now_hour % 60;
//�Ÿ����� ������Ʈ
	move_distance_4 = move_dis * 1000;
	move_distance_4 = move_distance_4 % 1;
	move_distance_3 = move_dis * 1000 / 10;
	move_distance_3 = move_distance_3 % 10;
	move_distance_2 = move_dis * 1000 / 100;
	move_distance_2 = move_distance_2 % 100;
	move_distance_1 = move_dis;
	move_distance_1 = move_distance_1 % 1000;
//���ӵ�-->����ӵ�
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

void Type_Check_fun() {
	if (Type_sel == 1) {
		if (move_dis > target_dis) {
			end_bit = 1;
			sprintf(BT1, "!e?");
			BT_Put_String(BT1);
		}
	}

	else if (Type_sel == 2) {
		if (time_now > target_sec) {
			end_bit = 1;
			sprintf(BT1, "!e?");
			BT_Put_String(BT1);
		}
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
	if ((break_timer < 400) && (start_bit == 1))   //2�ʵ��� 1���Լ��׷����� �극��ũ ��Ƽ ���
			{
		break_timer++;
		break_duty = 0.002075 * break_timer;
	}
	if (break_timer >= 400) {
		break_timer = 401;
	}
}

void Gait_score_calculation(){
	Gait_score_degree=Encoder_deg_new-180;

	if(Gait_score_degree<0)
		Gait_score_degree=Gait_score_degree+180;



}

interrupt void cpu_timer0_isr(void) // cpu timer ���� �������ļ� 100Hz
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

	IncreaseTime();
	TrainAbnormalPerson();
	Gait_score_calculation();

	if (IsEnd())
		BeNormal();
	UpdateInformation();
	RETURN: OutputPWM();
}

//============================================================================================
