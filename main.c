/* INFORMATIONEN
 * GPIOS wurden konfiguriert
 * Can fehlen noch einige informationen und Interrupts
 * Timer fehlen noch die genauen frequenzen und die Berechnungen der Tastverhältnisse
 * Es fehlt noch das freischalten der buffer gates
 * es fehlt noch der gearcut
 *
 * Berechnung der Kupplungssollwerte fehlt noch
 * EEProm konfiguration fehlt noch
 *
 *
 * Version: 0.1
 * Test
 *
 *
*/
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_adc.h"
//#include "stm32f4xx_can.h"
#include "stm32f4xx_dma.h"
//#include "stm32f4xx_iwdg.h"
//#include "stm32f4xx_flash.h"
//#include "stm32_ub_ee_flash.h"

enum { Standardzustand, Upshift, Downshift, Korrekturmodus, Kalibriermodus, Neutralgang_request, Upshift_delay, Downshift_delay, Neutralgang, Automatikmodus_request, Automatikmodus};

#define t_shift_delay 400
#define t_shift_cooldown 300
#define t_neutralgang_request 1000
#define t_downshift_clutch 1
#define Position_Upshift 84
#define Position_Downshift 16
#define Position_neutral 69
#define Position_home 50
#define Position_Clutch_Downshift 50

void stateMashine(void);
//void kalibrieren(void);
//void Automatik(void);
void GPIOinit(void);
void Timinit(void);
void setPWM(int pulse, int channel);
//void CANinit(void);
//void TransmitData(void);
void Analoginit(void);
void Werteladen(void);



int static state= Standardzustand;

int static B_up=1, B_down=1, // Taster am Lenkrad sind schließer
		gear=3,  gear_max=6, gear_min=1, k=0,
		K_requ=0,	// Kupplungspedalwert
		K_soll,		// Kupplungssollwert
		B_neu=0,	// Neutralgang
		Sh_soll=50,	// Schaltservo sollwert
		i=0, B_cal=0, B_aut=0, gear_old=0, z=0;

volatile uint16_t ADCConvertedValues[1];

int main(void)
{
	SystemInit();
	GPIOinit();
	Timinit();

	setPWM(50,2);
	setPWM(50,1);	// Servos auf Homeposition setzen



	Analoginit();
//	CANinit();



	GPIO_SetBits(GPIOB, GPIO_Pin_11);	// Gearcut

	GPIO_ResetBits(GPIOA, GPIO_Pin_2);	//Das Buffer Gate freigeben
	GPIO_ResetBits(GPIOA, GPIO_Pin_9);

	GPIO_SetBits(GPIOB, GPIO_Pin_11); // Gearcut einschalten

/*	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_4);
	IWDG_SetReload(4095);
	IWDG_Enable();
*/

	SysTick_Config(SystemCoreClock/1000); // 0,001s


	/*	ErrorStatus check;
	int32_t ee_wert;

	check=UB_EE_FLASH_Init();
*/

	int test5;

	RCC_ClocksTypeDef clocks;
	    	RCC_GetClocksFreq(&clocks);

    while(1)
    {


    	/*Testlauf der Kupplung
    	 * Ruhelage 50
    	 * Kupplung komplett gezogen 10
    	 *
    	 * Kupplungspedal komplett gezogen	3200
    	 * Kupplungspedal ruhelage			3600
    	 *
    	 */




    	/* Testlauf der Schaltung
    	 * 50 = Ruhelage
    	 * 69 = Neutralgang
    	 * 77 = Hochschalten
    	 * 22 = Runterschalten
    	 *
    	 * PB14 = Hochschalten
    	 * PB15 = Herunterschalten
    	 *
    	 * PC6 = Trigger Rot
    	 * PC7 = Neutralgang Grün
    	 */

    	}


}



void stateMashine (void){
/*Evaluiert welche Aktionen ausgeführt werden dürfen
 *
 */
switch( state ) {
    // Zustand 1
	case Standardzustand:



	if(B_up == 0 ) {
		state = Upshift;
	}
	if(B_down == 0) {
		state = Downshift;
	}
/*	if(k=1){	//Korrekturbedingung
		state=Korrekturmodus;
	}
	*/
	if(B_neu==1){
		state = Neutralgang_request;
	}
	break;
	//Zustand 1 Ende

	//Zustand 2
	case Upshift:
		i++;
		Sh_soll = Position_Upshift;
		GPIO_ResetBits(GPIOB, GPIO_Pin_11);	// Gearcut

		if (i<3 && B_down ==0){				// Fragt ab ob, während der Taster für Hochschalten noch gedrückt ist auch gleichzeitig Herunterschalten gedrückt ist.
			Sh_soll = Position_home;		// Soll denn fall des abziehens des Lenkrads absichern
			GPIO_SetBits(GPIOB, GPIO_Pin_11);
			i=0;
			state = Standardzustand;
		}

	if(i>t_shift_delay){
		state = Upshift_delay;
		GPIO_SetBits(GPIOB, GPIO_Pin_11);
		i=0;
	}

	break;
	// Zustand 2 Ende

	// Zustand 3
	case Downshift:
		i++;
		Sh_soll = Position_Downshift;
		//GPIO_ResetBits(GPIOB, GPIO_Pin_11);	// Gearcut

/*		if(i< t_downshift_clutch){
			K_soll = Position_Clutch_Downshift;
		}
*/
		if (i<2 && B_up ==0){				// Fragt ab ob, während der Taster für Hochschalten noch gedrückt ist auch gleichzeitig Herunterschalten gedrückt ist.
			Sh_soll = Position_home;		// Soll denn fall des abziehens des Lenkrads absichern
		//	GPIO_SetBits(GPIOB, GPIO_Pin_11);
			i=0;
			state = Standardzustand;
		}

	if(i>t_shift_delay){
		state = Downshift_delay;
		// GPIO_SetBits(GPIOB, GPIO_Pin_11);
		i=0;
	}
	break;
	// Zustand 3 Ende

	// Zustand 4
	case Upshift_delay:
		i++;
		Sh_soll = Position_home;
/*	if(B_down == 1 && i>15){
		state = Downshift;
		i=0;
	}
	*/
/*	if (i>10 && gear == gear_old ){
		state = Upshift;
		i=0;
	}
	*/
	if (i>t_shift_cooldown){
		state = Standardzustand;
		i=0;
	}
/*	if(k=1){	//Korrekturbedingung
		state=Korrekturmodus;
	}
	*/
	break;
	// Zustand 4 Ende

	// Zustand 5
	case Downshift_delay:
		i++;
		Sh_soll = Position_home;
/*	if(B_up==1 && i>15){
		state = Upshift;
		i=0;
	}
	*/
/*		if (i>10 && gear == gear_old ){
		state = Downshift;
		i=0;
	}
	*/
	if (i>t_shift_cooldown){
		state = Standardzustand;
		i=0;
	}
	break;
	// Zustand 5 Ende

	// Zustand 6
	case Korrekturmodus:
//	K_soll = K_soll - 5 *i/10;

	state= Standardzustand;


	break;
	// Zustand 6 Ende

	// Zustand 7
	case Kalibriermodus:

	//kalibrieren();
	//K_soll = 100;	// Kupplung geschlossen lassen

	/*if(B_cal==0){
	state = Neutralgang;
	}
	*/
		state = Standardzustand;
	break;
	// Zustand 7 Ende

	// Zustand 8
	case Neutralgang_request:
	i++;
	if (B_neu==0){
		state = Standardzustand;
		i=0;
	}
	if (i>t_neutralgang_request){
		state = Neutralgang;
		i = 0;
	}

	break;
	// Zustand 8 Ende

	// Zustand 9
	case Neutralgang:
	i++;
	if ( i< t_shift_delay){
		Sh_soll = Position_neutral;
	}
	else{
		Sh_soll = Position_home;
	}

/*	if (B_cal ==1){
		state = Kalibriermodus;
	}
	*/
	if (B_up==0){
		state = Upshift;
		i=3;
	}
	if (B_down==0){
		state = Downshift;
		i=3;
	}
	if(B_neu==1){
		state = Neutralgang_request;
	}
	break;
/*	if(B_aut ==1){
		state = Automatikmodus_request;
	}
	*/
	// Zustand 9 Ende

	// Zustand 10
	case Automatikmodus_request:
	i++;
/*	if(B_aut==0){
		state = Neutralgang;
		i=0;
	}
	if (i<100){
		state = Automatikmodus;
		i=0;
	}
	*/
	break;
	state = Standardzustand;
	// Zustand 10 Ende

	// Zustand 11
	case Automatikmodus:
	//Automatik();
				// Abfrage die den Automatikmodus verlässt (vielleicht geschwindigkeit)
	state = Standardzustand;
	break;
	// Zustand 11 Ende
  }
}

/*void kalibrieren(void){
/*Hier kommt der Code zum calibrieren via Can rein
 *

}
*/

/*void Automatik(void){
/*Hier kommt der Code für den Automatikmodus rein
 *

}
*/

void GPIOinit(void){
	/*Initialisiert alle GPIO
	 *
	 */
	GPIO_InitTypeDef GPIO_InitDef;

	// PWM
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);



	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);

    GPIO_InitDef.GPIO_Pin =  GPIO_Pin_8| GPIO_Pin_1;
    GPIO_InitDef.GPIO_OType = GPIO_OType_PP;
    GPIO_InitDef.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitDef.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitDef.GPIO_Speed = GPIO_Speed_100MHz;

    GPIO_Init(GPIOA, &GPIO_InitDef);




    // PWM Ende

	// SH_Ctrl
    GPIO_InitDef.GPIO_Pin =  GPIO_Pin_2;
    GPIO_InitDef.GPIO_OType = GPIO_OType_PP;
    GPIO_InitDef.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitDef.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitDef.GPIO_Speed = GPIO_Speed_100MHz;

    GPIO_Init(GPIOA, &GPIO_InitDef);
    // SH_Ctrl Ende

	// Clutch_Ctrl
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitDef.GPIO_Pin =  GPIO_Pin_9;
    GPIO_InitDef.GPIO_OType = GPIO_OType_PP;
    GPIO_InitDef.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitDef.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitDef.GPIO_Speed = GPIO_Speed_100MHz;

    GPIO_Init(GPIOC, &GPIO_InitDef);
    // Clutch_Ctrl Ende

	// Digital_Out
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitDef.GPIO_Pin =  GPIO_Pin_11; // PB10 ist der andere Digitalausgang
    GPIO_InitDef.GPIO_OType = GPIO_OType_PP;
    GPIO_InitDef.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitDef.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitDef.GPIO_Speed = GPIO_Speed_100MHz;

    GPIO_Init(GPIOB, &GPIO_InitDef);
    // Digital_Out Ende


	// Analog_In
    GPIO_InitDef.GPIO_Pin =  GPIO_Pin_7; // PA4 - 6 sind die anderen Analog Eingänge
    GPIO_InitDef.GPIO_OType = GPIO_OType_PP;
    GPIO_InitDef.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitDef.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitDef.GPIO_Speed = GPIO_Speed_100MHz;

    GPIO_Init(GPIOA, &GPIO_InitDef);
    // Analog_In Ende

	// Digital_In 3&4 setze ich zu Shift UP = 14 und Shift Down = 15
    GPIO_InitDef.GPIO_Pin =  GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitDef.GPIO_OType = GPIO_OType_PP;
    GPIO_InitDef.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitDef.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitDef.GPIO_Speed = GPIO_Speed_100MHz;

    GPIO_Init(GPIOB, &GPIO_InitDef);
    // Digital_In 3&4 Ende

	// Digital_In 1&2
    GPIO_InitDef.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitDef.GPIO_OType = GPIO_OType_PP;
    GPIO_InitDef.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitDef.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitDef.GPIO_Speed = GPIO_Speed_100MHz;

    GPIO_Init(GPIOC, &GPIO_InitDef);
    // Digital_In 1&2 Ende

	// Can
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);

    GPIO_InitDef.GPIO_Pin =  GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitDef.GPIO_OType = GPIO_OType_PP;
    GPIO_InitDef.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitDef.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitDef.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOA, &GPIO_InitDef);
    // Can Ende

}

void Timinit(void){
	/* Initialisiert Timer 1 und 2
	 * werden für die PWM Signale gebraucht
	*/

	TIM_TimeBaseInitTypeDef TIM_TimeBase_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);

	TIM_TimeBase_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBase_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBase_InitStructure.TIM_Period = 41995;
	TIM_TimeBase_InitStructure.TIM_Prescaler = 3;
	TIM_TimeBase_InitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBase_InitStructure);

	TIM_Cmd(TIM5, ENABLE);



	TIM_TimeBase_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBase_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBase_InitStructure.TIM_Period = 41995;
	TIM_TimeBase_InitStructure.TIM_Prescaler = 7;
	TIM_TimeBase_InitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBase_InitStructure);

	TIM_Cmd(TIM1, ENABLE);

}

void setPWM(int duty, int channel){
	/*Channel 1 = Clutch
	 *Channel 2 = Shifting
	 *duty ist das Tastverhältnis
	*/
	TIM_OCInitTypeDef TIM_OC_InitStructure;

	if (duty>97){
		duty = 97;
	}

	if (duty<3){
		duty =3;
	}

	int pulse;


	pulse = ((41995 + 1) * duty) / 100 - 1;


	TIM_OC_InitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OC_InitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OC_InitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC_InitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC_InitStructure.TIM_Pulse = pulse;



	if(channel == 1){
		TIM_OC1Init(TIM1, &TIM_OC_InitStructure);
		TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
		TIM_CtrlPWMOutputs(TIM1, ENABLE);

	}
	if(channel == 2){
		TIM_OC2Init(TIM5, &TIM_OC_InitStructure);
		TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);

	}

}

void SysTick_Handler(void){

	int test;

	z++;

	Werteladen();
	stateMashine();
	setPWM(K_soll,1);
	setPWM(Sh_soll,2);
//	IWDG_ReloadCounter();


/*	if (z>50){


		TransmitData();
		test = CAN_TransmitStatus(CAN1,0);
		z=0;
	}
*/





}
/*
void CANinit(void){
	/*Initialisiert den CAN1
	 *Code wurde noch nie getestet
	 *Empfangsfilter fehlen noch


	CAN_InitTypeDef CAN_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);


	CAN_InitStructure.CAN_Prescaler = 2;
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS1_14tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_6tq;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = DISABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_NART = ENABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;
	CAN_Init(CAN1, &CAN_InitStructure);


	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	CAN_FilterInitStructure.CAN_FilterNumber = 0;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x777;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x777;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);

}

*/
	
/*
void TransmitData(void){
	/*Sendet alle daten die zyklisch übergeben werden
	 *




	int status=0;

	CanTxMsg canMessage;

	canMessage.StdId = 0x601;
	canMessage.ExtId = 0;
	canMessage.RTR = CAN_RTR_DATA;
	canMessage.IDE = CAN_ID_STD;
	canMessage.DLC = 8;

/*	if (B_down==0){
		status = status & 1<<0;
	}

	if (B_up==0){
		status = status & 1<<1;
	}

	if (B_neu==1){
		status = status & 1<<2;
	}
	if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11)==1){

		status = status & 1<<3;
	}


	canMessage.Data[0] = 1;
	canMessage.Data[1] = 2;
	canMessage.Data[2] = 3;
	canMessage.Data[3] = 4;
	canMessage.Data[4] = 4;
	canMessage.Data[5] = 5;
	canMessage.Data[6] = 6;
	canMessage.Data[7] = 7;

 	CAN_Transmit(CAN1, &canMessage);

}

*/
void Analoginit(void){
	/*Initialisiert die Analod-Digital Wandler
	 *
	 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	ADC_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	DMA_InitTypeDef       DMA_InitStructure;

	DMA_DeInit(DMA2_Stream0);
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADCConvertedValues[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);

	DMA_Cmd(DMA2_Stream0, ENABLE);

	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	//ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_480Cycles);
	//ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_480Cycles);
	//ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 1, ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_480Cycles);

	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

	ADC_DMACmd(ADC1, ENABLE);

	ADC_Cmd(ADC1, ENABLE);





	ADC_SoftwareStartConv(ADC1);
}

void Werteladen(void){
	/*hier werden alle werte aus den Tastern und aus den ADC_Values in variablen geladen
	 *
	 */

	B_up = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14);
	B_down = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15);





	if ( ADCConvertedValues[0] >2800){	// checkt ob das lenkrad gezogen ist

		K_requ = (ADCConvertedValues[0] -3100 )/(36-31);	// Angeforderter winkel wird in Prozent umgerechnet
	}

	if ( K_requ < 30){
		K_requ =30;
	}
	if (K_requ> 100){

		K_requ =100;
	}


	if (B_up == 0 && B_down == 0){	// Wenn beide Taster gleichzeitig gezogen werden, passiert nichts
		B_up = 1;
		B_down = 1;
		K_requ=100;
	}


	K_soll = 10+ (K_requ * 40)/100;		// Kupplungsstellwert wird berechnet

	B_neu = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7);



}

