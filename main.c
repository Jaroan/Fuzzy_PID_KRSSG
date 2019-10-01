#include "stm32f4xx.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_exti.h"
#include "misc.h"


volatile int Fuzzy_Matrix[3][3];
volatile int NE,NDE,PE,PDE,ZE,ZDE,err_sum,error,d_err,Kp,Ki,Kd,prev_err;
//volatile float
volatile int PWM_Fuzzy,ticks;
volatile int target=50;
volatile int Kp_small,Kp_medium,Kp_large;
volatile int Ki_small,Ki_medium,Ki_large;
volatile int Kd_small,Kd_medium,Kd_large;
volatile int out;
const int Kp_s=1,Kp_m=2,Kp_l=3;
const int Ki_s=1,Ki_m=2,Ki_l=3;
const int Kd_s=1,Kd_m=2,Kd_l=3;

void init(){
	int i,j;
	NE=0; PDE=0; NDE=0; PE=0; ZE=0; ZDE=0;
	Kp=0; Ki=0; Kd=0;
	error=0;
	prev_err=0;
	PWM_Fuzzy=0;
	err_sum=0;
	for(i=0;i<3;i++){
		for(j=0;j<3;j++) Fuzzy_Matrix[i][j]=0;
	}
}

void swap(int* a, int* b)  
{  
    int t = *a;  
    *a = *b;  
    *b = t;  
}  

int max(int x,int y){
	return x>y?x:y;
}
int min(int x,int y){
	return x>y?y:x;
}
void Fuzzification(){
	NE=0; PDE=0; NDE=0; PE=0;
	if(error<=-20) NE=20;
	else if(error>=20) PE=20;
	else{
		if(error>=0){
			PE=error;
			NDE=20-error;
		}
		else{
			NE=-error;
			ZDE=20+error;
		}
	}
	if(d_err<=-20) NDE=20;
	else if(d_err>=20) PDE=20;
	else{
		if(d_err>=0){
			PDE=d_err;
			ZDE=20-d_err;
		}
		else{
			NDE=-d_err;
			ZDE=20+d_err;
		}
	}
}
void Create_Fuzzy_Matrix(){
	Fuzzy_Matrix[0][0]=min(NE,NDE);
	Fuzzy_Matrix[0][1]=min(NE,ZDE);
	Fuzzy_Matrix[0][2]=min(NE,PDE);
	Fuzzy_Matrix[1][0]=min(ZE,NDE);
	Fuzzy_Matrix[1][1]=min(ZE,ZDE);
	Fuzzy_Matrix[1][2]=min(ZE,PDE);
	Fuzzy_Matrix[2][0]=min(PE,NDE);
	Fuzzy_Matrix[2][1]=min(PE,ZDE);
	Fuzzy_Matrix[2][2]=min(PE,PDE);
}
void Defuzzification(){
	Kp_small=max(Fuzzy_Matrix[0][0],Fuzzy_Matrix[0][1]);
	Kp_large=max(Fuzzy_Matrix[2][1],Fuzzy_Matrix[2][2]);
	Kp_medium=max(Fuzzy_Matrix[0][1],max(Fuzzy_Matrix[1][0],max(Fuzzy_Matrix[1][1],max(Fuzzy_Matrix[1][2],Fuzzy_Matrix[2][1]))));

	Ki_small=max(Fuzzy_Matrix[0][0],Fuzzy_Matrix[0][1]);
	Ki_large=max(Fuzzy_Matrix[2][1],Fuzzy_Matrix[2][2]);
	Ki_medium=max(Fuzzy_Matrix[0][1],max(Fuzzy_Matrix[1][0],max(Fuzzy_Matrix[1][1],max(Fuzzy_Matrix[1][2],Fuzzy_Matrix[2][1]))));

	Kd_large=max(Fuzzy_Matrix[0][0],Fuzzy_Matrix[0][1]);
	Kd_small=max(Fuzzy_Matrix[2][1],Fuzzy_Matrix[2][2]);
	Kd_medium=max(Fuzzy_Matrix[0][1],max(Fuzzy_Matrix[1][0],max(Fuzzy_Matrix[1][1],max(Fuzzy_Matrix[1][2],Fuzzy_Matrix[2][1]))));


	if(Kp_small!=0 || Kp_medium!=0 || Kp_large!=0) Kp=(Kp_s*Kp_small+Kp_m*Kp_medium+Kp_l*Kp_large)/(Kp_small+Kp_medium+Kp_large);
	if(Ki_small!=0 || Ki_medium!=0 || Ki_large!=0) Ki=(Ki_s*Ki_small+Ki_m*Ki_medium+Ki_l*Ki_large)/(Ki_small+Ki_medium+Ki_large);
	if(Kd_small!=0 || Kd_medium!=0 || Kd_large!=0) Kd=(Kd_s*Kd_small+Kd_m*Kd_medium+Kd_l*Kd_large)/(Kd_small+Kd_medium+Kd_large);
}
int Compute_PWM(){
	err_sum+=error;
	out=Kp*error + Ki*err_sum + Kd*d_err;
	if(out>127) out=127;
	else if(out<0) out=0;
	prev_err=error;
	return out;
}

//int PWM_Fuzzy = 200;

void Initialise_LED()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitTypeDef GPIO_Init_Struct;
	GPIO_Init_Struct.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_Init_Struct.GPIO_OType=GPIO_OType_PP;
	GPIO_Init_Struct.GPIO_Pin=GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_Init_Struct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD,&GPIO_Init_Struct);
	GPIO_SetBits(GPIOD,GPIO_Pin_12);
}

void Initialise_UART()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0,   GPIO_AF_UART4);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1,   GPIO_AF_UART4);
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA,&GPIO_InitStruct);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	USART_InitTypeDef USART_InitStruct;
	USART_InitStruct.USART_BaudRate=250000;
	USART_InitStruct.USART_WordLength=USART_WordLength_8b ; //Change this to increase the size of data send
	USART_InitStruct.USART_StopBits=USART_StopBits_1;
	USART_InitStruct.USART_Parity=USART_Parity_No;
	USART_InitStruct.USART_Mode=USART_Mode_Rx|USART_Mode_Tx ;
	USART_InitStruct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_Init(UART4,&USART_InitStruct);

	USART_Cmd(UART4,ENABLE);
	USART_ITConfig(UART4,USART_IT_RXNE ,ENABLE);
}

void Initialise_NVIC()
{
	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel=UART4_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=0;
	NVIC_Init(&NVIC_InitStruct);
}

void UART4_IRQHandler()
{
	GPIO_SetBits(GPIOD,GPIO_Pin_13); //Pin_13_LED set during Recieving

	//Recieving Data as ticks
	ticks=USART_ReceiveData(UART4);

	GPIO_ResetBits(GPIOD,GPIO_Pin_13);

	//Call the Fuzzy Code Function Here and return the value to PWM_Fuzzy
	error=target-ticks;
	d_err=error-prev_err;
	Fuzzification();
	Create_Fuzzy_Matrix();
	Defuzzification();
	PWM_Fuzzy=Compute_PWM();
	GPIO_SetBits(GPIOD,GPIO_Pin_12); //Pin_12_LED set during Transmitting

	//Transmitting Data as Fuzzy_PWM
	while(USART_GetFlagStatus( UART4,USART_FLAG_TXE)!=SET){}
		USART_SendData(UART4, PWM_Fuzzy);

	GPIO_ResetBits(GPIOD,GPIO_Pin_12);
}

int main(void)
{
	init();
	Initialise_LED();
	Initialise_UART();
	Initialise_NVIC();

	int PWM = 60;
		//Send a PWM for the First Time

	while(USART_GetFlagStatus( UART4,USART_FLAG_TXE)!=SET){}
		USART_SendData(UART4, PWM);
		//Subsequent Transmission will be initialise when any data is recieved in the IRQ_Handler();

    while(1)
    {

    	//Send PWM;
    }
}
