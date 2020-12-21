#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "misc.h"

#define SOUND_TRIG_GO              GPIO_Pin_2 
#define SOUND_ECHO_GO              GPIO_Pin_3
#define SOUND_TRIG_BACK            GPIO_Pin_4
#define SOUND_ECHO_BACK            GPIO_Pin_5
#define IR_STUFF                   GPIO_Pin_15

#define TURNING_TIME               5000000
#define TURNING_FIXTIME            750000
#define TURNING_GO                 50000
#define SAFETY                     20
#define IGNORE_TIME                1000000

enum _MotorState{
  M_Stop, 
  Go_Forward, Go_Turn_Left,Go_Tank_Left, Go_Turn_Right,Go_Tank_Right,
  Back_Forward, Back_Turn_Left, Back_Tank_Left, Back_Turn_Right, Back_Tank_Right
} MotorState;

enum _RobotState{
  Stop, Run, Arrive, Return, Done, Emergency
} RobotState;

enum _IR{
  FL,FR,BL,BR, STUFF
}IR;

enum _Flag{
  Stop_Stuff, Stop_Target,Emergency_From
}Flag;

enum _Array{
    IGNORE, LEFT, RIGHT, D_ARRIVE, S_ARRIVE
};

uint8_t IR_Array[5];
uint8_t EventFlag[10]; 

uint8_t GoArray[5][10] = {
  {0,0,0,0,0},
  {1,3,0,0,0},
  {1,0,3,0,0},
  {2,3,0,0,0},
  {2,0,3,0,0}
};

uint8_t BackArray[5][10] = {
    {0,0,0,0,0},
    {2,4,0,0,0},
    {0,2,4,0,0},
    {0,1,4,0,0},
    {0,1,4,0,0}
};

uint32_t usTime=0;
int MoveIdx = 0;
uint8_t Go_Idx = 0;
uint8_t Back_Idx = 0;
int distance=0 ;



// Configurations 
void RCC_Configure(void);
void GPIO_Configure(void);
void USART12_Init(void);
void NVIC_Configure(void);
void Time_Configure(void);


//IRQ 
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void TIM2_IRQHandler(void);

//user function

int Read_Distance_Go(void);
int Read_Distance_Back(void);
void Delay(uint32_t delayTime);
void Stay(uint32_t StayTime);
void M_StateChange(uint8_t m_state);
void IR_Sensing_Motor(void);

// debug function
void SendInt(int data);
void SendData(uint16_t data);
void Led_On(int num);
void Led_Off(int num);
void Var_Reset_All(void);