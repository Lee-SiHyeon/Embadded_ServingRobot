#include "linetracer.h"
/***********************
Configuration Setting function
*************************/

void RCC_Configure(void)
{
  /* Alternate Function IO clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
 
  /* USART1 clock enable */  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  /* USART2 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

  // TIM2 clock enable
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  /* GPIOA~E clock enable*/
  RCC_APB2PeriphClockCmd(
      RCC_APB2Periph_GPIOA  |
      RCC_APB2Periph_GPIOB  | //PB0 for ADC12_IN8
      RCC_APB2Periph_GPIOC  |
      RCC_APB2Periph_GPIOD  |
      RCC_APB2Periph_GPIOE
      , ENABLE);
  
  /* ADC1 Enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

}

void GPIO_Configure(void)
{
  GPIOA->CRL = 0xCCCCCCCC;  
  GPIOA->CRH = 0xCCCCCCCC;

  GPIOB->CRL = 0xCCCCCCCC;
  GPIOB->CRH = 0xCCCCCCCC;

  GPIOC->CRL = 0xCCCCCCCC;
  GPIOC->CRH = 0xCCCCCCCC;

  GPIOD->CRL = 0xCCCCCCCC;
  GPIOD->CRH = 0xCCCCCCCC;
  
  GPIOE->CRL = 0xCCCCCCCC;
  GPIOE->CRH = 0xCCCCCCCC;
  
    GPIO_InitTypeDef GPIO_InitStructure;

    /* UART pin setting */
    //USART1 Tx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    //USART1 RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    //IR sensor
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    
    // LED
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    // Ultrasound 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_4; // TRIG
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
        
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5; // ECHO
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    // Motor 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

}
void USART12_Init(void)
{
    USART_InitTypeDef USART_InitStructure;

    // USART1 Init
    USART_Cmd(USART1, ENABLE);

    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &USART_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE , ENABLE);

    // USART2 Init
    USART_Cmd(USART2, ENABLE);
    
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART2, &USART_InitStructure);
    
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}

void NVIC_Configure(void) {

    NVIC_InitTypeDef NVIC_InitStructure;
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    // UART1
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    //USART2
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //TIMER 2
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void Time_Configure(void) 
{
  //set 1us
    TIM_TimeBaseInitTypeDef TIM_InitStructure;
    TIM_InitStructure.TIM_Prescaler = 72;
    TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_InitStructure.TIM_Period = 1;
    TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &TIM_InitStructure);
    
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); 
    TIM_Cmd(TIM2, ENABLE);
}

/***************************************
IRQ Handler
****************************************/
void USART1_IRQHandler() {
   uint16_t word;

       if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET){
       // the most recent received data by the USART1 peripheral
        word = USART_ReceiveData(USART1);
        
        //모터 디버그용 코드
//        if( word == 'w') M_StateChange(Go_Forward);        
//        else if (word == 's') M_StateChange(M_Stop);
//        else if( word == 'q') {
//          M_StateChange(Go_Turn_Left);
//          Stay(TURNING_TIME);
//          M_StateChange(Stop);
//        }
//        else if( word == 'e') {
//          M_StateChange(Go_Turn_Right);
//          Stay(TURNING_TIME);
//          M_StateChange(Stop);
//        }
//        else if( word == 'z'){
//            M_StateChange(Back_Turn_Left);
//            Stay(TURNING_TIME);
//            M_StateChange(Stop);
//        }
//        else if( word == 'c'){
//            M_StateChange(Back_Turn_Right);
//            Stay(TURNING_TIME);
//            M_StateChange(Stop);
//        }
//        else if( word == 'x')
//          M_StateChange(Back_Forward);
//        
//        
//
//        //초음파센서 디버그용
//        if(word == 'u'){
//          distance = Read_Distance_Go();
//          SendInt(distance);
//        }
//        
//        //State Change
//        if( word == '1' || word == '2'|| word == '3' || word =='4')
//        {
//          
//          if(EventFlag[Stop_Target] ==0) // 커맨드가 안들어와있는 상태면.
//          {
//            MoveIdx = word-'0';        // 커맨드를 입력시켜준다.
//            EventFlag[Stop_Target] = 1;
//          }
//        }
//
//        if(word =='p') {
//          Var_Reset_All();
//        }
        
        //SendData(word);
        //USART_SendData(USART2, word);
        // clear 'Read data register not empty' flag
        USART_ClearITPendingBit(USART1,USART_IT_RXNE);
    }
}


void USART2_IRQHandler(){ 
    uint16_t word;
    //SendData('K');
    if(USART_GetITStatus(USART2,USART_IT_RXNE)!=RESET){
        //SendData('K');
       // the most recent received data by the USART1 peripheral
        word = USART_ReceiveData(USART2);
        //모터 디버그용 코드
        if( word == 'w') M_StateChange(Go_Forward);        
        else if (word == 's') M_StateChange(M_Stop);
        else if( word == 'q') {
          M_StateChange(Go_Turn_Left);
          Stay(TURNING_TIME);
          M_StateChange(Stop);
        }
        else if( word == 'e') {
          M_StateChange(Go_Turn_Right);
          Stay(TURNING_TIME);
          M_StateChange(Stop);
        }
        else if( word == 'z'){
            M_StateChange(Back_Turn_Left);
            Stay(TURNING_TIME);
            M_StateChange(Stop);
        }
        else if( word == 'c'){
            M_StateChange(Back_Turn_Right);
            Stay(TURNING_TIME);
            M_StateChange(Stop);
        }
        else if( word == 'x')
          M_StateChange(Back_Forward);
        
        

        //초음파센서 디버그용
        if(word == 'u'){
          distance = Read_Distance_Back();
          SendInt(distance);
        }
        
        //State Change
        if( word == '1' || word == '2'|| word == '3' || word =='4')
        {
          
          if(EventFlag[Stop_Target] ==0) // 커맨드가 안들어와있는 상태면.
          {
            MoveIdx = word-'0';        // 커맨드를 입력시켜준다.
            EventFlag[Stop_Target] = 1;
          }
        }

        if(word =='p') {
          Var_Reset_All();
          RCC_Configure();
          GPIO_Configure();
          USART12_Init();
          NVIC_Configure();
          Time_Configure();
        }
        // //USART2가 받은 데이터를 USART2으로 전달.
        USART_SendData(USART2, word); // 핸드폰에서 받아보기 위해서
        while ((USART2->SR & USART_SR_TC) == 0);
        // // clear 'Read data register not empty' flag
        USART_ClearITPendingBit(USART2,USART_IT_RXNE);
    }
}

void TIM2_IRQHandler(){
  
   if(TIM_GetITStatus(TIM2,TIM_IT_Update)!=RESET){
        usTime++;
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
   }
}
/**************************
User Defined Function
***************************/
int Read_Distance_Go(void){
  uint32_t prev=0;
  GPIO_SetBits(GPIOE, SOUND_TRIG_GO);
  GPIO_ResetBits(GPIOE, SOUND_ECHO_GO);
  Delay(10);
  GPIO_ResetBits(GPIOE, SOUND_TRIG_GO);
  uint8_t val = GPIO_ReadInputDataBit(GPIOE, SOUND_ECHO_GO);
  prev = usTime;
  //초기값은 ECHO가 RESET일테니까.
  while(val == RESET){ //바로 SET되지 않고 RESET인 경우에
    if(usTime - prev >= 5000) break; // 5ms 동안
    else{
      val = GPIO_ReadInputDataBit(GPIOE, SOUND_ECHO_GO); //계속 갱신하면서 5ms가 넘으면 빠져나옴.
    }
  }
  //빠져나왔는데
  if(val == SET) // 5ms안에 SET이 되었으면
  {
    prev = usTime;
    while(GPIO_ReadInputDataBit(GPIOE, SOUND_ECHO_GO) != RESET)
    {
    }
    return (usTime-prev)*34/1000; // 다시 SET -> RESET이 될때까지 시간 (usTime -prev)으로 distance계산해서 반환.
  }else{
      //5ms안에 감지가 안됐으면
      //너무 거리가 멀다는 의미니까 큰값 반환.
      return 150;
  }
}

int Read_Distance_Back(void){
  uint32_t prev=0;
  GPIO_SetBits(GPIOE, SOUND_TRIG_BACK);
  GPIO_ResetBits(GPIOE, SOUND_ECHO_BACK);
  Delay(10);
  GPIO_ResetBits(GPIOE, SOUND_TRIG_BACK);
  uint8_t val = GPIO_ReadInputDataBit(GPIOE, SOUND_ECHO_BACK);
  prev = usTime;
  //초기값은 ECHO가 RESET일테니까.
  while(val == RESET){ //바로 SET되지 않고 RESET인 경우에
    if(usTime - prev >= 5000) break; // 5ms 동안
    else{
      val = GPIO_ReadInputDataBit(GPIOE, SOUND_ECHO_BACK); //계속 갱신하면서 5ms가 넘으면 빠져나옴.
    }
  }
  //빠져나왔는데
  if(val == SET) // 5ms안에 SET이 되었으면
  {
    prev = usTime;
    while(GPIO_ReadInputDataBit(GPIOE, SOUND_ECHO_BACK) != RESET)
    {
    }
    return (usTime-prev)*34/1000; // 다시 SET -> RESET이 될때까지 시간 (usTime -prev)으로 distance계산해서 반환.
  }else{
      //5ms안에 감지가 안됐으면
      //너무 거리가 멀다는 의미니까 큰값 반환.
      return 150;
  }
}
void Delay(uint32_t delayTime){
  uint32_t prev_time = usTime;
  while(1)
  {
    if(usTime - prev_time > delayTime) break;
  }
}
void Stay(uint32_t StayTime){
  for(int i = 0; i< StayTime; i++);
}

void M_StateChange(uint8_t m_state){
  switch(m_state){
  case M_Stop:
    GPIO_ResetBits(GPIOC, GPIO_Pin_10); // Back Right -
    GPIO_ResetBits(GPIOC, GPIO_Pin_11); // Back_Right +
    
    GPIO_ResetBits(GPIOD, GPIO_Pin_14); // Back Left -
    GPIO_ResetBits(GPIOD, GPIO_Pin_15); // Back Left +
    
    GPIO_ResetBits(GPIOC, GPIO_Pin_6); // Front Right -
    GPIO_ResetBits(GPIOC, GPIO_Pin_7); // Front Right +
    
    GPIO_ResetBits(GPIOC, GPIO_Pin_8); // Front Left -
    GPIO_ResetBits(GPIOC, GPIO_Pin_9); // Front Left +
    MotorState = M_Stop;
    break;
  case Go_Forward:
    GPIO_ResetBits(GPIOC, GPIO_Pin_10); // Back Right -
    GPIO_SetBits(GPIOC, GPIO_Pin_11); // Back_Right +
    
    GPIO_ResetBits(GPIOD, GPIO_Pin_14); // Back Left -
    GPIO_SetBits(GPIOD, GPIO_Pin_15); // Back Left +
    
    GPIO_ResetBits(GPIOC, GPIO_Pin_6); // Front Right -
    GPIO_SetBits(GPIOC, GPIO_Pin_7); // Front Right +
    
    GPIO_ResetBits(GPIOC, GPIO_Pin_8); // Front Left -
    GPIO_SetBits(GPIOC, GPIO_Pin_9); // Front Left +    
    MotorState = Go_Forward;
    break;
  case Go_Turn_Left:
    GPIO_ResetBits(GPIOC, GPIO_Pin_10); // Back Right -
    GPIO_SetBits(GPIOC, GPIO_Pin_11); // Back_Right +
    
    GPIO_SetBits(GPIOD, GPIO_Pin_14); // Back Left -
    GPIO_ResetBits(GPIOD, GPIO_Pin_15); // Back Left +
    
    GPIO_ResetBits(GPIOC, GPIO_Pin_6); // Front Right -
    GPIO_SetBits(GPIOC, GPIO_Pin_7); // Front Right +
    
    GPIO_ResetBits(GPIOC, GPIO_Pin_8); // Front Left -
    GPIO_ResetBits(GPIOC, GPIO_Pin_9); // Front Left + 
    MotorState = Go_Turn_Left;
    break;
  case Go_Tank_Left:
    GPIO_ResetBits(GPIOC, GPIO_Pin_10); // Back Right -
    GPIO_SetBits(GPIOC, GPIO_Pin_11); // Back_Right +
    
    GPIO_SetBits(GPIOD, GPIO_Pin_14); // Back Left -
    GPIO_ResetBits(GPIOD, GPIO_Pin_15); // Back Left +
    
    GPIO_ResetBits(GPIOC, GPIO_Pin_6); // Front Right -
    GPIO_SetBits(GPIOC, GPIO_Pin_7); // Front Right +
    
    GPIO_SetBits(GPIOC, GPIO_Pin_8); // Front Left -
    GPIO_ResetBits(GPIOC, GPIO_Pin_9); // Front Left + 
    MotorState = Go_Tank_Left;
    break;
  case Go_Turn_Right:
    GPIO_ResetBits(GPIOC, GPIO_Pin_10); // Back Right -
    GPIO_ResetBits(GPIOC, GPIO_Pin_11); // Back_Right +
    
    GPIO_ResetBits(GPIOD, GPIO_Pin_14); // Back Left -
    GPIO_SetBits(GPIOD, GPIO_Pin_15); // Back Left +
    
    GPIO_SetBits(GPIOC, GPIO_Pin_6); // Front Right -
    GPIO_ResetBits(GPIOC, GPIO_Pin_7); // Front Right +
    
    GPIO_ResetBits(GPIOC, GPIO_Pin_8); // Front Left -
    GPIO_SetBits(GPIOC, GPIO_Pin_9); // Front Left + 
    MotorState = Go_Turn_Right;
    break;
  case Go_Tank_Right:
    GPIO_SetBits(GPIOC, GPIO_Pin_10); // Back Right -
    GPIO_ResetBits(GPIOC, GPIO_Pin_11); // Back_Right +
    
    GPIO_ResetBits(GPIOD, GPIO_Pin_14); // Back Left -
    GPIO_SetBits(GPIOD, GPIO_Pin_15); // Back Left +
    
    GPIO_SetBits(GPIOC, GPIO_Pin_6); // Front Right -
    GPIO_ResetBits(GPIOC, GPIO_Pin_7); // Front Right +
    
    GPIO_ResetBits(GPIOC, GPIO_Pin_8); // Front Left -
    GPIO_SetBits(GPIOC, GPIO_Pin_9); // Front Left + 
    MotorState = Go_Tank_Right;
    break;
  case Back_Forward:
    GPIO_SetBits(GPIOC, GPIO_Pin_10); // Back Right -
    GPIO_ResetBits(GPIOC, GPIO_Pin_11); // Back_Right +
    
    GPIO_SetBits(GPIOD, GPIO_Pin_14); // Back Left -
    GPIO_ResetBits(GPIOD, GPIO_Pin_15); // Back Left +
    
    GPIO_SetBits(GPIOC, GPIO_Pin_6); // Front Right -
    GPIO_ResetBits(GPIOC, GPIO_Pin_7); // Front Right +
    
    GPIO_SetBits(GPIOC, GPIO_Pin_8); // Front Left -
    GPIO_ResetBits(GPIOC, GPIO_Pin_9); // Front Left +    
    MotorState = Back_Forward;
    break;
  case Back_Turn_Left:
    GPIO_ResetBits(GPIOC, GPIO_Pin_10); // Back Right -
    GPIO_ResetBits(GPIOC, GPIO_Pin_11); // Back_Right +
    
    //
    GPIO_SetBits(GPIOD, GPIO_Pin_14); // Back Left -
    GPIO_ResetBits(GPIOD, GPIO_Pin_15); // Back Left +
    
    GPIO_ResetBits(GPIOC, GPIO_Pin_6); // Front Right -
    GPIO_SetBits(GPIOC, GPIO_Pin_7); // Front Right +
    
    //
    GPIO_SetBits(GPIOC, GPIO_Pin_8); // Front Left -
    GPIO_ResetBits(GPIOC, GPIO_Pin_9); // Front Left +    
    MotorState = Back_Turn_Left;
    break;
  case Back_Tank_Left:
    GPIO_ResetBits(GPIOC, GPIO_Pin_10); // Back Right -
    GPIO_SetBits(GPIOC, GPIO_Pin_11); // Back_Right +
    
    //
    GPIO_SetBits(GPIOD, GPIO_Pin_14); // Back Left -
    GPIO_ResetBits(GPIOD, GPIO_Pin_15); // Back Left +
    
    GPIO_ResetBits(GPIOC, GPIO_Pin_6); // Front Right -
    GPIO_SetBits(GPIOC, GPIO_Pin_7); // Front Right +
    
    //
    GPIO_SetBits(GPIOC, GPIO_Pin_8); // Front Left -
    GPIO_ResetBits(GPIOC, GPIO_Pin_9); // Front Left +    
    MotorState = Back_Tank_Left;
    break;
  case Back_Turn_Right:
    GPIO_SetBits(GPIOC, GPIO_Pin_10); // Back Right -
    GPIO_ResetBits(GPIOC, GPIO_Pin_11); // Back_Right +
    
    //
    GPIO_ResetBits(GPIOD, GPIO_Pin_14); // Back Left -
    GPIO_ResetBits(GPIOD, GPIO_Pin_15); // Back Left +
    
    GPIO_SetBits(GPIOC, GPIO_Pin_6); // Front Right -
    GPIO_ResetBits(GPIOC, GPIO_Pin_7); // Front Right +
    
    //
    GPIO_ResetBits(GPIOC, GPIO_Pin_8); // Front Left -
    GPIO_SetBits(GPIOC, GPIO_Pin_9); // Front Left +    
    MotorState = Back_Turn_Right;
    break;
  case Back_Tank_Right:
    GPIO_SetBits(GPIOC, GPIO_Pin_10); // Back Right -
    GPIO_ResetBits(GPIOC, GPIO_Pin_11); // Back_Right +
    
    //
    GPIO_ResetBits(GPIOD, GPIO_Pin_14); // Back Left -
    GPIO_SetBits(GPIOD, GPIO_Pin_15); // Back Left +
    
    GPIO_SetBits(GPIOC, GPIO_Pin_6); // Front Right -
    GPIO_ResetBits(GPIOC, GPIO_Pin_7); // Front Right +
    
    //
    GPIO_ResetBits(GPIOC, GPIO_Pin_8); // Front Left -
    GPIO_SetBits(GPIOC, GPIO_Pin_9); // Front Left +    
    MotorState = Back_Tank_Right;
    break;
  default:
    break;
    
  }
}

void IR_Sensing_Motor(void)
{
    IR_Array[FL] = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_11);
    IR_Array[FR] = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_12);
    IR_Array[BL] = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_14);
    IR_Array[BR] = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_13);

    if(RobotState == Run){
        if(IR_Array[FL] == 1 && IR_Array[FR] == 1) //앞으로 가는데, 아무것도 센싱안됐음.
        { 
            M_StateChange(Go_Forward);
        }
        else if(IR_Array[FL] == 0 && IR_Array[FR] ==1) // 왼쪽 센싱됐으면 좌회전.
        {
            M_StateChange(Go_Tank_Left);
            Stay(TURNING_FIXTIME);
            M_StateChange(Go_Forward);
            Stay(TURNING_GO);
        }
        else if(IR_Array[FL] == 1 && IR_Array[FR] == 0)
        {
            M_StateChange(Go_Tank_Right);
            Stay(TURNING_FIXTIME);
            M_StateChange(Go_Forward);
            Stay(TURNING_GO);
            
        }
        else{ //양쪽 다 감지되는 상황.         
            int dir = GoArray[MoveIdx][Go_Idx];
            if(dir == D_ARRIVE ){
              //도착이면.
              RobotState = Arrive;
              M_StateChange(M_Stop);
            } 
            else if(dir == LEFT){
              //좌회전이면
              M_StateChange(Go_Turn_Left);
              Stay(TURNING_TIME);
            } 
            else if(dir ==RIGHT){
              //우회전이면
              M_StateChange(Go_Turn_Right);
              Stay(TURNING_TIME);
            }
            //시작지점 도착했으면
            else if(dir ==S_ARRIVE){ 
              RobotState = Done;
              M_StateChange(Stop);
            }
            else if(dir == IGNORE)
            {
              M_StateChange(Go_Forward);
              Stay(IGNORE_TIME);
            }
            Go_Idx++;
        }
    }
    else if(RobotState == Return){
        if(IR_Array[BL] == 1 && IR_Array[BR] ==1)
        {
            M_StateChange(Back_Forward);
        }
        else if(IR_Array[BL] == 0 && IR_Array[BR] ==1)
        {
            M_StateChange(Back_Tank_Left);
            Stay(TURNING_FIXTIME);
            M_StateChange(Back_Forward);
            Stay(TURNING_GO);
        }
        else if(IR_Array[BL] == 1 && IR_Array[BR] ==0)
        {
            M_StateChange(Back_Tank_Right);
            Stay(TURNING_FIXTIME);
            M_StateChange(Back_Forward);
            Stay(TURNING_GO);
        }
        else{
            int dir = BackArray[MoveIdx][Back_Idx];
            if(dir == D_ARRIVE ){
              //도착이면.
              RobotState = Arrive;
              M_StateChange(Stop);
            } 
            else if(dir == LEFT){
              //좌회전이면
              M_StateChange(Back_Turn_Left);
              Stay(TURNING_TIME);

            } 
            else if(dir ==RIGHT){
              //우회전이면
              M_StateChange(Back_Turn_Right);
              Stay(TURNING_TIME);
            }
            else if(dir ==S_ARRIVE){ 
              RobotState = Done;
              M_StateChange(Stop);
            }
            else if(dir == IGNORE)
            {
              M_StateChange(Back_Forward);
              Stay(IGNORE_TIME);
            }
            Back_Idx++;
        }
    }
}

/******************************
Debug function 
*******************************/
void SendInt(int data)
{
  if(data == -1){
    
    return;
  }
  
  if( data < 10 ) {
    SendData('0');
  }
  else if( data >= 10 && data <= 20 )
  {
    SendData('1');
  }
  else if( data >= 20 && data <= 30 )
  {
    SendData('2');
  }
  else if( data >= 30 && data <= 40 )
  {
    SendData('3');
  }
  else if( data >= 40 && data <= 50 )
  {
    SendData('4');
  }
  else if( data >= 50 && data <= 60 )
  {
    SendData('5');
  }
  else if( data >= 60 && data <= 70 )
  {
    SendData('6');
  }
  else if( data >= 70 && data <= 80 )
  {
    SendData('7');
  }
  else if( data >= 80 && data <= 90 )
  {
    SendData('8');
  }
  else{
    SendData('9');
  }
  SendData('\r');
  SendData('\n');

}

void SendData(uint16_t data){
  USART_SendData(USART2, (uint16_t) data);
  while ((USART2->SR & USART_SR_TC) == 0);
}

void Led_On(int num){ //On
  switch(num){
    case 1:
       // LED 1
      GPIO_SetBits(GPIOD, GPIO_Pin_2);
      break;
    case 2:
       // LED 2
      GPIO_SetBits(GPIOD, GPIO_Pin_3);
      break;
    case 3:
       // LED 3
      GPIO_SetBits(GPIOD, GPIO_Pin_4);
      break;
    case 4:
       // LED 4
      GPIO_SetBits(GPIOD, GPIO_Pin_7);
      break;
    case 5:
      GPIO_SetBits(GPIOD, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_7);
      break;
    default:
      break;
  }
}
void Led_Off(int num){
  switch(num){
    case 1:
       // LED 1
      GPIO_ResetBits(GPIOD, GPIO_Pin_2);
      break;
    case 2:
       // LED 2
      GPIO_ResetBits(GPIOD, GPIO_Pin_3);
      break;
    case 3:
       // LED 3
      GPIO_ResetBits(GPIOD, GPIO_Pin_4);
      break;
    case 4:
       // LED 4
      GPIO_ResetBits(GPIOD, GPIO_Pin_7);
      break;
    case 5:
      GPIO_ResetBits(GPIOD, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_7);
      break;
      
    default:
      break;
  }
}

void Var_Reset_All(void)
{
  MoveIdx = 0;
  Go_Idx = 0;
  Back_Idx = 0;
  RobotState = Stop;
  MotorState = M_Stop;
  M_StateChange(Stop);
  for(int i = 0; i<5; i++)
  {
    IR_Array[i] = 0;
  }
  for(int i = 0; i<10 ; i++)
  {
    EventFlag[i] = 0;
  }
  distance = 0;
}
