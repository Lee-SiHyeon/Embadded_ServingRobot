#include "linetracer.h"
int main(void)
{
    
    RCC_Configure();
    GPIO_Configure();
    USART12_Init();
    NVIC_Configure();
    Time_Configure();

    
    RobotState = Stop;

    
    while (1) {
      
      switch(RobotState){
        case Stop:
          IR_Array[STUFF] = GPIO_ReadInputDataBit(GPIOE, IR_STUFF); // 배달물품 감지
          //목적지가 설정 되어 있고, 물체가 감지 된 상태라면 Run State로 이동.
          if(EventFlag[Stop_Target] ==1 && IR_Array[STUFF] ==1){ 
            RobotState = Run;
            SendData('R');
          }
          break;

        case Run:
        //적외선 센싱과 모터 동작
          IR_Sensing_Motor();

          //충돌 방지를 위한 초음파센서 거리계산
          distance = Read_Distance_Go();
          IR_Array[STUFF] = GPIO_ReadInputDataBit(GPIOE, IR_STUFF);

          //RUN하던 도중에 초음파 거리 SAFETY이하가 되거나 물체가 사라지면 Emergency State로 이동.
          if(distance <= SAFETY || IR_Array[STUFF] == 0){
            EventFlag[Emergency_From] = Run;
            RobotState = Emergency;
          }
          break;


        case Arrive:
            //물체 감지해보고
          IR_Array[STUFF] = GPIO_ReadInputDataBit(GPIOE, IR_STUFF);
          while(IR_Array[STUFF] !=0) // 물건이 사라질때까지 기다림.
          {
            IR_Array[STUFF] = GPIO_ReadInputDataBit(GPIOE, IR_STUFF); // 지속적으로 감지.
          }
          //while loop를 빠져나오면 Return State로 감.
          RobotState = Return;
          break;

        case Return:
          IR_Sensing_Motor();

          distance = Read_Distance_Back();
          //Run State랑 로직 동일. 다만 물체 감지안해도 됨.
          if(distance <= SAFETY) {
            EventFlag[Emergency_From] = Return;
            RobotState = Emergency;
          }
          break;

        case Emergency:
          M_StateChange(Stop);
          while(EventFlag[Emergency_From] == Run) 
          { 
            distance=Read_Distance_Go();
            IR_Array[STUFF] = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_15);
            if(distance >= SAFETY && IR_Array[STUFF] ==1)
            {
              RobotState = Run;
              break;
            }
          }
          while(EventFlag[Emergency_From] == Return)
          {
            distance = Read_Distance_Back();
            if(distance >= SAFETY)
            {
              RobotState = Return;
              break;
            }
          }

          break;

        case Done:
        // 다시 시작지점으로 온 상태.
          Var_Reset_All(); // 모든값을 초기화 해주고
          RobotState = Stop; // 정지상태로 loop back
          break;

      }

    }
  
    return 0;
}