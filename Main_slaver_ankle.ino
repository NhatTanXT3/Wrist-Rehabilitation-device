#include "PID.h"
#include "IR_sensor.h"
#include "Communication.h"



/*====================================
          Biến chương trình, 
          phục vụ truyền nhận lệnh
    ======================================*/
struct{
	unsigned char run:1;
	unsigned char display:1;
	unsigned char test_motor:1;
}flag;
unsigned char run_mode=1;

char command[100];
unsigned char command_index=0;
boolean stringComplete = false;  // whether the string is complete


/*=====================================
            biến motor và encoder
    =====================================*/
#define MAX_POSITION 20000
#define MIN_POSITION 100


int encorder_1_intPin = 2;   //encoder 1 chanel A
int encorder_1_inputPin = 3; //encoder 1 chanel B
int pinOut = 15;  //pin for testing

const int PWM_1 = 5;  //pin for PWM motor SEA
const int DIR_1 = 6;  // pin for DIR motor SEA

float motor_output;

volatile long encoder_1=0;
unsigned long pre_encoder_1=0;
unsigned long velo_1=0;

const int LED=13;  //pin for LED
int ledState = LOW; 

/*==========================================
             Chương trình Encoder read
============================================*/

void readEncoder_1()
{
	if (digitalRead(encorder_1_intPin) == digitalRead(encorder_1_inputPin)) {
		encoder_1--;
	} else {
		encoder_1++;
	}
}


/*=====================================
            biến cho timer
    =====================================*/
unsigned long currentMicros;
unsigned long previousMicros_100Hz = 0;        // will store last time it was called for 100Hz timer
const long interval_100Hz = 10000;           // interval at which to do 100Hz task(micros)

unsigned long previousMicros_50Hz = 0;        // will store last time it was called for 50Hz timer
const long interval_50Hz = 20000;           // interval at which to do 50Hz task (micros)

unsigned long previousMicros_500Hz= 0;        // will store last time it was called for 50Hz timer
const long interval_500Hz = 2000;           // interval at which to do 500Hz task (micros)

unsigned long position_timer_t0= 0;
unsigned long position_timer_t= 0;

/*=====================================
         biên cho quỹ đạo lực và vị trí
    =====================================*/
float omega=0;//2pi/T
float position_attitude=1000;

/*=====================================
            biến cho IR sensor
    =====================================*/
int as[9];
float as_filter[9];

/*=====================================
       Biến, chương trình cho controller
    =====================================*/
#define LIMIT_FORCE 2
//PID PID_position; //position control
//PID PID_admittance; // addmitance control
//PID PID_force; // force control

PID PID_position; //position control
PID PID_admittance; // admittance control
PID PID_force; // force control
PID PID_velo;  //velocity control

float K_spring=10;//7.54
float offset_setpoint=0;
float position_setpoint=0;
float position_test=0;
//PID PID_v;
void PID_init()
{
	PID_position.set_point=0;
	PID_position.KP=1.5;//4;//5;
	PID_position.KI=4;//10;//10;
	PID_position.KD=0.03;//0.4;//0.5;
	PID_position.output=0;
	PID_position.er=0;
	PID_position.pre_er=0;
	PID_position.pre_pre_er=0;
	PID_position.I_limit=1000;

	PID_position.sampling_time=0.01;// cho bộ điều khiển chạy ở 100Hz, xác định cứng.

	PID_admittance.set_point=0;
	PID_admittance.KP=50;//10;
	PID_admittance.KI=0;
	PID_admittance.KD=0;//0.2;
	PID_admittance.output=0;
	PID_admittance.er=0;
	PID_admittance.pre_er=0;
	PID_admittance.pre_pre_er=0;
	PID_admittance.I_limit=1000;

	PID_admittance.sampling_time=0.01;// cho bộ điều khiển chạy ở 100Hz, xác định cứng.


	PID_force.set_point=5;
	PID_force.KP=5;
	PID_force.KI=0;
	PID_force.KD=0;
	PID_force.output=0;
	PID_force.er=0;
	PID_force.pre_er=0;
	PID_force.pre_pre_er=0;
	PID_force.I_limit=1000;  
	PID_force.sampling_time=0.01;// cho bộ điều khiển chạy ở 100Hz, xác định cứng.
}
void PID_reset()
{
	PID_position.output=0;
	PID_position.er=0;
	PID_position.pre_er=0;
	PID_position.pre_pre_er=0;
	PID_position.set_point=0;
	encoder_1=0;
	position_setpoint=0;


	PID_admittance.output=0;
	PID_admittance.er=0;
	PID_admittance.pre_er=0;
	PID_admittance.pre_pre_er=0;

	PID_force.output=0;
	PID_force.er=0;
	PID_force.pre_er=0;
	PID_force.pre_pre_er=0;

}

/*=====================================
             Chương trình chạy động cơ
    =====================================*/
void motor_SEA(float pwm)
{
	//     
	if(pwm<0)
	{
		analogWrite(PWM_1,255+pwm);
		digitalWrite(DIR_1, 0);
	}
	else
	{
		analogWrite(PWM_1, 255-pwm);
		digitalWrite(DIR_1, 1);
	}

	//      if(pwm<0)
	//      {
	//        analogWrite(PWM_1,-pwm);
	//        digitalWrite(DIR, 0);
	//      }
	//      else
	//      {
	//         analogWrite(PWM_1, pwm);
	//         digitalWrite(DIR, 1);
	//      }
}

/*=====================================
             Setup phần cứng
    =====================================*/
void setup() {
	PID_init(); //khởi tạo giá trị ban đầu của bộ điều khiển
	Serial.begin(115200);

	pinMode(PWM_1, OUTPUT);   // Chân xung động cơ
	pinMode(DIR_1, OUTPUT);   // Chân xung động cơ       
	pinMode(LED, OUTPUT);  // chân xuất xung để test

	attachInterrupt(digitalPinToInterrupt(encorder_1_intPin), readEncoder_1, CHANGE); //encoder_1 chanel A
        pinMode(encorder_1_inputPin, INPUT);                                              //encoder_1 chanel B  
	motor_SEA(0);
	while(Spring_calib());
	//while(Loadcell_calib());
}
/*==============================================
             Chương trình chính
             Task 500Hz: đọc cảm biến và lọc
             Task 100Hz: chạy bộ điều khiển
             Task 50Hz: hiễn thị dữ liệu
    ===============================================*/
unsigned char flag_force_control=0;
void loop() {
	serialEvent();

	currentMicros = micros();
	position_timer_t=currentMicros-position_timer_t0;
	if(currentMicros - previousMicros_500Hz >= interval_500Hz)
	{
		previousMicros_500Hz= currentMicros;
		//your code begin from here
		Sensor_update();
	}

	if(currentMicros - previousMicros_100Hz >= interval_100Hz)
	{
		previousMicros_100Hz = currentMicros;
		//your code begin from here
		if(flag.run){  
//			if((abs(as_filter[0])>LIMIT_FORCE)&&(run_mode!=1))
//			{
//				if(flag_force_control==0)
//				{
//					PID_admittance.I_term=0;
//					PID_admittance.pre_er=0;
//					PID_admittance.pre_pre_er=0;
//
//					PID_force.I_term=0;
//					PID_force.pre_er=0;
//					PID_force.pre_pre_er=0;          
//				}

				if(run_mode==2){
					PID_type_3(as_filter[0], &PID_admittance);
					//position_setpoint=1000+position_attitude*sin(omega*(float)position_timer_t/1000000-0.785);     
					PID_position.set_point=position_setpoint+PID_admittance.output+position_attitude*sin(omega*(float)position_timer_t/1000000-0.785);;
                          		
                                }
				else if(run_mode==3)
				{
					PID_type_3(as_filter[0], &PID_force);
					//PID_position.set_point=position_setpoint+PID_force.output+PID_force.set_point*K_spring;        
					PID_position.set_point +=PID_force.output;    
				}
//				flag_force_control=1;
//			}
			else if(run_mode==1)
			{
				if(flag_force_control==1)
				{
					position_timer_t0=currentMicros;
					position_timer_t=0;
                                        position_setpoint=PID_position.set_point;                                       
				} 
				flag_force_control=0; 
				PID_position.set_point=position_setpoint+position_attitude*sin(omega*(float)position_timer_t/1000000-0.785);      							
                        }
                        
                        PID_position.set_point=constrain(PID_position.set_point,MIN_POSITION,MAX_POSITION);
			PID_type_3(encoder_1, &PID_position);
			motor_output=PID_position.output;
			motor_output=constrain(motor_output,-255,255);
			motor_SEA(motor_output);
		}       
	}

	if(currentMicros - previousMicros_50Hz >= interval_50Hz)
	{
		previousMicros_50Hz = currentMicros;
		//your code begin from here
		//    velo_1=encoder_1-pre_encoder_1;
		//    velo_2=encoder_2-pre_encoder_2;
		//    velo=(velo_1+velo_2)*25.92;
		//    pre_encoder_1=encoder_1;
		//    pre_encoder_2=encoder_2;
		if(flag.display)
		{
			display();
		}
		ledState^=1;
		digitalWrite(LED,ledState);
	}
}//loop

/*============================================
         Chương trình hiễn thị dữ liệu
    =============================================*/
void display(){
	//your code start from here
	Serial.write(CN_1);
	Serial.print(encoder_1);
	Serial.write(CN_2);
	Serial.print(as[0]);
	Serial.write(CN_3);
	Serial.print(as_filter[0]);
	Serial.write(CN_4);
	Serial.print(PID_position.set_point);
	Serial.write(CN_5);
	Serial.print(PID_position.output);
	Serial.write(CN_6);
	Serial.print(PID_admittance.output);
	Serial.write(CN_7);
	Serial.println(PID_force.output);
//	Serial.write(CN_8);
//	Serial.println(position_setpoint);
	//        Serial.write(CN_9);
	//        Serial.println(as_filter[5]);


	//  Serial1.print(xlt_filter);
	//  Serial1.write('a');
	//  Serial1.println(PID_x.output);


}
/*============================================
         Chương trình đọc và xữ lý cám biến
    =============================================*/
float spring_offset=0;
float spring_gain=1;
#define  SPRING_NOISE_RANGE  10
unsigned char Spring_calib(){
	unsigned long sum=0;
	unsigned int numOfsamples=50;

	unsigned int buffer;
	unsigned int max_value;
	unsigned int min_value;
	unsigned char i,j;


	buffer = analogRead(SPRING_CHANNEL);
	max_value= buffer;
	min_value= buffer;

	for (i=1; i<=numOfsamples;i++)
	{
		buffer = analogRead(SPRING_CHANNEL);
		if(buffer>max_value) max_value=buffer;
		else if(buffer<min_value)  min_value=buffer;
		if((max_value-min_value)>=SPRING_NOISE_RANGE)
		{
			Serial.println("error spring");
			Serial.println(max_value-min_value);  
			return 1;
		}

		// code sum
		sum += buffer;
		delay(10);
	}


	spring_offset =sum/numOfsamples;

	Serial.print(" Spring offset: ");
	Serial.println(spring_offset);
	return 0;

}


void Sensor_update(){
	as[0] = (analogRead(SPRING_CHANNEL) - spring_offset)*spring_gain;
	//  as[2] = analogRead(analogInPin_2) - MIN_A2;
	//as[3] = analogRead(analogInPin_3)-spring_offset;
	//  as[4] = analogRead(analogInPin_4) - MIN_A4;
	//as[5] = analogRead(analogInPin_5);
	//  as[6] = analogRead(analogInPin_6) - MIN_A6;

	//  as[0]=constrain_lower(as[0],1);
	//  as[1]=constrain_lower(as[1],1);
	//  as[2]=constrain_lower(as[2],1);
	//  as[3]=constrain_lower(as[3],1);
	//  as[4]=constrain_lower(as[4],1);
	//  as[5]=constrain_lower(as[5],1);
	//  as[6]=constrain_lower(as[6],1);

	//as_filter[0]=as_filter[0]*0.8+as[0]*0.2;
	as_filter[0]=as_filter[0]*0.8+as[0]*0.2;
	//  as_filter[2]=as_filter[2]*0.8+as[2]*0.2;
	//as_filter[3]=as_filter[3]*0.98+as[3]*0.02;
	//  as_filter[4]=as_filter[4]*0.8+as[4]*0.2;
	//as_filter[5]=as_filter[5]*0.8+as[5]*0.2;
	//  as_filter[6]=as_filter[6]*0.8+as[6]*0.2;

}//irSensor_update


/*============================================
             Chương trình giao tiếp, nhận
             dữ liệu từ máy tính, được 
             gọi giữa mỗi loop, cho nên 
             hạn chế sử dụng delay trong
             để không mất dữ liệu đọc được
    ==============================================*/
void serialEvent() {
	while (Serial.available()) {
		// get the new byte:
		char inChar = (char)Serial.read();
		// add it to the inputString:
		command[command_index]=inChar;
		command_index++;

		// if the incoming character is a newline, set a flag
		// so the main loop can do something about it:
		if (inChar == '\n') {
			stringComplete = true;
		}
	}

	if (stringComplete){
		Serial.print(command);
		switch(command[0])
		{
		case RUN_:
			flag.run=1;
			run_mode=1; 
			position_timer_t0=currentMicros;
			position_timer_t=0;
			while(Spring_calib());
			//while(Loadcell_calib());   
			PID_reset();
			break;
		case STOP_:
			flag.run=0;
			run_mode=1;
			motor_SEA(0);         
			break;
		case DISPLAY_ON_:
			flag.display=1;
			break;
		case DISPLAY_OFF_:
			flag.display=0;
			break;
		case MODE_SET_:
			//               motor_output_1=atoi(command+1);
			if(command[1]==MODE_2_)
                        {
                                Serial.println("Mode 2");
				run_mode=2;
                        }
			else if(command[1]==MODE_3_)
                        {
                                Serial.println("Mode 3");
				run_mode=3;
                        }
			else 
                        {
                            Serial.println("Mode 1");
				run_mode=1; 
                        }
			break;
		case SET_FORCE_:
			PID_force.set_point=atoi(command+1);
			Serial.print("SF: ");
			Serial.println(PID_force.set_point);
			break;
                case SET_OMEGA_:
			omega=atof(command+1);
			Serial.print("Omega: ");
			Serial.println(omega);
			break;
                 case SET_POSITION_ATTITUDE_:
			position_attitude=atof(command+1);
			Serial.print("Position_attitude: ");
			Serial.println(position_attitude);
			break;
		case SET_KP_:
                        switch(command[1])
                        {
                          case PID_POSITION_:
                              PID_position.KP=atof(command+2);
                              Serial.print("KP_position: ");
			      Serial.println(PID_position.KP);
                              break;
                          case PID_ADMITTANCE_:
                              PID_admittance.KP=atof(command+2);
                              Serial.print("KP_admittance: ");
			      Serial.println(PID_admittance.KP);
                              break; 
                          case PID_FORCE_:
                              PID_force.KP=atof(command+2);
                              Serial.print("KP_force: ");
			      Serial.println(PID_force.KP);  
                              break;
                          case PID_VELO_:
                              PID_velo.KP=atof(command+2);
                              Serial.print("KP_velo: ");
			      Serial.println(PID_velo.KP);
                              break;
                          default:
                              Serial.print("KP_error: ");
			      break;                      
                        }
                        break;
		case SET_KI_:
			 switch(command[1])
                        {
                          case PID_POSITION_:
                              PID_position.KI=atof(command+2);
                              Serial.print("KI_position: ");
			      Serial.println(PID_position.KI);
                              break;
                          case PID_ADMITTANCE_:
                              PID_admittance.KI=atof(command+2);
                              Serial.print("KI_admittance: ");
			      Serial.println(PID_admittance.KI);
                              break; 
                          case PID_FORCE_:
                              PID_force.KI=atof(command+2);
                              Serial.print("KI_force: ");
			      Serial.println(PID_force.KI);  
                              break;
                          case PID_VELO_:
                              PID_velo.KI=atof(command+2);
                              Serial.print("KI_velo: ");
			      Serial.println(PID_velo.KI);
                              break;
                          default:
                              Serial.print("KI_error: ");
			      break;                      
                        }
                        break;  
		case SET_KD_:
			 switch(command[1])
                        {
                          case PID_POSITION_:
                              PID_position.KD=atof(command+2);
                              Serial.print("KD_position: ");
			      Serial.println(PID_position.KD);
                              break;
                          case PID_ADMITTANCE_:
                              PID_admittance.KD=atof(command+2);
                              Serial.print("KD_admittance: ");
			      Serial.println(PID_admittance.KD);
                              break; 
                          case PID_FORCE_:
                              PID_force.KD=atof(command+2);
                              Serial.print("KD_force: ");
			      Serial.println(PID_force.KD);  
                              break;
                          case PID_VELO_:
                              PID_velo.KD=atof(command+2);
                              Serial.print("KD_velo: ");
			      Serial.println(PID_velo.KD);
                              break;
                          default:
                              Serial.print("KD_error: ");
			      break;                      
                        }
                        break;
                case SET_SETPOINT_:
                         switch(command[1])
                        {
                          case PID_POSITION_:
//                              PID_position.set_point=atof(command+2);
//                              Serial.print("set_point_position: ");
//			      Serial.println(PID_position.set_point);
                              position_setpoint=atof(command+2);
                              Serial.print("set_point_position: ");
			      Serial.println(position_setpoint);
                              break;
                          case PID_ADMITTANCE_:
                              PID_admittance.set_point=atof(command+2);
                              Serial.print("set_point_admittance: ");
			      Serial.println(PID_admittance.set_point);
                              break; 
                          case PID_FORCE_:
                              PID_force.set_point=atof(command+2);
                              Serial.print("set_point_force: ");
			      Serial.println(PID_force.set_point);  
                              break;
                          case PID_VELO_:
                              PID_velo.set_point=atof(command+2);
                              Serial.print("set_point_velo: ");
			      Serial.println(PID_velo.set_point);
                              break;
                          default:
                              Serial.print("set_point_error: ");
			      break;                      
                        }
                        break;
                case HELP_:
                       Serial.print("help: ");
                       Serial.println(HELP_);
                       
                       Serial.print("display on: ");
                       Serial.println(DISPLAY_ON_);
                       
                       Serial.print("display off: ");
                       Serial.println(DISPLAY_OFF_);
                       
                       Serial.print("run on: ");
                       Serial.println(RUN_);
                       
                       Serial.print("stop: ");
                       Serial.println(STOP_);
                       break;
                case SYSTEM_INFOR:
                       Serial.print("PID_position: ");
                       Serial.print(PID_position.KP);
                       Serial.print(" : ");
                       Serial.print(PID_position.KI);
                       Serial.print(" : ");
                       Serial.print(PID_position.KD);
                       Serial.print(" : ");
                       Serial.println(PID_position.set_point);
                       
                       Serial.print("PID_admittance: ");
                       Serial.print(PID_admittance.KP);
                       Serial.print(" : ");
                       Serial.print(PID_admittance.KI);
                       Serial.print(" : ");
                       Serial.print(PID_admittance.KD);
                       Serial.print(" : ");
                       Serial.println(PID_admittance.set_point);
                       
                        Serial.print("PID_force: ");
                       Serial.print(PID_force.KP);
                       Serial.print(" : ");
                       Serial.print(PID_force.KI);
                       Serial.print(" : ");
                       Serial.print(PID_force.KD);
                       Serial.print(" : ");
                       Serial.println(PID_force.set_point);
                       
                       Serial.print("PID_velo: ");
                       Serial.print(PID_velo.KP);
                       Serial.print(" : ");
                       Serial.print(PID_velo.KI);
                       Serial.print(" : ");
                       Serial.print(PID_velo.KD);
                       Serial.print(" : ");
                       Serial.println(PID_velo.set_point);
                       break;           
		default:
			break;
		}
		memset(command, 0, sizeof(command));
		command_index=0;
		// clear the string:
		stringComplete = false;
	}
}
