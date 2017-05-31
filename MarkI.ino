  
  /*
  @author Ravi Shekhar Sinha
  Note: Has originally been derived from Joop Brooking's code, to adapt to 
  a new 10DOF sensor and Blutooth(HC-05) as a reciever. 
  
  You can refer the orignal code here which is designed for MPU-6050 and RF 
  Transciever: http://www.brokking.net/ 
  */
  
  
 //Nose up and right wing up is +ve in signal

 /*
  * Fw pitch comes -ve from Phone
  */

#include <SoftwareSerial.h>
#include<Servo.h>   //Using servo library to control ESC

//Sensor Libraries and Settings/////////////////////////////////////////////

#include <Wire.h> 
#include <Adafruit_L3GD20.h>
#include <Adafruit_LSM303_U.h>

Adafruit_L3GD20 gyro;
Adafruit_LSM303 accel;


void initSensors()
{
  accel.begin();
}

//End Sensor Configs///////////////////////////////////////////////////////////////


SoftwareSerial bluetooth(2,3);    //as RX,TX(REDTAPE) in Arduino .. reverse wire to BT Module

Servo escM1;        
Servo escM2;
Servo escM3;
Servo escM4;


boolean start=false;
int throttle=1000;
int roll=0;
int pitch=0;
int yaw=0;
int maxRange=30; //Max input bend of 5deg, hence max = 1500+(5*15)=1575
int minRange=-30; //Min input  bend of 5deg, hence max = 1500-(5*15)=1425
int esc_1 = 1000;
int esc_2 = 1000;
int esc_3 = 1000;
int esc_4 = 1000;
int minESC=1000;
int maxESC=2000;

float sensor_roll;
float sensor_pitch;
float sensor_yaw;

//Raw Data Inclusion ====================================================================================
double mag_x, mag_y, mag_z;
double acc_x, acc_y, acc_z;
static float ACC_RESOLUTION = 0.001F;   // 1, 2, 4 or 12 mg per lsb var _lsm303Accel_MG_LSB
static float EARTH_STANDARD = 9.80665F; //var SENSORS_GRAVITY_STANDARD 



//For fastWrite()=========================================================================================
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long loop_timer, curTime;

//For PID=====================================================
float pid_error_temp;
float pid_i_mem_roll,rollOffset, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch,pitchOffset, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, yawOffset,pid_output_yaw, pid_last_yaw_d_error;
float yawSetPoint;


//PID gain and limit settings======================================================
float pid_p_gain_roll = 0;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 0;              //Gain setting for the roll D-controller
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0;               //Gain setting for the pitch I-controller. //0.0
float pid_d_gain_yaw = 0;                //Gain setting for the pitch D-controller. //26
int pid_max_yaw = 100;                     //Maximum output of the PID-controller (+/-)

int battery_voltage;

//Body//////////////////////////////////////////////////////////////////////////////////////////

void setup() {
Serial.begin(115200);
initSensors();

DDRD |= B11110000;  

bluetooth.begin(115200);            
digitalWrite(12,HIGH);                                                    //Turn on the Ready led.
loop_timer=micros();
battery_voltage = (analogRead(0) + 35) * 1.2317;
}



void loop() {                               //Ardound 3800 without the gyro

  // Taking and processing BT Input
    if(bluetooth.available())
  {
    setCommandFromBT();
  }
  if(start){
  fastWrite();      //Inbuilt call to gyroSensor to update values also calc esc through pid
  }else{
    idleWrite();
  }
}





//Transciver Functions/////////////////////////////////////////////////////////////////

void setCommandFromBT(){
  String content = "";
  char type=bluetooth.read();


    switch(type){
      case 'R': resetAngles();
                break;
      case 'K': kill();
                break;
      case 'B': start=true;
                Serial.println("Starting==================");
                calSensor();
                break;
      case 'T':{    int tempT = ((int)bluetooth.read()-48)*1000;
                    tempT+= ((int)bluetooth.read()-48)*100;
                    tempT+= ((int)bluetooth.read()-48)*10;
                    tempT+= ((int)bluetooth.read()-48);
                    if(isThrottleSafe(tempT)){
                    throttle=tempT;
                    }
      }
                    break;
                    
      case 'S':     {//In the sequence of roll,Pitch and Yaw
                    int tRoll=decodeSignal(bluetooth.read());
                    if(isEngageDataSafe(tRoll))roll=tRoll;
                    else break;
                    int tPitch=decodeSignal(bluetooth.read());
                    if(isEngageDataSafe(tPitch))pitch=tPitch;
                    else break;
                    int tYaw=decodeSignal(bluetooth.read());
                    if(isEngageDataSafe(tYaw))yaw=tYaw;
                    }
                    break;
      
      case 'C':     {int choice = (int)bluetooth.read(); choice-=48;
                    String content =bluetooth.readStringUntil('\n');
                    float data=(content.toFloat())/100;
               switch(choice){
        
                case 0: //if(adjDataSafe(data,e1)){e1=data-100;}
                        if(isPSafe(data)){pid_p_gain_roll =data;//Serial.println(data);
                        } 
                        pid_p_gain_pitch=pid_p_gain_roll;
                        break;
                case 1: //if(adjDataSafe(data,e2)){e2=data-100;}
                        if(isISafe(data)){pid_i_gain_roll =data;//Serial.println(data);
                        }
                        pid_i_gain_pitch=pid_i_gain_roll;
                        break;
                case 2: //if(adjDataSafe(data,e3)){e3=data-100;}
                        if(isDSafe(data)){pid_d_gain_roll =data;//Serial.println(data);
                        }
                        pid_d_gain_pitch=pid_d_gain_roll;
                        break;
                  }}     
    }

}



//IMU Sensor Data Related//////////////////////////////////////////////////////////////////////
void getSensorData(){
  accel.read();
  acc_x=accel.accelData.x;
  acc_y=accel.accelData.y;
  acc_z=accel.accelData.z;

  mag_x=accel.magData.x;
  mag_y=accel.magData.y;
  mag_z=accel.magData.z;

  rawToStandardAccel();
}

void rawToStandardAccel(){
  acc_x = (float)acc_x * ACC_RESOLUTION * EARTH_STANDARD;
  acc_y = (float)acc_y * ACC_RESOLUTION * EARTH_STANDARD;
  acc_z = (float)acc_z * ACC_RESOLUTION * EARTH_STANDARD;  
  }
  
//The calculation imported from 10DOF Adafruit library
void unifySensorData(){
  
  float const PI_F = 3.14159265F;
  sensor_roll = (float)atan2(acc_y, acc_z);
  
  if (acc_y * sin(sensor_roll) + acc_z * cos(sensor_roll) == 0)
    sensor_pitch = acc_x > 0 ? (PI_F / 2) : (-PI_F / 2);
  else
    sensor_pitch = (float)atan(-acc_x / (acc_y * sin(sensor_roll) + acc_z * cos(sensor_roll)));

  sensor_yaw = (float)atan2(mag_z * sin(sensor_roll) - mag_y * cos(sensor_roll), \
                                      mag_x * cos(sensor_pitch) + \
                                      mag_y * sin(sensor_pitch) * sin(sensor_roll) + \
                                      mag_z * sin(sensor_pitch) * cos(sensor_roll));

  /*Convert data to degrees*/
  sensor_roll = sensor_roll * 180 / PI_F;
  sensor_pitch = sensor_pitch * 180 / PI_F;
  sensor_yaw = sensor_yaw * 180 / PI_F;

  }

// ESC Calculation and Writing/////////////////////////////////////////////////////

void idleWrite(){
    PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    delay(7);
}

void fastWrite(){

   unifySensorData();
   sensor_roll-=rollOffset;
   sensor_pitch-=pitchOffset;
  
  if(micros() - loop_timer > 4000) digitalWrite(12, HIGH);                   //Turn on the LED if the loop time exceeds 4050us. 

  while(micros() - loop_timer < 4000);                                      //We wait until 4000us are passed.
  loop_timer = micros();                                                    //Set the timer for the next loop.
  
  //Serial.print(sensor_roll);Serial.print(",");Serial.print(sensor_pitch);Serial.print(",");Serial.println(sensor_yaw);
  calculate_pid();        //300us
  escCalculate();
  limitEsc();
  //Serial.print(esc_1);Serial.print(",");Serial.print(esc_2);Serial.print(",");Serial.print(esc_3);Serial.print(",");Serial.print(esc_4);Serial.println();
  
  
  PORTD |= B11110000;                                                       //Set digital outputs 4,5,6 and 7 high.
  curTime=micros();
  timer_channel_1 = 1000 + curTime;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  timer_channel_2 = esc_2 + curTime;                                     //Calculate the time of the faling edge of the esc-2 pulse.
  timer_channel_3 = 1000 + curTime;                                     //Calculate the time of the faling edge of the esc-3 pulse.
  timer_channel_4 = esc_4 + curTime;                                     //Calculate the time of the faling edge of the esc-4 pulse.
 
  //There is always 1000us of spare time. So let's do something usefull that is very time consuming.
    
  getSensorData();  

  while(PORTD >= 16){                                                       //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                              //Read the current time.
    if(timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;                //Set digital output 4 to low if the time is expired.
    if(timer_channel_2 <= esc_loop_timer)PORTD &= B11011111;                //Set digital output 5 to low if the time is expired.
    if(timer_channel_3 <= esc_loop_timer)PORTD &= B10111111;                //Set digital output 6 to low if the time is expired.
    if(timer_channel_4 <= esc_loop_timer)PORTD &= B01111111;                //Set digital output 7 to low if the time is expired.
  }
 // Serial.println(micros()-temp);
}

void calculate_pid(){
 
  //Roll calculations
  pid_error_temp = sensor_roll - roll;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = sensor_pitch - pitch;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = sensor_yaw - (yawSetPoint + yaw);
  if(pid_error_temp > 180){
    pid_error_temp-=360;
  }else if(pid_error_temp < -179){
    pid_error_temp+=360;
  }
  
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;

}

void escCalculate(){
    esc_1 = round(throttle + pid_output_pitch + pid_output_roll + pid_output_yaw); //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = round(throttle - pid_output_pitch + pid_output_roll - pid_output_yaw); //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = round(throttle - pid_output_pitch - pid_output_roll + pid_output_yaw);//Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = round(throttle + pid_output_pitch - pid_output_roll - pid_output_yaw); //Calculate the pulse for esc 4 (front-left - CW)

      
  //The battery voltage is needed for compensation.
  //A complementary filter is used to reduce noise.
  //0.09853 = 0.08 * 1.2317.
  battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 35) * 0.09853;
  //Serial.println(battery_voltage);

   if (battery_voltage < 1240 && battery_voltage > 800){                   //Is the battery connected?
      esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-1 pulse for voltage drop.
      esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-2 pulse for voltage drop.
      esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-3 pulse for voltage drop.
      esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-4 pulse for voltage drop.
    } 
}


//Utlity Functions////////////////////////////////////////////////////
boolean isPSafe(float data){
  if(data>=0 && data<=5)
  return true;
  else return false;
}

boolean isISafe(float data){
  if(data>=0 && data<=5)
  return true;
  else return false;
}

boolean isDSafe(float data){
  if(data>=0 && data<=20)
  return true;
  else return false;
}

boolean isEngageDataSafe(int ahrs){
  if(ahrs >= minRange && ahrs <= maxRange){
    return true;
  }else return false;
}

boolean isThrottleSafe(int data){
int lBuffer=throttle-40;
int hBuffer=throttle+40;

if(data >= lBuffer && data <= throttle){
  return true;  
}else if(data <= hBuffer && data >= throttle){
  return true;
}
return false;
}

boolean adjDataSafe(int data, int ref){
  data-=100;
  int lref=ref-10;
  int uref=ref+10;
  
  if(data>=ref && data <=uref){
    return true;
  }else if(data<=ref && data >=lref){
    return true;
  }
  return false;
}
int decodeSignal(char s){
  if(s=='0'){
    return 0;
  }
  
 if(s<='Z'){
   return s-91;
 }else if(s<='z'){
  return s-96;
 }
}

void limitEsc(){

    if (esc_1 < minESC) esc_1 = minESC;                                         //Keep the motors running.
    if (esc_2 < minESC) esc_2 = minESC;                                         //Keep the motors running.
    if (esc_3 < minESC) esc_3 = minESC;                                         //Keep the motors running.
    if (esc_4 < minESC) esc_4 = minESC;                                         //Keep the motors running.

    if(esc_1 > maxESC)esc_1 = maxESC;                                           //Limit the esc-1 pulse to 2000us.
    if(esc_2 > maxESC)esc_2 = maxESC;                                           //Limit the esc-2 pulse to 2000us.
    if(esc_3 > maxESC)esc_3 = maxESC;                                           //Limit the esc-3 pulse to 2000us.
    if(esc_4 > maxESC)esc_4 = maxESC;                                           //Limit the esc-4 pulse to 2000us.
}

void calSensor(){
  float calRoll=0;
  float calPitch=0;
  float calYaw=0;
  
  for(int i=0;i<500;i++){
    getSensorData();
    unifySensorData();
    calRoll+=sensor_roll;
    calPitch+=sensor_pitch;
    calYaw+=sensor_yaw;
        
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while calibrating the gyro.
    PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    delay(3);                                                               //Wait 3 milliseconds before the next loop.
  }

 rollOffset=calRoll/500;
 pitchOffset=calPitch/500;
 yawSetPoint=calYaw/500;

 //Serial.println("Exiting calSensor");
}

void resetAngles(){
  roll=0;
  pitch=0;
  yaw=0;

  int inCount=0; 
  
  while(sensor_yaw==0 && inCount < 5){              
  getSensorData();  
  ++inCount;

  PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
  delayMicroseconds(throttle);                                                
  PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
  delay(3);  
  }
  
  yawSetPoint=sensor_yaw;                 
}

void resetPID(){
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;

  pid_p_gain_roll = 0;               //Gain setting for the roll P-controller
  pid_i_gain_roll = 0;              //Gain setting for the roll I-controller
  pid_d_gain_roll = 0;              //Gain setting for the roll D-controller
  pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

  pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
  pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
  pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
  pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

  pid_p_gain_yaw = 0;                //Gain setting for the pitch P-controller. //4.0
  pid_i_gain_yaw = 0;               //Gain setting for the pitch I-controller. //0.02
  pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
  pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)
}

void kill(){
  throttle=1000;
  start=false;
  resetAngles();
  resetPID();
}
