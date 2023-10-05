#include <Arduino.h>
#include <Xicro_demo_ID_1.h>
#include <Arduino_LSM6DS3.h>
// global variables

int counter_milli = 0;

float ax,ay,az;
float gx,gy,gz;

Xicro xicro;

void setup_timer(float timer_rate){
  TCB0.CTRLB = TCB_CNTMODE_INT_gc; 
  TCB0.CCMP  = F_CPU/timer_rate-1;
  TCB0.INTCTRL = TCB_CAPT_bm; 
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;
}

void setup_button(){
  pinMode(A1,INPUT); // button
  pinMode(A2,INPUT); // button
  pinMode(A3,INPUT); // button
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  xicro.begin(&Serial);
  setup_timer(1000);
  setup_button();
  IMU.begin();
}

void loop() {
  // put your main code here, to run repeatedly:  
  xicro.Spin_node();
}

void read_imu(){
  if(IMU.readAcceleration(ax,ay,az) && IMU.readGyroscope(gx,gy,gz)){
    xicro.Publisher_raw_accel.message.x = ax*9.80665;
    xicro.Publisher_raw_accel.message.y = ay*9.80665;
    xicro.Publisher_raw_accel.message.z = az*9.80665;
    xicro.Publisher_raw_gyro.message.x = gx*3.14159/180.0;
    xicro.Publisher_raw_gyro.message.y = gy*3.14159/180.0;
    xicro.Publisher_raw_gyro.message.z = gz*3.14159/180.0;
    xicro.publish_raw_accel();
    xicro.publish_raw_gyro();
    }
  }
 void read_buttons(){
  xicro.Publisher_button_1.message.data = !digitalRead(A1);
  xicro.Publisher_button_2.message.data = !digitalRead(A2);
  xicro.Publisher_button_3.message.data = !digitalRead(A3);
  xicro.publish_button_1();
  xicro.publish_button_2();
  xicro.publish_button_3();
  }

ISR(TCB0_INT_vect)
{
  counter_milli++;
  if (counter_milli%100==0){
    counter_milli=0;
    read_imu();
    read_buttons();
    }
     
  TCB0.INTFLAGS = TCB_CAPT_bm; 
}