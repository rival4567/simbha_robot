
#include <ros.h>   //ros header file
#include <limits.h> //  to limit pwm speed of motors
#include <std_msgs/Int64.h>  //send encode data 
#include <std_msgs/Float32.h> // subscribe motor pwm data
#include <simbha_msgs/simbha_imu.h> // publish motor imu data
#include <Wire.h>  // for I2C library
#include <MPU6050.h> //put this library folder in arduino-folder>libraries
#include <math.h>   //math library
#include "I2Cdev.h"  //i2c library
#include "HMC5883L.h" // magnetometer libray
#include <ros/time.h> // ros time library
#define SERIAL_BUFFER_SIZE 64
////////////////////////// right and left motor pin intialization for controlling direction//////////////////////
#define left_b 25      
#define left_f 24
#define right_b 23
#define right_f 22
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////// PWM pin for left and right motor//////////////////////////////////////////////////////

#define pwm_right 7
#define pwm_left 6

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////// MPU caluation variable initialization/////////////////////////////////////
#define delta_t .01
float pitch=0;
float pitchAcc;
float P_CompCoeff= 0.98;
float yaw=0;
float output_[3];
float output_gyro[3];
float prev_output[3];
float prev_output_gyro[3];
float prev_input_gyro[3];
extern uint16_t cycleTime = 1000;
float acc[3];
float gyro[3];
float *filter_gyro;
float *filter_acc ;

HMC5883L compass;
MPU6050 mpu;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////encode data variable defination /////////////////////////////////////////
#define Left_Encoder_PinA 19
#define Left_Encoder_PinB 18
#define Right_Encoder_PinA 25
#define Right_Encoder_PinB 26

static char global_m1a;
static char global_m2a;
static char global_m1b;
static char global_m2b;

static int global_counts_m1;
static int global_counts_m2;

static char global_error_m1;
static char global_error_m2;

static char global_last_m1a_val;
static char global_last_m2a_val;
static char global_last_m1b_val;
static char global_last_m2b_val;

static int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
static uint8_t enc1_val = 0;
static uint8_t enc2_val = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////// inital value variable required for functions ///////////////////////////////////////
float motor_left_speed = 0;
float motor_right_speed = 0;
float left_value = 0;
float right_value = 0;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////// Ros initialization (publisher) encoder publisher//////////////////////////
ros::NodeHandle  nh;    // node handler to handle publisher subscriber
std_msgs::Int64 int_msg;  // creating object for publisher
simbha_msgs::simbha_imu imu;

ros::Publisher leftmotor("simbha/lw_encoder", &int_msg);   //left encoder publisher on topic "lwheel"
ros::Publisher rightmotor("simbha/rw_encoder", &int_msg);  //right encoder publisher on topic "rwheel"
ros::Publisher imu_data("simbha/imu", &imu);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////// function : setup the motor and pwm pins as I/O ////////////////////////////////////////////
void setup_motor()
{
  pinMode (right_b ,OUTPUT);
  pinMode (right_f ,OUTPUT);
  pinMode (left_b ,OUTPUT);
  pinMode (left_f ,OUTPUT);
  pinMode (pwm_right ,OUTPUT);
  pinMode (pwm_left ,OUTPUT);

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////// encoder interupts initialtization ///////////////////////////////////////////////
void encoder_init()
{
   // disable interupt
   noInterrupts(); 
   
   // initialize interupt
   pinMode(Left_Encoder_PinA, INPUT_PULLUP);
   pinMode(Right_Encoder_PinA, INPUT_PULLUP);
   pinMode(Left_Encoder_PinB, INPUT_PULLUP);
   pinMode(Right_Encoder_PinB, INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(Right_Encoder_PinA), do_Right_Encoder, CHANGE);   // set interrupt on change
   attachInterrupt(digitalPinToInterrupt (Left_Encoder_PinA), do_Left_Encoder, CHANGE);
   attachInterrupt(digitalPinToInterrupt(Right_Encoder_PinB), do_Right_Encoder, CHANGE);
   attachInterrupt(digitalPinToInterrupt (Left_Encoder_PinB), do_Left_Encoder, CHANGE);

   // initialize the global state
   global_counts_m1 = 0;
   global_counts_m2 = 0;
   global_error_m1 = 0;
   global_error_m2 = 0;
   
   // checking left and right encoder roatation using masking 
   enc1_val = enc1_val | ((PIOD->PIO_PDSR & 0b0011));      //right encoder value
   enc2_val = enc2_val | ((PIOD->PIO_PDSR & 0b1100) >> 2 );  //left encoder value   

   // enable interrupts
   interrupts();
 }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////// ISR for right encoder //////////////////////////////////////////////////////////
void do_Right_Encoder()
 {
   
   enc1_val = enc1_val << 2;
   enc1_val = enc1_val | (PIOD->PIO_PDSR & 0b0011);
   
   global_counts_m1 = global_counts_m1 + lookup_table[enc1_val & 0b1111];
 }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////// ISR for left motor ////////////////////////////////////////////////////////
void do_Left_Encoder()
 {
   
   enc2_val = enc2_val << 2;
   enc2_val = enc2_val | ((PIOD->PIO_PDSR & 0b1100) >> 2) ;
   
   global_counts_m2 = global_counts_m2 + lookup_table[enc2_val & 0b1111];

 }
///////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////// various function for getiing the count in variable //////////////////////
 int getCountsM1()
 {
  noInterrupts();
   int tmp = global_counts_m1;
  interrupts();
   return tmp;
 }

 int getCountsM2()
 {
  noInterrupts();
   int tmp = global_counts_m2;
   interrupts();
   return tmp;
 }

 int getCountsAndResetM1()
 {
   noInterrupts();
   int tmp = global_counts_m1;
   global_counts_m1 = 0;
    interrupts();
   return tmp;
 }

 int getCountsAndResetM2()
 {
   noInterrupts();
   int tmp = global_counts_m2;
   global_counts_m2 = 0;
   interrupts();
   return tmp;
 }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////// subscribe function of left motor speed ///////////////////////////////////////////////
 
void left_movement( const std_msgs::Float32 &data)
{
  left_value = data.data;
  
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////// subscribe function for right motor speed ////////////////////////////////////////
void right_movement(const std_msgs::Float32 &data)
{
 right_value = data.data;
  
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////// initialization of subsciber for left and right motor speed ////////////////

ros::Subscriber<std_msgs::Float32> lwheel_sub("simbha/left_motor", &left_movement );
ros::Subscriber<std_msgs::Float32> rwheel_sub("simbha/right_motor", &right_movement);

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////// MPU initialization /////////////////////////////////////////////
void MPU_init()
{
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    delay(500);
  }

  mpu.setI2CMasterModeEnabled(false);
  mpu.setI2CBypassEnabled(true) ;
  mpu.setSleepEnabled(false);
  while (!compass.begin())
  {
    delay(500);
  }
  mpu.calibrateGyro();
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);

  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(0, 0); 
 
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////// Get data from MPU and mangentometer for further calculation /////////////////////////
void get_MPU_data()
{
  Vector normAccel = mpu.readNormalizeAccel();
  Vector normgyro = mpu.readNormalizeGyro();

    acc[0] = normAccel.XAxis;
    acc[1] = normAccel.YAxis;
    acc[2] = normAccel.ZAxis;
    gyro[0] = normgyro.XAxis;
    gyro[1] = normgyro.YAxis;
    gyro[2] = normgyro.ZAxis;
     
     ////////////////// filter data /////////////////////
    
    filter_acc = lowpassfilter(acc,15);
    filter_gyro = highpassfilter(gyro, 15);
    
    
    float pub_pitch = ComplementaryFilter(filter_acc[0],filter_acc[1],filter_acc[2],filter_gyro[1],filter_gyro[2]);
    imu.pitch = pub_pitch;
    ////////////////////////////////////////////////////       
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////// 1st order complementry filter for pitch //////////////////////////////////
float ComplementaryFilter(int ax,int ay,int az,int gy,int gz) {
 long squaresum=(long)ay*ay+(long)az*az;
 pitch+=((-gy/32.8f)*(delta_t/1000000.0f));
 pitchAcc =atan(ax/sqrt(squaresum))*RAD_TO_DEG;
 pitch =P_CompCoeff*pitch + (1.0f-P_CompCoeff)*pitchAcc;
 //SerialUSB.println(pitch);
 delay(2);
 return pitch;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

 
////////////////////////////////Low pass filter for accelerlometer /////////////////////////////////////////
float *lowpassfilter(float *input_, uint8_t f_cut) {
   
   float dT = (float)cycleTime * 0.000001f;
   float RC= 1.0f / ( 2.0f * (float)M_PI * f_cut );
   float alpha = dT /(RC +dT);
   for(int i=0;i<3;i++)
   {
    output_[i] = (*(input_+i)* alpha) + (1-alpha) * (prev_output[i]);
    prev_output[i]=output_[i];
    //SerialUSB.println(output_[i]);
   }
  float *ptr = output_;
  return ptr;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////// high pass filter for gyroscope ///////////////////////////////////////////////
float *highpassfilter(float *input_gyro, uint8_t f_cut) {
   
   float dT = (float)cycleTime * 0.000001f;
   float RC= 1.0f / ( 2.0f * (float)M_PI * f_cut );
   float alpha = RC /(RC +dT);
   for(int i=0;i<3;i++)
   {
    output_gyro[i] = ((prev_output_gyro[i]* alpha) +  ((*(input_gyro+i) - prev_input_gyro[i]) * alpha)) ;
    prev_output_gyro[i]=output_gyro[i];
    prev_input_gyro[i]= *(input_gyro+i);
    //SerialUSB.println(output_[i]);
   }
  float *pt = output_gyro;
  return pt;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////// arduino setup fuction /////////////////////////////////////////////
void setup()
{
   setup_motor();    // motor initialization 
   encoder_init();   // encoder initialization
   MPU_init();       // gy-80 initialization
   nh.initNode();     // ros node initialization
  // setupSharp();
   nh.subscribe(lwheel_sub);  // initialization leftwheel subscriber
   nh.subscribe(rwheel_sub); // initialization rightwheel subscriber
   nh.advertise(leftmotor);  // initialization leftwheel encoder publisher
   nh.advertise(rightmotor); // initialization rightwheel encoder publisher
   nh.advertise(imu_data); // initialization imu encoder publisher

  
}

/////////////////////////// fuction to convert speed into RPM //////////////////
int speedToRPM(float speed)
{
    int rpm=0;
  rpm = (int)(speed * 293.707);
  return rpm;
}
//////////////////////////////////////////////////////////////////////////////

////////////////////////////////// function to pass the subscriber funtion value to motor pwm //////////////////
void Update_Motors()
{
  
  moverightMotor(right_value);
  moveleftMotor(left_value);
  
}
void Publish_imu()
{    
   Vector norm = compass.readNormalize();

  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);
  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
  {
    heading += 2 * PI;
  }
 
  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }

  // Convert to degrees
  float headingDegrees = heading * 180/M_PI; 
  get_MPU_data();
     
    imu.header.frame_id =  "simbha/imu";
    imu.header.stamp = nh.now();
    imu.roll = 0;
    imu.yaw = headingDegrees;
    imu_data.publish(&imu);   
    
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////// left motor speed controlling /////////////////////////////////////////////////////////
void moveleftMotor(float leftmotorValue)
{
 // int right_pwm = (abs(rightServoValue));
  if (leftmotorValue>0)
  {
 //int  leftmotorValueMap=  map(leftmotorValue,0,255,20,255);
 digitalWrite(left_f,HIGH);
 digitalWrite(left_b,LOW);
//analogWrite(pwm_left,(right_pwm + 30));
analogWrite(pwm_left,leftmotorValue);    
  }
  else if(leftmotorValue<0)
  {
   // int  leftmotorValueMap=  map(abs(leftmotorValue),0,255,20,255);
 digitalWrite(left_f,LOW);
 digitalWrite(left_b,HIGH);
 //analogWrite(pwm_left,(right_pwm + 30));
 analogWrite(pwm_left,abs(leftmotorValue)); 
 
  }
  
  else if(leftmotorValue == 0)
  {
 digitalWrite(left_f,LOW);
 digitalWrite(left_b,LOW);
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// right motor speed controlling ///////////////////////////////////////////////
void moverightMotor(float rightmotorValue)
{
 //int left_pwm = (abs(leftServoValue)); 
 if (rightmotorValue > 0)
  {
  //  int  rightmotorValueMap=  map(rightmotorValue,0,255,20,255);

digitalWrite(right_b,LOW);
digitalWrite(right_f,HIGH);
 //analogWrite(pwm_right,(left_pwm+ 30));
  analogWrite(pwm_right,rightmotorValue);
  }
  else if(rightmotorValue < 0)
  {
    //int  rightmotorValueMap=  map(abs(rightmotorValue),0,255,20,255); 
 digitalWrite(right_b,HIGH);
 digitalWrite(right_f,LOW);
 //analogWrite(pwm_right,(left_pwm +30));
 analogWrite(pwm_right,abs(rightmotorValue)); 

  }
  else if(rightmotorValue == 0)
  {

   digitalWrite(right_f,LOW);
   digitalWrite(right_b,LOW);
  
   }  
  
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////// update right and left encoder value for publisher function ///////////////////////////////////
int Update_Encoders()
{
  return global_counts_m2;
}
int Update_Encoders2()
{
  return global_counts_m1;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop()
{
  int_msg.data = Update_Encoders();
  leftmotor.publish( &int_msg );
  int_msg.data = Update_Encoders2();
  rightmotor.publish( &int_msg );

  Publish_imu();
//publishsharp();
  nh.spinOnce();
  delay(100);
    
}




