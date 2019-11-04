/*
   This entire work was done by me and by the help of Arduino Website in
   Ultrasonic website.
   And of the MPU6050 by me and the help of the website (https://proteshea.com)
   (MPU6050) as all of them are public domain for only MPU6050 which anyone can
   redistribut or modify part any part of their code but with the condition of
   mentioning them in the main program
 * ******************************************************
 * ******************************************************
   In this program, the measurements of all attached ultrasonic sensors and
   MPU6050 sensors are displayed using Arduino and ROS environments
*/

#include <ros.h>                         // the main code should in ros using the package rosserial
#include <std_msgs/String.h>             // the output should be words and number, therefore, the data is in String
#include <std_msgs/Float64MultiArray.h>  // it was needed for the main array of all sensors
#include<Wire.h>                         // adding the library of wire to initiliaze the MPU6050
#define MPUaddr0 0x68                    // if the left pin is not connected
#define MPUaddr1 0x69                    // the set from 0 to 1

/* the node handle, according to ROS wiki
  it creates the serial port communications
  and it the publisher which is TheProject
*/
ros::NodeHandle nh;
std_msgs::String Measurements;

ros::Publisher Project("/ProjectofSmartMotorizedBedMeasurements", &Measurements);

/*  First part:
    The acceleration x y z begin with 0 of
    the MPU6050 sensro, same for the angle
    adn the Gyro, rotation, pitch, and angle
    from the last and the real time of the process
    ----------------------------------------------------------
    Second Part:
    Declaring the pins of all HC-SR04 sensors
    The trigger pins of HC-SR04 Ultrasonic
    are all mounted in the digital pin number 2 of
    the arduino Mega and the other echo pins for
    all five are mounted on the digital pins (3, 4, 8, 10, 11)
*/

float accelerationX = 0;
float accelerationY = 0;
float accelerationZ = 0;

float angular_accelerationX = 0;
float angular_accelerationY = 0;

float gyroscopeX = 0;
float gyroscopeY = 0;
float gyroscopeZ = 0;

float gyroscope_angleX = 0;
float gyroscope_angleY = 0;
float gyroscope_angleZ = 0;

float Angle_Smart_Bed = 0;
float pitch = 0;
float Rotation_Stand_Up = 0;

float final_period = 0;
float real_time = 0;
float s = 0;

int trigPin1 = 2, echoPin1 = 3;
int trigPin2 = 2, echoPin2 = 4;
int trigPin5 = 2, echoPin5 = 8;
int trigPin7 = 2, echoPin7 = 10;
int trigPin8 = 2, echoPin8 = 11;

void setup() {

  /*
     We advertise the published topic
  */

  nh.initNode();
  nh.advertise(Project);
  //reset MPU to default settings
  /*
     Both the Gyro and the accelerometer has ranges
     for the Gyro are between
     +/-250, +/-500, +/-1000, +/-2000 DPS
     for the acclerometer
     +/-2g, +/-4g, +/-8g, +/-16g
  */


  /*  Setting the band to 57600 is very important
      because our serial node in ROS is only work
      with this band number, moreover the mpu6050
      sensor also works with it. But HC-SR04 sensors
      work with any band and that's why it was
      crucial to set the band the same for all
      methods.
  */

  Serial.begin (57600);
  /*
     Declaring the pin mode (INPUT,OUTPUT)
     Trigger is the output
     Echo is the input
  */
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin5, OUTPUT);
  pinMode(echoPin5, INPUT);
  pinMode(trigPin7, OUTPUT);
  pinMode(echoPin7, INPUT);
  pinMode(trigPin8, OUTPUT);
  pinMode(echoPin8, INPUT);

  Wire.begin();
  /*
     Process of start and restart of the sensor MPU6050 to make sure
     that the sensor is on the default settings
     And make the sensistivity for bot the gyroscope and accerlerometer
     to be zero and at rest
  */
  OPEN();
  GS(0x00);
  AS(0x00);
  delay(10000);
}

long publisher_timer;

void loop() {
  /*
     The general domain of the two kits
     Acc = 16384 = 16*1024 with  //+/-2g
     Gyro = 131
     And fusing the sensor
  */
  display_data_accelerometer(16384.0);
  display_data_gyroscope(131.0);
  fusion();
  displaySerial();
  delay(10);
  //********************************************************************************
  /*
     Left back sensor measurements
     Explain More
  */
  float duration_Left_Back_Sensor, distance_Left_Back_Sensor;
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);

  duration_Left_Back_Sensor = pulseIn(echoPin1, HIGH);
  distance_Left_Back_Sensor = (duration_Left_Back_Sensor / 2) / 29.1;

  Serial.print ("LEFT BACK SENSOR:  ");
  Serial.print (distance_Left_Back_Sensor);
  Serial.println("cm");
  delay(10);
  //********************************************************************************
  /*
     Right back sensor measurements
     Explain More
  */
  float duration_Right_Back_Sensor, distance_Right_Back_Sensor;
  digitalWrite(trigPin2, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin2, LOW);

  duration_Right_Back_Sensor = pulseIn(echoPin2, HIGH);
  distance_Right_Back_Sensor = (duration_Right_Back_Sensor / 2) / 29.1;

  Serial.print("RIGHT BACK SENSOR:  ");
  Serial.print(distance_Right_Back_Sensor);
  Serial.println("cm");
  delay(10);
  //********************************************************************************
  /*
     Right up sensor measurements
     Explain More
  */

  float duration_Right_Up_Sensor, distance_Right_Up_Sensor;
  digitalWrite(trigPin5, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin5, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin5, LOW);

  duration_Right_Up_Sensor = pulseIn(echoPin5, HIGH);
  distance_Right_Up_Sensor = (duration_Right_Up_Sensor / 2) / 29.1;

  Serial.print("Right Up Sensor:  ");
  Serial.print(distance_Right_Up_Sensor);
  Serial.println("cm");
  delay(10);
  //********************************************************************************
  /*
     Right front sensor measurements
     Explain More
  */

  float duration_Right_Front_Sensor, distance_Right_Front_Sensor;
  digitalWrite(trigPin7, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin7, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin7, LOW);

  duration_Right_Front_Sensor = pulseIn(echoPin7, HIGH);
  distance_Right_Front_Sensor = (duration_Right_Front_Sensor / 2) / 29.1;

  Serial.print("Right Front Sensor:  ");
  Serial.print(distance_Right_Front_Sensor);
  Serial.println("cm");
  delay(10);
  //********************************************************************************
  /*
     Left front sensor measurements
     Explain More
  */

  float duration_Left_Front_Sensor, distance_Left_Front_Sensor;
  digitalWrite(trigPin8, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin8, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin8, LOW);

  duration_Left_Front_Sensor = pulseIn(echoPin8, HIGH);
  distance_Left_Front_Sensor = (duration_Left_Front_Sensor / 2) / 29.1;

  Serial.print("Left Front Sensor:  ");
  Serial.print(distance_Left_Front_Sensor);
  Serial.println("cm");
  delay(10);

  /**************************************************
     Since we would like to show the result mixed between
     Names of the sensors and the results of the sensor
     We had to change the type of the measurements from
     float to String of all ultrasonic sensors
  */

  String distance_Left_Back   = String(distance_Left_Back_Sensor);
  String distance_Right_Back  = String(distance_Right_Back_Sensor);
  String distance_Right_Up    = String(distance_Right_Up_Sensor);
  String distance_Right_Front = String(distance_Right_Front_Sensor);
  String distance_Left_Front  = String(distance_Left_Front_Sensor);

  /*
     String Data is our output methods for all inputs
  */
  String data = "LeftBack: " + distance_Left_Back + " RightBack: " + distance_Right_Back + " RighyUp: " + distance_Right_Up + " RightFront: " + distance_Right_Front + " LeftFront: " + distance_Left_Front + " RotationStandUp: " + Rotation_Stand_Up + " AngleWholeFrame: " + Angle_Smart_Bed + "E" ;

  Serial.println(data);

  int length = data.indexOf("E") + 2;
  char final[length + 1];
  data.toCharArray(final, length + 1);

  if (millis() > publisher_timer) {
    Measurements.data = final;
    Project.publish(&Measurements);
    publisher_timer = millis() + 100;
    nh.spinOnce();
  }
}


void displaySerial(void) {

  Serial.print("RotationStandUp: ");
  Serial.print(Rotation_Stand_Up);
  Serial.print(" AngleWholeFrame: ");
  Serial.println(Angle_Smart_Bed);

}

void fusion(void) {

  /*Fusion is very important methods because sometimes
    this tiny sensor MPU6050 may be slower of the data
    or the gyro will be drifted or the accelerometer
    will give a wrong output because of the exernal forces
  */
  Angle_Smart_Bed = 0.96 * (Angle_Smart_Bed + gyroscope_angleX * s) + 0.04 * angular_accelerationX;
}


void display_data_accelerometer(float aDivisor) {

  Wire.beginTransmission(MPUaddr0);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(MPUaddr0, 6);

  // 6 consecutive registers
  /*
     Reading the upper byte and the lower byte
     from X to Y to Z of the acceleration
  */

  // Calculating the angle comes from only the accelerometer

  if (Wire.available() >= 6) {
    /*
        the accelerationX is NOT needed only if you want
        to calculate the pitch of the bed
    */
    int16_t temp0 = Wire.read() << 8;
    int16_t temp1 = Wire.read();
    accelerationX = (float) (temp0 | temp1);
    accelerationX = accelerationX / aDivisor;

    temp0 = Wire.read() << 8;
    temp1 = Wire.read();
    accelerationY = (float) (temp0 | temp1);
    accelerationY = accelerationY / aDivisor;

    temp0 = Wire.read() << 8;
    temp1 = Wire.read();
    accelerationZ = (float) (temp0 | temp1);
    accelerationZ = accelerationZ / aDivisor;

  }

  angular_accelerationX = (atan2(accelerationY, accelerationZ)) * 180 / PI;

}

void display_data_gyroscope(float gDivisor) {

  final_period = real_time;
  real_time = millis();
  s = (real_time - final_period) / 1000;
  //Serial.print(Ts);

  Wire.beginTransmission(MPUaddr0);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(MPUaddr0, 6);
  if (Wire.available() >= 6) {
    // 6 consecutive registers
    /*
       Reading the upper byte and the lower byte
       of Z axis of the Gyro
    */

    int16_t temp0 = Wire.read() << 8;
    int16_t temp1 = Wire.read();
    gyroscopeZ = (float) (temp0 | temp1);
    gyroscopeZ = gyroscopeZ / gDivisor + 0.79;

  }

  Rotation_Stand_Up += gyroscopeZ * s;

}

/*
   The upcoming functions (AS, GS, OPEN)
   are NOT so important for the project
   but it was posted on the public
   domain of the MPU6050 for resetting
   the sensor or checking the sensisitivity
   of the gyroscope or the accelerometer
*/

void AS(uint8_t ACC) {

  Wire.beginTransmission(MPUaddr0);
  Wire.write(0x1C);
  Wire.write(ACC);
  Wire.endTransmission();
}

void GS(uint8_t SES) {
  Wire.beginTransmission(MPUaddr0);
  Wire.write(0x1B);
  Wire.write(SES);
  Wire.endTransmission();
}

void OPEN(void) {
  Wire.beginTransmission(MPUaddr0);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);
}
