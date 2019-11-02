#include <Wire.h>
#include <RPLidar.h>
RPLidar lidar;          // define lidar as RPLIDAR Object
#define RPLIDAR_MOTOR 3 // motor pin for lidar speed control

void setup() {
  pinMode(RPLIDAR_MOTOR, OUTPUT); // set lidar speed control pin as output
  lidar.begin(Serial);            // begin communication with lidar
  Wire.begin(8);                  // join i2c bus with address 8 as SLAVE
  Wire.onRequest(requestEvent);   // call "requestEvent" function when receives a request
  Wire.onReceive(receiveEvent);   // call "receiveEvent" function when receives a byte
}
int left = 0;                     // variable for detected points on left hand side
int right = 0;                    // variable for detected points on right hand side
int front = 0;                    // varaible for detected points on front side
unsigned long timer = millis();    // time variable for reseting variables
float distance;
float angle;
int c1;                           // variable for received integer

void receiveEvent(int bytes)
{
       // read the received byte as integer. This indicates what data to send back when master is requesting data
       c1 = Wire.read();
}

void requestEvent() 
{

  if(c1 == 1){
    //sennd back right
    if(right >=7 )
      Wire.write(1);
    else Wire.write(0);
  }
  else if(c1 == 2){
    //send back left
    if(left >= 7)
      Wire.write(1);
    else Wire.write(0);
  }
  else if(c1 == 3){
    //send back front
    if(front >= 7)
      Wire.write(1);
    else Wire.write(0);
  }
  
   // receive message byte as a character
   // if master's request is right side data, ("1"), send back the right side data
   // if master's request is left side data, ("2"), send back the left side data
  
}

void loop() 
{
  if (IS_OK(lidar.waitPoint())) { // if lidar is working properly (waiting time less than timeout)
    // read angle and distance of the obstacle
    angle = lidar.getCurrentPoint().angle;
    distance = lidar.getCurrentPoint().distance;
    // filter data (keep only the data in desired range and with desired angle) 
    if(distance > 9 && distance < 1200){
      if(angle >= 340 || angle <= 20)
              front++;
      else if(angle >= 310 && angle < 340)
              left++;
      else if(angle > 20 && angle <= 50)
              right++;
    }
    // COUNT the number of obstacles on LEF and RIGHT side
    // reset obstacle variables every 1 second
    
    if(millis() > timer + 1000){
      front = 0;
      right = 0;
      left = 0;
      timer = millis();
    }
      


    
  } else {                                                  // if lidar is not responding           // Dont change this......
    analogWrite(RPLIDAR_MOTOR, 0);                          //stop the rplidar motor                // Dont change this......
    rplidar_response_device_info_t info;                    // try to detect RPLIDAR...             // Dont change this......
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {            // if detected,                         // Dont change this......
      lidar.startScan();                                    // start scan                           // Dont change this......
      analogWrite(RPLIDAR_MOTOR, 255);                      // start motor rotating at max speed    // Dont change this......
      delay(1000);                                                                                  // Dont change this......
    }
  }
}


