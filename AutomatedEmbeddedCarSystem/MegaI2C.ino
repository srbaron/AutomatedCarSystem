#include <Adafruit_BNO055.h>
#include <FlexiTimer2.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include <Adafruit_GPS.h>
#include "DFR_Key.h"

Adafruit_GPS GPS(&Serial3);                   // define GPS object DO NOT USE Serial0
DFR_Key keypad;                               // define keypad object
Servo myservo;                                // define servo object
Adafruit_BNO055 bno = Adafruit_BNO055(55);    // define BNO sensor object

#define GPSECHO  false                        // echo GPS Sentence 
#define Threshold 5                           // Threshold for Obstacle avoidance (number of obstacles)

LiquidCrystal lcd( 8, 9, 4, 5, 6, 7); // define lcd pins use these default values for OUR LCD

// Global variables that change across functions
int STEERANGLE = 90;        // servo initial angle (range is 0:180)
float HEADING = 0;          // Heading in degree
int LidarRight;             // LIDAR left
int LidarLeft;              // LIDAR right
int LidarFront;             //LIDAR front
boolean usingInterrupt = false;
int carSpeedPin = 2;              // pin for DC motor (PWM for motor driver). don't use other pins....
float errorHeadingRef = 0;        // error
long int lat;                     // GPS latitude in degree decimal * 100000   |     we multiply decimal degree by 100000 to convert it to meter  https://en.wikipedia.org/wiki/Decimal_degrees
long int lon;                     // GPS latitude in degree decimal * 100000   |     0.00001 decimal degree is equal to 1.0247 m at 23 degree N/S
long int latDestination = 33.425891 * 100000;       // define an initial reference Latitude of destination
long int lonDestination =  -111.940458 * 100000;    // define an initial reference Longitude of destination
float Bearing = 0;                                  // initialize bearing
int localkey = 0;                                   // var
float distance;
const float pi = 3.14159256;
bool gps_set = false;
imu::Vector<3> euler;
unsigned long timer = millis();

void setup() {
  
 myservo.attach(48);     // servo is connected to pin 44
  lcd.begin( 16, 2 );     // LCD type is 16x2 (col & row)
  Serial.begin(9600);     // serial for monitoring
  Serial.println("Orientation Sensor Calibration"); Serial.println("");
  if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF)) { //if you want to calibrate using another mode, set it here. OPERATION_MODE_COMPASS for a precise tilt compensated compass (Section 3.3.2 / 3.3.3)
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
byte c_data[22] = {0, 0, 0, 0, 0, 0, 93, 1, 124, 253, 143, 0, 253, 255, 254, 255, 255, 255, 232, 3, 82, 3};

  bno.setCalibData(c_data);                                                                                       // SET CALIBRATION DATA
  bno.setExtCrystalUse(true);


 noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 59016;           // every 0.1 second
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << TOIE1);  // enable timer compare interrupt

  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4  = 336;             // every 1 second
  TCCR4B |= (1 << CS42);    // 256 prescaler
  TIMSK4 |= (1 << TOIE4);  // enable timer compare interrupt

interrupts();
  
myservo.write(90);

 
 GPS.begin(9600);
 GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
 GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate // it's more stable than 10Hz
 GPS.sendCommand(PGCMD_ANTENNA); 
    useInterrupt(true);

  Wire.begin();
     
 while(!GPS.fix && (GPS.satellites < 8)){
  lcd.clear();
  
  lcd.print("#satellites: ");
  lcd.print(GPS.satellites);
  
  lcd.setCursor(0,1);
  lcd.print("Fix: ");
  
 if(GPS.fix)
    lcd.print("Yes");
  else
    lcd.print("No");
    
  delay(80);
 }

 lcd.clear();
 lcd.print("Ready to Drive!");
 delay(2000);
  ///Setting the reference (Lat and Lon)///
  localkey = 0;
  while (localkey != 1) {    // wait for select button
    lcd.clear();
    localkey = keypad.getKey();
    lcd.print("Press Select");
    lcd.setCursor(0, 1);
    lcd.print("To Save Dest");
    delay(100);               // delay to make display visible
  }
  GPSRead();
  latDestination = lat;     // saving the destiantion point
  lonDestination = lon;     // saving the destiantion point
  localkey = 0;
  
  while (localkey != 1) {   // wait for select button
    lcd.clear();
    localkey = keypad.getKey();
    lcd.print("Press Select");
    lcd.setCursor(0, 1);
    lcd.print("To Drive!");
    delay(100);              // delay to make display visible
  }

  gps_set = true;

  // set timer interrupts
 
}

SIGNAL(TIMER0_COMPA_vect) {       // don't change this !!
  char c = GPS.read();            // interrupt for reading GPS sentences char by char....
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
#endif
}

void useInterrupt(boolean v) {    // enable inttrupt for GPS, don't change this!!
  if (v) {
    OCR0A = 0xAF;               // Timer0 is already used for millis() - we'll just interrupt somewhere
    TIMSK0 |= _BV(OCIE0A);      // in the middle by Output Compare Register A (OCR0A) and "TIMER0_COMPA_vect" function is called
    usingInterrupt = true;
  } else {
    TIMSK0 &= ~_BV(OCIE0A); // do not call the interrupt function COMPA anymore
    usingInterrupt = false;
  }
}

ISR(TIMER4_OVF_vect) {  // Timer interrupt for reading GPS DATA
  sei();        //   reset interrupt flag
  TCNT4  = 336; //   re-initialize timer value
  GPSRead();    //   read GPS data
}

void GPSRead() {
  // read GPS data
    if(GPS.newNMEAreceived())
    GPS.parse(GPS.lastNMEA());
    
    if(GPS.fix){
    lat = GPS.latitudeDegrees*100000;
  lon = GPS.longitudeDegrees*100000;
    }
}


void ReadHeading()
{
  // calculate HEADING
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  HEADING = euler.x();
}

void CalculateBearing() {
  // Calculate Bearing
    float deg;
  float dx;
  float dy;
  dx = abs(lon) - abs(lonDestination);
  dy = abs(lat) - abs(latDestination);
  deg = atan(dy/dx);
  deg = ((deg*180)/pi);

  
  if(dx > 0)
    Bearing = deg + 270;
    
  else if(dx < 0)
   Bearing = deg + 90;

  else if(dx == 0 && dy != 0){
      if(dy > 0)
        Bearing = 0;
      else
        Bearing = 180;  
    }
  else if(dy == 0 && dx!=0){
      if(dx > 0)
        Bearing = 90;
       else
        Bearing = 270;   
    }


  else if(dx == 0 && dy == 0)
      Bearing = HEADING;
}

void CalculateSteer() {
  // Calculate Steer angle based on GPS data and IMU
    // calculate steering angle based on heading and bearing
      if(Bearing == 0 || Bearing == 360){
  if(HEADING >= 2.5 && HEADING <=180){
    if(HEADING < 10)
      STEERANGLE = 75;
    else
      STEERANGLE = 50;
  }
    
    
  else if(HEADING > 180 && HEADING <= 358.5){
    if(HEADING > 350)
      STEERANGLE = 105;
    else
      STEERANGLE = 130;
  }
    
  else if(HEADING >358.5 || HEADING <2.5)
    STEERANGLE = 90;
  }




  else{
    if(HEADING < (Bearing+2.5) && HEADING > (Bearing-2.5))
        STEERANGLE = 90;
    
    else if(HEADING >= (Bearing+2.5)){
      
            if(HEADING >= (Bearing + 10))
              STEERANGLE = 50;
            else
              STEERANGLE = 75;
    }

    else if(HEADING <= (Bearing -2.5)){
            if(HEADING <= (Bearing -10))
              STEERANGLE = 130;
            else
              STEERANGLE = 105;
    }
  }
}

void CalculateDistance(){
  // calculate distance to destination based on current and destination coordinates
  float d;
  d = sqrt((lat-latDestination)*(lat-latDestination) + (lon-lonDestination)*(lon-lonDestination));
  distance = d;
  
}

void SetCarDirection() {    // Input: Lidar data
  // Set Steering angle,
  // If any obstacle is detected by Lidar, Ignore steering angle and turn left or right based on observation
  ReadLidar();
  
  if(LidarFront == 1){
    if(LidarLeft == 1 && LidarRight == 0)
        myservo.write(120);
    else if(LidarRight == 1 && LidarLeft == 0)
           myservo.write(60); 
    else myservo.write(120);          
  }
  else if(LidarRight == 1 && LidarLeft == 0)
          myservo.write(82);
       else if(LidarLeft == 1 && LidarRight == 0)
          myservo.write(98);
       else   
        myservo.write(STEERANGLE);
}

void SetCarSpeed() {  // Input: GPS data
  // set speed,
  // if destination is within 5 meters of current position, set speed to zero.
  if(distance <= 3)
    analogWrite(carSpeedPin,0);
  else
    analogWrite(carSpeedPin,30);
}

void ReadLidar() {    // Output: Lidar Data
  // read Lidar Data from Nano Board (I2C)
  // you should request data from Nano and read the number of obstacle (within the range) on your rightside and leftside
  // Then, you can decide to either do nothing, turn left or turn right based on threshold. For instance, 0 = do nothing, 1= left and 2 = right
  Wire.beginTransmission(8);
  Wire.write(1);
  Wire.endTransmission();
  
  Wire.requestFrom(8,1);
  while(Wire.available()){
    LidarRight = Wire.read();
  }

  Wire.beginTransmission(8);
  Wire.write(2);
  Wire.endTransmission();
  
  Wire.requestFrom(8,1);
   while(Wire.available()){
    LidarLeft = Wire.read();
   }

  Wire.beginTransmission(8);
  Wire.write(3);
  Wire.endTransmission();
  
  Wire.requestFrom(8,1);
   while(Wire.available()){
    LidarFront = Wire.read();
   }
  
}

ISR(TIMER1_OVF_vect) {        // function will be call every 0.1 seconds
  sei();                  // reset interrupt flag
  TCNT1  = 59016;
  ReadHeading();
  CalculateBearing();
  CalculateSteer();
  CalculateDistance();
  if((millis() > timer + 250) && gps_set)
    SetCarDirection();
  if(GPS.fix && gps_set)
    SetCarSpeed();
}


void printHeadingOnLCD() {
  lcd.setCursor(0,0);
  lcd.print("Heading:");
  lcd.print(HEADING);

}

void printLocationOnLCD() {
  lcd.setCursor(0,0);
lcd.print("Lat: ");
lcd.print(lat);

lcd.setCursor(0,1);
lcd.print("Lon: ");
lcd.print(lon);

}

void printDistanceOnLCD() {
      lcd.setCursor(0,1);
    lcd.print("Dis: ");
    lcd.print(distance);

}

void printObstacleOnLCD() {
  lcd.setCursor(0,1);
  lcd.print("F:");
  lcd.print(LidarFront);
  lcd.print(" R:");
  lcd.print(LidarRight);
  lcd.print(" L:");
  lcd.print(LidarLeft);

}

void loop() {
  lcd.clear();      // clear lcd
  // Pring data to LCD in order to debug your program!
  printHeadingOnLCD();
  printObstacleOnLCD();
  if(millis() > timer + 1000)
    timer = millis();
  delay(100);
}
