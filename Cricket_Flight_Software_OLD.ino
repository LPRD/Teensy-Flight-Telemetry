#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h> //all 100hz except mag(20) and temp(1)
#include <avr/pgmspace.h>
#include <Adafruit_BMP3XX.h>  //bmp.setOutputDataRate(BMP3_ODR_50_HZ);  ...ODR_200_HZ, _12_5_HZ, _3_1_HZ, _1_5_HZ, etc
#include <TinyGPS++.h>  //10hz update rate, ~5 w/ RTK
#include <Telemetry.h>
#include <PWMServo.h>

//GPS setup
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
#define gps_dt 100 //time in ms between samples for neo m8n GPS

//radio setup
#define TELEMETRY_SERIAL Serial1 //Teensy 3.6 has to use Serial1 or higher
// 2/4/20 telem ACSII msgs are ~300 chars long= ~300 bytes ... 57600bits/s / (8bits+2extra)= 5760 bytes/s... = ~19 radio msgs/s MAX
// 2-5 msgs/s is probably fine for the future, so we want ~600-1500 bytes/s, so no lower than 6000-15000 bits/s baud rate
//I'm only showing the math here now b/c eventually we will want to lower the radio baud rates to get better range
#define radio_dt 100 //time in ms between sending telemetry packets

//BMP388 setup
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 15

//UPDATE B4 FLIGHT!!!
//Calibration Factor for BMP388, chech loacl pressure b4 flight!
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BMP3XX bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);  //software SPI
#define bmp_dt 10 //time in ms between samples for bmp388

//BNO setup
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);//ID, Address, Wire
//use the syntax &Wire1, 2,... for SDA1, 2,... //55 as the only argument also works
#define bno_dt 10 //time in ms between samples for bno055

//internal clock setup
RTC_DS1307 rtc;

//Remaining issues: RTC doesn't work right, still not sure why
//I also haven't been able to get I2C ports on the teensy working 
//other than SDA0 and SCL0... there is some sytax involving "&Wire" 
//above in the BNO setup that I think is supposed to be changed but 
//I have already tried using Wire1 and &Wire1 but the BNO 
//didnt give data when I connected it to the SCL1/SDA1 ports on the
//teensy... long story short I could use some help here


//SD logging setup
File dataFile;
char filename[] = "DATA000.csv";
#define sd_dt 10 //time in ms between data points in csv file logging

//Battery Reading Setup
#define Batt_V_Read A0
double reading;
double vbatt1;
int batt1_num_cells 4;  //UPDATE B4 FLIGHT!!!

//Pin setup
#define LED 13 //Error LED, refers to builtin LED on teensy
#define PYRO1 24
#define PYRO2 25
#define PYRO3 26
#define PYRO4 27
#define PYRO5 28  //Camera on/off Pin
#define PWM1 2
#define PWM2 3
#define PWM3 4
#define PWM4 5
#define PWM5 6
#define PWM6 7
#define PWM7 8
#define PWM8 23 //A9
#define PWM9 22 //A8
#define PWM10 21 //A7
#define PWM11 20 //A6
#define PWM12 17 //A3
#define PWM13 16 //A2
#define PWM14 36 //A17
#define PWM15 35 //A16

PWMServo S1;

int pos1 = 90;

#define SEND_VECTOR_ITEM(field, value)\
  SEND_ITEM(field, value.x())         \
  SEND_GROUP_ITEM(value.y())          \
  SEND_GROUP_ITEM(value.z())

#define WRITE_CSV_ITEM(value)         \
  dataFile.print(F(", ")); dataFile.print(value);

#define WRITE_CSV_VECTOR_ITEM(value)  \
  WRITE_CSV_ITEM(value.x())           \
  WRITE_CSV_ITEM(value.y())           \
  WRITE_CSV_ITEM(value.z())

unsigned int missed_deadlines = 0;

//BMP calibration factor is ABOVE in the code ^
#define Launch_ALT 300  //Launch Alt above sea level in m- UPDATE B4 FLIGHT!!!
#define ATST 100 //m above launch height- UPDATE B4 FLIGHT!!!
//Apogee Trigger Safety Threshold- apogee detection/(parachute) triggering will not work below this pt
                        
//carry working gps to points and record positions- UPDATE B4 FLIGHT!!!
#define launch_lat 44.975313  
#define launch_lon -93.232216
#define land_lat (44.975313+.00035)
#define land_lon (-93.232216+.00035)

long gpstimer= 0;
long radiotimer= 0;
long bmptimer= 0;
long bnotimer= 0;
long sdtimer= 0;
long falltimer=0;
#define fall_dt 10

imu::Vector<3> gyroscope;
imu::Vector<3> euler;
imu::Vector<3> accelerometer;
imu::Vector<3> magnetometer;
//imu::Quaternion quat; //double qw; ordouble q[4];
double temp;

  double bno_x= 0;
  double bno_y= 0;
  double bno_z= 0;
  double bno_vx= 0;
  double bno_vy= 0;
  double bno_vz= 0;
double bno_alt= Launch_ALT; //will be -bno_x
double bno_alt_last [10]= {0,0,0,0,0,0,0,0,0,0};
double bno_alt_new [10]= {0,0,0,0,0,0,0,0,0,0};
double bno_alt_last_avg= 0;
double bno_alt_new_avg= 0;
int bno_descending_counter= 0;
bool bno_descending= 0;  

double bmp_temp;
double bmp_pressure;
double bmp_alt= 0;  
//can review alt data to get bmp vel, accel
double bmp_alt_last [10]= {0,0,0,0,0,0,0,0,0,0};
double bmp_alt_new [10]= {0,0,0,0,0,0,0,0,0,0};
double bmp_alt_last_avg= 0;
double bmp_alt_new_avg= 0;
int bmp_descending_counter= 0;
bool bmp_descending= 0;  

//Cardinal= in the North/East/South/West plane, NO up/down component!
int16_t sats;
float fix_hdop; //horiz. diminution of precision
double gps_lat;
double gps_lon;
double gps_alt; //alt above SL in m
double gps_vel; //abs Cardinal speed in m/s
double gps_dir; //abs Cardinal course in deg, N=0, E=90...
double xy_from_lanch; //Cardinal distance in m from launch pt
double dir_from_launch; //Cardinal dir from launch pt in deg, N=0, E=90...
double xy_to_land; //Cardinal distance in m to land pt
double xy_dir_to_land; //Cardinal direction in deg to land pt
double x_to_land;
double y_to_land;
//NorthEastDown Frame
double gps_n= 0; //m north of launch spot
double gps_e= 0; //m east of launch spot
double gps_d= 0; //m down of launch spot (will be -)
//can review data to get gps vel, accel, use gps_alt for data analysis
double gps_alt_last [10]= {0,0,0,0,0,0,0,0,0,0};
double gps_alt_new [10]= {0,0,0,0,0,0,0,0,0,0};
double gps_alt_last_avg= 0;
double gps_alt_new_avg= 0;
int gps_descending_counter= 0;
bool gps_descending= 0;  

bool Apogee_Passed=0;

double sum=0;

void setup() {  
  Serial2.begin(GPSBaud); //Serial2 is the radio
  TELEMETRY_SERIAL.begin(57600); TELEMETRY_SERIAL.println();
  
  rtc.begin();  //ensures that rtc actually begins...
  
  while (!bmp.begin()) {                         //flashes to signal error
    TELEMETRY_SERIAL.println(F("BMP388 err"));
    digitalWrite(LED,LOW); delay(2000); digitalWrite(LED,HIGH);
  } 
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_100_HZ);
  
  while (!bno.begin()) {                         //flashes to signal error
    TELEMETRY_SERIAL.println(F("BNO055 err"));
    digitalWrite(LED,LOW); delay(1000); digitalWrite(LED,HIGH);
  }
  
  if (!rtc.isrunning()) { rtc.adjust(DateTime(__DATE__, __TIME__)); }
  if (!SD.begin(BUILTIN_SDCARD)){
    TELEMETRY_SERIAL.println(F("SD err"));
    digitalWrite(LED,LOW); delay(500); digitalWrite(LED,HIGH);
  }
  else {                                            // generates file name
    for (uint16_t nameCount = 0; nameCount < 1000; nameCount++) {
      filename[4] = nameCount/100 + '0';
      filename[5] = (nameCount%100)/10 + '0';
      filename[6] = nameCount%10 + '0';
      if (!SD.exists(filename)) {                   // opens if file doesn't exist
        dataFile = SD.open(filename, FILE_WRITE);
        TELEMETRY_SERIAL.print(F("\twriting "));
        TELEMETRY_SERIAL.println(filename);
        dataFile.println(F("abs time,sys date,sys time,x angle,y angle,z angle,x gyro,y gyro,z gyro,bno temp,x mag,y mag,z mag,x accel,y accel,z accel,bmp alt,gps alt,gps lat,gps lon,gps vel,gps dir,xy_from_lanch,dir_from_launch,xy_to_land,xy_dir_to_land,x_to_land,y_to_land,bmp temp,bmp pressure,sats,hdop"));
        break;
      }
    }
  }
  //Pin Initialization
  pinMode(PYRO1, OUTPUT);
  pinMode(PYRO2, OUTPUT);
  pinMode(PYRO3, OUTPUT);
  pinMode(PYRO4, OUTPUT);
  pinMode(PYRO5, OUTPUT);
  digitalWrite(PYRO1,LOW);
  digitalWrite(PYRO2,LOW);
  digitalWrite(PYRO3,LOW);
  digitalWrite(PYRO4,LOW);
  digitalWrite(PYRO5,LOW);
  //this could probably be done w/ a loop in fewer lines
  //so if someone wants to do that that'll work 

  S1.attach(PWM1);
  //S1.attach(SERVO_PIN_A, 1000, 2000); //some motors need min/max setting
  
}




void loop() {
  long time0 = millis();  //I don't think we need this
  
  if(millis()-bnotimer > bno_dt){
    gyroscope     = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    euler         = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    accelerometer = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    magnetometer  = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    //quat = bno.getQuat(); //qw= quat.w(); or q[4]=[quat.w(),quat.x()...
    temp = bno.getTemp();

    for (int i=0;i<9;i++){
      bno_alt_new[i+1]=bno_alt_new[i]; //move every element 1 back 
    }
    bno_alt_new[0]= bno_alt; //now array fully updated
    
    for (int i=0;i<10;i++){
      sum=sum+bno_alt_new[i];
    }
    bno_alt_new_avg= sum/10.0;
    sum=0;  //reset sum var for next use

    if((bno_alt_last_avg > bno_alt_new_avg) && (bno_alt > Launch_ALT + ATST)){
      bno_descending_counter= bno_descending_counter + 1;
    }
    
    for (int i=0;i<10;i++){
      bno_alt_last[i]= bno_alt_new[i];
    } //prev alts now = current alts
    bno_alt_last_avg= bno_alt_new_avg;
  
  
    
    bnotimer= millis();
  }

  if(bno_descending != 1){
    if((bno_descending_counter>9)&& (bno_alt > Launch_ALT + ATST)){ 
      bno_descending = 1;
    }
  }

  
  if(millis()-bmptimer > bmp_dt){
    bmp_temp= bmp.temperature; //in Celcius, do I need int8_t type????
    bmp_pressure= bmp.pressure / 100.0; //hPa or mbar 
    bmp_alt= bmp.readAltitude(SEALEVELPRESSURE_HPA); //m
    for (int i=0;i<9;i++){
      bmp_alt_new[i+1]=bmp_alt_new[i]; //move every element 1 back 
    }
    bmp_alt_new[0]= bmp_alt; //now array fully updated
    
    for (int i=0;i<10;i++){
      sum=sum+bmp_alt_new[i];
    }
    bmp_alt_new_avg= sum/10.0;
    sum=0;  //reset sum var for next use

    if((bmp_alt_last_avg > bmp_alt_new_avg) && (bmp_alt > Launch_ALT + ATST)){
      bmp_descending_counter= bmp_descending_counter + 1;
    }
    
    for (int i=0;i<10;i++){
      bmp_alt_last[i]= bmp_alt_new[i];
    } //prev alts now = current alts
    bmp_alt_last_avg= bmp_alt_new_avg;
  
    
    bmptimer= millis();
  }

  if(bmp_descending != 1){
    if((bmp_descending_counter>9)&& (bmp_alt > Launch_ALT + ATST)){ 
      bmp_descending = 1;
    }
  }
  
  
  if(millis()-gpstimer > gps_dt){
    //battery voltage read code will also go here:
    reading= analogRead(Batt_V_Read);
    vbatt1= reading*(3.3/1023.00)*batt1_num_cells;
    //while (Serial2.available())
    //    gps.encode(Serial2.read());
    sats= gps.satellites.value();
    fix_hdop= gps.hdop.hdop(); 
    gps_lat= gps.location.lat();
    gps_lon= gps.location.lng();
    gps_alt= gps.altitude.meters();
    gps_vel= gps.speed.mps(); 
    gps_dir= gps.course.deg(); 
    xy_from_lanch= TinyGPSPlus::distanceBetween(launch_lat, launch_lon, gps_lat, gps_lon);
    dir_from_launch= TinyGPSPlus::courseTo(launch_lat, launch_lon, gps_lat, gps_lon);  
    xy_to_land= TinyGPSPlus::distanceBetween(gps_lat, gps_lon, land_lat, land_lon);
    xy_dir_to_land= TinyGPSPlus::courseTo(gps_lat, gps_lon, launch_lat, launch_lon);
    x_to_land= TinyGPSPlus::distanceBetween(0, gps_lon, 0, land_lon);
    y_to_land= TinyGPSPlus::distanceBetween(gps_lat, 0, land_lat, 0);
    gps_n= TinyGPSPlus::distanceBetween(launch_lat, 0, gps_lat, 0);
    gps_e= TinyGPSPlus::distanceBetween(0, launch_lon, 0, gps_lon);
    gps_d=-gps_alt;
    
    for (int i=0;i<9;i++){
      gps_alt_new[i+1]=gps_alt_new[i]; //move every element 1 back 
    }
    gps_alt_new[0]= gps_alt; //now array fully updated
    
    for (int i=0;i<10;i++){
      sum=sum+gps_alt_new[i];
    }
    gps_alt_new_avg= sum/10.0;
    sum=0;  //reset sum var for next use

    if((gps_alt_last_avg > gps_alt_new_avg) && (bmp_alt > Launch_ALT + ATST)){
      gps_descending_counter= gps_descending_counter + 1;
    }
    
    for (int i=0;i<10;i++){
      gps_alt_last[i]= gps_alt_new[i];
    } //prev alts now = current alts
    gps_alt_last_avg= gps_alt_new_avg;

    
    gpstimer=millis();
  }   

  if(gps_descending != 1){
    if((gps_descending_counter>9)&& (bmp_alt > Launch_ALT + ATST)){ 
      gps_descending = 1;
    }
  }


  if(millis()-falltimer > fall_dt){
    //Code that is active before Apogee is Reached/Passed
    if(Apogee_Passed !=1){
      
      //S1.write(pos1);
      
      //The following loop only trigger ONCE (when apogee is detected)
      if((gps_descending*1)+(bmp_descending*1)+(bno_descending*1) > 1){
        Apogee_Passed=1;
        //fire drouge chute
        digitalWrite(PYRO1,HIGH);        
        //for(pos = 180; pos >=1; pos -= 1){ //close a servo all the way
        //  S1.write(pos); 
        //  delay(30); 
        //}
      }
    }
    //Continuous code that runs once Apogee is Reached/Passed
    if(Apogee_Passed = 1){
      //insert code here, ex: wait to fire main chutes
  
      if(bmp_alt < Launch_ALT + ATST + 50){
        digitalWrite(PYRO2,HIGH); //fire main chute, just an example

        //would need another trigger lock to ensure this loop doesn't 
        //keep repeating on every iteration...
        //for(pos = 0; pos < 180; pos+=1) { //open a servo all the way
        //  S1.write(pos);
        //  delay(30);
        //}
        
        //S1.write(pos1);
      }
      
    }

    falltimer=millis();
  } 




  
  
  // Downlink
  if(millis()-radiotimer > radio_dt){
  BEGIN_SEND
  SEND_VECTOR_ITEM(euler_angle  , euler);
  SEND_VECTOR_ITEM(gyro         , gyroscope);
  SEND_ITEM(temperature         , temp);
  SEND_VECTOR_ITEM(magnetometer , magnetometer);
  SEND_VECTOR_ITEM(acceleration , accelerometer);
  SEND_ITEM(bmp_alt             , bmp_alt);
  SEND_ITEM(gps_alt             , gps_alt);
  //SEND_ITEM(gps_lat             , gps_lat);
  TELEMETRY_SERIAL.print(F(";"));               
  TELEMETRY_SERIAL.print(F("gps_lat"));            
  TELEMETRY_SERIAL.print(F(":"));               
  TELEMETRY_SERIAL.print(gps_lat,5);//more digits of precision
  //SEND_ITEM(gps_lon             , gps_lon);
  TELEMETRY_SERIAL.print(F(";"));               
  TELEMETRY_SERIAL.print(F("gps_lon"));            
  TELEMETRY_SERIAL.print(F(":"));               
  TELEMETRY_SERIAL.print(gps_lon,5);//more digits of precision
  SEND_ITEM(gps_vel             , gps_vel);
  SEND_ITEM(gps_dir             , gps_dir);
  SEND_ITEM(xy_from_lanch       , xy_from_lanch);
  SEND_ITEM(dir_from_launch     , dir_from_launch);
  SEND_ITEM(sats                , sats);
  END_SEND
  radiotimer=millis();  
  }
  
  
  // Writing to SD Card
  if(millis()-sdtimer > sd_dt){
  DateTime now = rtc.now();
  //writing abs time,sys date,sys time 
  dataFile.print(millis());           dataFile.print(',');
  dataFile.print(now.year()  ,DEC);   dataFile.print('/');
  dataFile.print(now.month() ,DEC);   dataFile.print('/');
  dataFile.print(now.day()   ,DEC);   dataFile.print(',');
  dataFile.print(now.hour()  ,DEC);   dataFile.print(':');
  dataFile.print(now.minute(),DEC);   dataFile.print(':');
  dataFile.print(now.second(),DEC);
  //writing sensor values
  WRITE_CSV_VECTOR_ITEM(euler)
  WRITE_CSV_VECTOR_ITEM(gyroscope)
  WRITE_CSV_ITEM(temp)
  WRITE_CSV_VECTOR_ITEM(magnetometer)
  WRITE_CSV_VECTOR_ITEM(accelerometer)
  WRITE_CSV_ITEM(bmp_alt)
  WRITE_CSV_ITEM(gps_alt)
  //WRITE_CSV_ITEM(gps_lat) //change to have more precision
  dataFile.print(F(", ")); dataFile.print(gps_lat,8);
  //WRITE_CSV_ITEM(gps_lon) //change to have more precision
  dataFile.print(F(", ")); dataFile.print(gps_lat,8);
  WRITE_CSV_ITEM(gps_vel)
  WRITE_CSV_ITEM(gps_dir)
  WRITE_CSV_ITEM(xy_from_lanch)
  WRITE_CSV_ITEM(dir_from_launch)
  WRITE_CSV_ITEM(xy_to_land)
  WRITE_CSV_ITEM(xy_dir_to_land)
  WRITE_CSV_ITEM(x_to_land)
  WRITE_CSV_ITEM(y_to_land)
  WRITE_CSV_ITEM(bmp_temp)
  WRITE_CSV_ITEM(bmp_pressure)
  WRITE_CSV_ITEM(sats)
  WRITE_CSV_ITEM(fix_hdop)
  dataFile.println();
  dataFile.flush();
  sdtimer=millis();  
  }
   
}
