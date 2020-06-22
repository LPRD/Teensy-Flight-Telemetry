#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h> //all 100hz except mag(20) and temp(1)
#include <avr/pgmspace.h>
#include <Adafruit_BMP3XX.h>  //bmp.setOutputDataRate(BMP3_ODR_50_HZ);  ...ODR_200_HZ, _12_5_HZ, _3_1_HZ, _1_5_HZ, etc
#include <TinyGPS++.h>  //10hz update rate, ~5 w/ RTK
#include <Telemetry.h>
#include <PWMServo.h>
#include <TimeLib.h>

#define DEMO 0
#define FLIGHT 2

// Change this line to set configuration
#define CONFIGURATION FLIGHT    //UPDATE b4 FLIGHT!!!


#define HEARTBEAT_TIMEOUT  5000
long heartbeat_time = 0;
bool link2ground = 1;
bool cmd; //for pyro channels
float val; //read value for ground config
bool P1_setting,P2_setting,P3_setting,P4_setting,P5_setting= 0;

char send1[150]= "";
char send2[150]= "";
char send3[150]= "";
char send4[150]= "";


#if CONFIGURATION == DEMO
#define COUNTDOWN_DURATION 10000 // 10 sec
#else
#define COUNTDOWN_DURATION 60000 // 1 min
#endif

long abort_time= 0;
long start_time = 0;
long run_time= 0;
bool ss = true; //ss stands for sensor_status


char data[10] = "";
char data_name[20] = "";

typedef enum {
  STAND_BY,
  TERMINAL_COUNT,   //button activated
  POWERED_ASCENT,   //if (run_time>0) && (state==TERMINAL_COUNT) && (run_time<BURN_TIME)
  UNPOWERED_ASCENT, //if (state==POWERED_ASCENT) && (run_time>BURN_TIME)
  FREEFALL,         //if (state==UNPOWERED_ASCENT) && (Apogee_Passed == 1)
  DROGUE_DESCENT,   //if (state==FREEFALL) && (DROGUE_FIRED==1)
  MAIN_DESCENT,     //if (state==DROGUE_DESCENT) && (MAIN_FIRED==1)
  LANDED            //if (state==MAIN_DESCENT) && (bmp_alt < Launch_ALT + 5)
} state_t;

state_t state = STAND_BY;

bool IS_RISING=0;
bool IS_FALLING=0;
int BURN_TIME= 3000; //ms
bool DROGUE_FIRED= 0;
bool MAIN_FIRED= 0;

 /*
// Convenience
#define SET_STATE(STATE) //{    \
    state = STATE;      write_state(#STATE);       \
         \
//}
 */

 // /*
void SET_STATE(state_t STATE){
  state= STATE;
}
// */


void (*reset)(void) = 0;

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
#define read_dt 100 //time in ms between recieving telemetry packets


//BMP388 setup
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 15

//UPDATE B4 FLIGHT!!!
//Calibration Factor for BMP388, chech local pressure b4 flight!
float SEALEVELPRESSURE_HPA= 1014.22; //1013.25;  //in units of 100* Pa
Adafruit_BMP3XX bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);  //software SPI
#define bmp_dt 100 //time in ms between samples for bmp388

//BNO setup
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);//ID, Address, Wire
//use the syntax &Wire1, 2,... for SDA1, 2,... //55 as the only argument also works
#define bno_dt 50 //time in ms between samples for bno055

//Remaining issues: I haven't been able to get I2C ports on the teensy working
//other than SDA0 and SCL0... there is some sytax involving "&Wire"
//above in the BNO setup that I think is supposed to be changed but
//I have already tried using Wire1 and &Wire1 but the BNO
//didnt give data when I connected it to the SCL1/SDA1 ports on the
//teensy... long story short I could use some help here


//SD logging setup
File dataFile;
char filename[] = "DATA000.csv";
#define sd_dt 1000 //time in ms between data points in csv file logging

//Battery Reading Setup
#define Batt_V_Read 14  %A0
double reading;
double vbatt1;
int voltage_divider_ratio= 6;

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
float Launch_ALT= 190;  //Launch Alt above sea level in m- UPDATE B4 FLIGHT!!!
float ATST= 25; //m above launch height- UPDATE B4 FLIGHT!!!
//Apogee Trigger Safety Threshold- apogee detection/(parachute) triggering will not work below this pt

//carry working gps to points and record positions- UPDATE B4 FLIGHT!!!
float launch_lat= 44.975313;
double launch_lon= -93.232216;
float land_lat= (44.975313+.00035);
float land_lon= (-93.232216+.00035);

long gpstimer= 0;
long radiotimer= 0;
long readtimer= 0;
long bmptimer= 0;
long bnotimer= 0;
long sdtimer= 0;
long falltimer=0;
#define fall_dt 50


imu::Vector<3> gyroscope;
imu::Vector<3> euler;
imu::Vector<3> accelerometer;
imu::Vector<3> magnetometer;
imu::Quaternion q1; //double qw; // double q[4]; //both might also work, not sure
double temp;
double sqw;
double sqx;
double sqy;
double sqz;
double unit;
double test;
double heading;
double attitude;
double bank;

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
double bmp_alt_last [20]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//double bmp_alt_new [10]= {0,0,0,0,0,0,0,0,0,0};
double bmp_alt_last_avg= 0;
double bmp_alt_new_avg= 0;
int bmp_descending_counter= 0;
bool bmp_descending= 0;
int bmp_descending_counter2= 0;
bool bmp_descending2= 0;

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

//time_t getTeensy3Time;


time_t getTeensy3Time(){
  return Teensy3Clock.get();
}

static void smartDelay(unsigned long ms){
  unsigned long start = millis();
  do
  {
    while (Serial2.available()){
      gps.encode(Serial2.read());
    }
  } while (millis() - start < ms);
}

void start_countdown() {
  #if CONFIGURATION != DEMO
    if (!ss) {
      TELEMETRY_SERIAL.println(F("Countdown aborted due to sensor failure"));
      SET_STATE(STAND_BY); // Set state to signal countdown was aborted
    } 
    else{
      //do nothing
    }
  #endif
  if (0) {  //replace 0 with any unacceptable initial states
    TELEMETRY_SERIAL.println(F("Countdown aborted due to unexpected initial state"));
    SET_STATE(STAND_BY); // Set state to signal countdown was aborted
  }
  else {
    TELEMETRY_SERIAL.println(F("Countdown started"));
    SET_STATE(TERMINAL_COUNT);
    start_time = millis();
    abort_time= 0;
    heartbeat();
  }
}

void heartbeat() {
  heartbeat_time = millis();
}

void write_state(const char *state_name) {
  //SEND(status, state_name);
}

void abort_autosequence() {   //need to check if data is still logged after an abort is triggered
  TELEMETRY_SERIAL.println(F("Run aborted"));
  switch (state) {
    case STAND_BY:
    case TERMINAL_COUNT:
      SET_STATE(STAND_BY);
      abort_time = millis();
      run_time=0;
      start_time=0;
      break;

    case POWERED_ASCENT:
      //SET_STATE(STAND_BY);
      abort_time = millis();
      break;

    case UNPOWERED_ASCENT:
      //SET_STATE(STAND_BY)'
      abort_time = millis();
      break;
      
    case FREEFALL:
      //SET_STATE(STAND_BY)
      abort_time = millis();
      break;

    case DROGUE_DESCENT:
      //SET_STATE(STAND_BY)
      abort_time = millis();
      break;
      
    case MAIN_DESCENT:
      //SET_STATE(STAND_BY)
      abort_time = millis();
      break;
      
    case LANDED:
      SET_STATE(STAND_BY);
      abort_time = millis();
      break;
  }
}






void setup() {
  setSyncProvider(getTeensy3Time);
  Serial2.begin(GPSBaud); //Serial2 is the radio
  TELEMETRY_SERIAL.begin(57600); TELEMETRY_SERIAL.println();

  // Teensy RTC error/success config
  while (timeStatus()!= timeSet) {
    TELEMETRY_SERIAL.println(F("Teensy RTC err"));
    digitalWrite(LED,LOW); delay(5000); digitalWrite(LED,HIGH);delay(5000);
    ss=0;
  }

  while (!bmp.begin()) {                         //flashes to signal error
    TELEMETRY_SERIAL.println(F("BMP388 err"));
    digitalWrite(LED,LOW); delay(2000); digitalWrite(LED,HIGH);delay(2000);
    ss=0;
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_100_HZ);

  //bno.begin() is ran when the while loop criteria is tested...
  //default bno mode is OPERATION_MODE_NDOF
  while (!bno.begin()) {                         //flashes to signal error
    TELEMETRY_SERIAL.println(F("BNO055 err"));
    digitalWrite(LED,LOW); delay(100); digitalWrite(LED,HIGH);delay(100);
    ss=0;
  }
  //set BNO mode
  bno.setMode(Adafruit_BNO055::OPERATION_MODE_NDOF);  //OPERATION_MODE_NDOF_FMC_OFF, see .cpp for all modes
  bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P1); //REMAP_CONFIG_P0 to P7 (1 default)
  //bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P7);  //REMAP_SIGN_P0 to P7 (1 default)
  bno.setExtCrystalUse(true);

  if (!SD.begin(BUILTIN_SDCARD)){
    TELEMETRY_SERIAL.println(F("SD err"));
    digitalWrite(LED,LOW); delay(500); digitalWrite(LED,HIGH);delay(500);
    ss=0;
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
        dataFile.print(F("abs time,sys date,sys time,heading (psy),attitude (theta),bank (phi),x accel,x gyro,bmp alt,gps alt,bno alt"));
        dataFile.print(F(",sats,hdop,vbatt1,y accel,z accel,y gyro,z gyro"));
        dataFile.print(F(",gps lat,gps lon,gps vel,gps dir,xy_from_lanch,dir_from_launch"));
        dataFile.print(F(",xy_to_land,xy_dir_to_land,x_to_land,y_to_land"));
        dataFile.println(F(",bmp pressure,bmp temp,bno temp,qw,qx,qy,qz,test,x euler,y euler,z euler,x mag,y mag,z mag,l2g"));
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
  //S1.attach(SERVO_PIN_A, 1000, 2000); //some motors need min/max setting ,ESCs go 1k-2k
  
  for(int q=0; q<10;q++){
    digitalWrite(LED,LOW); delay(100); digitalWrite(LED,HIGH);delay(100); 
  }
  smartDelay(1000*30);
  for(int q=0; q<20;q++){
    digitalWrite(LED,LOW); delay(50); digitalWrite(LED,HIGH);delay(50); 
  }
  
  //Launch_ALT= bmp.readAltitude(SEALEVELPRESSURE_HPA);   //keeps giving 1900m
  if(gps.satellites.value() > 3){
    gps_alt= gps.altitude.meters();
    launch_lat= gps.location.lat();
    launch_lon= gps.location.lng();
    land_lat= launch_lat + 0.0002;
    land_lon= launch_lon + 0.0002;
  }

  
}







void loop() {
  long time0 = millis();  //I don't think we need this

  if(millis()-bnotimer > bno_dt){
    gyroscope     = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    euler         = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    accelerometer = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    magnetometer  = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    q1 = bno.getQuat(); //qw= quat.w(); or q[4]=[quat.w(),quat.x()...
    temp = bno.getTemp();

    sqw = q1.w()*q1.w();
    sqx = q1.x()*q1.x();
    sqy = q1.y()*q1.y();
    sqz = q1.z()*q1.z();
    unit = sqx + sqy + sqz + sqw; // if normalised is one, otherwise is correction factor
    test = q1.x()*q1.y() + q1.z()*q1.w();
    //when test is close to +/- .5, the imu is close to an euler singularity
    if (test > 0.499*unit) { // singularity at north pole, >86.3 deg
      heading = 2 * atan2(q1.x(),q1.w());
      attitude = PI/2;
      bank = 0;
    }
    if (test < -0.499*unit) { // singularity at south pole, <-86.3 deg
      heading = -2 * atan2(q1.x(),q1.w());
      attitude = -PI/2;
      bank = 0;
    }
    heading = RAD_TO_DEG*atan2(2*q1.y()*q1.w()-2*q1.x()*q1.z() , sqx - sqy - sqz + sqw);
    attitude = RAD_TO_DEG*asin(2*test/unit);
    bank = RAD_TO_DEG*atan2(2*q1.x()*q1.w()-2*q1.y()*q1.z() , -sqx + sqy - sqz + sqw);



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

    //shift everthing one back
    for (int i=0;i<19;i++){
      bmp_alt_last[i+1]=bmp_alt_last[i]; //move every element 1 back, [19] now forgotten
    }
    bmp_alt_last[0]= bmp_alt; //now array fully updated

    for (int i=0;i<10;i++){
      sum=sum+bmp_alt_last[i];
    }
    bmp_alt_new_avg= sum/10.0;  //avg alt over the most recent 10 data points
    sum=0;  //reset sum var for next use

    for (int i=10;i<20;i++){
      sum=sum+bmp_alt_last[i];
    }
    bmp_alt_last_avg= sum/10.0; //avg alt over the oldest 10 data points
    sum=0;

    if((bmp_alt_last_avg > bmp_alt_new_avg) && (bmp_alt > (Launch_ALT + ATST))){
      bmp_descending_counter= bmp_descending_counter + 1;
      IS_RISING= 0;
      IS_FALLING= 1;
    }

    if((bmp_alt_last[19] > bmp_alt_last[0]) && (bmp_alt > (Launch_ALT + ATST))){
      bmp_descending_counter2= bmp_descending_counter2 + 1;
    }

    if((bmp_alt_last_avg < bmp_alt_new_avg)){
      IS_RISING= 1;
      IS_FALLING= 0;
    }
    else if( abs(bmp_alt_last_avg - bmp_alt_new_avg) < .3){  //vertical velocity less than .3 m over the past avg sec
      IS_RISING= 0;
      IS_FALLING= 0;  
    }
    else{
      IS_RISING= 0;
      IS_FALLING= 1; 
    }

    bmptimer= millis();
  }

  if(bmp_descending != 1){
    if((bmp_descending_counter>9) && (bmp_alt > Launch_ALT + ATST)){
      bmp_descending = 1;
    }
  }

  if(bmp_descending2 != 1){
    if((bmp_descending_counter2 >9) && (bmp_alt > Launch_ALT + ATST)){
      bmp_descending2 = 1;
    }
  }


  if(millis()-gpstimer > gps_dt){
    //battery voltage read code will also go here:
    reading= analogRead(Batt_V_Read);
    vbatt1= reading*(3.3/1023.00)* voltage_divider_ratio;  //batt1_num_cells;

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

    smartDelay(300);
    /*
    for(int n=0; n < 200 ;n++){ //100-300 is good... ~700 iterations can run per sec
      while (Serial2.available()){
        gps.encode(Serial2.read());
      }
    }
    */
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
    //state machine:
    if ( (run_time>0) && (state==TERMINAL_COUNT) && (run_time<BURN_TIME) ){
      SET_STATE(POWERED_ASCENT);
      //launch rocket
      digitalWrite(PYRO3,HIGH);
      //DROGUE_FIRED= 1;
      P3_setting=1;
    }    
    if ( (state==POWERED_ASCENT) && (run_time>BURN_TIME) ){
      SET_STATE(UNPOWERED_ASCENT);
    }
    if ( (state==UNPOWERED_ASCENT) && (Apogee_Passed == 1) ){
      SET_STATE(FREEFALL);
    }
    if ( (state==FREEFALL) && (DROGUE_FIRED==1) ){
      SET_STATE(DROGUE_DESCENT);
    }
    if ( (state==DROGUE_DESCENT) && (MAIN_FIRED==1) ){
      SET_STATE(MAIN_DESCENT);
    }
    if ( (state==MAIN_DESCENT) && (bmp_alt < Launch_ALT + 8) ){
      SET_STATE(LANDED);
    }
  
    //Code that is active before Apogee is Reached/Passed
    if(Apogee_Passed !=1){

      //S1.write(pos1);

      //The following loop only trigger ONCE (when apogee is detected)
      if((gps_descending*1)+(bmp_descending*1)+(bmp_descending2*1)+(bno_descending*1) > 1){
        Apogee_Passed=1;
        //fire drouge chute
        digitalWrite(PYRO1,HIGH);
        DROGUE_FIRED= 1;
        P1_setting=1;
        
        //for(pos = 180; pos >=1; pos -= 1){ //close a servo all the way
        //  S1.write(pos);
        //  delay(30);
        //}
      }
    }
    //Continuous code that runs once Apogee is Reached/Passed
    if(Apogee_Passed == 1){
      //insert code here, ex: wait to fire main chutes

      if(bmp_alt < Launch_ALT + ATST + 10){
        digitalWrite(PYRO2,HIGH); //fire main chute, just an example
        MAIN_FIRED= 1;
        P2_setting=1;
        
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




  //Read Commands (STAND_BY / TERMINAL_COUNT aka on pad)
  if(millis()-readtimer > read_dt){

    BEGIN_READ
    READ_FLAG(c) {
      heartbeat();
    }
    READ_FLAG(reset) {
      TELEMETRY_SERIAL.println(F("Resetting board"));
      reset();
    }
    READ_FLAG(s) {
      start_countdown();
    }
    READ_FLAG(a) {
      TELEMETRY_SERIAL.println(F("Manual abort initiated"));
      abort_autosequence();
    }
    READ_FIELD(P1cmd, "%d", cmd) {
      P1_setting= cmd;
      digitalWrite(PYRO1,cmd);
    }
    READ_FIELD(P2cmd, "%d", cmd) {
      P2_setting= cmd;
      digitalWrite(PYRO2,cmd);
    }
    READ_FIELD(P3cmd, "%d", cmd) {
      P3_setting= cmd;
      digitalWrite(PYRO3,cmd);
    }
    READ_FIELD(P4cmd, "%d", cmd) {
      P4_setting= cmd;
      digitalWrite(PYRO4,cmd);
    }
    READ_FIELD(P5cmd, "%d", cmd) {
      P5_setting= cmd;
      digitalWrite(PYRO5,cmd);
    }
    READ_FIELD(Launch_ALT, "%f", val) {
      Launch_ALT= val;
    }
    READ_FIELD(BMP_cf, "%f", val) {
      SEALEVELPRESSURE_HPA= val;
    }
    READ_FIELD(ATST, "%f", val) {
      ATST= val;
    }
    READ_FIELD(launch_lat, "%f", val) {
      launch_lat= val;
    }
    READ_FIELD(launch_lon, "%f", val) {
      launch_lon= val;
    }
    READ_FIELD(land_lat, "%f", val) {
      land_lat= val;
    }
    READ_FIELD(land_lon, "%f", val) {
      land_lon= val;
    }

    //not sure where the UI code sends this data- I
    //don't see any variable named data_name in the
    //static test driver py file, @Lucas?
    READ_DEFAULT(data_name, data) {
      TELEMETRY_SERIAL.print(F("Invalid data field recieved: "));
      TELEMETRY_SERIAL.print(data_name);
      TELEMETRY_SERIAL.print(":");
      TELEMETRY_SERIAL.println(data);
    }
    END_READ

    readtimer=millis();
  }


  // Downlink
  if(millis()-radiotimer > radio_dt){
    if (millis() > heartbeat_time + HEARTBEAT_TIMEOUT) {
      //TELEMETRY_SERIAL.println(F("Loss of data link"));
      link2ground=0;
      //abort_autosequence();
    }
    else{
      link2ground=1;
    }

    //both run_time and start_time = 0 until countdown started
    if((run_time >0) || (start_time >0)){
      if((state != LANDED) && (abort_time == 0)){
        run_time= millis() - start_time - COUNTDOWN_DURATION;
      }
    }

    //abs time, sys time, sys date
    //Send Fomat for Matlab UI (uncomment as needed)
      /*
    sprintf(send1,"@@@@@,%u,%u/%u/%u,%u:%u:%u,%.2f,%.2f,%.5f,%.5f,%.2f,%.2f,%.2f,%u",millis(),year(),month(),day(),hour(),minute(),second(),bmp_alt,gps_alt,gps_lat,gps_lon,vbatt1,test,fix_hdop,sats);
    sprintf(send2,",%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f",heading,attitude,bank,euler.x(),euler.y(),euler.z(),magnetometer.x(),magnetometer.y(),magnetometer.z(),gyroscope.x(),gyroscope.y(),gyroscope.z(),accelerometer.x(),accelerometer.y(),accelerometer.z());
    sprintf(send3,",%.1f,%.1f,%.1f,%.1f,%.1f,%u,%u,%u,%u,%u,%u,%.1f,%.1f,%.1f,%.5f,%.5f,%.5f,%.5f",temp,gps_vel,gps_dir,xy_from_lanch,dir_from_launch,run_time,P1_setting,P2_setting,P3_setting,P4_setting,P5_setting,ATST,Launch_ALT,SEALEVELPRESSURE_HPA,launch_lat,launch_lon,land_lat,land_lon);
    sprintf(send4,",%u,%u,%u,%u,%s,%u,%u,%u,&&&&&",gps_descending,bmp_descending,bmp_descending2,bno_descending,state,Apogee_Passed,link2ground,ss);
    TELEMETRY_SERIAL.print(send1);
    TELEMETRY_SERIAL.print(send2);
    TELEMETRY_SERIAL.print(send3);
    TELEMETRY_SERIAL.println(send4);
      */
    
    //Send Fomat for Python UI (uncomment as needed)
    // /*
    BEGIN_SEND
    SEND_VECTOR_ITEM(euler_angle  , euler)
    SEND_VECTOR_ITEM(gyro         , gyroscope)
    SEND_ITEM(temperature         , temp)
    SEND_VECTOR_ITEM(magnetometer , magnetometer)
    SEND_VECTOR_ITEM(acceleration , accelerometer)
    SEND_ITEM(bmp_alt             , bmp_alt)
    SEND_ITEM(gps_alt             , gps_alt)
    //SEND_ITEM(gps_lat           , gps_lat)
    TELEMETRY_SERIAL.print(F(";"));
    TELEMETRY_SERIAL.print(F("gps_lat"));
    TELEMETRY_SERIAL.print(F(":"));
    TELEMETRY_SERIAL.print(gps_lat,5);//more digits of precision
    //SEND_ITEM(gps_lon           , gps_lon)
    TELEMETRY_SERIAL.print(F(";"));
    TELEMETRY_SERIAL.print(F("gps_lon"));
    TELEMETRY_SERIAL.print(F(":"));
    TELEMETRY_SERIAL.print(gps_lon,5);//more digits of precision
    SEND_ITEM(heading             , heading)
    SEND_ITEM(attitude            , attitude)
    SEND_ITEM(bank                , bank)
    SEND_ITEM(test                , test)
    SEND_ITEM(gps_vel             , gps_vel)
    SEND_ITEM(gps_dir             , gps_dir)
    SEND_ITEM(xy_from_lanch       , xy_from_lanch)
    SEND_ITEM(dir_from_launch     , dir_from_launch)
    SEND_ITEM(sats                , sats)
    SEND_ITEM(hdp                 , fix_hdop)
    SEND_ITEM(vb1                 , vbatt1)
    SEND_ITEM(ss                  , ss)
    //SEND_ITEM(run_time            , millis()-start_time-COUNTDOWN_DURATION);
    SEND_ITEM(l2g                 , link2ground)
    SEND_ITEM(P1_setting          , P1_setting)
    SEND_ITEM(P2_setting          , P2_setting)
    SEND_ITEM(P3_setting          , P3_setting)
    SEND_ITEM(P4_setting          , P4_setting)
    SEND_ITEM(P5_setting          , P5_setting)
    SEND_ITEM(gps_d               , gps_descending)
    SEND_ITEM(bmp_d               , bmp_descending)
    SEND_ITEM(bmp_d2              , bmp_descending2)
    SEND_ITEM(bno_d               , bno_descending)
    SEND_ITEM(Apogee_Passed       , Apogee_Passed)

    SEND_ITEM(ATST                , ATST)
    SEND_ITEM(Launch_ALT          , Launch_ALT)
    SEND_ITEM(BMPcf               , SEALEVELPRESSURE_HPA)
    SEND_ITEM(launch_lat          , launch_lat)
    SEND_ITEM(launch_lon          , launch_lon)
    SEND_ITEM(land_lat            , land_lat)
    SEND_ITEM(land_lon            , land_lon)

    SEND_ITEM(Apogee_Passed       , Apogee_Passed)
    SEND_ITEM(run_time            , run_time)
    SEND_ITEM(up                  , IS_RISING)
    SEND_ITEM(down                , IS_FALLING)
    SEND_ITEM(gps_n               , gps_n)
    SEND_ITEM(gps_e               , gps_e)
    
    if(state==STAND_BY){
      SEND_ITEM(status, "STAND_BY" )
    }
    if(state==TERMINAL_COUNT){
      SEND_ITEM(status, "TERMINAL_COUNT" )
    }
    if(state==POWERED_ASCENT){
      SEND_ITEM(status, "POWERED_ASCENT" )
    }
    if(state==UNPOWERED_ASCENT){
      SEND_ITEM(status, "UNPOWERED_ASCENT" )
    }
    if(state==FREEFALL){
      SEND_ITEM(status, "FREEFALL" )
    }
    if(state==DROGUE_DESCENT){
      SEND_ITEM(status, "DROGUE_DESCENT" )
    }
    if(state==MAIN_DESCENT){
      SEND_ITEM(status, "MAIN_DESCENT" )
    }
    if(state==LANDED){
      SEND_ITEM(status, "LANDED" )
    }
    
    //SEND_ITEM(status            , int(state) )
    //SEND_ITEM(status            , char(state) )
    
    END_SEND
    // */

    radiotimer=millis();
  }


  // Writing to SD Card
  if(millis()-sdtimer > sd_dt){
    //writing abs time,sys date,sys time
    dataFile.print(millis());           dataFile.print(',');
    //might need to do: dataFile.print(year()  ,DEC);   DEC format
    dataFile.print(year());   dataFile.print('/');
    dataFile.print(month());   dataFile.print('/');
    dataFile.print(day());   dataFile.print(',');
    dataFile.print(hour());   dataFile.print(':');
    dataFile.print(minute());   dataFile.print(':');
    dataFile.print(second());
    //writing sensor values
    WRITE_CSV_ITEM(heading)
    WRITE_CSV_ITEM(attitude)
    WRITE_CSV_ITEM(bank)
    WRITE_CSV_ITEM(accelerometer.x())
    WRITE_CSV_ITEM(gyroscope.x())
    WRITE_CSV_ITEM(bmp_alt)
    WRITE_CSV_ITEM(gps_alt)
    WRITE_CSV_ITEM(bno_alt)
    WRITE_CSV_ITEM(sats)
    WRITE_CSV_ITEM(fix_hdop)
    WRITE_CSV_ITEM(vbatt1)
      //WRITE_CSV_VECTOR_ITEM(gyroscope)
    WRITE_CSV_ITEM(accelerometer.y())
    WRITE_CSV_ITEM(accelerometer.z())
    WRITE_CSV_ITEM(gyroscope.y())
    WRITE_CSV_ITEM(gyroscope.z())
      //WRITE_CSV_VECTOR_ITEM(accelerometer)
    //WRITE_CSV_ITEM(gps_lat) //change to have more precision
    dataFile.print(F(", ")); dataFile.print(gps_lat,8);
    //WRITE_CSV_ITEM(gps_lon) //change to have more precision
    dataFile.print(F(", ")); dataFile.print(gps_lon,8);
    WRITE_CSV_ITEM(gps_vel)
    WRITE_CSV_ITEM(gps_dir)
    WRITE_CSV_ITEM(xy_from_lanch)
    WRITE_CSV_ITEM(dir_from_launch)
    WRITE_CSV_ITEM(xy_to_land)
    WRITE_CSV_ITEM(xy_dir_to_land)
    WRITE_CSV_ITEM(x_to_land)
    WRITE_CSV_ITEM(y_to_land)
    WRITE_CSV_ITEM(bmp_pressure)
    WRITE_CSV_ITEM(bmp_temp)
    WRITE_CSV_ITEM(temp)  //bno temp
    WRITE_CSV_ITEM(q1.w())
    WRITE_CSV_ITEM(q1.x())
    WRITE_CSV_ITEM(q1.y())
    WRITE_CSV_ITEM(q1.z())
    WRITE_CSV_ITEM(test)
    WRITE_CSV_VECTOR_ITEM(euler)
    WRITE_CSV_VECTOR_ITEM(magnetometer)
    WRITE_CSV_ITEM(link2ground)
    WRITE_CSV_ITEM(state)
    WRITE_CSV_ITEM(Apogee_Passed)

    dataFile.println();
    dataFile.flush();
    sdtimer=millis();
  }

}
