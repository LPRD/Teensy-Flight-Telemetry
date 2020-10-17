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
#include <Matrix.h>

#define DEMO 0
#define FLIGHT 2

#define CONFIGURATION FLIGHT    //UPDATE b4 FLIGHT!!!

#define HEARTBEAT_TIMEOUT  5000
unsigned long heartbeat_time = 0;
bool link2ground = 1;
//bool cmd; //for pyro channels
int cmd;
float val; //read value for ground config
bool P1_setting,P2_setting,P3_setting,P4_setting,P5_setting= 0;

//char send1[150], send2[150], send3[150], send4[150]= ""; //For Matlab UI

#if CONFIGURATION == DEMO
#define COUNTDOWN_DURATION 10000 // 10 sec
#else
#define COUNTDOWN_DURATION 60000 // 1 min
#endif

long abort_time, start_time, run_time= 0;
bool ss = true; //ss stands for sensor_status

//char data[10], data_name[20] = "";
char data;
char data_name;

typedef enum {
  STAND_BY,
  TERMINAL_COUNT,   //button activated
  POWERED_ASCENT,   //if (run_time>0) && (state==TERMINAL_COUNT) && (run_time<BURN_TIME) //end based on acceleration TBD
  UNPOWERED_ASCENT, //if (state==POWERED_ASCENT) && (run_time>BURN_TIME)
  FREEFALL,         //if (state==UNPOWERED_ASCENT) && (Apogee_Passed == 1)
  DROGUE_DESCENT,   //if (state==FREEFALL) && (DROGUE_FIRED==1)
  MAIN_DESCENT,     //if (state==DROGUE_DESCENT) && (MAIN_FIRED==1)
  LANDED            //if (state==MAIN_DESCENT) && (bmp_alt < Launch_ALT + 5)
} state_t;

state_t state = STAND_BY;

//bool IS_RISING, IS_FALLING= 0;    //May not need
bool DROGUE_FIRED, MAIN_FIRED =0;
int BURN_TIME= 1600; //ms //Update b4 Flight! J425=1.6s burn time

 /*
// Convenience
#define SET_STATE(STATE) //{    \
    state = STATE;      write_state(#STATE);       \
         \
//}
 */

 // /*
void SET_STATE(state_t STATE){
  //TELEMETRY_SERIAL.println(F("Hello????"));
  state= STATE;
  //TELEMETRY_SERIAL.println(F("Hello???"));
}
// */

void (*reset)(void) = 0;

//GPS setup
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
#define gps_dt 200 //time in ms between samples for neo m8n GPS

//radio setup
#define TELEMETRY_SERIAL Serial //Serial for USB, Serial1 for 3.6, Serial5 or Serial8 for 4.1
// 2/4/20 telem ACSII msgs are ~300 chars long= ~300 bytes ... 57600bits/s / (8bits+2extra)= 5760 bytes/s... = ~19 radio msgs/s MAX
// 2-5 msgs/s is probably fine for the future, so we want ~600-1500 bytes/s, so no lower than 6000-15000 bits/s baud rate
//I'm only showing the math here now b/c eventually we will want to lower the radio baud rates to get better range
#define radio_dt 100 //time in ms between sending telemetry packets
#define read_dt 200 //time in ms between recieving telemetry packets

//BMP388 setup
#define BMP_SCK 32   //pin 27 for Teensy 4.1  //13 og    //LED no longer looks on because it sck won't share the same pin anymore!
#define BMP_MISO 1  //pin 39 for Teensy 4.1   //12 og
#define BMP_MOSI 0  //pin 26 for Teensy 4.1   //11 og
#define BMP_CS 31  //pin 38 for Teensy 4.1    //15 og    
  //UPDATE B4 FLIGHT!!!
  //Calibration Factor for BMP388, chech local pressure b4 flight!
float SEALEVELPRESSURE_HPA= 1014.22; //1013.25;  //in units of 100* Pa
Adafruit_BMP3XX bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);  //software SPI
#define bmp_dt 100 //time in ms between samples for bmp388

//BNO setup       //for Teensy 4.1, try just Adafruit_BNO055(55); or 0x29
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);//ID, Address, Wire   //try using &Wire1 for 4.1
//use the syntax &Wire1, 2,... for SDA1, 2,... //55 as the only argument also works
#define ori_dt 10   //time in ms between orientation updating
#define bno_dt 50.0 //time in ms between samples for bno055

//SD logging setup
File dataFile;
char filename[] = "DATA000.csv";
#define sd_dt 200 //time in ms between data points in csv file logging

//Battery Reading Setup
#define Batt_V_Read 14  //A0    //for Teensy 4.1, use A17 (41) & A16 (40) for batt & charge (grid) voltage
double reading;
double vbatt1;
int voltage_divider_ratio= 11; //(default 6) use 11 if using a 1k resistor instead of a 2k 

//Pin setup
#define LED 13 //Error LED, refers to builtin LED on teensy
#define PYRO1 24  //PWM capable on Teensy 4.1
#define PYRO2 25  //PWM capable on Teensy 4.1
#define PYRO3 26
#define PYRO4 27
#define PYRO5 28        //Camera on/off Pin
#define PWM1 2
#define PWM2 3
#define PWM3 4
#define PWM4 5
#define PWM5 6
#define PWM6 7          //LL1
#define PWM7 8          //LL2
#define PWM8 23 //A9    //LL3
#define PWM9 22 //A8    //LL4
#define PWM10 21 //A7   //EDF
#define PWM11 20 //A6     //Free
#define PWM12 17 //A3   //S1
#define PWM13 16 //A2   //S2
#define PWM14 36 //A17  //S3
#define PWM15 35 //A16  //S4

PWMServo S1;
PWMServo S2;
PWMServo S3;
PWMServo S4;
PWMServo LL1;
PWMServo LL2;
PWMServo LL3;
PWMServo LL4;
PWMServo EDF;

int pos;

int S1_Offset= 25;  //might need to change to double
int S2_Offset= 13;
int S3_Offset= 0;
int S4_Offset= 0;

//Telem Functions
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

//Launch Prep
  //BMP calibration factor is ABOVE in the code ^
float Launch_ALT= 274;  //Launch Alt above sea level in m- UPDATE B4 FLIGHT!!!
float ATST= 50; //m above launch height- UPDATE B4 FLIGHT!!!
  //Apogee Trigger Safety Threshold- apogee detection/(parachute) triggering
  //will not work below this pt
  //carry working gps to points and record positions- UPDATE B4 FLIGHT!!!
float launch_lat= 44.975313;
double launch_lon= -93.232216;
float land_lat= (44.975313+.00035);
float land_lon= (-93.232216+.00035);

//Creating Internal Variables
long gpstimer, radiotimer, readtimer, bmptimer, bnotimer, sdtimer, falltimer, oritimer =0;
float dtGyro= 0;
uint64_t thisLoopMicros, lastGyroUpdate =0;
#define fall_dt 50
//long KALMANtimer= 0;    //KALMAN operations take place w/ acceleration (BNO055)

//Vector<3> ori;
imu::Vector<3> ori;
imu::Vector<3> oriGyro;
imu::Vector<3> gyroscope;
imu::Vector<3> euler;
imu::Vector<3> Acc;
imu::Vector<3> magnetometer;
imu::Quaternion q1; //double qw; // double q[4]; //these won't work the same
imu::Quaternion oriQuat;  //orientation storing quaternion
imu::Quaternion rotQuat;  //new rotation quaternion based on gyro rates * dt for each iteration
double temp;
double sqw;
double sqx;
double sqy;
double sqz;
double unit_;
double test;
//double heading;
//double attitude;
//double bank;
double roll, pitch, yaw, oX, oY, oZ = 0;
double cy, sy, cp, sp, cr, sr;
double dyaw, dpitch, droll;
double bno_x, bno_y, bno_z, bno_vx, bno_vy, bno_vz= 0;
double bno_alt= 0;  //Launch_ALT;     //0;
double bno_alt_last [10]= {0,0,0,0,0,0,0,0,0,0};
double bno_alt_new [10]= {0,0,0,0,0,0,0,0,0,0};
double bno_alt_last_avg= 0;
double bno_alt_new_avg= 0;
int bno_descending_counter= 0;
bool bno_descending= 0;

  //Internal System States for Kalman Filter
double Px, Py, Pz, Vx, Vy, Vz, PzKp, PzCp = 0;
  //State Matrix X
double Xs[6][1]= {{Px},        //Array Format needed to fill XS
                  {Py},
                  {Pz},   //m obove Home/Launch (AGL)
                  {Vx},
                  {Vy},
                  {Vz}};
Matrix<double> XS(6,1,(double*)Xs);   //Matrix Lib Format, see example 4 help
  //double Alt_AGL= XS._entity[0][0]  (up to [5][0])
  //Matrix<int> C;
  //C= A*B;

  //State Covariance Matrix P
double P_[6][6]= {{1,0,0,0,0,0},   //low b/c GPS will give ACTUAL measurements
                 {0,1,0,0,0,0},
                 {0,0,1,0,0,0},
                 {0,0,0,100,0,0}, //high b/c no direct vel measurements
                 {0,0,0,0,100,0},
                 {0,0,0,0,0,100}};
Matrix<double> P(6,6,(double*)P_);

  //Kinematics Matrix A
Matrix<double> A(6,6,'I');

  //Kinematics Matrix B
Matrix<double> B(6,3,0);

  //Measurement Matrix U
Matrix<double> U(3,1,0);

  //Transition Matrix H (shouldn't change)
double H_[3][6]= {{1.0,0,0,0,0,0},
                  {0,1.0,0,0,0,0},
                  {0,0,1.0,0,0,0}};
Matrix<double> H(3,6,(double*)H_);
Matrix<double> H_T= Matrix<double>::transpose(H);

  //Identity Matrix I
Matrix<double> I(6,6,'I');

  //Measurement Covariance Matrix R
double R_[3][3]= {{.0225,0,0},    
                  {0,.0225,0},
                  {0,0,.0225}};
  //increasing the measurement covariances reduces the Kalman gain K
  //...This filters more noise but slows down the filter speed
Matrix<double> R(3,3,(double*)R_);

  //Process Noise Covariance Matrix Q
double noise_ax= 5;
double noise_ay= 5;
double noise_az= 5;
Matrix<double> Q(6,6,0);

  //Difference Matrix Y
Matrix<double> Y_diff(3,1,0);

  //Kalman Gain Matrix K
Matrix<double> K(6,3,0);
Matrix<double> K_denom(3,3,0);

  //GPS Measurement Matrix Z
Matrix<double> Z_Meas(3,1,0);









double dt= bno_dt/1000.0;   //.05
double Kdt, dt2, dt3, dt4;
double Kt0, Kt= 0; 
long GPS_Fixes, GPS_Fixes_prev= 0;

//other variables
double bmp_temp;
double bmp_pressure;
  //can review alt data to get bmp vel, accel
double bmp_alt_last [20]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
double bmp_alt_last_avg, bmp_alt_new_avg, bmp_alt= 0;
int bmp_descending_counter, bmp_descending_counter2= 0;
bool bmp_descending= 0;
//bool bmp_descending2= 0;

int16_t sats;
float fix_hdop; //horiz. diminution of precision
double gps_lat, gps_lon, gps_alt;
  //Cardinal= in the N/E/S/W plane, NO up/down component!
double gps_vel, gps_dir; //abs Cardinal course in deg, N=0, E=90...
double x_from_launch, y_from_launch; //Cardinal dist in m from launch pt
double x_to_land, y_to_land; //Cardinal dist in m to land pt
  //Not as important
double dir_from_launch; //Cardinal dir from launch pt in deg, N=0, E=90...
double xy_to_land; //Cardinal distance in m to land pt
double xy_dir_to_land; //Cardinal direction in deg to land pt

double gps_alt_last [10]= {0,0,0,0,0,0,0,0,0,0};
double gps_alt_new [10]= {0,0,0,0,0,0,0,0,0,0};
double gps_alt_last_avg, gps_alt_new_avg, sum= 0;
int gps_descending_counter= 0;
bool gps_descending, Apogee_Passed=0;


//Functions
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
    //TELEMETRY_SERIAL.println(F("Hello?"));
    start_time = millis();
    abort_time= 0;
    heartbeat();
  }
}

void heartbeat() {
  heartbeat_time = millis();
  //TELEMETRY_SERIAL.println(F("Hello??"));
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

  Wire1.begin();   //I was able to successfully use Wire.begin instead of Wire1.begin 
  //(might need to experiment w/ Teensy 4.1 if I wanted to use one of the 3 other I2C busses for some reason
  //alll that really matters is that I can use the pins I want
  //using Wire1.begin() without manually setting pins also works!
  
  //Wire.setSDA(38); //use SDA1 for Teensy 3.6
  //Wire.setSCL(37); //use SCL1 for Teensy 3.6
  //Wire.setSDA(17); //use SDA1 for Teensy 4.1
  //Wire.setSCL(16); //use SCL1 for Teensy 4.1

  //Similarly, SPI.setMOSI(pin), SPI.setMISO(pin), and SPI.setSCK(pin)
  SPI1.begin(); 
  
  setSyncProvider(getTeensy3Time);
  Serial2.begin(GPSBaud); //Serial2 is the radio        pins 9&10 are no longer serial on Teensy 4.1
  //use serial8 and serial5 for radio and GPS (whichever is closer for each respectively)
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
  bno.setMode(Adafruit_BNO055::OPERATION_MODE_NDOF);  //OPERATION_MODE_NDOF_FMC_OFF, see .cpp for all modes "AMG" is the non-fusion mode with all sensors
  //bno.setMode(Adafruit_BNO055::OPERATION_MODE_AMG);
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
        dataFile.print(F("abs time,sys date,sys time,yaw (psy),pitch (theta),roll (phi),Px,Py,Pz,oX,oY,oZ,bmp alt,gps alt,bno alt"));
        dataFile.print(F(",sats,hdop,vbatt1,x accel,y accel,z accel,x gyro,y gyro,z gyro"));
        dataFile.print(F(",gps lat,gps lon,gps vel,gps dir,x_from_launch,y_from_launch,dir_from_launch"));
        dataFile.print(F(",xy_to_land,xy_dir_to_land,x_to_land,y_to_land"));
        dataFile.println(F(",bmp pressure,bmp temp,bno temp,qw,qx,qy,qz,test,x euler,y euler,z euler,x mag,y mag,z mag,l2g,System State,Apogee Passed,dtGyro"));
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

  S1.attach(PWM1);  //17
  //S1.attach(SERVO_PIN_A, 1000, 2000); //some motors need min/max setting ,ESCs go 1k-2k
  S2.attach(PWM2);  //16
  S3.attach(PWM14);  //36
  S4.attach(PWM15);  //35
  LL1.attach(PWM6);  //7
  LL2.attach(PWM7);  //8
  LL3.attach(PWM8);  //23
  LL4.attach(PWM9);  //22
  EDF.attach(PWM11); //21

  for(int q=0; q<5;q++){
    digitalWrite(LED,LOW); delay(200); digitalWrite(LED,HIGH);delay(200);
  }
  //Automatically set values for flight

  //smartDelay(1000*10);      //Fixed time method

  do{
    //delay(1000);
    smartDelay(1000);
    digitalWrite(LED,LOW); delay(500); digitalWrite(LED,HIGH);delay(250);
    reading= analogRead(Batt_V_Read);
    vbatt1= reading*(3.3/1023.00)* voltage_divider_ratio;
    sats= gps.satellites.value();
    fix_hdop= gps.hdop.hdop();
    SEND_ITEM(sats                , sats)
    SEND_ITEM(hdp                 , fix_hdop)
    SEND_ITEM(vb1                 , vbatt1)
    SEND_ITEM(gps_fix             , gps.location.isValid() )
    TELEMETRY_SERIAL.print(F("\n"));
    
  }while( (!(gps.location.isValid())) || (gps.hdop.hdop() > 4.0) || (gps.satellites.value() < 5) );

    gps_alt= gps.altitude.meters();
    launch_lat= gps.location.lat();
    launch_lon= gps.location.lng();
    land_lat= launch_lat + 0.0002;
    land_lon= launch_lon + 0.0002;

  //Ready Lights
  for(int q=0; q<20;q++){
    digitalWrite(LED,LOW); delay(50); digitalWrite(LED,HIGH);delay(50);
  }

  //Lower Landing Legs
  for(pos = 0; pos <=180; pos += 18){
      LL1.write(pos);
      LL2.write(pos);
      LL3.write(pos);
      LL4.write(pos);
      delay(375);
  }

  //Actuate the Fins
  //1
  for(pos = 60; pos <=120; pos += 5){
      S1.write(pos);
      delay(50);
  }
  delay(100);
  for(pos = 120; pos >=60; pos -= 20){
      S1.write(pos);
      delay(200);
  }
  delay(200);
    for(pos = 60; pos <=120; pos += 5){
      S2.write(pos);
      delay(50);
  }
  delay(100);
  for(pos = 160; pos >=60; pos -= 20){
      S2.write(pos);
      delay(200);
  }
  delay(200);
  for(pos = 60; pos <=120; pos += 5){
      S3.write(pos);
      delay(50);
  }
  delay(100);
  for(pos = 120; pos >=60; pos -= 20){
      S3.write(pos);
      delay(200);
  }
  delay(200);
    for(pos = 60; pos <=120; pos += 5){
      S4.write(pos);
      delay(50);
  }
  delay(100);
  for(pos = 160; pos >=60; pos -= 20){
      S4.write(pos);
      delay(200);
  }



  //2
  for(pos = 45; pos <=135; pos += 2){
      S1.write(pos);
      S3.write(pos);
      delay(15);
  }
  delay(100);
  for(pos = 135; pos >=45; pos -= 10){
      S1.write(pos);
      S3.write(pos);
      delay(80);
  }
  delay(200);
    for(pos = 45; pos <=135; pos += 2){
      S2.write(pos);
      S4.write(pos);
      delay(15);
  }
  delay(100);
  for(pos = 135; pos >=45; pos -= 10){
      S2.write(pos);
      S4.write(pos);
      delay(80);
  }

  //all
  for(pos = 0; pos <=180; pos += 1){
      //analogWrite(17,pos);
      S1.write(pos);
      //analogWrite(16,pos);
      S2.write(pos);
      S3.write(pos);
      S4.write(pos);
      delay(10);
  }
  delay(200);
  for(pos = 180; pos >=90; pos -= 5){
      S1.write(pos + S1_Offset);
      S2.write(pos + S2_Offset);
      S3.write(pos + S3_Offset);
      S4.write(pos + S4_Offset);
      delay(50);
  }


  //things to run before starting the main loop:
  
  q1 = bno.getQuat();   //I don't think this needs to be run here, oriQuat can still equal q1 even if q1 doesn't have values yet, right?
  lastGyroUpdate= micros();
}







void loop() {
  
  if(millis()-oritimer > ori_dt){
    thisLoopMicros= micros();
    dtGyro = (float)(thisLoopMicros - lastGyroUpdate) / 1000000;
    lastGyroUpdate = thisLoopMicros;
    
    //get gyro data all the time for logging:
    gyroscope     = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);     //100hz (?), dps automatically set w/ ndof fusion mode
    //it might be a good idea to set the BNO to a non-fusion mode after launch so that the dps can be set to 250 vs 2000 for more accurate gyro orientation
    //on the other hand, it's probably best to keep the fusion on and just use a BMI088 to get super accurate vibration resistant gyro readings anyways. In addition, the fusion data
    //from the BNO can be compared to BMI088 after various flights

    if(1){        //state==STAND_BY || state==TERMINAL_COUNT      If I can ever get the gyro code working...
      //oX/oY/oZ each respectively represent orientation about the x/y/z body axis
      oX= roll;
      oY= pitch;
      oZ= yaw;
      //use sensor fusion quat for current orientation
      oriQuat= q1;
    }
    else{
      //convert gyro rates to euler rates to a quaternion
      dyaw =   ( gyroscope.x() )*dtGyro;   //.z *should* be the rads/s rate about the z axis of the BMI055, so yaw rate
      dpitch = ( gyroscope.y() )*dtGyro;   //.y *should* be the rads/s rate about the y axis of the BMI055, so pitch rate
      droll =  ( gyroscope.z() )*dtGyro;   //.x *should* be the rads/s rate about the z axis of the BMI055, so roll rate

      cy = cos(dyaw * 0.5);
      sy = sin(dyaw * 0.5);
      cp = cos(dpitch * 0.5);
      sp = sin(dpitch * 0.5);
      cr = cos(droll * 0.5);
      sr = sin(droll * 0.5);
      
      //rotQuat.w()= cr * cp * cy + sr * sp * sy;
      //rotQuat.x()= sr * cp * cy - cr * sp * sy;
      //rotQuat.y()= cr * sp * cy + sr * cp * sy;
      //rotQuat.z()= cr * cp * sy - sr * sp * cy;

      imu::Quaternion rotQuat(cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy);
      //imu::Quaternion oriQuat(cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy);
      //oriQuat(cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy);  //does NOT work

      
      //oriQuat= rotQuat*oriQuat;
      oriQuat= oriQuat*rotQuat;   //update the orientation storing quaternion
      
      oriGyro= oriQuat.toEuler(); //convert the ori storing quat to euler angles (no gimbal lock issues!)
      oX= RAD_TO_DEG*oriGyro.z();   //oX is roll, .z() represents the 3rd euler rotation which is roll
      oY= RAD_TO_DEG*oriGyro.y();   //oY is pitch, .y() represents the 2nd euler rotation which is pitch
      oZ= RAD_TO_DEG*oriGyro.x();   //oZ is yaw, .x() represents the 1rd euler rotation which is yaw
    }

    //Run the GPS encode function (not directly related to orientation, but want to run this frequently as well)
    smartDelay(5);  //will make a separate GPS encode timer that smart delays for ~1ms and runs every ~3+ ms (?)
    
    oritimer= millis();
  }
  
  if(millis()-bnotimer > bno_dt){
    //gyroscope     = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE); 
    euler         = bno.getVector(Adafruit_BNO055::VECTOR_EULER);         //sensor fusion
    Acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    magnetometer  = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    q1 = bno.getQuat(); //qw= quat.w(); or q[4]=[quat.w(),quat.x()...     //sensor fusion
    temp = bno.getTemp();

/*
    sqw = q1.w()*q1.w();
    sqx = q1.x()*q1.x();
    sqy = q1.y()*q1.y();
    sqz = q1.z()*q1.z();
    unit_ = sqx + sqy + sqz + sqw; // if normalised is one, otherwise is correction factor
    test = q1.x()*q1.y() + q1.z()*q1.w();
    //when test is close to +/- .5, the imu is close to an euler singularity
    if (test > 0.499*unit_) { // singularity at north pole, >86.3 deg
      heading = 2 * atan2(q1.x(),q1.w());
      attitude = PI/2;
      bank = 0;
    }
    else if (test < -0.499*unit_) { // singularity at south pole, <-86.3 deg
      heading = -2 * atan2(q1.x(),q1.w());
      attitude = -PI/2;
      bank = 0;
    }
    else{
    heading = RAD_TO_DEG*atan2(2*q1.y()*q1.w()-2*q1.x()*q1.z() , sqx - sqy - sqz + sqw); //PITCH
    attitude = RAD_TO_DEG*asin(2*test/unit_);     //YAW
    bank = RAD_TO_DEG*atan2(2*q1.x()*q1.w()-2*q1.y()*q1.z() , -sqx + sqy - sqz + sqw);  //ROLL
    }
*/
    //use built in quat function to get Euler Angles
    ori= q1.toEuler();
    yaw= RAD_TO_DEG*ori.x();    //x represents the 1rd rotation, not rotation about the x axis (which would be roll)
    pitch= RAD_TO_DEG*ori.y();  //y represents the 2rd rotation, not rotation about the y axis (although that would also be pitch)
    roll= RAD_TO_DEG*ori.z();   //z represents the 3rd rotation, not rotation about the z axis (which would be yaw)

    
    
    //***KALMAN Filter Code***********
      //also do Accel/GPS/BMP KF here (once GPS gets new data, that's the bottleneck)
    Kt= millis()/1000.0;    //time in s
    Kdt= Kt- Kt0; //dt for Kalman Filter, in s
      //technically I should be able to use (bno_dt/1000= .05) for Kdt
    if(Kdt > 10.0){    //can increase to 100 if necessary- see log
      Kdt= 10.0;
    }
    Kt0= Kt;

    dt2= Kdt*Kdt;
    dt3= dt2*Kdt;
    dt4= dt3*Kdt;
    
    //Update time step dependent matricies & Accel readings
      //log Kdt*1000 *as a treat* (compare to .05*1000 ms)
    A._entity[0][3]= Kdt;    //or dt (.05 seconds)
    A._entity[1][4]= Kdt;
    A._entity[2][5]= Kdt;

    B._entity[0][0]= .5*dt2;    //or dt (.05 seconds)
    B._entity[1][1]= .5*dt2;
    B._entity[2][2]= .5*dt2;
    B._entity[3][0]= Kdt;
    B._entity[4][1]= Kdt;
    B._entity[5][2]= Kdt;
     
    Q._entity[0][0] = dt4/4.0*noise_ax;
    Q._entity[0][3] = dt3/2.0*noise_ax;
    Q._entity[1][1] = dt4/4.0*noise_ay;
    Q._entity[1][4] = dt3/2.0*noise_ay;
    Q._entity[2][2] = dt4/4.0*noise_az;
    Q._entity[2][5] = dt3/2.0*noise_az;
    Q._entity[3][0] = dt3/2.0*noise_ax;
    Q._entity[3][3] = dt2*noise_ax;
    Q._entity[4][1] = dt3/2.0*noise_ay;
    Q._entity[4][4] = dt2*noise_ay;
    Q._entity[5][2] = dt3/2.0*noise_az;
    Q._entity[5][5] = dt2*noise_az;    

      //Note the ordering & frame change
      //Acc.x() is the sensor x direction (facing up)
      //might need to still change things around
    U._entity[0][0]= Acc.x(); //earth fixed x Accel
    U._entity[1][0]= Acc.y(); //earth fixed y Accel
    U._entity[2][0]= (Acc.z()*cos(DEG_TO_RAD*0)*cos(DEG_TO_RAD*0)) -9.81; //earth fixed z Accel
    
    //predict new altitude (XS continually updates)
    XS= (A*XS)+(B*U);
    P= ((A*P)*Matrix<double>::transpose(A))+Q;
    
    
  
    //Update States w/ Kalman Filter
    GPS_Fixes= gps.sentencesWithFix();
    if(gps.location.isValid() && (GPS_Fixes > GPS_Fixes_prev) ){
      gps_lon= gps.location.lng();
      gps_lat= gps.location.lat();
      x_from_launch= TinyGPSPlus::distanceBetween(0, launch_lon, 0, gps_lon);
      y_from_launch= TinyGPSPlus::distanceBetween(launch_lat, 0, gps_lat, 0);
      gps_alt= gps.altitude.meters() - Launch_ALT;
      Z_Meas._entity[0][0]= x_from_launch;
      Z_Meas._entity[1][0]= y_from_launch;
      Z_Meas._entity[2][0]= gps_alt;
      Y_diff= Z_Meas- H*XS;

      K_denom= ( ((H*P)*H_T) + R );
      K= (P*H_T)*Matrix<double>::inv(K_denom);

      //Calculate the new filtered states (only position updates)
      XS= XS+ (K*Y_diff);
      
      //Calculate the new P matrix
      P= (I-(K*H))*P;

      GPS_Fixes_prev= GPS_Fixes;
    }
    
    Px= XS._entity[0][0];
    Py= XS._entity[1][0];
    Pz= XS._entity[2][0]; //m AGL
    bno_alt= Launch_ALT + Pz;  //m ASL




      //Apogee Detection
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

    bmp_alt= bmp.readAltitude(SEALEVELPRESSURE_HPA); //m ASL     //estimate part
    //Because bmp_alt is already ASL, I can directly compare to gps_alt

    //Kalman filter for barometric/bmp alt












    //Complementary Filter for barometric/bmp alt







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
      //IS_RISING= 0;
      //IS_FALLING= 1;
    }

 /*
    if((bmp_alt_last[19] > bmp_alt_last[0]) && (bmp_alt > (Launch_ALT + ATST))){
      bmp_descending_counter2= bmp_descending_counter2 + 1;
    }

    if((bmp_alt_last_avg < bmp_alt_new_avg)){
      //IS_RISING= 1;
      //IS_FALLING= 0;
    }
    else if( abs(bmp_alt_last_avg - bmp_alt_new_avg) < .4){  //vertical velocity less than .3 m over the past avg sec
      //IS_RISING= 0;
      //IS_FALLING= 0;
    }
    else{
      //IS_RISING= 0;
      //IS_FALLING= 1;
    }
 */

    bmptimer= millis();
  }

  if(bmp_descending != 1){
    if((bmp_descending_counter>9) && (bmp_alt > Launch_ALT + ATST)){
      bmp_descending = 1;
    }
  }

 /*
  if(bmp_descending2 != 1){
    if((bmp_descending_counter2 >9) && (bmp_alt > Launch_ALT + ATST)){
      bmp_descending2 = 1;
    }
  }
 */

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
    x_from_launch= TinyGPSPlus::distanceBetween(0, launch_lon, 0, gps_lon);
    y_from_launch= TinyGPSPlus::distanceBetween(launch_lat, 0, gps_lat, 0);
    dir_from_launch= TinyGPSPlus::courseTo(launch_lat, launch_lon, gps_lat, gps_lon);
    xy_to_land= TinyGPSPlus::distanceBetween(gps_lat, gps_lon, land_lat, land_lon);
    xy_dir_to_land= TinyGPSPlus::courseTo(gps_lat, gps_lon, launch_lat, launch_lon);
    x_to_land= TinyGPSPlus::distanceBetween(0, gps_lon, 0, land_lon);
    y_to_land= TinyGPSPlus::distanceBetween(gps_lat, 0, land_lat, 0);

    //smartDelay(100);
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
    if ( (run_time>0) && (state==TERMINAL_COUNT) && (run_time<BURN_TIME) ){ //FIX!
      SET_STATE(POWERED_ASCENT);
      //launch rocket
      
      //digitalWrite(PYRO3,HIGH);
      //DROGUE_FIRED= 1;
      //P3_setting=1;
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
      if((gps_descending*1)+(bmp_descending*1)+(bno_descending*1) > 1){ //+(bmp_descending2*1)
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

      if(bmp_alt < Launch_ALT + 250){   //eject main at 250m
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




  //KALMAN Functions here









  //Read Commands (STAND_BY / TERMINAL_COUNT aka on pad)
  if(millis()-readtimer > read_dt){

    BEGIN_READ
    READ_FLAG(c) {
      heartbeat();
      
    }
    READ_FLAG(r) {
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
    if (millis() > (heartbeat_time + HEARTBEAT_TIMEOUT) ) {
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
    sprintf(send2,",%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f",yaw,pitch,roll,magnetometer.x(),magnetometer.y(),magnetometer.z(),gyroscope.x(),gyroscope.y(),gyroscope.z(),Acc.x(),Acc.y(),Acc.z());
    sprintf(send3,",%.1f,%.1f,%.1f,%.1f,%.1f,%u,%u,%u,%u,%u,%u,%.1f,%.1f,%.1f,%.5f,%.5f,%.5f,%.5f",temp,gps_vel,gps_dir,x_from_launch,y_from_launch,dir_from_launch,run_time,P1_setting,P2_setting,P3_setting,P4_setting,P5_setting,ATST,Launch_ALT,SEALEVELPRESSURE_HPA,launch_lat,launch_lon,land_lat,land_lon);
    sprintf(send4,",%u,%u,%u,%u,%s,%u,%u,%u,&&&&&",gps_descending,bmp_descending,bno_descending,state,Apogee_Passed,link2ground,ss);
    TELEMETRY_SERIAL.print(send1);
    TELEMETRY_SERIAL.print(send2);
    TELEMETRY_SERIAL.print(send3);
    TELEMETRY_SERIAL.println(send4);
      */

    //Send Fomat for Python UI (uncomment as needed)
    // /*
    BEGIN_SEND
    //SEND_VECTOR_ITEM(euler_angle  , euler)
    SEND_VECTOR_ITEM(gyro         , gyroscope)
    SEND_ITEM(tIMU         , temp)   //BNO055 Temp
    SEND_VECTOR_ITEM(magnetometer , magnetometer)
    SEND_VECTOR_ITEM(acceleration , Acc)
    SEND_ITEM(bmp_alt             , bmp_alt)
    SEND_ITEM(bno_alt             , bno_alt)
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
    SEND_ITEM(roll                , roll)
    SEND_ITEM(pitch               , pitch)
    SEND_ITEM(yaw                 , yaw)
    SEND_ITEM(Px                  , Px)
    SEND_ITEM(Py                  , Py)
    SEND_ITEM(Pz                  , Pz)
    SEND_ITEM(oX                  , oX)
    SEND_ITEM(oY                  , oY)
    SEND_ITEM(oZ                  , oZ)
    //SEND_ITEM(test                , test)
    SEND_ITEM(gps_vel             , gps_vel)
    SEND_ITEM(gps_dir             , gps_dir)
    SEND_ITEM(x_from_launch        , x_from_launch)
    SEND_ITEM(y_from_launch        , y_from_launch)
    SEND_ITEM(dir_from_launch     , dir_from_launch)
    SEND_ITEM(sats                , sats)
    SEND_ITEM(hdp                 , fix_hdop)
    SEND_ITEM(vb1                 , vbatt1)
    SEND_ITEM(ss                  , ss)
    SEND_ITEM(l2g                 , link2ground)
    SEND_ITEM(P1_setting          , P1_setting)
    SEND_ITEM(P2_setting          , P2_setting)
    SEND_ITEM(P3_setting          , P3_setting)
    SEND_ITEM(P4_setting          , P4_setting)
    SEND_ITEM(P5_setting          , P5_setting)
    SEND_ITEM(Apogee_Passed       , Apogee_Passed)

    SEND_ITEM(ATST                , ATST)
    SEND_ITEM(Launch_ALT          , Launch_ALT)
    SEND_ITEM(BMPcf               , SEALEVELPRESSURE_HPA)
    SEND_ITEM(launch_lat          , launch_lat)
    SEND_ITEM(launch_lon          , launch_lon)

    SEND_ITEM(Apogee_Passed       , Apogee_Passed)
    SEND_ITEM(run_time            , run_time)

    if(state==STAND_BY){
      SEND_ITEM(status, "STAND_BY" )
    }
    if(state==TERMINAL_COUNT){
      //TELEMETRY_SERIAL.print(F("QQQ?"));
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
    WRITE_CSV_ITEM(yaw)
    WRITE_CSV_ITEM(pitch)
    WRITE_CSV_ITEM(roll)
    WRITE_CSV_ITEM(Px)
    WRITE_CSV_ITEM(Py)
    WRITE_CSV_ITEM(Pz)
    WRITE_CSV_ITEM(oX)
    WRITE_CSV_ITEM(oY)
    WRITE_CSV_ITEM(oZ)
    WRITE_CSV_ITEM(bmp_alt)
    WRITE_CSV_ITEM(gps_alt)
    WRITE_CSV_ITEM(bno_alt)
    WRITE_CSV_ITEM(sats)
    WRITE_CSV_ITEM(fix_hdop)
    WRITE_CSV_ITEM(vbatt1)
      //WRITE_CSV_VECTOR_ITEM(gyroscope)
    WRITE_CSV_ITEM(Acc.x())  
    WRITE_CSV_ITEM(Acc.y())
    WRITE_CSV_ITEM(Acc.z())
    WRITE_CSV_ITEM(gyroscope.x())
    WRITE_CSV_ITEM(gyroscope.y())
    WRITE_CSV_ITEM(gyroscope.z())
      //WRITE_CSV_VECTOR_ITEM(Acc)
    //WRITE_CSV_ITEM(gps_lat) //change to have more precision
    dataFile.print(F(", ")); dataFile.print(gps_lat,8);
    //WRITE_CSV_ITEM(gps_lon) //change to have more precision
    dataFile.print(F(", ")); dataFile.print(gps_lon,8);
    WRITE_CSV_ITEM(gps_vel)
    WRITE_CSV_ITEM(gps_dir)
    WRITE_CSV_ITEM(x_from_launch)
    WRITE_CSV_ITEM(y_from_launch)
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
    WRITE_CSV_ITEM(dtGyro)
    
    dataFile.println();
    dataFile.flush();
    sdtimer=millis();
  }

}
