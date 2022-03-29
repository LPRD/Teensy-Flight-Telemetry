#include <Wire.h>                             //Built-in I2C Library
#include <SPI.h>                              //Built-in SPI Library
#include <SD.h>                               //Built-in SD Library
#include <avr/pgmspace.h>                     //Built-in Arduino Memory Libray
#include <Adafruit_Sensor.h>                  //Adafruit General Sensor Library (BNO055 uses this)
#include <Adafruit_BNO055.h>                  //Adafruit BNO055 Library
#include <Adafruit_BMP3XX.h>                  //Adafruit BMP388 Sne
#include <TinyGPS++.h>                        //Previous GPS lib, can still use for GPS coordinate distance math
#include <SparkFun_Ublox_Arduino_Library.h>   //Sparkfun Ublox GPS Library (I2C, Serial)
#include <Telemetry.h>                        //LPRD Telemetry Library
#include <PWMServo.h>                         //Servo Library immune to interrupts (vs normal servo.h)
#include <TimeLib.h>                          //built-in Teensy Time Library
#include <Matrix.h>                           //Open Source Matrix Library









//separate page

//Teensy 4.1 Compatible Restart
//#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
//#define CPU_RESTART_VAL 0x5FA0004
//#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);

//Teensy 3.X Compatible Restart- need to test
#define RESTART_ADDR       0xE000ED0C
#define READ_RESTART()     (*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))

//#define DEMO 0
//#define FLIGHT 2
//#define CONFIGURATION FLIGHT        //UPDATE b4 FLIGHT!!!

#define HEARTBEAT_TIMEOUT  5000     //5 sec
//#define LIFTOFF_ACC_THRESH 13.0     //Liftoff Acceleration Threshold in m/s^2- only arm once stationary on the pad

#define TELEMETRY_SERIAL Serial    //Serial for USB, Serial1 for 3.6, Serial5 or Serial8 for 4.1

#define Timer10ms   9    //will trigger when dt = 10 or higher
//#define BMP_SCK 27
//#define BMP_MISO 39
//#define BMP_MOSI 26
//#define BMP_CS 38  
//#define Grid_V 40                   //A16
//#define Batt_V 41                   //A17
#define LED 13 //Error LED, refers to builtin LED on teensy
//#define PYRO1 33  //PWM capable on Teensy 4.1
//#define PYRO2 36  //PWM capable on Teensy 4.1
//#define PYRO3 37  //PWM capable on Teensy 4.1
//#define PYRO4 14  //PWM capable on Teensy 4.1
//#define PYRO5 15  //PWM capable on Teensy 4.1
//#define PYRO5V 32
//#define PWM1 0
//#define PWM2 1
//#define PWM3 2
//#define PWM4 3
//#define PWM5 4
//#define PWM6 5         
//#define PWM7 5
//#define PWM8 7
//#define PWM9 8
//#define PWM10 9
//#define PWM11 10
//#define PWM12 11
//#define PWM13 12
//#define PWM14 24
//#define PWM15 25
//#define PWM16 28
//#define PWM17 29
//#define PWM18 23
//#define PWM19 22
//#define PWM20 19

//Telem Functions
#define SEND_VECTOR_ITEM(field, value)\
  SEND_ITEM(field, value.x())         \
  SEND_GROUP_ITEM(value.y())          \
  SEND_GROUP_ITEM(value.z())

#define WRITE_CSV_ITEM(value)         \
  dataFile.print(F(", ")); dataFile.print(value);
  //dataFile.printf(", %d",value); //would not work, format specifier depends on data type (if statements...)
  
#define WRITE_CSV_VECTOR_ITEM(value)  \
  WRITE_CSV_ITEM(value.x())           \
  WRITE_CSV_ITEM(value.y())           \
  WRITE_CSV_ITEM(value.z())


//Define State Names
typedef enum {
  STAND_BY,
  ARMED,            //button activated
  POWERED_ASCENT,   //if (state==ARMED) && (Data10[dataIndex/10].accZ > LIFTOFF_ACC_THRESH) )
  UNPOWERED_ASCENT, //if (state==POWERED_ASCENT) && (Data10[dataIndex/10].run_time>BURN_TIME)
  FREEFALL,         //if (state==UNPOWERED_ASCENT) && (Data10[dataIndex/10].Apogee_Passed == 1)
  DROGUE_DESCENT,   //if (state==FREEFALL) && (Data10[dataIndex/10].DROGUE_FIRED==1)
  MAIN_DESCENT,     //if (state==DROGUE_DESCENT) && (Data10[dataIndex/10].MAIN_FIRED==1)
  LANDED            //if (state==MAIN_DESCENT) && (Data10[dataIndex/10].bmp_alt < Launch_ALT + 5)
} state_t;

state_t state = STAND_BY;

//*******************Define data types that holds important information***********************************************

#define Num_Datapoints 400       //hold the last Num_Datapoints*100ms data (NEEDS TO BE DIVISIBLE BY 25, 10, 4, 1!!!!!)
uint64_t Timer100Hz= 0;           //millis timer that keeps track of the 10ms timer loop
uint32_t dataLoopIteration= 0;    //number of 10ms iterations have passed since starting void loop
uint32_t dataIndex= 0;            //index of Data array to store everything in for the moment
uint32_t i= 0;                    //shorthand for dataIndex
int dataIndexSigned;

struct data100hz_t{
  //BNO055/orientation
  double dtGyro;                            //in seconds
  double gyroX, gyroY, gyroZ;
  double oriQuatW, oriQuatX, oriQuatY, oriQuatZ;    //gyro orientation based quaternion values
  double oX, oY, oZ;
  //BMI088/orientation
  
};

struct data10hz_t{
  int state; 
  bool ss;                                //ss is sensor status
  uint32_t abort_time;
  int32_t run_time;                       //can be negative if countdown is set
  bool link2ground;
  //Kalman Filter
  float Px, Py, Pz, Vx, Vy, Vz; //need to be set to 0
  //GPS
  bool gps_descending;
  uint8_t fixType, SIV;                   //uint8_t in library
  float PDOP;                             //Positional diminution of precision
  double gps_lat, gps_lon, gps_alt;
    //Below values are Cardinal= in the N/E/S/W plane, NO up/down component!
  float gps_vel, gps_dir;                 //abs Cardinal course in deg, N=0, E=90...
  double x_from_launch, y_from_launch;    //Cardinal dist in m from launch pt
  double x_to_land, y_to_land;            //Cardinal dist in m to land pt
    //Not as important
  double xy_from_launch;
  double dir_from_launch;                 //Cardinal dir from launch pt in deg, N=0, E=90...
  double xy_to_land;                      //Cardinal distance in m to land pt
  double dir_to_land;                     //Cardinal direction in deg to land pt
  //BNO055
  float mx, my, mz;
  double eulerX, eulerY, eulerZ;
  double accX, accY, accZ;
  double qW, qX, qY, qZ;                  //sensor fusion quaternion values
  float bno_alt;                  //m AGL;
  double roll, pitch, yaw;
  //BMP388
  float bmp_pressure, bmp_alt;
  //fall
  bool bno_descending, bmp_descending;
  bool Apogee_Passed;
};

struct data1hz_t{
  //Flow Control
  float gpsRate, radioRate, readRate, bmpRate, bnoRate, sdRate, fallRate, oriRate;    //All in Hz
  //double gpsT, radioT, readT, bmpT, bnoT, sdT, fallT, oriT;                            //All in ms
  uint64_t gpsT, radioT, readT, bmpT, bnoT, sdT, fallT, oriT;                            //All in us
  uint16_t y;
  uint8_t mo, d, h, m, s;
  //Logistical
  float vbatt, vgrid;
  //BNO
  float temp;                     //bno temp
  //BMP388
  float bmp_temp;
  bool P1_setting, P2_setting, P3_setting, P4_setting, P5_setting, P5V_setting;
};

//Create arrays of various data type objects to store the last x seconds of data in flash memory

//data100hz_t Data100[Num_Datapoints] ;
data10hz_t  Data10[Num_Datapoints/10] ;
data1hz_t   Data1[Num_Datapoints/100] ;

//**********END Data Types*****************************************************************


//Initialize Sensors
//SFE_UBLOX_GPS myGPS;
//Adafruit_BMP3XX bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
//Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire2);    //ID, Address, Wire

//Cricket
/*
PWMServo S1;
PWMServo S2;
PWMServo S3;
PWMServo S4;
PWMServo LL1;
PWMServo LL2;
PWMServo LL3;
PWMServo LL4;
PWMServo EDF;

//can be negative
int pos;
int S1_Offset= 25;  //might need to change to double
int S2_Offset= 13;
int S3_Offset= 0;
int S4_Offset= 0;
*/

//******************Internal Variables (not recorded over time)*********************************
//Timers
uint32_t gpstimer=0, gpsCount=0, radiotimer=0, radioCount=0, readtimer=0, readCount=0, bmptimer=0, bmpCount=0, bnotimer=0, bnoCount=0, sdtimer=0, sdCount=0, falltimer=0, fallCount=0, oritimer=0, oriCount=0, heartbeat_time=0;
uint64_t startTimeMicros=0, lastGyroUpdate=0, startTime=0, loopIteration=0;
uint64_t thisLoopMillis=0, thisLoopMicros=0, thisLoopMicros2=0;

//SD
File dataFile;
char filename[] = "DATA000.csv";
bool DataWrittenOnLanding= 0;

//Batt
//double reading;
//uint32_t voltage_divider_ratio= 21;                         //20k drop resistor, 1k gnd resistor

//Launch Config
float Launch_ALT= 274.0;                                    //Launch Alt above sea level in m- UPDATE B4 FLIGHT!!!
float ATST= 50;                                             //Apogee Trigger Safety Threshold- m above launch altitude,  UPDATE B4 FLIGHT!!!
double launch_lat, launch_lon, land_lat, land_lon;
float SEALEVELPRESSURE_HPA= 1014.22;                        //1013.25;  //in units of 100* Pa

//Read Values
int cmd;
float val;

//Send Values
char data;
char data_name;

//quaternion math variables
//double sqw, sqx, sqy, sqz, unit_, test, cy, sy, cp, sp, cr, sr; 
//imu::Quaternion rotQuat;

//BNO
//double GyroAvgX=0, GyroAvgY=0, GyroAvgZ=0;
////uint16_t NumPrevGyroPts= 500;     //100 corresponds to 1 second, NumPrevDataPtIndices does the same thing
//double dyaw, dpitch, droll;       //gyro math variables
//double bno_alt_last [10]= {0,0,0,0,0,0,0,0,0,0};
//double bno_alt_new [10]= {0,0,0,0,0,0,0,0,0,0};
//double bno_alt_last_avg= 0;
//double bno_alt_new_avg= 0;
//int bno_descending_counter= 0;
//imu::Vector<3> magnetometer;
//imu::Vector<3> ori;
//imu::Vector<3> oriGyro;
//imu::Vector<3> gyroscope;
//imu::Vector<3> euler;
//imu::Vector<3> Acc;
//imu::Quaternion q1;
//imu::Quaternion oriQuat;        //orientation storing quaternion

//KALMAN Filter variables
  //State Covariance Matrix P
//double P_[6][6]= {{1,0,0,0,0,0},   //low b/c GPS will give ACTUAL measurements
//                 {0,1,0,0,0,0},
//                 {0,0,1,0,0,0},
//                 {0,0,0,100,0,0}, //high b/c no direct vel measurements
//                 {0,0,0,0,100,0},
//                 {0,0,0,0,0,100}};
//Matrix<double> P(6,6,(double*)P_);
//  //Kinematics Matrix A
//Matrix<double> A(6,6,'I');
//  //Kinematics Matrix B
//Matrix<double> B(6,3,0);
//  //Measurement Matrix U
//Matrix<double> U(3,1,0);
//  //Transition Matrix H (shouldn't change)
//double H_[3][6]= {{1.0,0,0,0,0,0},
//                  {0,1.0,0,0,0,0},
//                  {0,0,1.0,0,0,0}};
//Matrix<double> H(3,6,(double*)H_);
//Matrix<double> H_T= Matrix<double>::transpose(H);
//  //Identity Matrix I
//Matrix<double> I(6,6,'I');
//  //Measurement Covariance Matrix R
//double R_[3][3]= {{.0225,0,0},    
//                  {0,.0225,0},
//                  {0,0,.0225}};
//  //increasing the measurement covariances reduces the Kalman gain K
//  //...This filters more noise but slows down the filter speed
//Matrix<double> R(3,3,(double*)R_);
//  //Process Noise Covariance Matrix Q
//double noise_ax= 5;
//double noise_ay= 5;
//double noise_az= 5;
//Matrix<double> Q(6,6,0);
//  //Difference Matrix Y
//Matrix<double> Y_diff(3,1,0);
//  //Kalman Gain Matrix K
//Matrix<double> K(6,3,0);
//Matrix<double> K_denom(3,3,0);
//  //GPS Measurement Matrix Z
//Matrix<double> Z_Meas(3,1,0);
//  //double Array Xs needed to fill XS
//double Xs[6][1]= {{0},      
//                  {0},
//                  {0},         //m obove Home/Launch (AGL)
//                  {0},
//                  {0},
//                  {0}};
//  //State Matrix XS
//Matrix<double> XS(6,1,(double*)Xs);   //Matrix Lib Format, see example for help
////double dt= .01; //bno_dt/1000.0;   //.01
//double Kdt, dt2, dt3, dt4;
//double Kt0=0, Kt=0; 

//BMP
//double bmp_alt_last [20]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//double bmp_alt_last_avg=0, bmp_alt_new_avg=0;
//int bmp_descending_counter= 0;
//float bmp_pressure, bmp_alt;

//GPS
//double gps_alt_last [10]= {0,0,0,0,0,0,0,0,0,0};
//double gps_alt_new [10]= {0,0,0,0,0,0,0,0,0,0};
//double gps_alt_last_avg=0, gps_alt_new_avg=0;
//int gps_descending_counter= 0;
//uint8_t fixType, SIV;                   //uint8_t in library
//float PDOP;                             //Positional diminution of precision
//double gps_lat, gps_lon, gps_alt;
  //Below values are Cardinal= in the N/E/S/W plane, NO up/down component!
//float gps_vel, gps_dir;                 //abs Cardinal course in deg, N=0, E=90...
//double x_from_launch, y_from_launch;    //Cardinal dist in m from launch pt
//double x_to_land, y_to_land;            //Cardinal dist in m to land pt
//  //Not as important
//double xy_from_launch;
//double dir_from_launch;                 //Cardinal dir from launch pt in deg, N=0, E=90...
//double xy_to_land;                      //Cardinal distance in m to land pt
//double dir_to_land;                     //Cardinal direction in deg to land pt

//Other
double sum=0, sum2=0, sum3=0;
bool bno_descending=0, bmp_descending=0, Apogee_Passed=0, DROGUE_FIRED=0, MAIN_FIRED=0, gps_descending=0;     //HAVE TO HAVE THESE
uint32_t liftoff_time= 0;         //liftoff_time (static time in ms when system state becomes POWERED_ASCENT)
uint32_t BURN_TIME= 1600;         //since this is a variable, I could change it
uint32_t start_time= 0;           //start_time (static time in ms when system state becomes ARMED)
bool P1_setting= 0, P2_setting= 0, P3_setting= 0, P4_setting=0 , P5_setting=0, P5V_setting= 0;
double sdT;
float sdRate= 0;
char SendData[1000];
char AddData[350];
char AddData2[350];
uint64_t testTimer= 0, testT= 0, sumT=0, avgT=0;

//Helper functions  
//void SET_STATE(state_t STATE){
//  state= STATE;
//}

//double Avg_Last_N_Measurements(double prev[], uint32_t N){ //return the average of the last N data points
//  sum=  0;  //ensure that sum is reset
//  for(uint32_t i = 0; i < N; i++){
//    sum += prev[i];
//  }
//  double avg= sum/N;
//  sum=0;
//  return avg;
//}
 
//void start_countdown() {        //currently the countdown is 0 seconds as acceleromter readings will enable POWERED_ASCENT
//  #if CONFIGURATION != DEMO
//    if (!Data10[dataIndex/10].ss) {
//      TELEMETRY_SERIAL.println(F("Countdown aborted due to sensor failure"));
//      SET_STATE(STAND_BY); // Set state to signal countdown was aborted
//    }
//    else{
//      //do nothing
//    }
//  #endif
//  if (0) {  //replace 0 with any unacceptable initial states
//    TELEMETRY_SERIAL.println(F("Countdown aborted due to unexpected initial state"));
//    SET_STATE(STAND_BY); // Set state to signal countdown was aborted
//  }
//  else {
//    TELEMETRY_SERIAL.println(F("Countdown started"));
//    SET_STATE(ARMED);
//    start_time = millis();
//    Data10[dataIndex/10].abort_time= 0;
//    heartbeat();
//    Data10[dataIndex/10].Px= 0;
//    Data10[dataIndex/10].Py= 0;
//    Data10[dataIndex/10].Pz= 0;
//    Data10[dataIndex/10].Vx= 0;
//    Data10[dataIndex/10].Vy= 0;
//    Data10[dataIndex/10].Vz= 0;
//  }
//}

void heartbeat() {
  heartbeat_time = millis();
}

//void write_state(const char *state_name) {    //send state (can be asynchronous)
//  SEND(status, state_name)
//}

//void abort_autosequence() {   //need to check if data is still logged after an abort is triggered
//  TELEMETRY_SERIAL.println(F("Run aborted"));
//  switch (state) {
//    case STAND_BY:
//    case ARMED:
//      SET_STATE(STAND_BY);
//      Data10[dataIndex/10].abort_time = millis();
//      Data10[dataIndex/10].run_time=0;
//      start_time=0;
//      break;
//
//    //Aborting in any one of the following scenarios would require a reset if doing ground testing (new startup sequence, new datafile, etc)
//    case POWERED_ASCENT:
//      //SET_STATE(STAND_BY);
//      Data10[dataIndex/10].abort_time = millis();
//      break;
//
//    case UNPOWERED_ASCENT:
//      //SET_STATE(STAND_BY)'
//      Data10[dataIndex/10].abort_time = millis();
//      break;
//
//    case FREEFALL:
//      //SET_STATE(STAND_BY)
//      Data10[dataIndex/10].abort_time = millis();
//      break;
//
//    case DROGUE_DESCENT:
//      //SET_STATE(STAND_BY)
//      Data10[dataIndex/10].abort_time = millis();
//      break;
//
//    case MAIN_DESCENT:
//      //SET_STATE(STAND_BY)
//      Data10[dataIndex/10].abort_time = millis();
//      break;
//
//
//    //No point in aborting at this point...
//    case LANDED:
//      SET_STATE(STAND_BY);
//      Data10[dataIndex/10].abort_time = millis();
//      break;
//  }
//}








//#define BUFFER_LENGTH 1000      //redefine BUFFER_LENGTH in wire lib

char buf[1000];
//volatile boolean receiveFlag = false;

void receiveEvent(int howMany)
{
  Wire.readBytes(buf, howMany);
  //receiveFlag = true;
}



//char msg[1000];
//void receiveEvent(int bytes) {
//  msg = Wire.read();    // read one character from the I2C
//}





/*
time_t getTeensy3Time(){
  return Teensy3Clock.get();
}
*/

void setup() {
  //Start Teensy RTC- this line doesn't seem to matter, see Teensy Time Lib Documentation
  //setSyncProvider(getTeensy3Time);
  //Start Serial Radio- use serial5 or serial8
  TELEMETRY_SERIAL.begin(57600);
//  delay(2000);
//  TELEMETRY_SERIAL.println();
//  TELEMETRY_SERIAL.println();
  
  //Start I2C
  Wire.begin(11);
  Wire.onReceive(receiveEvent);   
  //Wire.setClock(400000);
  delay(500);
  //Wire.begin();
  //Wire.setClock(400000);
  //delay(1000);
  
  //Start SPI
  //Similarly, SPI.setMOSI(pin), SPI.setMISO(pin), and SPI.setSCK(pin)
  //SPI1.begin(); 
  
  //Start GPS
//  while(myGPS.begin(Wire1) == false){
//    TELEMETRY_SERIAL.println(F("GPS err"));
//    digitalWrite(LED,LOW); delay(1000); 
//    //2 Pulses
//    digitalWrite(LED,HIGH); delay(100); 
//    digitalWrite(LED,LOW); delay(100);
//    digitalWrite(LED,HIGH); delay(100);
//    Data10[0].ss=0;
//  }
//  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
//  myGPS.setNavigationFrequency(20); //Set output up to 25 times a second (Neo M9N Max Update Rate)
//  //Possible Dynamic Model options: PORTABLE, STATIONARY, PEDESTRIAN, AUTOMOTIVE, SEA, AIRBORNE1g, AIRBORNE2g, AIRBORNE4g, WRIST, BIKE
//  if (myGPS.setDynamicModel(DYN_MODEL_AIRBORNE4g) == false){ // Set the dynamic model to DYN_MODEL_AIRBORNE4g
//    TELEMETRY_SERIAL.println(F("***!!! Warning: setDynamicModel failed !!!***"));
//  }
//  else{
//    //TELEMETRY_SERIAL.println(F("Dynamic platform model changed successfully!"));
//  }
//  //Check Dynamic Model
////  uint8_t newDynamicModel = myGPS.getDynamicModel();
////  if (newDynamicModel == 255){
////    TELEMETRY_SERIAL.println(F("***!!! Warning: getDynamicModel failed !!!***"));
////  }
////  else{
////    TELEMETRY_SERIAL.print(F("The new dynamic model is: "));
////    TELEMETRY_SERIAL.println(newDynamicModel);
////  }  
//  //Check Nav Frequency
//  byte rate = myGPS.getNavigationFrequency(); //Get the update rate of this module
//  TELEMETRY_SERIAL.print("Current update rate:");
//  TELEMETRY_SERIAL.println(rate);
//  
//  myGPS.saveConfiguration();
  
  //Start BMP388
//  while (!bmp.begin()) {                         //flashes to signal error
//    TELEMETRY_SERIAL.println(F("BMP388 err"));
//    digitalWrite(LED,LOW); delay(1000);
//    //1 Pulse
//    digitalWrite(LED,HIGH); delay(100); 
//    Data10[0].ss=0;
//  }
//  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
//  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
//  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
//  bmp.setOutputDataRate(BMP3_ODR_25_HZ);   //ODR_200_, _100_, 50, 25, 12_5, 6_25, 3_1, 1_5, 0_78, ... 0_001
  
  //Start BNO055 in OPERATION_MODE_NDOF
//  while (!bno.begin()) {                         //flashes to signal error
//    TELEMETRY_SERIAL.println(F("BNO055 err"));
//    digitalWrite(LED,LOW); delay(1000);
//    //5 Pulses
//    digitalWrite(LED,HIGH); delay(100); 
//    digitalWrite(LED,LOW); delay(100);
//    digitalWrite(LED,HIGH); delay(100);
//    digitalWrite(LED,LOW); delay(100);
//    digitalWrite(LED,HIGH); delay(100);
//    digitalWrite(LED,LOW); delay(100);
//    digitalWrite(LED,HIGH); delay(100);
//    digitalWrite(LED,LOW); delay(100);
//    digitalWrite(LED,HIGH); delay(100);
//    Data10[0].ss=0;
//  }
//  bno.setMode(Adafruit_BNO055::OPERATION_MODE_NDOF);  //OPERATION_MODE_NDOF_FMC_OFF, see .cpp for all modes "AMG" is the non-fusion mode with all sensors
//  //bno.setMode(Adafruit_BNO055::OPERATION_MODE_AMG);
//  bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P1); //REMAP_CONFIG_P0 to P7 (1 default)
//  //bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P7);  //REMAP_SIGN_P0 to P7 (1 default)
//  bno.setExtCrystalUse(true);

  //Start SD Card
  if (!SD.begin(BUILTIN_SDCARD)){
    TELEMETRY_SERIAL.println(F("SD err"));
    digitalWrite(LED,LOW); delay(1000);
    //4 Pulses
    digitalWrite(LED,HIGH); delay(100); 
    digitalWrite(LED,LOW); delay(100);
    digitalWrite(LED,HIGH); delay(100);
    digitalWrite(LED,LOW); delay(100);
    digitalWrite(LED,HIGH); delay(100);
    digitalWrite(LED,LOW); delay(100);
    digitalWrite(LED,HIGH); delay(100);
    Data10[0].ss=0;
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
        dataFile.print(F("run time (ms),date (y/m/d),time (H:M:S),yaw (deg),pitch (deg),roll (deg),Px (m),Py (m),Pz (m),Vx (m/s),Vy (m/s),Vz (m/s),oX (deg),oY (deg),oZ (deg),bmp alt (m ASL),bno alt (m ASL),gyroX (dps),gyroY (dps),gyroZ (dps),accX (m/s^2),accY (m/s^2),accZ (m/s^2),bmp pressure (hpa),qW [Fusion],qX,qY,qZ"));
        dataFile.print(F(",oriQuatW [gyro],oriQuatX,oriQuatY,oriQuatZ,eulerX (deg),eulerY (deg),eulerZ (deg),state,Apogee Passed,bno descending,bmp descending,gps alt (m ASL),SIV,PDOP (m),Lattitude,Longitude"));
        //commentable one
        //dataFile.print(F(",x from launch (m),y from launch (m),dir from launch (deg CW from N),xy to land (m),dir to land (deg CW from N),x to land (m),y to land (m)"));   
        dataFile.print(F(",gps descending,mX (uT),mY (uT),mZ (uT),l2g,vbatt (V),vgrid (V),bmp temp (C),bno temp (C),gpsR (Hz),radioR (Hz),readR (Hz),bmpR (Hz),bnoR (Hz),sdR (Hz),fallR (Hz),oriR (Hz)"));
        dataFile.println(F(",gpsT (*ms*),radioT (us),readT (us),bmpT (us),bnoT (*ms*),sdT (us),fallT (us),oriT (us),P1,P2,P3,P4,P5,P5V"));
        
        break;
      }
    }
  }

pinMode(LED, OUTPUT);  
  //Pin Initialization
//  pinMode(PYRO1, OUTPUT);
//  pinMode(PYRO2, OUTPUT);
//  pinMode(PYRO3, OUTPUT);
//  pinMode(PYRO4, OUTPUT);
//  pinMode(PYRO5, OUTPUT);
//  pinMode(PYRO5V, OUTPUT);  
//  digitalWrite(PYRO1,LOW);
//  digitalWrite(PYRO2,LOW);
//  digitalWrite(PYRO3,LOW);
//  digitalWrite(PYRO4,LOW);
//  digitalWrite(PYRO5,LOW);
//  digitalWrite(PYRO5V,LOW);

  //Cricket
  /*
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
  */
  
  //Startup Lights
  for(int q=0; q<5;q++){
    digitalWrite(LED,LOW); delay(200); digitalWrite(LED,HIGH);delay(200);
  }

  //Obtain GPS Lock
//  TELEMETRY_SERIAL.println(F("Waiting for a 3D fix..."));
//  Data10[0].fixType= 0;
//  while (Data10[0].fixType != 3){
//    //obtain system voltages
//    digitalWrite(LED,LOW); delay(500); digitalWrite(LED,HIGH);delay(250);
//    reading= analogRead(Batt_V);
//    Data1[0].vbatt= reading*(3.3/1023.00)* voltage_divider_ratio;
//    reading= analogRead(Grid_V);
//    Data1[0].vgrid= reading*(3.3/1023.00)* voltage_divider_ratio;
//    //3D fix
//    Data10[0].fixType = myGPS.getFixType(); //Get fix type: 0= No Fix, 1= Dead Reckoning, 2= 2D, 3= 3D, 4= GNSS + Dead Reckoning, 5= Time only
//    Data10[0].SIV = myGPS.getSIV();
//    Data10[0].PDOP = myGPS.getPDOP() / 1.0E3;
//
//    Wire.beginTransmission(11); // transmit to device #11
//    Wire.printf("SIV:%u,PDOP:%.2f,fixType:%u,vbatt:%.2f,vgrid:%.2f", Data10[0].SIV, Data10[0].PDOP, Data10[0].fixType, Data1[0].vbatt, Data1[0].vgrid);
//    Wire.endTransmission();    // stop transmitting
//    
//    //fix this
////    SEND_ITEM(SIV                 , Data10[0].SIV)
////    SEND_ITEM(PDOP                , Data10[0].PDOP)
////    SEND_ITEM(fixType             , Data10[0].fixType)
////    SEND_ITEM(vb                  , Data1[0].vbatt)
////    SEND_ITEM(vg                  , Data1[0].vgrid)
//    //TELEMETRY_SERIAL.print(F("\n"));
//    delay(1000);
//  }
  
//  TELEMETRY_SERIAL.println(F("3D fix found!"));
//  delay(5000); //more time to get a more accurate GPS lock
//  
//  Data10[0].gps_alt= myGPS.getAltitude();
//  launch_lat= myGPS.getLatitude() / (1.0E7);
//  launch_lon= myGPS.getLongitude() / (1.0E7);
//  //set the Teensy internal RTC, approximately line up RTC second with GPS milliseconds (not perfect)
//  while(myGPS.getMillisecond() < 900){  //be VERY careful with this line, sensitive to NavigationFrequency
//    delay(1);
//    TELEMETRY_SERIAL.println(myGPS.getMillisecond());
//  }
//  setTime(myGPS.getHour(),myGPS.getMinute(),myGPS.getSecond(),myGPS.getDay(),myGPS.getMonth(),myGPS.getYear());
  
  //Ready Lights
  for(int q=0; q<20;q++){
    digitalWrite(LED,LOW); delay(50); digitalWrite(LED,HIGH);delay(50);
  }

  //Cricket
  /*
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
  */
  
//  q1 = bno.getQuat();   //I don't think this needs to be run here, oriQuat can still equal q1 even if q1 doesn't have values yet, right?
//  
  //Loop Timers
  startTimeMicros= micros();  //Time when loop was started (microseconds)
  lastGyroUpdate= startTimeMicros;
  startTime = millis();       //Time when loop was started
  Timer100Hz= millis();
  testT= 0;
  
}









void loop() {
 
  if( (millis()-Timer100Hz) > Timer10ms){      //100hz max
    Timer100Hz= millis();
    dataLoopIteration++;
    dataIndex= dataLoopIteration % Num_Datapoints;

//    if((dataIndex%100)==0){
//      testTimer= micros();
//    }
    testTimer= micros();

//    if((dataIndex%10)==0){
//      //Update run_time (time starting at liftoff (having a countdown can make run_time negative))
//      if( (state != ARMED) && (state != STAND_BY) && (state != LANDED) ){
//        Data10[dataIndex/10].run_time= millis() - start_time;  //OLD: - COUNTDOWN_DURATION
//      }
//      else if(state == STAND_BY){
//        Data10[dataIndex/10].run_time= 0;
//      }
//  
//      //Evaluate system status- set ss to 0 if there are any sensor issues
//      if(0){    //replace 0 w/ any something that tells sensors aren't working
//        Data10[dataIndex/10].ss= 0;
//      }
//      else{
//        Data10[dataIndex/10].ss= 1;
//      }
//    }
    
    //*****Orientations Section****************************************************************
//    thisLoopMicros= micros();
//    
//    Data100[dataIndex].dtGyro = (double)(thisLoopMicros - lastGyroUpdate) / 1000000.0;
//    lastGyroUpdate = thisLoopMicros;
//    
//    //get gyro data all the time for logging:
//    gyroscope= bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);     //100hz, dps
//    Data100[dataIndex].gyroX= gyroscope.x();
//    Data100[dataIndex].gyroY= gyroscope.y();
//    Data100[dataIndex].gyroZ= gyroscope.z();
//    //Data100[dataIndex].gyroscope.x()
//    //it might be a good idea to set the BNO to a non-fusion mode after launch so that the dps can be set to 250 vs 2000 for more accurate gyro orientation
//    //on the other hand, it's probably best to keep the fusion on and just use a BMI088 to get super accurate vibration resistant gyro readings anyways. In addition, the fusion data
//    //from the BNO can be compared to BMI088 after various flights
//
//    if(state==STAND_BY || state==ARMED){      //1  //state==STAND_BY || state==ARMED      If I can ever get the gyro code working...
//      //oX/oY/oZ each respectively represent orientation about the x/y/z body axis
//      Data100[dataIndex].oX= Data10[dataIndex/10].roll;
//      Data100[dataIndex].oY= Data10[dataIndex/10].pitch;
//      Data100[dataIndex].oZ= Data10[dataIndex/10].yaw;
//      //use sensor fusion quat for current orientation
//      oriQuat= q1;
//    }
//    else{
//      //convert gyro rates to euler rates to a quaternion
//      dyaw =   ( Data100[dataIndex].gyroX )*Data100[dataIndex].dtGyro;   //.z *should* be the rads/s rate about the z axis of the BMI055, so yaw rate
//      dpitch = ( Data100[dataIndex].gyroY )*Data100[dataIndex].dtGyro;   //.y *should* be the rads/s rate about the y axis of the BMI055, so pitch rate
//      droll =  ( Data100[dataIndex].gyroZ )*Data100[dataIndex].dtGyro;   //.x *should* be the rads/s rate about the z axis of the BMI055, so roll rate
//
//      cy = cos(dyaw * 0.5);
//      sy = sin(dyaw * 0.5);
//      cp = cos(dpitch * 0.5);
//      sp = sin(dpitch * 0.5);
//      cr = cos(droll * 0.5);
//      sr = sin(droll * 0.5);
//      
//      imu::Quaternion rotQuat(cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy);
//      
//      oriQuat= oriQuat*rotQuat;   //update the orientation storing quaternion
//      Data100[dataIndex].oriQuatW= oriQuat.w();
//      Data100[dataIndex].oriQuatX= oriQuat.x();
//      Data100[dataIndex].oriQuatY= oriQuat.y();
//      Data100[dataIndex].oriQuatZ= oriQuat.z();
//      
//      oriGyro= oriQuat.toEuler(); //convert the ori storing quat to euler angles (no gimbal lock issues!)
//      Data100[dataIndex].oX= oriGyro.z();   //oX is roll, .z() represents the 3rd euler rotation which is roll
//      Data100[dataIndex].oY= oriGyro.y();   //oY is pitch, .y() represents the 2nd euler rotation which is pitch
//      Data100[dataIndex].oZ= oriGyro.x();   //oZ is yaw, .x() represents the 1rd euler rotation which is yaw
//    }
//
//    oriCount++;
//    if(dataIndex%100 == 0){   //mod 100 -> 1Hz
//      Data1[dataIndex/100].oriRate= oriCount / ( (millis() - startTime) / 1000.0);   
//      Data1[dataIndex/100].oriT= ( micros() - thisLoopMicros );
//    }
    //*****END Orientations Section****************************************************************

    //*****Sensor Data Collection Section*****************************************************************
    
    //Data is obtained in a slightly offset fashion, but this spreads out the laggy I2C/SPI calls over 10 10ms time steps instead of just 1
//    switch (dataIndex%10) {
//      case 0:
//        //don't do anything here as the rest of the code usually runs for case 0
//        break;
//      case 1:
//        
//        //4-9ms, max ~45ms
//        euler         = bno.getVector(Adafruit_BNO055::VECTOR_EULER);         //default deg, ***sensor fusion***
//        //fixType = myGPS.getFixType();
//        
//        
//        break;
//      case 2:
//        
//        //1-2ms, max 2.7ms
//        q1            = bno.getQuat();                                        //unitless, ***sensor fusion***
//        SIV = myGPS.getSIV();
//
//        
//        break;
//      case 3:
//        
//        //1-2ms, max 2.2ms
//        Acc           = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER); //default m/s^2
//        PDOP = myGPS.getPDOP() / 100.0;  //m
//
//        
//        break;
//      case 4:
//        
//        //1-2ms, max 2.2 ms
//        magnetometer = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);  //default microT
//        gps_alt= myGPS.getAltitude() / 1.0E3; //m ASL
//
//        
//        break;
//      case 5:
//        
//        //0-1 us!!! 1us max
//        gps_lon= myGPS.getLongitude() / 1.0E7;
//
//        
//        break;
//      case 6:
//        
//        //0-1us, 1us max
//        gps_lat= myGPS.getLatitude() / 1.0E7;
//
//        
//        break;
//      case 7:
//        
//        //0-1us, 1us max
//        bmp_pressure=   bmp.pressure / 100.0; //hPa or mbar
//
//        
//        break;
//      case 8:
//        
//        //58us typ, max 59us
//        bmp_alt=        bmp.readAltitude(SEALEVELPRESSURE_HPA); //m ASL
//
//        
//        break;
//      case 9:
//        
//        break;
//        
//    }
    
    //*****END Sensor Data Collection Section*************************************************************
    
    //*****10Hz Section*************************************************************************************************
    //******************************************************************************************************************
    if((dataIndex%10)==0){
      //*****BNO, KALMAN, GPS Section****************************************************************
//      thisLoopMicros= micros();
//
//      //euler         = bno.getVector(Adafruit_BNO055::VECTOR_EULER);         //default deg, ***sensor fusion***
//      Data10[dataIndex/10].eulerX= euler.x();
//      Data10[dataIndex/10].eulerY= euler.y();
//      Data10[dataIndex/10].eulerZ= euler.z();
//      //q1            = bno.getQuat();                                        //unitless, ***sensor fusion***
//      Data10[dataIndex/10].qW= q1.w();
//      Data10[dataIndex/10].qX= q1.x();
//      Data10[dataIndex/10].qY= q1.y();
//      Data10[dataIndex/10].qZ= q1.z();
//      //Acc           = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER); //default m/s^2
//      Data10[dataIndex/10].accX= Acc.x();
//      Data10[dataIndex/10].accY= Acc.y();
//      Data10[dataIndex/10].accZ= Acc.z();
//      //magnetometer = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);  //default microT
//      Data10[dataIndex/10].mx= magnetometer.x();
//      Data10[dataIndex/10].my= magnetometer.y();
//      Data10[dataIndex/10].mz= magnetometer.z();
//        
//      if(dataIndex%100 == 0){          //1Hz
//        Data1[dataIndex/100].temp            = bno.getTemp();                                        //default C
//      }
//      
//      //use built in quat function to get Euler Angles
//      ori=   q1.toEuler();
//      Data10[dataIndex/10].yaw=   RAD_TO_DEG*ori.x();   //x represents the 1rd rotation, not rotation about the x axis (which would be roll)
//      Data10[dataIndex/10].pitch= RAD_TO_DEG*ori.y();   //y represents the 2rd rotation, not rotation about the y axis (although that would also be pitch)
//      Data10[dataIndex/10].roll=  RAD_TO_DEG*ori.z();   //z represents the 3rd rotation, not rotation about the z axis (which would be yaw)
//  
//      //***KALMAN Filter Code*********** (could always add more filters w/ different sensors for different estimates)
//      Kt= millis()/1000.0;    //time in s
//      Kdt= Kt- Kt0; //dt for Kalman Filter, in s
//        //technically I should be able to use (bno_dt/1000= .01) for Kdt
//      if(Kdt > 10.0){    //can increase to 100 if necessary- see log
//        Kdt= 10.0;
//      }
//      Kt0= Kt;
//  
//      dt2= Kdt*Kdt;
//      dt3= dt2*Kdt;
//      dt4= dt3*Kdt;
//      
//      //Update time step dependent matricies & Accel readings
//      A._entity[0][3]= Kdt;
//      A._entity[1][4]= Kdt;
//      A._entity[2][5]= Kdt;
//  
//      B._entity[0][0]= .5*dt2;
//      B._entity[1][1]= .5*dt2;
//      B._entity[2][2]= .5*dt2;
//      B._entity[3][0]= Kdt;
//      B._entity[4][1]= Kdt;
//      B._entity[5][2]= Kdt;
//       
//      Q._entity[0][0] = dt4/4.0*noise_ax;
//      Q._entity[0][3] = dt3/2.0*noise_ax;
//      Q._entity[1][1] = dt4/4.0*noise_ay;
//      Q._entity[1][4] = dt3/2.0*noise_ay;
//      Q._entity[2][2] = dt4/4.0*noise_az;
//      Q._entity[2][5] = dt3/2.0*noise_az;
//      Q._entity[3][0] = dt3/2.0*noise_ax;
//      Q._entity[3][3] = dt2*noise_ax;
//      Q._entity[4][1] = dt3/2.0*noise_ay;
//      Q._entity[4][4] = dt2*noise_ay;
//      Q._entity[5][2] = dt3/2.0*noise_az;
//      Q._entity[5][5] = dt2*noise_az;    
//      
//      //Z faces up in current config
//      if(state==STAND_BY || state==ARMED || state==POWERED_ASCENT || state==UNPOWERED_ASCENT || state==LANDED){ //Earth's gravity actin on IMU
//        U._entity[0][0]= Data10[dataIndex/10].accX; //earth fixed x Accel
//        U._entity[1][0]= Data10[dataIndex/10].accY; //earth fixed y Accel
//        U._entity[2][0]= (Data10[dataIndex/10].accZ*cos(DEG_TO_RAD*0)*cos(DEG_TO_RAD*0)) -9.81; //earth fixed z Accel
//      }
//      else{ //IMU in freefall
//        U._entity[0][0]= Data10[dataIndex/10].accX; //earth fixed x Accel
//        U._entity[1][0]= Data10[dataIndex/10].accY; //earth fixed y Accel
//        U._entity[2][0]= (Data10[dataIndex/10].accZ*cos(DEG_TO_RAD*0)*cos(DEG_TO_RAD*0)); //earth fixed z Accel
//      }
//      
//      //predict new states (XS continually updates)
//      XS= (A*XS)+(B*U);
//      P= ((A*P)*Matrix<double>::transpose(A))+Q;
//    
//      //****Update States w/ GPS Measurement***********************************
//      thisLoopMicros2= micros();
//      
//      Data10[dataIndex/10].fixType      = fixType;
//      Data10[dataIndex/10].SIV          = SIV;
//      Data10[dataIndex/10].PDOP         = PDOP;  //m
//      Data10[dataIndex/10].gps_alt      = gps_alt; //m ASL
//      Data10[dataIndex/10].gps_lon      = gps_lon;
//      Data10[dataIndex/10].gps_lat      = gps_lat;
//      
////      //can add in xy velocity and heading at some point (see library)
////      Data10[dataIndex/10].x_from_launch= TinyGPSPlus::distanceBetween(0, launch_lon, 0, Data10[dataIndex/10].gps_lon);
////      Data10[dataIndex/10].y_from_launch= TinyGPSPlus::distanceBetween(launch_lat, 0, Data10[dataIndex/10].gps_lat, 0);
////        //add Data10[dataIndex/10].xy_from_launch here at some point
////      Data10[dataIndex/10].dir_from_launch= TinyGPSPlus::courseTo(launch_lat, launch_lon, Data10[dataIndex/10].gps_lat, Data10[dataIndex/10].gps_lon);
////      Data10[dataIndex/10].xy_to_land= TinyGPSPlus::distanceBetween(Data10[dataIndex/10].gps_lat, Data10[dataIndex/10].gps_lon, land_lat, land_lon);
////      Data10[dataIndex/10].dir_to_land= TinyGPSPlus::courseTo(Data10[dataIndex/10].gps_lat, Data10[dataIndex/10].gps_lon, launch_lat, launch_lon);
////      Data10[dataIndex/10].x_to_land= TinyGPSPlus::distanceBetween(0, Data10[dataIndex/10].gps_lon, 0, land_lon);
////      Data10[dataIndex/10].y_to_land= TinyGPSPlus::distanceBetween(Data10[dataIndex/10].gps_lat, 0, land_lat, 0);
//      
//      //GPS apogee detection- determines an average alt from recent measurements, then compares to the average calculated on the previous cycle
//        //takes 11 measurements (11 cycles) to fully populate the array
//      for (int i=0;i<9;i++){
//        gps_alt_new[i+1]=gps_alt_new[i]; //move every element 1 back
//      }
//      gps_alt_new[0]= Data10[dataIndex/10].gps_alt; //now array fully updated
//      
//      gps_alt_new_avg= Avg_Last_N_Measurements(gps_alt_new, 10);
//      
//      if((gps_alt_last_avg > gps_alt_new_avg) && (Data10[dataIndex/10].bmp_alt > Launch_ALT + ATST)){
//        gps_descending_counter= gps_descending_counter + 1;
//      }
//      
//      for (int i=0;i<10;i++){
//        gps_alt_last[i]= gps_alt_new[i];
//      } //prev alts now = current alts
//      gps_alt_last_avg= gps_alt_new_avg;
//
//      gpsCount++;
//      if(dataIndex%100 == 0){   //mod 100 -> 1Hz
//        Data1[dataIndex/100].gpsRate= gpsCount / ( (millis() - startTime) / 1000.0);
//        Data1[dataIndex/100].gpsT= ( micros() - thisLoopMicros2 );
//      }
//
//      if(gps_descending != 1){
//        Data10[dataIndex/10].gps_descending = 0;
//        if((gps_descending_counter>9)&& (Data10[dataIndex/10].bmp_alt > (Launch_ALT + ATST) )){
//          gps_descending= 1;
//          Data10[dataIndex/10].gps_descending = 1;
//        }
//      }
//      else{
//        Data10[dataIndex/10].gps_descending = 1;
//      }
//
//      if( (Data10[dataIndex/10].fixType == 3) && (Data10[dataIndex/10].PDOP < 1.8) ){
//        //Measurement & Update steps are below, I *think* both should only happen when a new measurement occurs
//        //see https://towardsdatascience.com/sensor-fusion-part-2-kalman-filter-code-78b82c63dcd for reference
//          //Measurement Step
//        Z_Meas._entity[0][0]= Data10[dataIndex/10].x_from_launch;
//        Z_Meas._entity[1][0]= Data10[dataIndex/10].y_from_launch;
//        Z_Meas._entity[2][0]= (Data10[dataIndex/10].gps_alt - Launch_ALT);      //gps_alt is ASL
//        Y_diff= Z_Meas- H*XS;
//        K_denom= ( ((H*P)*H_T) + R );
//        K= (P*H_T)*Matrix<double>::inv(K_denom);
//        
//          //Update Step
//        XS= XS + (K*Y_diff);  //Calculate the new filtered state
//        P= (I-(K*H))*P;       //Calculate the new P matrix
//      }
//        
//      //****END Update States w/ GPS Measurement*************************************************
//      
//      Data10[dataIndex/10].Px= XS._entity[0][0];
//      Data10[dataIndex/10].Py= XS._entity[1][0];
//      Data10[dataIndex/10].Pz= XS._entity[2][0]; //m AGL
//      Data10[dataIndex/10].Vx= XS._entity[3][0];
//      Data10[dataIndex/10].Vy= XS._entity[4][0];
//      Data10[dataIndex/10].Vz= XS._entity[5][0];
//      //Trust Kalman Filter for bno_alt (not bmp_alt)
//      Data10[dataIndex/10].bno_alt= Launch_ALT + Data10[dataIndex/10].Pz;  //m ASL
//  
//      //***************END KALMAN Filter Code*****************************************
//      
//      //BNO apogee detection- determines an average alt from recent measurements, then compares to the average calculated on the previous cycle
//          //takes 11 measurements (11 cycles) to fully populate the array
//      for (int i=0;i<9;i++){
//        bno_alt_new[i+1]=bno_alt_new[i]; //move every element 1 back
//      }
//      bno_alt_new[0]= Data10[dataIndex/10].bno_alt; //now array fully updated
//      
//      bno_alt_new_avg= Avg_Last_N_Measurements(bno_alt_new, 10);
//      
//      if((bno_alt_last_avg > bno_alt_new_avg) && (Data10[dataIndex/10].bno_alt > Launch_ALT + ATST)){
//        bno_descending_counter= bno_descending_counter + 1;
//      }
//      
//      for (int i=0;i<10;i++){   
//        bno_alt_last[i]= bno_alt_new[i];
//      } 
//      bno_alt_last_avg= bno_alt_new_avg;    //prev alts now = current alts
//      
//      if(bno_descending != 1){
//        Data10[dataIndex/10].bno_descending = 0;
//        if((bno_descending_counter>9) && (Data10[dataIndex/10].bno_alt > (Launch_ALT + ATST) )){
//          bno_descending= 1;
//          Data10[dataIndex/10].bno_descending = 1;
//        }
//      }
//      else{
//        Data10[dataIndex/10].bno_descending = 1;
//      }
//  
//      bnoCount++;
//      if(dataIndex%100 == 0){   //mod 100 -> 1Hz
//        Data1[dataIndex/100].bnoRate= bnoCount / ( (millis() - startTime) / 1000.0);
//        Data1[dataIndex/100].bnoT= ( micros() - thisLoopMicros );
//      }
      //*****END BNO, KALMAN, GPS Section****************************************************************
  
      //**************BMP Section****************************************************************
//      thisLoopMicros= micros();
//      thisLoopMillis= millis();
//  
//      if(dataIndex%100 == 0){
//        Data1[dataIndex/100].bmp_temp=       bmp.temperature; //C
//      }
//      Data10[dataIndex/10].bmp_pressure=   bmp_pressure;
//      Data10[dataIndex/10].bmp_alt=        bmp_alt;
//      //Also, the BMP 388 is only good up to ~9000m... look into ones that can work higher than that
//  
//      //BMP apogee detection- determines an average alt from last 1-10 measurements and last 11-20 measurements
//        //compares the averages... takes 20 measurements (20 cycles) to fully populate the array
//      for (int i=0;i<19;i++){
//        bmp_alt_last[i+1]=bmp_alt_last[i]; //move every element 1 back, [19] now forgotten
//      }
//      bmp_alt_last[0]= Data10[dataIndex/10].bmp_alt; //now array fully updated
//      
//      bmp_alt_new_avg= Avg_Last_N_Measurements(bno_alt_new, 10);
//      
//      for (int i=10;i<20;i++){
//        sum += bmp_alt_last[i];
//      }
//      bmp_alt_last_avg= sum/10.0; //avg alt over the oldest 10 data points
//      sum=0;
//      
//      if((bmp_alt_last_avg > bmp_alt_new_avg) && (Data10[dataIndex/10].bmp_alt > (Launch_ALT + ATST))){
//        bmp_descending_counter= bmp_descending_counter + 1;
//      }
//    
//      if(bmp_descending != 1){
//        Data10[dataIndex/10].bmp_descending = 0;
//        if((bmp_descending_counter>9) && (Data10[dataIndex/10].bmp_alt > Launch_ALT + ATST)){
//          bmp_descending= 1;
//          Data10[dataIndex/10].bmp_descending = 1;
//        }
//      }
//      else{
//        Data10[dataIndex/10].bmp_descending = 1;
//      }
//      
//      bmpCount++;
//      if(dataIndex%100 == 0){   //mod 100 -> 1Hz
//        Data1[dataIndex/100].bmpRate= bmpCount / ( (millis() - startTime) / 1000.0);
//        //Data1[dataIndex/100].bmpT= (millis() - thisLoopMillis)*1000.0;     
//        Data1[dataIndex/100].bmpT= ( micros() - thisLoopMicros );
//      }
      //*****END BMP Section****************************************************************
  
      //*****Fall Section****************************************************************
//      thisLoopMicros= micros();
//      thisLoopMillis= millis();
//      
//      //Code that is active before Apogee is Reached/Passed
//      if( (Apogee_Passed == 0) && (state != ARMED) && (state != STAND_BY) ){
//        Data10[dataIndex/10].Apogee_Passed = 0;
//        //The following loop only trigger ONCE (when apogee is detected)
//        if((gps_descending*1)+(bmp_descending*1)+(bno_descending*1) > 1){   //should be able to use either the struct or the normal version
//          Apogee_Passed=1;
//          Data10[dataIndex/10].Apogee_Passed = 1;
//          digitalWrite(PYRO1,HIGH);   //fire drouge chute
//          DROGUE_FIRED= 1;                                //represents if the drogue chute has actually been fired
//          P1_setting= 1;                                  //represents that P1 is high for the GUI
//          //Data1[dataIndex/100].P1_setting= 1;           //will fill in during read section
//        }
//      }
//      else if(Apogee_Passed == 1) {         //Continuous code that runs once Apogee is Reached/Passed
//        Data10[dataIndex/10].Apogee_Passed = 1;
//        //insert code here, ex: wait to fire main chutes or fire main chute when below a certain altitude
//        if( Data10[dataIndex/10].bmp_alt < (Launch_ALT + 250) ){   //eject main at 250m
//          digitalWrite(PYRO2,HIGH);                       //fire main chute, just an example
//          MAIN_FIRED= 1;                                  //mark that the main chute has actually been fired
//          P2_setting= 1;
//          //Data1[dataIndex].P2_setting=1;                //for logging
//        }
//      }
//  
//      //Test if gyro measurements have settled
//      sum= 0;   //leave this here in case I switch back to the avg function for the 2nd bmp avg
//      dataIndexSigned= (int) dataIndex;
//      if(dataIndexSigned > 499){
//        for(int i = ( dataIndexSigned - 500 ); i < dataIndexSigned; i++){
//          sum  += Data100[i].gyroX;   //all in dps
//          sum2 += Data100[i].gyroY;
//          sum3 += Data100[i].gyroZ;
//        }
//        GyroAvgX= sum/500;
//        GyroAvgY= sum/500;
//        GyroAvgZ= sum/500;
//        sum= 0;
//        sum2= 0;
//        sum3= 0;
//      }
//      
//      //state machine:
//      if ( (state==ARMED) && (Data10[dataIndex/10].accZ > LIFTOFF_ACC_THRESH) ){
//        SET_STATE(POWERED_ASCENT);
//        liftoff_time= millis();
//        Data10[dataIndex/10].Px= 0;
//        Data10[dataIndex/10].Py= 0;
//        Data10[dataIndex/10].Pz= 0;
//        Data10[dataIndex/10].Vx= 0;
//        Data10[dataIndex/10].Vy= 0;
//        Data10[dataIndex/10].Vz= 0;
//      }
//      if ( (state==POWERED_ASCENT) && ( millis() > (BURN_TIME + liftoff_time)) ){
//        SET_STATE(UNPOWERED_ASCENT);
//      }
//      if ( (state==UNPOWERED_ASCENT) && (Apogee_Passed == 1) ){
//        SET_STATE(FREEFALL);
//      }
//      if ( (state==FREEFALL) && (DROGUE_FIRED==1) ){
//        SET_STATE(DROGUE_DESCENT);
//      }
//      if ( (state==DROGUE_DESCENT) && (MAIN_FIRED==1) ){
//        SET_STATE(MAIN_DESCENT);
//      }
//        //reference prev avg gyro values (dps) to determine state
//      if ( (state==MAIN_DESCENT) && (Data10[dataIndex/10].bmp_alt<(Launch_ALT+15)) && (GyroAvgX < 2.0) && (GyroAvgY < 2.0) && (GyroAvgZ < 2.0) ){
//        SET_STATE(LANDED);
//      }
//  
//      Data10[dataIndex/10].state= state;          //THIS IS KEY FOR UPDATING STATE ARRAY!!!!
//      
//      fallCount++;
//      if(dataIndex%100 == 0){   //mod 100 -> 1Hz
//        Data1[dataIndex/100].fallRate= fallCount / ( (millis() - startTime) / 1000.0);
//        //Data1[dataIndex/100].fallT= (millis() - thisLoopMillis)*1000.0;        
//        Data1[dataIndex/100].fallT= ( micros() - thisLoopMicros );
//      }
      //*****END Fall Section****************************************************************
      
      //*****SEND I2C Data Section******************************************************* 10Hz
      i= dataIndex;
      //strcpy(SendData, );
                                                                                                                                        //abs_time              run_time  
//      sprintf(AddData,"%lu,%ld,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.2f,%.2f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,", millis(), Data10[dataIndex/10].run_time, i, Data10[i/10].yaw, Data10[i/10].pitch, Data10[i/10].roll, Data10[i/10].Px, Data10[i/10].Py, Data10[i/10].Pz, Data10[i/10].Vx, Data10[i/10].Vy, Data10[i/10].Vz, Data100[i].oX, Data100[i].oY, Data100[i].oZ, Data10[i/10].bmp_alt, Data10[i/10].bno_alt, Data100[i].gyroX, Data100[i].gyroY, Data100[i].gyroZ, Data10[i/10].accX, Data10[i/10].accY, Data10[i/10].accZ);
//      strcat(SendData,AddData);
//      AddData[0]= '\0';
//      
//      sprintf(AddData,"%.2f,%.4f,%.4f,%.4f,%.4f,%f,%f,%f,%f,%.2f,%.2f,%.2f,%d,%d,%d,%d,%.2f,%.2f,%.2f,%f,%f,", Data10[i/10].bmp_pressure, Data10[i/10].qW, Data10[i/10].qX, Data10[i/10].qY, Data10[i/10].qZ, Data100[i].oriQuatW, Data100[i].oriQuatX, Data100[i].oriQuatY, Data100[i].oriQuatZ, Data10[i/10].eulerX, Data10[i/10].eulerY, Data10[i/10].eulerZ, Data10[i/10].state, Data10[i/10].Apogee_Passed, Data10[i/10].bno_descending, Data10[i/10].bmp_descending, ATST, Launch_ALT, SEALEVELPRESSURE_HPA, launch_lat, launch_lon);
//      strcat(SendData,AddData);
//      AddData[0]= '\0';
//  
//      //***radioRate, readRate, sdRate, radioT, readT, sdT should no longer be included in uC1 code
//      
//      sprintf(AddData,"%.2f,%u,%.2f,%.7f,%.7f,%d,", Data10[dataIndex/10].gps_alt, Data10[dataIndex/10].SIV, Data10[dataIndex/10].PDOP, Data10[dataIndex/10].gps_lat, Data10[dataIndex/10].gps_lon, Data10[dataIndex/10].gps_descending);
//      strcat(SendData,AddData);
//      AddData[0]= '\0';
//      
//      sprintf(AddData,"%.2f,%.2f,%.2f,", Data10[dataIndex/10].mx, Data10[dataIndex/10].my, Data10[dataIndex/10].mz);
//      strcat(SendData,AddData);
//      AddData[0]= '\0';
//      
//      if( (dataIndex%100) == 0){   //add any data that can be sent or logged at 1hz
//        sprintf(AddData,"%d,%d,%d,%d,%d,%d,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,", Data1[dataIndex/100].y, Data1[dataIndex/100].mo, Data1[dataIndex/100].d, Data1[dataIndex/100].h, Data1[dataIndex/100].m, Data1[dataIndex/100].s, Data1[dataIndex/100].vbatt, Data1[dataIndex/100].vgrid, Data1[dataIndex/100].bmp_temp, Data1[dataIndex/100].temp, Data1[dataIndex/100].gpsRate, Data1[dataIndex/100].bmpRate, Data1[dataIndex/100].bnoRate, Data1[dataIndex/100].fallRate, Data1[dataIndex/100].oriRate);
//        sprintf(AddData2,"%llu,%llu,%llu,%llu,%llu,%d,%d,%d,%d,%d,%d,%llu,%llu",Data1[dataIndex/100].gpsT, Data1[dataIndex/100].bmpT, Data1[dataIndex/100].bnoT, Data1[dataIndex/100].fallT, Data1[dataIndex/100].oriT, Data1[dataIndex/100].P1_setting, Data1[dataIndex/100].P2_setting, Data1[dataIndex/100].P3_setting, Data1[dataIndex/100].P4_setting, Data1[dataIndex/100].P5_setting, Data1[dataIndex/100].P5V_setting, testT, avgT );    
//      }
//      else{
//        sprintf(AddData," , , , , , , , , , , , , , , ,");
//        sprintf(AddData2," , , , , , , , , , , , , ");
//      }
//      strcat(SendData,AddData);
//      strcat(SendData,AddData2);
//      AddData[0]= '\0';
//      AddData2[0]= '\0';
//      
//      Wire.beginTransmission(11); // transmit to device #11
//      Wire.write(SendData);
//      //Wire.printf("%s",SendData);     //another option
//      Wire.printf(" hello??? %f \n", 3.1415);
//      Wire.endTransmission();    // stop transmitting
//  
//      if( (dataIndex%100) == 0){
//      TELEMETRY_SERIAL.println(SendData);   //debugging, but adds lag :(
//      //TELEMETRY_SERIAL.println();
//      //testT= (micros() - testTimer);
//      }
//      
//      SendData[0]= '\0';

      
      //add I2C Read Section





        if( (dataIndex%100) == 0){
          TELEMETRY_SERIAL.println(buf);   //debugging, but adds lag :(
          //TELEMETRY_SERIAL.println("abcdefghijklmnopqrstuvwxyz1234567890____abcdefghijklmnopqrstuvwxyz1234567890____");
          //testT= (micros() - testTimer);
        }


      






      
      
      //*****END I2C Data Section******************************************************* 100Hz
      
    }
    //***********END 10Hz Section********************************************************************************************
    //***********************************************************************************************************************
    
    //*****1Hz Section**************************************************************** 1Hz
    if(dataIndex%100 == 0){
      
      //Update the Time keeping array
      Data1[dataIndex/100].y= year();
      Data1[dataIndex/100].mo= month();
      Data1[dataIndex/100].d= day();
      Data1[dataIndex/100].h= hour();
      Data1[dataIndex/100].m= minute();
      Data1[dataIndex/100].s= second();
    
      //*****Read Section**************************************************************
      
      thisLoopMicros= micros();
      thisLoopMillis= millis();
  
      BEGIN_READ
      READ_FLAG(c) {
        heartbeat();
      }
      READ_FLAG(r) {      //@@@@@:r&&&&& if manually sending commands
        TELEMETRY_SERIAL.println(F("Resetting board"));
        //CPU_RESTART;              //Teensy 4.1
        Wire.printf("r");           //send reset command

        delay(1000);
        WRITE_RESTART(0x5FA0004);   //Teensy 3.6
      }
      READ_FLAG(s) {
        //start_countdown();
        Wire.printf("s");           //send start command
      }
      READ_FLAG(a) {
        TELEMETRY_SERIAL.println(F("Manual abort initiated"));
        Wire.printf("a");           //send abort command
        //abort_autosequence();
      }
      READ_FIELD(P1cmd, "%d", cmd) {
        //P1_setting= cmd;
        //digitalWrite(PYRO1,cmd);
        if(cmd){
          Wire.printf("P1-ON");           //send P1 command
        }
        else{
          Wire.printf("P1-OFF");           //send P1 command
        }
      }
      READ_FIELD(P2cmd, "%d", cmd) {
        //P2_setting= cmd;
        //digitalWrite(PYRO2,cmd);
        if(cmd){
          Wire.printf("P2-ON");           //send P2 command
        }
        else{
          Wire.printf("P2-OFF");           //send P2 command
        }
      }
      READ_FIELD(P3cmd, "%d", cmd) {
        //P3_setting= cmd;
        //digitalWrite(PYRO3,cmd);
        if(cmd){
          Wire.printf("P3-ON");           //send P3 command
        }
        else{
          Wire.printf("P3-OFF");           //send P3 command
        }
      }
      READ_FIELD(P4cmd, "%d", cmd) {
        //P4_setting= cmd;
        //digitalWrite(PYRO4,cmd);
        if(cmd){
        Wire.printf("P4-ON");           //send P4 command
        }
        else{
          Wire.printf("P4-OFF");           //send P4 command
        }
      }
      READ_FIELD(P5cmd, "%d", cmd) {
        //P5_setting= cmd;
        //digitalWrite(PYRO5,cmd);
        if(cmd){
          Wire.printf("P5-ON");           //send P5 command
        }
        else{
          Wire.printf("P5-OFF");           //send P5 command
        }
      }
      READ_FIELD(P5Vcmd, "%d", cmd) {
        //P5V_setting= cmd;
        //digitalWrite(PYRO5V,cmd);
        if(cmd){
          Wire.printf("P5V-ON");           //send P5V command
        }
        else{
          Wire.printf("P5V-OFF");           //send P5V command
        }
      }
      READ_FIELD(Launch_ALT, "%f", val) {
        //Launch_ALT= val;
        Wire.printf("Launch_ALT:%f", val);           //send Launch_ALT command
      }
      READ_FIELD(BMP_cf, "%f", val) {
        //SEALEVELPRESSURE_HPA= val;
        Wire.printf("BMP_cf:%f", val);           //send BMP_cf command
      }
      READ_FIELD(ATST, "%f", val) {
        //ATST= val;
        Wire.printf("ATST:%f", val);           //send ATST command
      }
      READ_FIELD(launch_lat, "%f", val) {
        //launch_lat= val;
        //Wire.printf("launch_lat:%f", val);           //send launch_lat command
      }
      READ_FIELD(launch_lon, "%f", val) {
        //launch_lon= val;
        //Wire.printf("SEALEVELPRESSURE_HPA:%f", val);           //send SEALEVELPRESSURE_HPA command
      }
      READ_FIELD(land_lat, "%f", val) {
        //land_lat= val;
        //Wire.printf("SEALEVELPRESSURE_HPA:%f", val);           //send SEALEVELPRESSURE_HPA command
      }
      READ_FIELD(land_lon, "%f", val) {
        //land_lon= val;
        //Wire.printf("SEALEVELPRESSURE_HPA:%f", val);           //send SEALEVELPRESSURE_HPA command
      }            
      
      READ_DEFAULT(data_name, data) {
        TELEMETRY_SERIAL.print(F("Invalid data field recieved: "));
        TELEMETRY_SERIAL.print(data_name);
        TELEMETRY_SERIAL.print(":");
        TELEMETRY_SERIAL.println(data);
      }
      END_READ

      Data1[dataIndex/100].P1_setting= P1_setting;
      Data1[dataIndex/100].P2_setting= P2_setting;
      Data1[dataIndex/100].P3_setting= P3_setting;
      Data1[dataIndex/100].P4_setting= P4_setting;
      Data1[dataIndex/100].P5_setting= P5_setting;

      Data1[dataIndex/100].sdT= sdT;
      Data1[dataIndex/100].sdRate= sdRate;
      
      //NEED TO ADD THE ABILITY TO FIRE PYRO 5V
      Data1[dataIndex/100].P5V_setting= P5V_setting;
      
      //Read battery voltages
//      reading= analogRead(Batt_V);
//      Data1[dataIndex/100].vbatt= reading*(3.3/1023.00)* voltage_divider_ratio;
//      reading= analogRead(Grid_V);
//      Data1[dataIndex/100].vgrid= reading*(3.3/1023.00)* voltage_divider_ratio;
      
      readCount++;
      if(dataIndex%100 == 0){   //mod 100 -> 1Hz, keep this just in case the read section is moved from 1Hz
        Data1[dataIndex/100].readRate= readCount / ( (millis() - startTime) / 1000.0);
        //Data1[dataIndex/100].readT= (millis() - thisLoopMillis)*1000.0;
        Data1[dataIndex/100].readT= ( micros() - thisLoopMicros );
      }
      
      //*****END Read Section**************************************************************
      
    }
    //*****END 1Hz Section**************************************************************** 1Hz

    //*****Downlink Section****************************************************************
//    if(dataIndex%25 == 0){    //4Hz
//      //*****Downlink Section****************************************************************
//      
//      thisLoopMicros= micros();
//      
//      if (millis() > (heartbeat_time + HEARTBEAT_TIMEOUT) ) {
//        //TELEMETRY_SERIAL.println(F("Loss of data link"));
//        Data10[dataIndex/10].link2ground=0;
//        //abort_autosequence();
//      }
//      else{
//        Data10[dataIndex/10].link2ground= 1;
//      }
//      
//      BEGIN_SEND
//      SEND_ITEM(gyrX                , Data100[dataIndex].gyroX)
//      SEND_ITEM(gyrY                , Data100[dataIndex].gyroY)
//      SEND_ITEM(gyrZ                , Data100[dataIndex].gyroZ)
//      SEND_ITEM(aX                  , Data10[dataIndex/10].accX)
//      SEND_ITEM(aY                  , Data10[dataIndex/10].accY)
//      SEND_ITEM(aZ                  , Data10[dataIndex/10].accZ)
//      SEND_ITEM(bmp_alt             , Data10[dataIndex/10].bmp_alt)
//      SEND_ITEM(bno_alt             , Data10[dataIndex/10].bno_alt)
//      SEND_ITEM(roll                , Data10[dataIndex/10].roll)
//      SEND_ITEM(pitch               , Data10[dataIndex/10].pitch)
//      SEND_ITEM(yaw                 , Data10[dataIndex/10].yaw)
//      SEND_ITEM(Px                  , Data10[dataIndex/10].Px)
//      SEND_ITEM(Py                  , Data10[dataIndex/10].Py)
//      SEND_ITEM(Pz                  , Data10[dataIndex/10].Pz)
//      SEND_ITEM(oX                  , Data100[dataIndex].oX)
//      SEND_ITEM(oY                  , Data100[dataIndex].oY)
//      SEND_ITEM(oZ                  , Data100[dataIndex].oZ)
//      SEND_ITEM(ss                  , Data10[dataIndex/10].ss)
//      SEND_ITEM(P1_setting          , P1_setting)
//      SEND_ITEM(P2_setting          , P2_setting)
//      SEND_ITEM(P3_setting          , P3_setting)
//      SEND_ITEM(P4_setting          , P4_setting)
//      SEND_ITEM(P5_setting          , P5_setting)
//      SEND_ITEM(P5V_setting         , P5V_setting)
//      SEND_ITEM(Apogee_Passed       , Apogee_Passed)
//      SEND_ITEM(run_time            , Data10[dataIndex/10].run_time)
////      SEND_ITEM(gps_vel             , Data10[dataIndex/10].gps_vel)
////      SEND_ITEM(gps_dir             , Data10[dataIndex/10].gps_dir)
//      SEND_ITEM(x_from_launch       , Data10[dataIndex/10].x_from_launch)
//      SEND_ITEM(y_from_launch       , Data10[dataIndex/10].y_from_launch)
//      SEND_ITEM(dir_from_launch     , Data10[dataIndex/10].dir_from_launch)
//      TELEMETRY_SERIAL.print(F(";"));
//      TELEMETRY_SERIAL.print(F("gps_lat"));
//      TELEMETRY_SERIAL.print(F(":"));
//      TELEMETRY_SERIAL.print(Data10[dataIndex/10].gps_lat,5);            //more digits of precision
//      TELEMETRY_SERIAL.print(F(";"));
//      TELEMETRY_SERIAL.print(F("gps_lon"));
//      TELEMETRY_SERIAL.print(F(":"));
//      TELEMETRY_SERIAL.print(Data10[dataIndex/10].gps_lon,5);            //more digits of precision
//      SEND_ITEM(gps_alt             , Data10[dataIndex/10].gps_alt)
//      SEND_ITEM(sats                , Data10[dataIndex/10].SIV)
//      SEND_ITEM(hdp                 , Data10[dataIndex/10].PDOP)
//      SEND_ITEM(l2g                 , Data10[dataIndex/10].link2ground)
//      SEND_ITEM(tIMU                , Data1[dataIndex/100].temp)                         //BNO055 Temp
//      SEND_ITEM(vb                  , Data1[dataIndex/100].vbatt)
//      SEND_ITEM(vg                  , Data1[dataIndex/100].vgrid)
//      SEND_ITEM(gR                  , Data1[dataIndex/100].gpsRate)
//      SEND_ITEM(tR                  , Data1[dataIndex/100].radioRate)
//      SEND_ITEM(rR                  , Data1[dataIndex/100].readRate)
//      SEND_ITEM(bmpR                , Data1[dataIndex/100].bmpRate)
//      SEND_ITEM(bnoR                , Data1[dataIndex/100].bnoRate)
//      SEND_ITEM(sdR                 , Data1[dataIndex/100].sdRate)
//      SEND_ITEM(fR                  , Data1[dataIndex/100].fallRate)
//      SEND_ITEM(oR                  , Data1[dataIndex/100].oriRate)
//      
//      SEND_ITEM(gT                  , Data1[dataIndex/100].gpsT)
//      SEND_ITEM(tT                  , Data1[dataIndex/100].radioT)
//      SEND_ITEM(rT                  , Data1[dataIndex/100].readT)
//      SEND_ITEM(bmpT                , Data1[dataIndex/100].bmpT)
//      SEND_ITEM(bnoT                , Data1[dataIndex/100].bnoT)
//      SEND_ITEM(sdT                 , sdT)                          //just send the raw value itself
//      SEND_ITEM(fT                  , Data1[dataIndex/100].fallT)
//      SEND_ITEM(oT                  , Data1[dataIndex/100].oriT)
//
//      //constants
//      SEND_ITEM(ATST                , ATST)
//      SEND_ITEM(Launch_ALT          , Launch_ALT)
//      SEND_ITEM(BMPcf               , SEALEVELPRESSURE_HPA)
//      SEND_ITEM(launch_lat          , launch_lat)
//      SEND_ITEM(launch_lon          , launch_lon)
//  
//      //Fix to use the send_state function
////      if(state==STAND_BY){
////        SEND_ITEM(status, "STAND_BY" )
////      }
////      if(state==ARMED){
////        SEND_ITEM(status, "ARMED" )
////      }
////      if(state==POWERED_ASCENT){
////        SEND_ITEM(status, "POWERED_ASCENT" )
////      }
////      if(state==UNPOWERED_ASCENT){
////        SEND_ITEM(status, "UNPOWERED_ASCENT" )
////      }
////      if(state==FREEFALL){
////        SEND_ITEM(status, "FREEFALL" )
////      }
////      if(state==DROGUE_DESCENT){
////        SEND_ITEM(status, "DROGUE_DESCENT" )
////      }
////      if(state==MAIN_DESCENT){
////        SEND_ITEM(status, "MAIN_DESCENT" )
////      }
////      if(state==LANDED){
////        SEND_ITEM(status, "LANDED" )
////      }
//      
//      //write_state(status);
//      SEND(status, state)
//      
//      END_SEND
//  
//      radioCount++;
//      if(dataIndex%100 == 0){   //mod 100 -> 1Hz
//        Data1[dataIndex/100].radioRate= radioCount / ( (millis() - startTime) / 1000.0);
//        Data1[dataIndex/100].radioT= ( micros() - thisLoopMicros);
//      }
//      
//      //*****END Downlink Section****************************************************************
//    }
    //*****END Downlink Section**************************************************************** 4Hz

    //*****SD Logging Section (every Num_Datapoints*10ms)****************************************************************
    /*
    if( (dataIndex == (Num_Datapoints-1)) && (state != LANDED) ){  //if the flash memory data structure arrays are full
      thisLoopMicros= micros();
      
      for(int i = 0; i < (Num_Datapoints/10); i++){     //size of 10Hz array
        
        dataFile.print(Data10[i].run_time);         dataFile.print(',');      //run_time (T=0 at liftoff)
        if((i%10) == 0){       //1Hz
          dataFile.print(Data1[i/10].y);            dataFile.print('/');    
          dataFile.print(Data1[i/10].mo);           dataFile.print('/');
          dataFile.print(Data1[i/10].d);            dataFile.print(',');
          dataFile.print(Data1[i/10].h);            dataFile.print(':');
          dataFile.print(Data1[i/10].m);            dataFile.print(':');
          dataFile.print(Data1[i/10].s);            
        }
        else{
          //dataFile.print(',');
        }
        
        //writing sensor values
        WRITE_CSV_ITEM(Data10[i].yaw)
        WRITE_CSV_ITEM(Data10[i].pitch)
        WRITE_CSV_ITEM(Data10[i].roll)
        WRITE_CSV_ITEM(Data10[i].Px)
        WRITE_CSV_ITEM(Data10[i].Py)
        WRITE_CSV_ITEM(Data10[i].Pz)
        WRITE_CSV_ITEM(Data10[i].Vx)
        WRITE_CSV_ITEM(Data10[i].Vy)
        WRITE_CSV_ITEM(Data10[i].Vz)
        WRITE_CSV_ITEM(Data100[i*10].oX)
        WRITE_CSV_ITEM(Data100[i*10].oY)
        WRITE_CSV_ITEM(Data100[i*10].oZ)
        WRITE_CSV_ITEM(Data10[i].bmp_alt)
        WRITE_CSV_ITEM(Data10[i].bno_alt)
        WRITE_CSV_ITEM(Data100[i*10].gyroX)
        WRITE_CSV_ITEM(Data100[i*10].gyroY)
        WRITE_CSV_ITEM(Data100[i*10].gyroZ)
        WRITE_CSV_ITEM(Data10[i].accX)
        WRITE_CSV_ITEM(Data10[i].accY)
        WRITE_CSV_ITEM(Data10[i].accZ)
        WRITE_CSV_ITEM(Data10[i].bmp_pressure)
        WRITE_CSV_ITEM(Data10[i].qW)
        WRITE_CSV_ITEM(Data10[i].qX)
        WRITE_CSV_ITEM(Data10[i].qY)
        WRITE_CSV_ITEM(Data10[i].qZ)
        WRITE_CSV_ITEM(Data100[i*10].oriQuatW)
        WRITE_CSV_ITEM(Data100[i*10].oriQuatX)
        WRITE_CSV_ITEM(Data100[i*10].oriQuatY)
        WRITE_CSV_ITEM(Data100[i*10].oriQuatZ)
        WRITE_CSV_ITEM(Data10[i].eulerX)
        WRITE_CSV_ITEM(Data10[i].eulerY)
        WRITE_CSV_ITEM(Data10[i].eulerZ)
        WRITE_CSV_ITEM(Data10[i].state)
        WRITE_CSV_ITEM(Data10[i].Apogee_Passed)
        WRITE_CSV_ITEM(Data10[i].bno_descending)
        WRITE_CSV_ITEM(Data10[i].bmp_descending)
        WRITE_CSV_ITEM(Data10[i].gps_alt)
        WRITE_CSV_ITEM(Data10[i].SIV)
        WRITE_CSV_ITEM(Data10[i].PDOP)
        dataFile.print(F(", ")); dataFile.print(Data10[i].gps_lat,7);
        dataFile.print(F(", ")); dataFile.print(Data10[i].gps_lon,7);
//          WRITE_CSV_ITEM(Data10[i].gps_vel)
//          WRITE_CSV_ITEM(Data10[i].gps_dir)
//          
//          WRITE_CSV_ITEM(Data10[i].x_from_launch)
//          WRITE_CSV_ITEM(Data10[i].y_from_launch)
//          WRITE_CSV_ITEM(Data10[i].dir_from_launch)
//          WRITE_CSV_ITEM(Data10[i].xy_to_land)
//          WRITE_CSV_ITEM(Data10[i].dir_to_land)
//          WRITE_CSV_ITEM(Data10[i].x_to_land)
//          WRITE_CSV_ITEM(Data10[i].y_to_land)
        WRITE_CSV_ITEM(Data10[i].gps_descending)
        WRITE_CSV_ITEM(Data10[i].mx)
        WRITE_CSV_ITEM(Data10[i].my)
        WRITE_CSV_ITEM(Data10[i].mz)
        WRITE_CSV_ITEM(Data10[i].link2ground)

        if((i%10) == 0){       //1Hz
          WRITE_CSV_ITEM(Data1[i/10].vbatt)
          WRITE_CSV_ITEM(Data1[i/10].vgrid)
          WRITE_CSV_ITEM(Data1[i/10].bmp_temp)
          WRITE_CSV_ITEM(Data1[i/10].temp)       //bno temp
          WRITE_CSV_ITEM(Data1[i/10].gpsRate)
          WRITE_CSV_ITEM(Data1[i/10].radioRate)
          WRITE_CSV_ITEM(Data1[i/10].readRate)
          WRITE_CSV_ITEM(Data1[i/10].bmpRate)
          WRITE_CSV_ITEM(Data1[i/10].bnoRate)
          WRITE_CSV_ITEM(Data1[i/10].sdRate)
          WRITE_CSV_ITEM(Data1[i/10].fallRate)
          WRITE_CSV_ITEM(Data1[i/10].oriRate)
          WRITE_CSV_ITEM(Data1[i/10].gpsT)
          WRITE_CSV_ITEM(Data1[i/10].radioT)
          WRITE_CSV_ITEM(Data1[i/10].readT)
          WRITE_CSV_ITEM(Data1[i/10].bmpT)
          WRITE_CSV_ITEM(Data1[i/10].bnoT)
          WRITE_CSV_ITEM(Data1[i/10].sdT)
          WRITE_CSV_ITEM(Data1[i/10].fallT)
          WRITE_CSV_ITEM(Data1[i/10].oriT)
          WRITE_CSV_ITEM(Data1[i/10].P1_setting)
          WRITE_CSV_ITEM(Data1[i/10].P2_setting)
          WRITE_CSV_ITEM(Data1[i/10].P3_setting)
          WRITE_CSV_ITEM(Data1[i/10].P4_setting)
          WRITE_CSV_ITEM(Data1[i/10].P5_setting)
          WRITE_CSV_ITEM(Data1[i/10].P5V_setting)
        }
        else{
          //dataFile.print(',');
        }
        
        dataFile.println();   //go to the next line
      }
      
      dataFile.flush();     //ensures data is written to sd card (might need to put inside the for loop)
      
      sdCount++;
      sdRate= sdCount / ( (millis() - startTime) / 1000.0);
      sdT= ( micros() - thisLoopMicros );  
    }
    else if( (dataIndex == (Num_Datapoints-1)) && (DataWrittenOnLanding == 0) ){
      DataWrittenOnLanding= 1;    //can only do this once upon landing
      
      //copy above for loop here
      
      dataFile.flush();     //ensures data is written to sd card (might need to put inside the for loop)
      dataFile.close();
    }
    */
    //*****END SD Logging Section**************************************************************************
//    if((dataIndex%100)==0){
//      testT= (micros() - testTimer);
//    }

    //testT= ((micros() - testTimer) + testT) / 2.0;
    //testT= max(micros() - testTimer, testT);
    testT= (micros() - testTimer);
    sumT += testT;
    avgT= sumT/dataLoopIteration;
  }
  
}
