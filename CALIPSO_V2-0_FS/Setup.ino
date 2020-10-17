#define DEMO 0
#define FLIGHT 2
#define CONFIGURATION FLIGHT        //UPDATE b4 FLIGHT!!!

/*
#if CONFIGURATION == DEMO
#define COUNTDOWN_DURATION 10000    // 10 sec
#else
#define COUNTDOWN_DURATION 0        // 0 seconds
#endif
*/

#define HEARTBEAT_TIMEOUT  5000     //5 sec
#define BURN_TIME 1600              //1.6 sec, Update b4 Flight! J425=1.6s burn time
#define LIFTOFF_ACC_THRESH 13.0     //Liftoff Acceleration Threshold in m/s^2- only arm once stationary on the pad

#define TELEMETRY_SERIAL Serial    //Serial for USB, Serial1 for 3.6, Serial5 or Serial8 for 4.1

#define Timer10ms   10    //100Hz-   dataIndex%1==0    Timer100Hz, Ori, Kalman, 100Hz sensors (BNO, BMP), falling logic, state change logic, sd logging logic,
  //25Hz sd logging ex: if(i%4==0){ WRITE_CSV_ITEM(Data25[i/4].SIV) }
  //25Hz data collection ex: if(dataIndex%4==0){ Data25[dataIndex/4].SIV= myGPS.getSIV(); }
/*  
#define Timer40ms   40    //25Hz- dataIndex%4 == 0
#define Timer100ms  100   //10Hz- dataIndex%10 == 0
#define Timer250ms  250   //4Hz-  dataIndex%25 == 0
#define Timer1000ms 1000  //1Hz-  dataIndex%100 == 0
*/

/*
#define radio_dt 250                //time in ms between sending telemetry packets
#define read_dt 500                 //time in ms between recieving telemetry packets
#define gps_dt 40                   //time in ms between samples for Neo M9N GPS
#define bmp_dt 100                  //time in ms between samples for bmp388
#define ori_dt 10                   //time in ms between orientation updating
#define bno_dt 10.0                 //time in ms between samples for bno055
#define sd_dt 200                   //time in ms between data points in csv file logging
#define fall_dt 100
*/

#define BMP_SCK 27
#define BMP_MISO 39
#define BMP_MOSI 26
#define BMP_CS 38  
#define Grid_V 40                   //A16
#define Batt_V 41                   //A17
#define LED 13 //Error LED, refers to builtin LED on teensy
#define PYRO1 33  //PWM capable on Teensy 4.1
#define PYRO2 36  //PWM capable on Teensy 4.1
#define PYRO3 37  //PWM capable on Teensy 4.1
#define PYRO4 14  //PWM capable on Teensy 4.1
#define PYRO5 15  //PWM capable on Teensy 4.1
#define PYRO5V 32
#define PWM1 0
#define PWM2 1
#define PWM3 2
#define PWM4 3
#define PWM5 4
#define PWM6 5         
#define PWM7 5
#define PWM8 7
#define PWM9 8
#define PWM10 9
#define PWM11 10
#define PWM12 11
#define PWM13 12
#define PWM14 24
#define PWM15 25
#define PWM16 28
#define PWM17 29
#define PWM18 23
#define PWM19 22
#define PWM20 19
#define NumPrevDataPtIndices 500   //Number of previous data point indices that will be stored, 100 corresponds to 1 second

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
  POWERED_ASCENT,   //if (state==ARMED) && (data100[dataIndex].Acc.z() > LIFTOFF_ACC_THRESH) )
  UNPOWERED_ASCENT, //if (state==POWERED_ASCENT) && (data100[dataIndex].run_time>BURN_TIME)
  FREEFALL,         //if (state==UNPOWERED_ASCENT) && (data100[dataIndex].Apogee_Passed == 1)
  DROGUE_DESCENT,   //if (state==FREEFALL) && (data100[dataIndex].DROGUE_FIRED==1)
  MAIN_DESCENT,     //if (state==DROGUE_DESCENT) && (data100[dataIndex].MAIN_FIRED==1)
  LANDED            //if (state==MAIN_DESCENT) && (data100[dataIndex].bmp_alt < Launch_ALT + 5)
} state_t;

state_t state = STAND_BY;


//**********Define data types that holds important information***************************************

uint32_t Num_Datapoints= 10000;   //Define the number of Data objects to store in flash (NEEDS TO BE DIVISIBLE BY 25, 10, 4, 1!!!!!)
uint64_t Timer100Hz= 0;           //millis timer that keeps track of the 10ms timer loop
uint32_t dataLoopIteration= 0;    //number of 10ms iterations have passed since starting void loop
uint32_t dataIndex= 0;            //index of Data array to store everything in for the moment
uint32_t dataLast[NumPrevDataPtIndices];   //arry that stores the indices of the last 1 to n data sets (for Data array)

struct{
  //Flow Control
  uint64_t loopIteration;         //how many times the void loop has iterated (not super important, just interesting)
  state_t state;                  //Data100[dataIndex].state= state; 
  //Logistical
  bool ss;                        //ss is sensor status
  uint32_t abort_time, start_time, liftoff_time;
  int32_t run_time;
  //I/O
  bool P1_setting, P2_setting, P3_setting, P4_setting, P5_setting;
  //orientation
  double dtGyro;                  //in seconds
  //Kalman Filter
  float Px, Py, Pz, Vx, Vy, Vz; //, PzKp, PzCp;    //need to be set to 0
  //BNO055
  imu::Vector<3> ori;
  imu::Vector<3> oriGyro;
  imu::Vector<3> gyroscope;
  imu::Vector<3> euler;
  imu::Vector<3> Acc;
  imu::Quaternion q1;
  imu::Quaternion oriQuat;        //orientation storing quaternion
  imu::Quaternion rotQuat;        //new rotation quaternion based on gyro rates * dt for each iteration
  double roll, pitch, yaw, oX, oY, oZ;
  /*
  double heading;
  double attitude;
  double bank;
  */
  float bno_alt;                  //m AGL;
  //BMP388
  float bmp_pressure, bmp_alt;
  //fall
  bool bno_descending, bmp_descending;
  bool Apogee_Passed, DROGUE_FIRED, MAIN_FIRED;
}data100hz_t;

struct{
  //GPS
  bool gps_descending;
  uint8_t SIV, fixType;
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
  double xy_dir_to_land;                  //Cardinal direction in deg to land pt
}data25hz_t;

struct{
  imu::Vector<3> magnetometer;
}data10hz_t;

struct{
  bool link2ground;
}data4hz_t;

struct{
  //Flow Control
  float gpsRate, radioRate, readRate, bmpRate, bnoRate, sdRate, fallRate, oriRate;    //All in Hz
  float gpsT, radioT, readT, bmpT, bnoT, sdT, fallT, oriT;                            //All in ms
  //Logistical
  float vbatt, vgrid;
  //BNO
  float temp;                     //bno temp
  //BMP388
  float bmp_temp
}data1hz_t;

//Create arrays of various data type objects to store the last x seconds of data in flash memory
data100hz_t Data100[Num_Datapoints] PROGMEM;
data25hz_t  Data25[Num_Datapoints/4] PROGMEM;
data10hz_t  Data10[Num_Datapoints/10] PROGMEM;
data4hz_t   Data4[Num_Datapoints/25] PROGMEM;
data1hz_t   Data1[Num_Datapoints/100] PROGMEM;


//**********END Data Types*****************************************************************


//Initialize Sensors
SFE_UBLOX_GPS myGPS;
Adafruit_BMP3XX bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);    //ID, Address, Wire

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
uint32_t gpstimer, gpsCount, radiotimer, radioCount, readtimer, readCount, bmptimer, bmpCount bnotimer, bnoCount, sdtimer, sdCount, falltimer, fallCount, oritimer, oriCount, heartbeat_time;
gpstimer = gpsCount = radiotimer = radioCount = readtimer = readCount = bmptimer = bmpCount = bnotimer = bnoCount = sdtimer = sdCount = falltimer = fallCount = oritimer = oriCount = heartbeat_time = 0;
uint64_t thisLoopMicros, startTimeMicros, lastGyroUpdate, startTime, loopIteration;
thisLoopMicros = startTimeMicros = lastGyroUpdate = startTime = loopIteration = 0;
Data100[dataIndex].start_time = Data100[dataIndex].run_time = 0;

//SD
File dataFile;
char filename[] = "DATA000.csv";
bool DataWrittenOnLanding= 0;

//Batt
double reading;
uint32_t voltage_divider_ratio= 21;                         //20k drop resistor, 1k gnd resistor

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
double sqw, sqx, sqy, sqz, unit_, test, cy, sy, cp, sp, cr, sr; 

//BNO
double GyroAvgX, GyroAvgY, GyroAvgZ;
GyroAvgX = GyroAvgY = GyroAvgZ = 0;
//uint16_t NumPrevGyroPts= 500;     //100 corresponds to 1 second, NumPrevDataPtIndices does the same thing
double dyaw, dpitch, droll;       //gyro math variables
double bno_alt_last [10]= {0,0,0,0,0,0,0,0,0,0};
double bno_alt_new [10]= {0,0,0,0,0,0,0,0,0,0};
double bno_alt_last_avg= 0;
double bno_alt_new_avg= 0;
int bno_descending_counter= 0;
data100[dataIndex].bno_descending= 0;

//KALMAN Filter variables
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
  //double Array Xs needed to fill XS
double Xs[6][1]= {{0},      
                  {0},
                  {0},         //m obove Home/Launch (AGL)
                  {0},
                  {0},
                  {0}};
  //State Matrix XS
Matrix<double> XS(6,1,(double*)Xs);   //Matrix Lib Format, see example for help
double dt= bno_dt/1000.0;   //.01
double Kdt, dt2, dt3, dt4;
double Kt0, Kt; 
Kt0 = Kt = 0;


//BMP
double bmp_alt_last [20]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
double bmp_alt_last_avg, bmp_alt_new_avg;
bmp_alt_last_avg = bmp_alt_new_avg = 0;
int bmp_descending_counter;
bmp_descending_counter= 0;
data100[dataIndex].bmp_descending= 0;

//GPS
double gps_alt_last [10]= {0,0,0,0,0,0,0,0,0,0};
double gps_alt_new [10]= {0,0,0,0,0,0,0,0,0,0};
double gps_alt_last_avg, gps_alt_new_avg;
gps_alt_last_avg = gps_alt_new_avg = 0;
int gps_descending_counter= 0;

//Other
double sum, sum2, sum3;
sum = sum2 = sum3 = 0;


//Helper functions  
void SET_STATE(state_t STATE){
  state= STATE;
}

double Avg_Last_N_Measurements(double prev[], uint32_t N)[ //return the average of the last N data points
  sum=  0;  //ensure that sum is reset
  for(int i = 0; i < N; i++){
    sum += prev[i];
  }
  return (sum/N);
}
 
void start_countdown() {        //currently the countdown is 0 seconds as acceleromter readings will enable POWERED_ASCENT
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
    SET_STATE(ARMED);
    data100[dataIndex].start_time = millis();
    data100[dataIndex].abort_time= 0;
    heartbeat();
  }
}

void heartbeat() {
  heartbeat_time = millis();
}

//FIX
void (*reset)(void) = 0;

void write_state(const char *state_name) {    //send state (can be asynchronous)
  SEND(status, state_name);
}

void abort_autosequence() {   //need to check if data is still logged after an abort is triggered
  TELEMETRY_SERIAL.println(F("Run aborted"));
  switch (state) {
    case STAND_BY:
    case ARMED:
      SET_STATE(STAND_BY);
      data100[dataIndex].abort_time = millis();
      data100[dataIndex].run_time=0;
      data100[dataIndex].start_time=0;
      break;

    //Aborting in any one of the following scenarios would require a reset if doing ground testing (new startup sequence, new datafile, etc)
    case POWERED_ASCENT:
      //SET_STATE(STAND_BY);
      data100[dataIndex].abort_time = millis();
      break;

    case UNPOWERED_ASCENT:
      //SET_STATE(STAND_BY)'
      data100[dataIndex].abort_time = millis();
      break;

    case FREEFALL:
      //SET_STATE(STAND_BY)
      data100[dataIndex].abort_time = millis();
      break;

    case DROGUE_DESCENT:
      //SET_STATE(STAND_BY)
      data100[dataIndex].abort_time = millis();
      break;

    case MAIN_DESCENT:
      //SET_STATE(STAND_BY)
      data100[dataIndex].abort_time = millis();
      break;


    //No point in aborting at this point...
    case LANDED:
      SET_STATE(STAND_BY);
      data100[dataIndex].abort_time = millis();
      break;
  }
}
