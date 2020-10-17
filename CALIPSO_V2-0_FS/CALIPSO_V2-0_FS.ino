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

/*
time_t getTeensy3Time(){
  return Teensy3Clock.get();
}
*/

void setup() {
  //Start Teensy RTC- this line doesn't seem to matter, see Teensy Time Lib Documentation
  //setSyncProvider(getTeensy3Time);
  
  //Start I2C
  //Wire.setSDA(17); Wire.setSCL(16); //use SDA1, SCL1 for Teensy 4.1
  Wire1.begin();   
  Wire1.setClock(400000);
  delay(1000);

  //Start SPI
  //Similarly, SPI.setMOSI(pin), SPI.setMISO(pin), and SPI.setSCK(pin)
  SPI1.begin(); 
  
  //Start GPS
  while(myGPS.begin(Wire1) == false){
    TELEMETRY_SERIAL.println(F("GPS err"));
    digitalWrite(LED,LOW); delay(1000); 
    //2 Pulses
    digitalWrite(LED,HIGH); delay(100); 
    digitalWrite(LED,LOW); delay(100);
    digitalWrite(LED,HIGH); delay(100);
    Data100[dataIndex].ss=0;
  }
  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.setNavigationFrequency(25); //Set output to 25 times a second (Neo M9N Max Update Rate)
  //Possible Dynamic Model options: PORTABLE, STATIONARY, PEDESTRIAN, AUTOMOTIVE, SEA, AIRBORNE1g, AIRBORNE2g, AIRBORNE4g, WRIST, BIKE
  if (myGPS.setDynamicModel(DYN_MODEL_AIRBORNE4g) == false){ // Set the dynamic model to DYN_MODEL_AIRBORNE4g
    TELEMETRY_SERIAL.println(F("***!!! Warning: setDynamicModel failed !!!***"));
  }
  else{
    //TELEMETRY_SERIAL.println(F("Dynamic platform model changed successfully!"));
  }
  //Check Dynamic Model
  /*
  uint8_t newDynamicModel = myGPS.getDynamicModel();
  if (newDynamicModel == 255){
    TELEMETRY_SERIAL.println(F("***!!! Warning: getDynamicModel failed !!!***"));
  }
  else{
    TELEMETRY_SERIAL.print(F("The new dynamic model is: "));
    TELEMETRY_SERIAL.println(newDynamicModel);
  }  
  */
  //Check Nav Frequency
  /*
  byte rate = myGPS.getNavigationFrequency(); //Get the update rate of this module
  TELEMETRY_SERIAL.print("Current update rate:");
  TELEMETRY_SERIAL.println(rate);
  */
  myGPS.saveConfiguration();
  
  //Start Serial Radio- use serial5 or serial8
  while(TELEMETRY_SERIAL.begin(57600) == false){
    TELEMETRY_SERIAL.println(F("Radio err"));
    digitalWrite(LED,LOW); delay(1000); 
    //3 Pulses
    digitalWrite(LED,HIGH); delay(100); 
    digitalWrite(LED,LOW); delay(100);
    digitalWrite(LED,HIGH); delay(100);
    digitalWrite(LED,LOW); delay(100);
    digitalWrite(LED,HIGH); delay(100);
    Data100[dataIndex].ss=0;
  }
  
  //Start BMP388
  while (!bmp.begin()) {                         //flashes to signal error
    TELEMETRY_SERIAL.println(F("BMP388 err"));
    digitalWrite(LED,LOW); delay(1000);
    //1 Pulse
    digitalWrite(LED,HIGH); delay(100); 
    Data100[dataIndex].ss=0;
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_100_HZ);   //bmp.setOutputDataRate(BMP3_ODR_50_HZ);  ...ODR_200_HZ, _12_5_HZ, _3_1_HZ, _1_5_HZ, etc

  //Start BNO055 in OPERATION_MODE_NDOF
  while (!bno.begin()) {                         //flashes to signal error
    TELEMETRY_SERIAL.println(F("BNO055 err"));
    digitalWrite(LED,LOW); delay(1000);
    //5 Pulses
    digitalWrite(LED,HIGH); delay(100); 
    digitalWrite(LED,LOW); delay(100);
    digitalWrite(LED,HIGH); delay(100);
    digitalWrite(LED,LOW); delay(100);
    digitalWrite(LED,HIGH); delay(100);
    digitalWrite(LED,LOW); delay(100);
    digitalWrite(LED,HIGH); delay(100);
    digitalWrite(LED,LOW); delay(100);
    digitalWrite(LED,HIGH); delay(100);
    Data100[dataIndex].ss=0;
  }
  bno.setMode(Adafruit_BNO055::OPERATION_MODE_NDOF);  //OPERATION_MODE_NDOF_FMC_OFF, see .cpp for all modes "AMG" is the non-fusion mode with all sensors
  //bno.setMode(Adafruit_BNO055::OPERATION_MODE_AMG);
  bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P1); //REMAP_CONFIG_P0 to P7 (1 default)
  //bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P7);  //REMAP_SIGN_P0 to P7 (1 default)
  bno.setExtCrystalUse(true);

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
    Data100[dataIndex].ss=0;
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
        dataFile.print(F(",sats,hdop,vbatt,x accel,y accel,z accel,x gyro,y gyro,z gyro"));
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
  pinMode(PYRO5V, OUTPUT);  
  digitalWrite(PYRO1,LOW);
  digitalWrite(PYRO2,LOW);
  digitalWrite(PYRO3,LOW);
  digitalWrite(PYRO4,LOW);
  digitalWrite(PYRO5,LOW);
  digitalWrite(PYRO5V,LOW);

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
  TELEMETRY_SERIAL.println(F("Waiting for a 3D fix..."));
  while (fixType < 3){
    //obtain system voltages
    digitalWrite(LED,LOW); delay(500); digitalWrite(LED,HIGH);delay(250);
    reading= analogRead(Batt_V);
    Data1[dataIndex].vbatt= reading*(3.3/1023.00)* voltage_divider_ratio;
    reading= analogRead(Grid_V);
    Data1[dataIndex].vgrid= reading*(3.3/1023.00)* voltage_divider_ratio;
    //3D fix
    Data25[dataIndex].fixType = myGPS.getFixType(); //Get fix type: 0= No Fix, 1= Dead Reckoning, 2= 2D, 3= 3D, 4= GNSS + Dead Reckoning, 5= Time only
    Data25[dataIndex].SIV = myGPS.getSIV() / 100.0;
    Data25[dataIndex].PDOP = myGPS.getPDOP() / 1.0E3;
    
    SEND_ITEM(SIV                 , Data25[dataIndex].SIV)
    SEND_ITEM(PDOP                , Data25[dataIndex].PDOP)
    SEND_ITEM(fixType             , Data25[dataIndex].fixType)
    SEND_ITEM(vb                  , Data25[dataIndex].vbatt)
    SEND_ITEM(vg                  , Data25[dataIndex].vgrid)
    
    TELEMETRY_SERIAL.print(F("\n"));
    delay(1000);
  }
  
  TELEMETRY_SERIAL.println(F("3D fix found!"));
  delay(5000); //more time to get a more accurate GPS lock
  
  Data25[dataIndex].gps_alt= myGPS.getAltitude();
  launch_lat= myGPS.getLatitude() / (1.0E7);
  launch_lon= myGPS.getLongitude() / (1.0E7);
  //set the Teensy internal RTC, approximately line up RTC second with GPS milliseconds (not perfect)
  while(myGPS.getMillisecond() < 900){  //be VERY careful with this line, sensitive to NavigationFrequency
    delay(1);
    TELEMETRY_SERIAL.println(myGPS.getMillisecond());
  }
  setTime(myGPS.getHour(),myGPS.getMinute(),myGPS.getSecond(),myGPS.getDay(),myGPS.getMonth(),myGPS.getYear());
  
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
  
  Data100[dataIndex].q1 = bno.getQuat();   //I don't think this needs to be run here, oriQuat can still equal q1 even if q1 doesn't have values yet, right?
  
  //Loop Timers
  startTimeMicros= micros();  //Time when loop was started (microseconds)
  lastGyroUpdate= startTimeMicros;
  startTime = millis();       //Time when loop was started
  Timer100Hz= millis();

}









void loop() {
  if(millis()-Timer100Hz > Timer10ms){      //100hz max
    dataLoopIteration++;
    dataIndex= dataLoopIteration % Num_Datapoints;
    Data100[dataIndex].loopIteration = loopIteration;

    // /*
    if(dataLoopIteration > NumPrevDataPtIndices){
      for(int i = 0; i < NumPrevDataPtIndices; i++){
        dataLast[i]= (dataLoopIteration - i - 1) % Num_Datapoints;
      }
    }
    // */
    
    //it makes more sense to use if(dataLoopIteration > NumPrevDataPtIndices) before each code section that references previous values
     /*
    else{
      for(int i = 0; i < NumPrevDataPtIndices; i++){
        dataLast[i]= 0;
      }
    }
     */
    
    //Update run_time (time starting at liftoff (having a countdown can make run_time negative))
    if( (state != ARMED) && (state != STAND_BY) && (state != LANDED) ){
        Data100[dataIndex].run_time= millis() - Data100[dataIndex].start_time;  //OLD: - COUNTDOWN_DURATION
    }


    //*****Orientations Section****************************************************************
    thisLoopMicros= micros();
    
    Data100[dataIndex].dtGyro = (double)(thisLoopMicros - lastGyroUpdate) / 1000000.0;
    lastGyroUpdate = thisLoopMicros;
    
    //get gyro data all the time for logging:
    //Data100[dataIndex].gyroscope= bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);     //100hz, dps
    //Data100[dataIndex].gyroscope.x()
    //it might be a good idea to set the BNO to a non-fusion mode after launch so that the dps can be set to 250 vs 2000 for more accurate gyro orientation
    //on the other hand, it's probably best to keep the fusion on and just use a BMI088 to get super accurate vibration resistant gyro readings anyways. In addition, the fusion data
    //from the BNO can be compared to BMI088 after various flights

    if(state==STAND_BY || state==ARMED){      //1  //state==STAND_BY || state==ARMED      If I can ever get the gyro code working...
      //oX/oY/oZ each respectively represent orientation about the x/y/z body axis
      Data100[dataIndex].oX= Data100[dataIndex].roll;
      Data100[dataIndex].oY= Data100[dataIndex].pitch;
      Data100[dataIndex].oZ= Data100[dataIndex].yaw;
      //use sensor fusion quat for current orientation
      Data100[dataIndex].oriQuat= Data100[dataIndex].q1;
    }
    else{
      //convert gyro rates to euler rates to a quaternion
      dyaw =   ( Data100[dataIndex].gyroscope.x() )*Data100[dataIndex].dtGyro;   //.z *should* be the rads/s rate about the z axis of the BMI055, so yaw rate
      dpitch = ( Data100[dataIndex].gyroscope.y() )*Data100[dataIndex].dtGyro;   //.y *should* be the rads/s rate about the y axis of the BMI055, so pitch rate
      droll =  ( Data100[dataIndex].vgyroscope.z() )*Data100[dataIndex].dtGyro;   //.x *should* be the rads/s rate about the z axis of the BMI055, so roll rate

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

      //not sure which I need
      imu::Quaternion Data100[dataIndex].rotQuat(cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy);
      //Data100[dataIndex].rotQuat(cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy);
      
      //imu::Quaternion oriQuat(cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy);
      //oriQuat(cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy);  //does NOT work

      
      //oriQuat= rotQuat*oriQuat;
      Data100[dataIndex].oriQuat= Data100[dataIndex].oriQuat*Data100[dataIndex].rotQuat;   //update the orientation storing quaternion
      
      Data100[dataIndex].oriGyro= Data100[dataIndex].oriQuat.toEuler(); //convert the ori storing quat to euler angles (no gimbal lock issues!)
      Data100[dataIndex].oX= RAD_TO_DEG*Data100[dataIndex].oriGyro.z();   //oX is roll, .z() represents the 3rd euler rotation which is roll
      Data100[dataIndex].oY= RAD_TO_DEG*Data100[dataIndex].oriGyro.y();   //oY is pitch, .y() represents the 2nd euler rotation which is pitch
      Data100[dataIndex].oZ= RAD_TO_DEG*Data100[dataIndex].oriGyro.x();   //oZ is yaw, .x() represents the 1rd euler rotation which is yaw
    }

    oriCount++;
    if(dataIndex%100 == 0){   //mod 100 -> 1Hz
      Data1[dataIndex/100].oriRate= oriCount / ( (millis() - startTime) / 1000.0);
      Data1[dataIndex/100].oriT= ( thisLoopMicros - micros() ) / 1000.0;
    }
    //*****END Orientations Section****************************************************************

    //*****BNO, KALMAN, GPS Section****************************************************************
    thisLoopMicros= micros();

    Data100[dataIndex].euler         = bno.getVector(Adafruit_BNO055::VECTOR_EULER);         //default deg, ***sensor fusion***
    Data100[dataIndex].q1            = bno.getQuat();                                        //unitless, ***sensor fusion***
    Data100[dataIndex].gyroscope     = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);     //default dps
    Data100[dataIndex].Acc           = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER); //default m/s^2
    if(dataIndex%10 == 0){          //20Hz
      Data10[dataIndex/10].magnetometer = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);  //default microT
    }
    if(dataIndex%100 == 0){          //1Hz
    Data1[dataIndex/100].temp            = bno.getTemp();                                        //default C
    }
    
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
    Data100[dataIndex].ori=   Data100[dataIndex].q1.toEuler();
    Data100[dataIndex].yaw=   RAD_TO_DEG*ori.x();    //x represents the 1rd rotation, not rotation about the x axis (which would be roll)
    Data100[dataIndex].pitch= RAD_TO_DEG*ori.y();  //y represents the 2rd rotation, not rotation about the y axis (although that would also be pitch)
    Data100[dataIndex].roll=  RAD_TO_DEG*ori.z();   //z represents the 3rd rotation, not rotation about the z axis (which would be yaw)

    //***KALMAN Filter Code*********** (could always add more filters w/ different sensors for different estimates)
    Kt= millis()/1000.0;    //time in s
    Kdt= Kt- Kt0; //dt for Kalman Filter, in s
      //technically I should be able to use (bno_dt/1000= .01) for Kdt
    if(Kdt > 10.0){    //can increase to 100 if necessary- see log
      Kdt= 10.0;
    }
    Kt0= Kt;

    dt2= Kdt*Kdt;
    dt3= dt2*Kdt;
    dt4= dt3*Kdt;
    
    //Update time step dependent matricies & Accel readings
    A._entity[0][3]= Kdt;
    A._entity[1][4]= Kdt;
    A._entity[2][5]= Kdt;

    B._entity[0][0]= .5*dt2;
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
    
    //Z faces up in current config
    if(state==STAND_BY || state==ARMED || state==POWERED_ASCENT || state==UNPOWERED_ASCENT || state==LANDED){ //Earth's gravity actin on IMU
      U._entity[0][0]= Data100[dataIndex].Acc.x(); //earth fixed x Accel
      U._entity[1][0]= Data100[dataIndex].Acc.y(); //earth fixed y Accel
      U._entity[2][0]= (Data100[dataIndex].Acc.z()*cos(DEG_TO_RAD*0)*cos(DEG_TO_RAD*0)) -9.81; //earth fixed z Accel
    }
    else{ //IMU in freefall
      U._entity[0][0]= Data100[dataIndex].Acc.x(); //earth fixed x Accel
      U._entity[1][0]= Data100[dataIndex].Acc.y(); //earth fixed y Accel
      U._entity[2][0]= (Data100[dataIndex].Acc.z()*cos(DEG_TO_RAD*0)*cos(DEG_TO_RAD*0)); //earth fixed z Accel
    }
    
    //predict new states (XS continually updates)
    XS= (A*XS)+(B*U);
    P= ((A*P)*Matrix<double>::transpose(A))+Q;
  
    //****Update States w/ GPS Measurement*********
    if(dataIndex%4 == 0){
      thisLoopMicros= micros();

      Data25[dataIndex/4].fixType = myGPS.getFixType();
      Data25[dataIndex/4].SIV = myGPS.getSIV();
      Data25[dataIndex/4].PDOP = myGPS.getPDOP() / 100.0;  //m
      Data25[dataIndex/4].gps_alt= myGPS.getAltitude() / 1.0E3; //m ASL
      Data25[dataIndex/4].gps_lon= myGPS.getLongitude() / 1.0E7;
      Data25[dataIndex/4].gps_lat= myGPS.getLatitude() / 1.0E7;
      //can add in xy velocity and heading at some point (see library)
      Data25[dataIndex/4].x_from_launch= TinyGPSPlus::distanceBetween(0, launch_lon, 0, Data25[dataIndex/4].gps_lon);
      Data25[dataIndex/4].y_from_launch= TinyGPSPlus::distanceBetween(launch_lat, 0, Data25[dataIndex/4].gps_lat, 0);
        //add Data25[dataIndex/4].xy_from_launch here at some point
      Data25[dataIndex/4].dir_from_launch= TinyGPSPlus::courseTo(launch_lat, launch_lon, Data25[dataIndex/4].gps_lat, Data25[dataIndex/4].gps_lon);
      Data25[dataIndex/4].xy_to_land= TinyGPSPlus::distanceBetween(Data25[dataIndex/4].gps_lat, Data25[dataIndex/4].gps_lon, land_lat, land_lon);
      Data25[dataIndex/4].xy_dir_to_land= TinyGPSPlus::courseTo(Data25[dataIndex/4].gps_lat, Data25[dataIndex/4].gps_lon, launch_lat, launch_lon);
      Data25[dataIndex/4].x_to_land= TinyGPSPlus::distanceBetween(0, Data25[dataIndex/4].gps_lon, 0, land_lon);
      Data25[dataIndex/4].y_to_land= TinyGPSPlus::distanceBetween(Data25[dataIndex/4].gps_lat, 0, land_lat, 0);

      //GPS apogee detection- determines an average alt from recent measurements, then compares to the average calculated on the previous cycle
        //takes 11 measurements (11 cycles) to fully populate the array
      for (int i=0;i<9;i++){
        gps_alt_new[i+1]=gps_alt_new[i]; //move every element 1 back
      }
      gps_alt_new[0]= Data25[dataIndex/4].gps_alt; //now array fully updated
  
      /*
      for (int i=0;i<10;i++){
        sum += gps_alt_new[i];
      }
      gps_alt_new_avg= sum/10.0;
      sum=0;  //reset sum var for next use
      */
  
      gps_alt_new_avg= Avg_Last_N_Measurements(gps_alt_new, 10);
      
      if((gps_alt_last_avg > gps_alt_new_avg) && (Data25[dataIndex/4].bmp_alt > Launch_ALT + ATST)){
        gps_descending_counter= gps_descending_counter + 1;
      }

      /*
      for (int i=0;i<10;i++){
        gps_alt_last[i]= gps_alt_new[i];
      } //prev alts now = current alts
      gps_alt_last_avg= gps_alt_new_avg;
      */

      gps_alt_last= gps_alt_new;
      gps_alt_last_avg= gps_alt_new_avg;
      
      gpsCount++;
      if(dataIndex%100 == 0){   //mod 100 -> 1Hz
        Data1[dataIndex/100].gpsRate= gpsCount / ( (millis() - startTime) / 1000.0);
        Data1[dataIndex/100].gpsT= ( thisLoopMicros - micros() ) / 1000.0;
      }

      if(Data25[dataIndex/4].gps_descending != 1){
        if((gps_descending_counter>9)&& (Data100[dataIndex].bmp_alt > (Launch_ALT + ATST) )){
          Data25[dataIndex/4].gps_descending = 1;
        }
      }

      //Measurement & Update steps are below, I *think* both should only happen when a new measurement occurs
      //see https://towardsdatascience.com/sensor-fusion-part-2-kalman-filter-code-78b82c63dcd for reference
        //Measurement Step
      Z_Meas._entity[0][0]= Data25[dataIndex/4].x_from_launch;
      Z_Meas._entity[1][0]= Data25[dataIndex/4].y_from_launch;
      Z_Meas._entity[2][0]= (Data25[dataIndex/4].gps_alt - Launch_ALT);      //gps_alt is ASL
      Y_diff= Z_Meas- H*XS;
      K_denom= ( ((H*P)*H_T) + R );
      K= (P*H_T)*Matrix<double>::inv(K_denom);
      
        //Update Step
      XS= XS + (K*Y_diff);  //Calculate the new filtered state
      P= (I-(K*H))*P;       //Calculate the new P matrix
      
    }
    //****END Update States w/ GPS Measurement***********
    
    Data100[dataIndex].Px= XS._entity[0][0];
    Data100[dataIndex].Py= XS._entity[1][0];
    Data100[dataIndex].Pz= XS._entity[2][0]; //m AGL
    Data100[dataIndex].Vx= XS._entity[3][0];
    Data100[dataIndex].Vy= XS._entity[4][0];
    Data100[dataIndex].Vz= XS._entity[5][0];
    //Trust Kalman Filter for bno_alt (not bmp_alt)
    Data100[dataIndex].bno_alt= Launch_ALT + Data100[dataIndex].Pz;  //m ASL

    //***END KALMAN Filter Code***********
    
    //BNO apogee detection- determines an average alt from recent measurements, then compares to the average calculated on the previous cycle
        //takes 11 measurements (11 cycles) to fully populate the array
    for (int i=0;i<9;i++){
      bno_alt_new[i+1]=bno_alt_new[i]; //move every element 1 back
    }
    bno_alt_new[0]= Data100[dataIndex].bno_alt; //now array fully updated

    /*
    for (int i=0;i<10;i++){
      sum=sum+bno_alt_new[i];
    }
    bno_alt_new_avg= sum/10.0;
    sum=0;  //reset sum var for next use
    */

    bno_alt_new_avg= Avg_Last_N_Measurements(bno_alt_new, 10);
    
    if((bno_alt_last_avg > bno_alt_new_avg) && (Data100[dataIndex].bno_alt > Launch_ALT + ATST)){
      bno_descending_counter= bno_descending_counter + 1;
    }

    /*
    for (int i=0;i<10;i++){
      bno_alt_last[i]= bno_alt_new[i];
    } //prev alts now = current alts
    bno_alt_last_avg= bno_alt_new_avg;
    */

    bno_alt_last= bno_alt_new;  //arrays store the memory address of the first elm
    bno_alt_last_avg= bno_alt_new_avg;
    
    if(Data100[dataIndex].bno_descending != 1){
      if((bno_descending_counter>9)&& (Data100[dataIndex].bno_alt > Launch_ALT + ATST)){
        Data100[dataIndex].bno_descending = 1;
      }
    }

    bnoCount++;
    if(dataIndex%100 == 0){   //mod 100 -> 1Hz
      Data1[dataIndex/100].bnoRate= bnoCount / ( (millis() - startTime) / 1000.0);
      Data1[dataIndex/100].bnoT= ( thisLoopMicros - micros() ) / 1000.0;
    }
    //*****END BNO, KALMAN, GPS Section****************************************************************

    //*****BMP Section****************************************************************
    thisLoopMicros= micros();

    if(dataIndex%100 == 0){
      Data1[dataIndex/100].bmp_temp=       bmp.temperature; //C
    }
    Data100[dataIndex].bmp_pressure=   bmp.pressure / 100.0; //hPa or mbar
    Data100[dataIndex].bmp_alt=        bmp.readAltitude(SEALEVELPRESSURE_HPA); //m ASL
    //could add more filters here at some point (GPS/baro)...
    //Also, the BMP 388 is only good up to ~9000m... look into ones that can work higher than that

    //BMP apogee detection- determines an average alt from last 1-10 measurements and last 11-20 measurements
      //compares the averages... takes 20 measurements (20 cycles) to fully populate the array
    for (int i=0;i<19;i++){
      bmp_alt_last[i+1]=bmp_alt_last[i]; //move every element 1 back, [19] now forgotten
    }
    bmp_alt_last[0]= Data100[dataIndex].bmp_alt; //now array fully updated

    /*
    for (int i=0;i<10;i++){
      sum=sum+bmp_alt_last[i];
    }
    bmp_alt_new_avg= sum/10.0;  //avg alt over the most recent 10 data points
    sum=0;  //reset sum var for next use
    */

    bmp_alt_new_avg= Avg_Last_N_Measurements(bno_alt_new, 10);

    // /*
    sum=0;
    for (int i=10;i<20;i++){
      sum += bmp_alt_last[i];
    }
    bmp_alt_last_avg= sum/10.0; //avg alt over the oldest 10 data points
    sum=0;
    // */

    //not 100% sure about this one
    //bmp_alt_last_avg= Avg_Last_N_Measurements( (bno_alt_last + 10), 10);  //use the bno_alt_last array, but starting at index 10

    if((bmp_alt_last_avg > bmp_alt_new_avg) && (Data100[dataIndex].bmp_alt > (Launch_ALT + ATST))){
      bmp_descending_counter= bmp_descending_counter + 1;
    }
  
    if(Data100[dataIndex].bmp_descending != 1){
      if((bmp_descending_counter>9) && (Data100[dataIndex].bmp_alt > Launch_ALT + ATST)){
        Data100[dataIndex].bmp_descending = 1;
      }
    }
    
    bmpCount++;
    if(dataIndex%100 == 0){   //mod 100 -> 1Hz
      Data1[dataIndex/100].bmpRate= bmpCount / ( (millis() - startTime) / 1000.0);
      Data1[dataIndex/100].bmpT= ( thisLoopMicros - micros() ) / 1000.0;
    }
    //*****END BMP Section****************************************************************

    //*****Fall Section****************************************************************
    thisLoopMicros= micros();

    //Code that is active before Apogee is Reached/Passed
    if(Apogee_Passed == 0){
      //The following loop only trigger ONCE (when apogee is detected)
      if((gps_descending*1)+(bmp_descending*1)+(bno_descending*1) > 1){
        Data100[dataIndex].Apogee_Passed=1;  
        digitalWrite(PYRO1,HIGH);   //fire drouge chute
        DROGUE_FIRED= 1;            //represents if the drogue chute has actually been fired
        P1_setting= 1;              //for telem (P1_setting can be set to 1 without reaching apogee via the GUI)
      }
    }
    else{         //Continuous code that runs once Apogee is Reached/Passed
      //insert code here, ex: wait to fire main chutes or fire main chute when below a certain altitude
      if( bmp_alt < (Launch_ALT + 250) ){   //eject main at 250m
        digitalWrite(PYRO2,HIGH);   //fire main chute, just an example
        MAIN_FIRED= 1;              //mark that the main chute has actually been fired
        P2_setting=1;               //for telem
      }
    }

    //Test if gyro measurements have settled
    sum= 0;   //leave this here in case I switch back to the avg function for the 2nd bmp avg
    if(dataLoopIteration > NumPrevDataPtIndices){
      for(int i=0; i<NumPrevGyroPts; i++){
        sum  += Data100[i].gyroscope.x();   //all in dps
        sum2 += Data100[i].gyroscope.y();
        sum3 += Data100[i].gyroscope.z();
      }
      GyroAvgX= sum/NumPrevDataPtIndices;
      GyroAvgY= sum/NumPrevDataPtIndices;
      GyroAvgZ= sum/NumPrevDataPtIndices;
      sum= 0;
      sum2= 0;
      sum3= 0;
    }
    
    //state machine:
    if ( (state==ARMED) && (Data100[dataIndex].Acc.z() > LIFTOFF_ACC_THRESH) ){
      SET_STATE(POWERED_ASCENT);
      Data100[dataIndex].liftoff_time= millis();
    }
    if ( (state==POWERED_ASCENT) && (Data100[dataIndex].run_time > BURN_TIME) ){
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
      //reference prev avg gyro values (dps) to determine state
    if ( (state==MAIN_DESCENT) && (bmp_alt<(Launch_ALT+15)) && (GyroAvgX < 2.0) && (GyroAvgY < 2.0) && (GyroAvgZ < 2.0) ){
      SET_STATE(LANDED);
    }

    Data100[dataIndex].state= state;
    
    fallCount++;
    if(dataIndex%100 == 0){   //mod 100 -> 1Hz
      Data1[dataIndex/100].fallRate= fallCount / ( (millis() - startTime) / 1000.0);
      Data1[dataIndex/100].fallT= ( thisLoopMicros - micros() ) / 1000.0;
    }
    //*****END Fall Section****************************************************************

    //*****1Hz Section**************************************************************** 1Hz
    if(dataIndex%100 == 0){
      
      //*****Read Section**************************************************************
      thisLoopMicros= micros();
  
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
  
      READ_DEFAULT(data_name, data) {
        TELEMETRY_SERIAL.print(F("Invalid data field recieved: "));
        TELEMETRY_SERIAL.print(data_name);
        TELEMETRY_SERIAL.print(":");
        TELEMETRY_SERIAL.println(data);
      }
      END_READ
      
      //Read battery voltages
      reading= analogRead(Batt_V);
      Data1[dataIndex/100].vbatt= reading*(3.3/1023.00)* voltage_divider_ratio;
      reading= analogRead(Grid_V);
      Data1[dataIndex/100].vgrid= reading*(3.3/1023.00)* voltage_divider_ratio;
      
      readCount++;
      if(dataIndex%100 == 0){   //mod 100 -> 1Hz, keep this just in case the read section is moved from 1Hz
        Data1[dataIndex/100].readRate= readCount / ( (millis() - startTime) / 1000.0);
        Data1[dataIndex/100].readT= ( thisLoopMicros - micros() ) / 1000.0;
      }
      //*****END Read Section**************************************************************
      
    }
    //*****END 1Hz Section**************************************************************** 1Hz

    //*****4Hz Section**************************************************************** 4Hz
    if(dataIndex%25 == 0){
      //*****Downlink Section****************************************************************
      thisLoopMicros= micros();
    
      if (millis() > (heartbeat_time + HEARTBEAT_TIMEOUT) ) {
        //TELEMETRY_SERIAL.println(F("Loss of data link"));
        Data4[dataIndex/25].link2ground=0;
        //abort_autosequence();
      }
      else{
        Data4[dataIndex/25].link2ground= 1;
      }
      
      BEGIN_SEND
      SEND_VECTOR_ITEM(euler_angle  , Data100[dataIndex].euler)
      SEND_VECTOR_ITEM(gyro         , Data100[dataIndex].gyroscope)
      SEND_VECTOR_ITEM(acceleration , Data100[dataIndex].Acc)
      SEND_ITEM(bmp_alt             , Data100[dataIndex].bmp_alt)
      SEND_ITEM(bno_alt             , Data100[dataIndex].bno_alt)
      SEND_ITEM(roll                , Data100[dataIndex].roll)
      SEND_ITEM(pitch               , Data100[dataIndex].pitch)
      SEND_ITEM(yaw                 , Data100[dataIndex].yaw)
      SEND_ITEM(Px                  , Data100[dataIndex].Px)
      SEND_ITEM(Py                  , Data100[dataIndex].Py)
      SEND_ITEM(Pz                  , Data100[dataIndex].Pz)
      SEND_ITEM(oX                  , Data100[dataIndex].oX)
      SEND_ITEM(oY                  , Data100[dataIndex].oY)
      SEND_ITEM(oZ                  , Data100[dataIndex].oZ)
      SEND_ITEM(sats                , Data100[dataIndex].SIV)
      SEND_ITEM(hdp                 , Data100[dataIndex].PDOP)
      SEND_ITEM(ss                  , Data100[dataIndex].ss)
      SEND_ITEM(P1_setting          , Data100[dataIndex].P1_setting)
      SEND_ITEM(P2_setting          , Data100[dataIndex].P2_setting)
      SEND_ITEM(P3_setting          , Data100[dataIndex].P3_setting)
      SEND_ITEM(P4_setting          , Data100[dataIndex].P4_setting)
      SEND_ITEM(P5_setting          , Data100[dataIndex].P5_setting)
      SEND_ITEM(Apogee_Passed       , Data100[dataIndex].Apogee_Passed)
      SEND_ITEM(run_time            , Data100[dataIndex].run_time)

      //Not 100Hz- integer division will occur but shouldn't cause a fault
      /*
      SEND_ITEM(gps_vel             , Data25[dataIndex/4].gps_vel)
      SEND_ITEM(gps_dir             , Data25[dataIndex/4].gps_dir)
      */
      SEND_ITEM(x_from_launch       , Data25[dataIndex/4].x_from_launch)
      SEND_ITEM(y_from_launch       , Data25[dataIndex/4].y_from_launch)
      SEND_ITEM(dir_from_launch     , Data25[dataIndex/4].dir_from_launch)
      //SEND_ITEM(gps_lat           , gps_lat)
      TELEMETRY_SERIAL.print(F(";"));
      TELEMETRY_SERIAL.print(F("gps_lat"));
      TELEMETRY_SERIAL.print(F(":"));
      TELEMETRY_SERIAL.print(Data25[dataIndex/4].gps_lat,5);            //more digits of precision
      //SEND_ITEM(gps_lon           , gps_lon)
      TELEMETRY_SERIAL.print(F(";"));
      TELEMETRY_SERIAL.print(F("gps_lon"));
      TELEMETRY_SERIAL.print(F(":"));
      TELEMETRY_SERIAL.print(Data25[dataIndex/4].gps_lon,5);            //more digits of precision
      SEND_ITEM(gps_alt             , Data25[dataIndex/4].gps_alt)
      SEND_VECTOR_ITEM(magnetometer , Data10[dataIndex/10].magnetometer)
      SEND_ITEM(l2g                 , Data4[dataIndex/25].link2ground)
      SEND_ITEM(tIMU                , Data1[dataIndex/100].temp)                         //BNO055 Temp
      SEND_ITEM(vb                  , Data1[dataIndex/100].vbatt)
      SEND_ITEM(vg                  , Data1[dataIndex/100].vgrid)

      SEND_ITEM(gR                  , Data1[dataIndex/100].gpsRate)
      SEND_ITEM(tR                  , Data1[dataIndex/100].radioRate)
      SEND_ITEM(rR                  , Data1[dataIndex/100].readRate)
      SEND_ITEM(bmpR                , Data1[dataIndex/100].bmpRate)
      SEND_ITEM(bnoR                , Data1[dataIndex/100].bnoRate)
      SEND_ITEM(sdR                 , Data1[dataIndex/100].sdRate)
      SEND_ITEM(fR                  , Data1[dataIndex/100].fallRate)
      SEND_ITEM(oR                  , Data1[dataIndex/100].oriRate)
      
      SEND_ITEM(gT                  , Data1[dataIndex/100].gpsT)
      SEND_ITEM(tT                  , Data1[dataIndex/100].radioT)
      SEND_ITEM(rT                  , Data1[dataIndex/100].readT)
      SEND_ITEM(bmpT                , Data1[dataIndex/100].bmpT)
      SEND_ITEM(bnoT                , Data1[dataIndex/100].bnoT)
      SEND_ITEM(sdT                 , Data1[dataIndex/100].sdT)
      SEND_ITEM(fT                  , Data1[dataIndex/100].fallT)
      SEND_ITEM(oT                  , Data1[dataIndex/100].oriT)

      //constants
      SEND_ITEM(ATST                , ATST)
      SEND_ITEM(Launch_ALT          , Launch_ALT)
      SEND_ITEM(BMPcf               , SEALEVELPRESSURE_HPA)
      SEND_ITEM(launch_lat          , launch_lat)
      SEND_ITEM(launch_lon          , launch_lon)
  
      //Fix to use the send_state function
      if(state==STAND_BY){
        SEND_ITEM(status, "STAND_BY" )
      }
      if(state==ARMED){
        SEND_ITEM(status, "ARMED" )
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
      
      //write_state(status);
      
      END_SEND
  
      radioCount++;
      if(dataIndex%100 == 0){   //mod 100 -> 1Hz
        Data1[dataIndex/100].radioRate= radioCount / ( (millis() - startTime) / 1000.0);
        Data1[dataIndex/100].radioT= ( thisLoopMicros - micros() ) / 1000.0;
      }
      //*****END Downlink Section****************************************************************
    }
    //*****END 4Hz Section**************************************************************** 4Hz

    //*****SD Logging Section (every Num_Datapoints*10ms)****************************************************************
    if( (dataIndex == (Num_Datapoints-1)) && (state != LANDED) ){  //if the flash memory data structure arrays are full
      thisLoopMicros= micros();
      
      //fix names at top of csv file...

      for(int i = 0; i < Num_Datapoints; i++){
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
        WRITE_CSV_ITEM(Data100[i].yaw)
        WRITE_CSV_ITEM(Data100[i].pitch)
        WRITE_CSV_ITEM(Data100[i].roll)
        WRITE_CSV_ITEM(Data100[i].Px)
        WRITE_CSV_ITEM(Data100[i].Py)
        WRITE_CSV_ITEM(Data100[i].Pz)
        WRITE_CSV_ITEM(Data100[i].Vx)
        WRITE_CSV_ITEM(Data100[i].Vy)
        WRITE_CSV_ITEM(Data100[i].Vz)
        WRITE_CSV_ITEM(Data100[i].oX)
        WRITE_CSV_ITEM(Data100[i].oY)
        WRITE_CSV_ITEM(Data100[i].oZ)
        WRITE_CSV_ITEM(Data100[i].bmp_alt)

        WRITE_CSV_ITEM(Data100[i].bno_alt)
        //WRITE_CSV_VECTOR_ITEM(gyroscope)
        WRITE_CSV_ITEM(Data100[i].Acc.x())  
        WRITE_CSV_ITEM(Data100[i].Acc.y())
        WRITE_CSV_ITEM(Data100[i].Acc.z())
        WRITE_CSV_ITEM(Data100[i].gyroscope.x())
        WRITE_CSV_ITEM(Data100[i].gyroscope.y())
        WRITE_CSV_ITEM(Data100[i].gyroscope.z())
        //WRITE_CSV_VECTOR_ITEM(Acc)
        WRITE_CSV_ITEM(Data100[i].bmp_pressure)
        WRITE_CSV_ITEM(Data100[i].q1.w())
        WRITE_CSV_ITEM(Data100[i].q1.x())
        WRITE_CSV_ITEM(Data100[i].q1.y())
        WRITE_CSV_ITEM(Data100[i].q1.z())
        WRITE_CSV_VECTOR_ITEM(Data100[i].euler)
        WRITE_CSV_ITEM(Data100[i].state)
        WRITE_CSV_ITEM(Data100[i].Apogee_Passed)
        WRITE_CSV_ITEM(Data100[i].dtGyro)

        if((i%4) == 0){         //25Hz
          WRITE_CSV_ITEM(Data25[i/4].gps_alt)
          WRITE_CSV_ITEM(Data25[i/4].SIV)
          WRITE_CSV_ITEM(Data25[i/4].PDOP)
          //WRITE_CSV_ITEM(gps_lat) //change to have more precision
          dataFile.print(F(", ")); dataFile.print(Data25[i/4].gps_lat,7);
          //WRITE_CSV_ITEM(gps_lon) //change to have more precision
          dataFile.print(F(", ")); dataFile.print(Data25[i/4].gps_lon,7);
          /*
          WRITE_CSV_ITEM(Data25[i/4].gps_vel)
          WRITE_CSV_ITEM(Data25[i/4].gps_dir)
          */
          WRITE_CSV_ITEM(Data25[i/4].x_from_launch)
          WRITE_CSV_ITEM(Data25[i/4].y_from_launch)
          WRITE_CSV_ITEM(Data25[i/4].dir_from_launch)
          WRITE_CSV_ITEM(Data25[i/4].xy_to_land)
          WRITE_CSV_ITEM(Data25[i/4].xy_dir_to_land)
          WRITE_CSV_ITEM(Data25[i/4].x_to_land)
          WRITE_CSV_ITEM(Data25[i/4].y_to_land)
        }

        if((i%10) == 0){       //10Hz
          WRITE_CSV_VECTOR_ITEM(Data10[i/10].magnetometer)
        }

        if((i%25) == 0){       //4Hz
          WRITE_CSV_ITEM(Data4[i/25].link2ground)
        }

        if((i%100) == 0){       //1Hz
          WRITE_CSV_ITEM(Data1[i/100].vbatt)
          WRITE_CSV_ITEM(Data1[i/100].vgrid)
          WRITE_CSV_ITEM(Data1[i/100].bmp_temp)
          WRITE_CSV_ITEM(Data1[i/100].temp)       //bno temp

          WRITE_CSV_ITEM(Data1[i/100].gpsRate)
          WRITE_CSV_ITEM(Data1[i/100].radioRate)
          WRITE_CSV_ITEM(Data1[i/100].readRate)
          WRITE_CSV_ITEM(Data1[i/100].bmpRate)
          WRITE_CSV_ITEM(Data1[i/100].bnoRate)
          WRITE_CSV_ITEM(Data1[i/100].sdRate)
          WRITE_CSV_ITEM(Data1[i/100].fallRate)
          WRITE_CSV_ITEM(Data1[i/100].oriRate)
          
          WRITE_CSV_ITEM(Data1[i/100].gpsT)
          WRITE_CSV_ITEM(Data1[i/100].radioT)
          WRITE_CSV_ITEM(Data1[i/100].readT)
          WRITE_CSV_ITEM(Data1[i/100].bmpT)
          WRITE_CSV_ITEM(Data1[i/100].bnoT)
          WRITE_CSV_ITEM(Data1[i/100].sdT)
          WRITE_CSV_ITEM(Data1[i/100].fallT)
          WRITE_CSV_ITEM(Data1[i/100].oriT) 
        }
        
        //constants
        WRITE_CSV_ITEM(ATST)
        WRITE_CSV_ITEM(Launch_ALT)
        WRITE_CSV_ITEM(SEALEVELPRESSURE_HPA)
        WRITE_CSV_ITEM(launch_lat)
        WRITE_CSV_ITEM(launch_lon)
        
        dataFile.println();   //go to the next line
      }
      
      dataFile.flush();     //ensures data is written to sd card (might need to put inside the for loop)
      
      sdCount++;
      if(dataIndex%100 == 0){   //mod 100 -> 1Hz
        Data1[dataIndex/100].sdRate= sdCount / ( (millis() - startTime) / 1000.0);
        Data1[dataIndex/100].sdT= ( thisLoopMicros - micros() ) / 1000.0;
      }    
    }
    else if( (dataIndex == (Num_Datapoints-1)) && (DataWrittenOnLanding == 0) ){
      DataWrittenOnLanding= 1;    //can only do this once upon landing
      //could just log up to dataIndex and be done or could wait until dataIndex is at its max value

      for(int i = 0; i < Num_Datapoints; i++){
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
        WRITE_CSV_ITEM(Data100[i].yaw)
        WRITE_CSV_ITEM(Data100[i].pitch)
        WRITE_CSV_ITEM(Data100[i].roll)
        WRITE_CSV_ITEM(Data100[i].Px)
        WRITE_CSV_ITEM(Data100[i].Py)
        WRITE_CSV_ITEM(Data100[i].Pz)
        WRITE_CSV_ITEM(Data100[i].Vx)
        WRITE_CSV_ITEM(Data100[i].Vy)
        WRITE_CSV_ITEM(Data100[i].Vz)
        WRITE_CSV_ITEM(Data100[i].oX)
        WRITE_CSV_ITEM(Data100[i].oY)
        WRITE_CSV_ITEM(Data100[i].oZ)
        WRITE_CSV_ITEM(Data100[i].bmp_alt)

        WRITE_CSV_ITEM(Data100[i].bno_alt)
        //WRITE_CSV_VECTOR_ITEM(gyroscope)
        WRITE_CSV_ITEM(Data100[i].Acc.x())  
        WRITE_CSV_ITEM(Data100[i].Acc.y())
        WRITE_CSV_ITEM(Data100[i].Acc.z())
        WRITE_CSV_ITEM(Data100[i].gyroscope.x())
        WRITE_CSV_ITEM(Data100[i].gyroscope.y())
        WRITE_CSV_ITEM(Data100[i].gyroscope.z())
        //WRITE_CSV_VECTOR_ITEM(Acc)
        WRITE_CSV_ITEM(Data100[i].bmp_pressure)
        WRITE_CSV_ITEM(Data100[i].q1.w())
        WRITE_CSV_ITEM(Data100[i].q1.x())
        WRITE_CSV_ITEM(Data100[i].q1.y())
        WRITE_CSV_ITEM(Data100[i].q1.z())
        WRITE_CSV_VECTOR_ITEM(Data100[i].euler)
        WRITE_CSV_ITEM(Data100[i].state)
        WRITE_CSV_ITEM(Data100[i].Apogee_Passed)
        WRITE_CSV_ITEM(Data100[i].dtGyro)

        if((i%4) == 0){         //25Hz
          WRITE_CSV_ITEM(Data25[i/4].gps_alt)
          WRITE_CSV_ITEM(Data25[i/4].SIV)
          WRITE_CSV_ITEM(Data25[i/4].PDOP)
          //WRITE_CSV_ITEM(gps_lat) //change to have more precision
          dataFile.print(F(", ")); dataFile.print(Data25[i/4].gps_lat,7);
          //WRITE_CSV_ITEM(gps_lon) //change to have more precision
          dataFile.print(F(", ")); dataFile.print(Data25[i/4].gps_lon,7);
          /*
          WRITE_CSV_ITEM(Data25[i/4].gps_vel)
          WRITE_CSV_ITEM(Data25[i/4].gps_dir)
          */
          WRITE_CSV_ITEM(Data25[i/4].x_from_launch)
          WRITE_CSV_ITEM(Data25[i/4].y_from_launch)
          WRITE_CSV_ITEM(Data25[i/4].dir_from_launch)
          WRITE_CSV_ITEM(Data25[i/4].xy_to_land)
          WRITE_CSV_ITEM(Data25[i/4].xy_dir_to_land)
          WRITE_CSV_ITEM(Data25[i/4].x_to_land)
          WRITE_CSV_ITEM(Data25[i/4].y_to_land)
        }

        if((i%10) == 0){       //10Hz
          WRITE_CSV_VECTOR_ITEM(Data10[i/10].magnetometer)
        }

        if((i%25) == 0){       //4Hz
          WRITE_CSV_ITEM(Data4[i/25].link2ground)
        }

        if((i%100) == 0){       //1Hz
          WRITE_CSV_ITEM(Data1[i/100].vbatt)
          WRITE_CSV_ITEM(Data1[i/100].vgrid)
          WRITE_CSV_ITEM(Data1[i/100].bmp_temp)
          WRITE_CSV_ITEM(Data1[i/100].temp)       //bno temp

          WRITE_CSV_ITEM(Data1[i/100].gpsRate)
          WRITE_CSV_ITEM(Data1[i/100].radioRate)
          WRITE_CSV_ITEM(Data1[i/100].readRate)
          WRITE_CSV_ITEM(Data1[i/100].bmpRate)
          WRITE_CSV_ITEM(Data1[i/100].bnoRate)
          WRITE_CSV_ITEM(Data1[i/100].sdRate)
          WRITE_CSV_ITEM(Data1[i/100].fallRate)
          WRITE_CSV_ITEM(Data1[i/100].oriRate)
          
          WRITE_CSV_ITEM(Data1[i/100].gpsT)
          WRITE_CSV_ITEM(Data1[i/100].radioT)
          WRITE_CSV_ITEM(Data1[i/100].readT)
          WRITE_CSV_ITEM(Data1[i/100].bmpT)
          WRITE_CSV_ITEM(Data1[i/100].bnoT)
          WRITE_CSV_ITEM(Data1[i/100].sdT)
          WRITE_CSV_ITEM(Data1[i/100].fallT)
          WRITE_CSV_ITEM(Data1[i/100].oriT) 
        }
        
        //constants
        WRITE_CSV_ITEM(ATST)
        WRITE_CSV_ITEM(Launch_ALT)
        WRITE_CSV_ITEM(SEALEVELPRESSURE_HPA)
        WRITE_CSV_ITEM(launch_lat)
        WRITE_CSV_ITEM(launch_lon)
        
        dataFile.println();   //go to the next line
      }
      
      dataFile.flush();     //ensures data is written to sd card (might need to put inside the for loop)
      
      SD.close();
    }
    //*****END SD Logging Section**************************************************************************
      
    Timer100Hz= millis();
  }

  loopIteration++;
  
}
