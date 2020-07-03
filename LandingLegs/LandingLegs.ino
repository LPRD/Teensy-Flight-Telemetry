#include <PWMServo.h>

  #define LED 13

  #define PWM1 2          //S1
  #define PWM2 3          //S2
  
  #define PWM6 7          //LL1
  #define PWM7 8          //LL2
  #define PWM8 23 //A9    //LL3
  #define PWM9 22 //A8    //LL4
  #define PWM10 21 //A7   //EDF
  #define PWM11 20 //A6     //Free
  #define PWM12 17 //17 //A3   //S1
  #define PWM13 16 //16 //A2   //S2
  #define PWM14 36 //A17  //S3    //working
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
  int pos1 = 90;


void setup() {

  S1.attach(PWM1);  //17
  delay(100);
  //S1.attach(SERVO_PIN_A, 1000, 2000); //some motors need min/max setting ,ESCs go 1k-2k
  S2.attach(PWM2);  //16
  delay(100);
  S3.attach(PWM14);  //36
  delay(100);
  S4.attach(PWM15);  //35
  delay(100);

  
  LL2.attach(PWM7);  //8
  delay(100);
  LL3.attach(PWM8);  //23
  delay(100);
  LL4.attach(PWM9);  //22
  delay(100);
  //EDF.attach(PWM10); //21
  EDF.attach(PWM11, 1000, 2000);
  delay(100);
  
  LL1.attach(PWM6);  //7
  delay(100);

  

  for(int q=0; q<10;q++){
    digitalWrite(LED,LOW); delay(100); digitalWrite(LED,HIGH);delay(100); 
  }
  
  delay(1000*3);       //changed from smartDelay
  
  for(int q=0; q<20;q++){
    digitalWrite(LED,LOW); delay(50); digitalWrite(LED,HIGH);delay(50); 
  }



/*
  //Lower Landing Legs
  LL1.write(0);   //pos1, 0 or 180, etc
  delay(100);
  LL2.write(0);
  delay(100);
  LL3.write(0);
  delay(100);
  LL4.write(0);
  delay(1000);
  
  LL1.write(180);
  delay(100);
  LL2.write(180);
  delay(100);
  LL3.write(180);
  delay(100);
  LL4.write(180);
  delay(1000);

*/
  
  for(pos = 0; pos <=180; pos += 1){
      //S1.analogWrite(pos);
      //analogWrite(17,pos);
      S1.write(pos);
      
      //delay(30);
      //S2.analogWrite(pos);
      //analogWrite(16,pos);
      S2.write(pos);
      
      //delay(30);
      S3.write(pos);
      //delay(30);
      S4.write(pos);
      delay(20);

  }

delay(2500);

  for(pos = 180; pos >=1; pos -= 5){
      //S1.analogWrite(pos);
      //analogWrite(17,pos);
      S1.write(pos);
      
      //delay(100);
      //S2.analogWrite(pos);
      //analogWrite(16,pos);
      S2.write(pos);
      
      //delay(100);
      S3.write(pos);
      //delay(100);
      S4.write(pos);
      delay(70);

  }



  //landing legs
  /*  
    //Pull LLs up
  for(pos = 180; pos >=1; pos -= 10){ 
      LL1.write(pos);
      //delay(30);
      LL2.write(pos);
      //delay(30);
      LL3.write(pos);
      //delay(30);
      LL4.write(pos);
      delay(200);
  }
  */
  delay(2500);

    //Bring LLs down
  for(pos = 0; pos <=180; pos += 18){
      //if(pos > 170){
        LL1.write(pos);
      //}
      //delay(30);
      LL2.write(pos);
      //delay(30);
      LL3.write(pos);
      //delay(30);
      LL4.write(pos);
      delay(400);
  }


}

int POS= 180;





void loop() {
  
  delay(4000);

    //Keep LLs down, this code doesn't appear to make things any better
  for(pos = 170; pos <=180; pos += 5){
      LL1.write(pos);
      LL2.write(pos);
      LL3.write(pos);
      LL4.write(pos);
      delay(100);
  }
  
  
  
  
  
  
  
  
  
  
  
  
  
  /*

 
  delay(100);
  while(LL1.read() !=180){
    LL1.write(POS+1);
  }
  while(LL2.read() !=180){
    LL2.write(POS+1);
  }
  while(LL3.read() !=180){
    LL3.write(POS+5);
  }
  while(LL4.read() !=180){
    LL4.write(POS+0);
  }
 */

/*
  for(pos = 180; pos < 180; pos += 18){
      if(pos > 170){
        LL1.write(pos);
      }
      //delay(30);
      LL2.write(pos);
      //delay(30);
      LL3.write(pos);
      //delay(30);
      LL4.write(pos);
      delay(100);
  }
*/


 // delay(1000);




}
