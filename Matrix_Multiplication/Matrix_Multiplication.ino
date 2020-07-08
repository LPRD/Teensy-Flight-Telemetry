#include <Matrix.h>
#include <Wire.h>
#include <SPI.h>
#include <avr/pgmspace.h>


int c;

void setup() {

  Matrix<int> A(5,2,5);
  Matrix<int> B(2,5,4);

  Matrix<int> C;
  C= A*B;

  c= C._entity[0][0];
  Serial.print( c );
  C.show();
  
}




void loop() {
  
}




/*

void MatrixMult( const int [][ 3 ] , const int [][ 3 ] );


void setup() {
  // put your setup code here, to run once:



int X= 4;
int Y= 4;


int A[][4] = { {2     2     1     3},
               {3     3     2     1},
               {3     3     4     4},
               {4     1     2     2} };

int B[][4] = { {3     3     4     4},
               {3     1     2     2},
               {4     1     4     1},
               {4     2     1     2} };

int Row= 1;
int Col= 1;


int Matrix[Y][X];
for (int i =0; i < Y; i++) {
  for (int j =0; j < X; j++) {
    Matrix[i][j] = A[i][j]*B[i][j] + A[i][j]*B[i][j] +;

    Serial.print(Matrix[i][j]);

    
  }
}




}

void loop() {
  
}


*/
