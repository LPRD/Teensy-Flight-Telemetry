#include <Matrix.h>

float array1[4][4] = {{16,19,2,7},{5,34,77,1},{23,37,38,7},{9,33,10,0}};
float array2[4][1] = {{5},{6},{7},{8}};
//create float type matrices
Matrix<float> A(4,4,(float*)array1);
Matrix<float> B(4,1,(float*)array2);


//Matrix<float> Q= {{1},{1}}; 

void setup() {
    Serial.begin(115200);

    Matrix<float> C = A*B;
    Matrix<float> D(4,4,'I');  //create identity matrix
    Serial.println("C: ");
    C.show();
    Serial.println("A transpose: ");
    Matrix<float>::transpose(A).show(4);
    Serial.println("A inverse: ");
    Matrix<float>::inv(A).show(4);
    Serial.println("D: ");
    D.show();

    Serial.println("New A: ");
    A= A*B;
    A.show();

    Serial.println("array1: ");
    float a= array1[0][0];
    float b= array1[1][0];
    float c= array1[2][0];
    float d= array1[3][0];
    
    Serial.println(a);
    Serial.println("\n");
    Serial.println(b);
    Serial.println("\n");
    Serial.println(c);
    Serial.println("\n");
    Serial.println(d);
    Serial.println("\n");

    //the original array 1 DOES NOT change based on if Matrix A changes
    
    Matrix<float> Q= A;   //works

    //Matrix<float> Q= {{1},{1}};   
    //compiles but doesn't work 
    
    //can't use .show fcn outside of setup or loop
    Q.show();
    //Matrix<float> Q[0][0]= 69.420;  //does not work!

    Q._entity[0][0]= 69.420;    //Changing individual values

    Q.show();
}

void loop() {

}
