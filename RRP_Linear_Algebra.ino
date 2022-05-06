#include <BasicLinearAlgebra.h> // library for matrices by Tom Steward
using namespace BLA;// All the functions in BasicLinearAlgebra are wrapped up inside the namespace BLA

// Spherical Robotic Arm (RRL)

// Insert the values
// a(x) = links: a1, a2, a3
float a1 = 4.2876; // Insert (a1), Length a1, inches
float a2 = 1.1022; // Insert (a2), Length a2, inches
float a3 = 4.1483; // Insert (a3), Length a3, inches
// T(x) = angles: 0-180 degrees
float T1 = 180*(PI/180); //  Angle T1, degree
float T2 = 180*(PI/180); // Angle T2, degree
float T3 = 180*(PI/180); // Angle T3, degree
// d(x) = Distance: Extending Arm
float d1 = 1.2319; // Distance: Extending Arm, inches

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(9600); // Read from Arduino
  // Each Homogeneous Transformation
  // The cols parameters has a default of 1 so to declare a vector of length 3 you can simply write:
  BLA::Matrix<4,4> H01; // H01
  BLA::Matrix<4,4> H12; // H12
  BLA::Matrix<4,4> H02; // H03
  BLA::Matrix<4,4> H23;// H23
  BLA::Matrix<4,4> H03; // H03

  // Matrices should be initialised to some value before you use them.
  H01.Fill(0);
  H12.Fill(0);
  H23.Fill(0);

  // To recieve the end effector, H03 must be completed: H03 = H01*H12*H23 in this order
  
  // H01 Matrix
  H01 << cos(T1), 0, sin(T1), 0,
       sin(T1), 0 , -1*cos(T1), 0,
       0, 1, 0, a1,
       0, 0, 0 ,1;
       // Print this Matrix
  Serial << "H01= " << H01 << '\n';   

  // H12 Matrix
  H12 << -1*sin(T2), 0, cos(T2), 0,
       cos(T2), 0, sin(T2), 0,
       0, 1, 0, 0,
       0, 0, 0, 1;
       // Print this Matrix
  Serial << "H12= " << H12 << '\n'; 

    // H02 = H01*H12
  Multiply(H01,H12,H02);
  // Print this Matrix
  Serial << "H02= " << H02 << '\n';

  // H23 Matrix
  H23 << 1, 0, 0, (a2+a2+d1)*cos(T2),
        0,1, 0, (a2+a2+d1)*sin(T2),
        0, 0 ,0 ,0,
        0, 0, 0, 1;
        // Print this Matrix
  Serial << "H23= " << H23 << '\n'; 

  // H03 = H02*H23
 
  Multiply(H02,H23,H03);
  Serial << "H03= " << H03 << '\n';

}

void loop() {
  // put your main code here, to run repeatedly:

}
  
