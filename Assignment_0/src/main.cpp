#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/GlobalFunctions.h>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include<iostream>
#define PI 3.1415926
int main(){

    // Basic Example of cpp
    std::cout << "Example of cpp \n";
    float a = 1.0, b = 2.0;
    std::cout << a << std::endl;
    std::cout << a/b << std::endl;
    std::cout << std::sqrt(b) << std::endl;
    std::cout << std::acos(-1) << std::endl;
    std::cout << std::sin(30.0/180.0*acos(-1)) << std::endl;

    // Example of vector
    std::cout << "Example of vector \n";
    // vector definition
    Eigen::Vector3f v(1.0f,2.0f,3.0f);
    Eigen::Vector3f w(1.0f,0.0f,0.0f);
    
    // vector output
    std::cout << "Example of output \n";
    std::cout << v << std::endl;
    // vector add
    std::cout << "Example of add \n";
    std::cout << v + w << std::endl;
    // vector scalar multiply
    std::cout << "Example of scalar multiply \n";
    std::cout << v * 3.0f << std::endl;
    std::cout << 2.0f * v << std::endl;
    // vector dot product
    std::cout << "Example of dot product \n";
    std::cout << v * w.transpose() << std::endl;
    std::cout << v.transpose() * w << std::endl;

    // vector cross product
    std::cout << "Example of cross product \n";
    std::cout << w.cross(v) << std::endl;
    // Example of matrix
    std::cout << "Example of matrix \n";
    // matrix definition
    Eigen::Matrix3f i,j;
    i << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
    j << 2.0, 3.0, 1.0, 4.0, 6.0, 5.0, 9.0, 7.0, 8.0;
    // matrix output
    std::cout << "Example of output \n";
    std::cout << i << std::endl;
    std::cout << "\n";
    std::cout << j << std::endl;
    std::cout << "Example of matrix add \n";
    std::cout << i+j << std::endl;
    std::cout << "Example of matrix scalar multiply \n";
    std::cout << i*2.0 << std::endl;
    std::cout << "Example of matrix multiply \n";
    std::cout << i*j << std::endl;
    std::cout << "Example of matrix multiply vector \n";
    std::cout << i*v << std::endl;
    /* 
    * PA 0
    */
    Eigen::Vector3f p;
    p << 2,1,1;
    std::cout << "P:" << std::endl;
    std::cout << p << std::endl;
    Eigen::Matrix3f M;
    double theta = 45*1.0/180*PI;
    M << cos(theta), -1.0*sin(theta), 1,
        sin(theta), cos(theta), 2,
        0,0,1;
    
    std::cout << "M:" << std::endl;
    std::cout << M << std::endl;
    std::cout << "M*p:" << std::endl;
    std::cout << M*p << std::endl;
    return 0;
}