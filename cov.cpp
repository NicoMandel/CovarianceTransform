// #include <eigen3/Eigen/Dense>
#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;

void compiletime(){
    Matrix3d m = Matrix3d::Random();
    m = (m + Matrix3d::Constant(1.2)) * 50;
    std::cout<< m << std::endl;
    Vector3d v(1,2,3);
    std::cout<< "m * v" << std::endl << m * v << std::endl;

}

void runtime(){
    MatrixXd m = MatrixXd::Random(3,3);
    m = (m + MatrixXd::Constant(3,3, 1.2)) * 50;
    VectorXd v(3);
    v << 1,2,3;
    std::cout<< "m * v" << std::endl << m * v << std::endl;
}


int main(int argc, char** argv){

    MatrixXd m(2,2);
    m(0,0) = 3;
    m(1,0) = 2.5;
    m(0,1) = -1;
    m(1,1) = m(1,0) + m(0,1);
    std::cout << m << std::endl;

    compiletime();
    runtime();

    return 0;
}