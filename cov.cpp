// #include <eigen3/Eigen/Dense>
#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;

void jacobian(MatrixXd& , Quaterniond);

void convert(Matrix3d& eul_cov, Quaterniond quat, const Matrix4d quatcov){

    MatrixXd jac = MatrixXd::Zero(3,4);
    jacobian(jac, quat);

    eul_cov = jac * quatcov * jac.transpose();
}

void jacobian(MatrixXd& jac, Quaterniond quat){
    // preliminary, temporary values
    double q32n = quat.y() - quat.x();
    double q32p = quat.y() + quat.x();
    double q41p = quat.z() + quat.w();
    double q41n = quat.z() - quat.x();

    double normalizerp = q32p*q32p + q41p*q41p;
    double normalizern = q32n*q32n + q41n*q41n;

    double q41pos = q41p / normalizerp;
    double q32pos = q32p / normalizerp;
    double q41neg = q41n / normalizern;
    double q32neg = q32n / normalizern;

    double theta_norm = 2 / sqrt(1.f - 4.f * (quat.x() * quat.y() + quat.z() * quat.w()) * (quat.x() * quat.y() + quat.z() * quat.w())); 
    // Matrix values - simplified
    double psi1 = q32pos * -1.f + q32neg;
    double psi2 = q41pos - q41neg;
    double psi3 = q41pos + q41neg;
    double psi4 = q32pos * -1.f - q32neg;

    // filling in the matrix - row 1
    jac(0,0) = psi1;
    jac(0,1) = psi2;
    jac(0,2) = psi3;
    jac(0,3) = psi4;

    // Row 3
    jac(2,0) = psi4;
    jac(2,1) = psi3;
    jac(2,2) = psi2;
    jac(2,3) = psi1;

    // Row 2
    jac(1,0) = theta_norm * quat.z();
    jac(1,1) = theta_norm * quat.y();
    jac(1,2) = theta_norm * quat.x();
    jac(1,3) = theta_norm * quat.w();
        
}

int main(int argc, char** argv){

    Matrix4d quatcov;
    Quaterniond quat = Quaterniond::UnitRandom(); //initialization order is important, see doc. w, x, y, z. Internal storage is xyz, w
    // TODO: fill in values

    // Get the converted data back
    Matrix3d eul_cov;
    convert(eul_cov, quat, quatcov);
    
    std::cout<<eul_cov<<std::endl;    

    return 0;
}