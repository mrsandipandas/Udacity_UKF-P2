#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict() {
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    VectorXd y = z - H_ * x_;
    MatrixXd Ht = H_.transpose();
    MatrixXd PHt = P_ * Ht;
    MatrixXd S = H_ * PHt + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K =  PHt * Si;
    
    //new state
    x_ = x_ + (K * y);
    long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    double px = x_(0);
    double py = x_(1);
    double vx = x_(2);
    double vy = x_(3);
    
    // Equations for Hj below
    const double eps = .00001;
    double rho = sqrt(px*px + py*py);
    rho = std::max(eps, rho);
    double theta = atan2(py, px);
    double rho_dot = (px*vx + py*vy) / rho;
    VectorXd Hj = VectorXd(3);
    Hj << rho, theta, rho_dot;
    
    VectorXd y = z - Hj;
    NormalizeAngle(y(1));
    MatrixXd Ht = H_.transpose();
    MatrixXd PHt = P_ * Ht;
    MatrixXd S = H_ * PHt + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K =  PHt * Si;
    
    //new state
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::NormalizeAngle(double &theta) {
    while(theta > M_PI){
        theta -= 2 * M_PI;
    }

    while(theta < -M_PI){
        theta += 2 * M_PI;
    }
}