#include "kalman_filter.h"
#include "tools.h"
#include <iostream>
using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  //cout << "Kalman Predict Function"<<endl;
  	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
	//cout << "Kalman Predict Function End"<<endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  //cout << "Kalman Update Laser Function"<<endl;
  	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
	//cout << "Kalman Update Laser Function End"<<endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  //cout << "Kalman Update Radar Function"<<endl;
  VectorXd jac_z = tool.polar_to_cartesian(z);
  //cout << "Kalman Update Radar Function1"<<endl;
  MatrixXd Hj = tool.CalculateJacobian(jac_z);
  //cout << "Kalman Update Radar Function2"<<endl;
   VectorXd x_1 = tool.cartesian_to_polar(x_);
   //cout << "Kalman Update Radar Function3"<<endl;
  //VectorXd z_pred = H_ * x_1;
	VectorXd y = z - x_1;
	//cout << "Kalman Update Radar Function4"<<endl;
	MatrixXd Ht = Hj.transpose();
	//cout << "Kalman Update Radar Function5"<<endl;
	//cout << Hj.size() << " "<< P_.size()<< " "<<Ht.size()<< " "<< R_.size()<<endl;
	MatrixXd S = Hj * P_ * Ht + R_;
	//cout << "Kalman Update Radar Function6"<<endl;
	MatrixXd Si = S.inverse();
	//cout << "Kalman Update Radar Function7"<<endl;
	MatrixXd PHt = P_ * Ht;
	//cout << "Kalman Update Radar Function8"<<endl;
	MatrixXd K = PHt * Si;
	//cout << "Kalman Update Radar Function9"<<endl;

	//Normalise
	y(1) = atan2(sin(y(1)),cos( y(1)));
	//cout << "Kalman Update Radar Function10"<<endl;
	//new estimate
	x_ = x_ + (K * y);
	//cout << "Kalman Update Radar Function11"<<endl;
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	//cout << "Kalman Update Radar Function12"<<endl;
	P_ = (I - K * Hj) * P_;
	//cout << "Kalman Update Radar Function End"<<endl;

}

