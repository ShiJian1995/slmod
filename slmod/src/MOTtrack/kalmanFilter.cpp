#include "kalmanFilter.h"
#include <Eigen/Cholesky>

namespace byte_kalman
{
	const double KalmanFilter::chi2inv95[10] = {
	0,
	3.8415,
	5.9915,
	7.8147,
	9.4877,
	11.070,
	12.592,
	14.067,
	15.507,
	16.919
	};
	KalmanFilter::KalmanFilter()
	{
		int ndim = 4;
		

		_motion_mat = Eigen::MatrixXf::Identity(8, 8);

		// for (int i = 0; i < ndim; i++) {
		// 	_motion_mat(i, ndim + i) = dt;
		// }

		_control_mat = Eigen::MatrixXf::Zero(8, 2);
		_update_mat = Eigen::MatrixXf::Identity(4, 8);

		this->_std_weight_position = 1. / 20;
		this->_std_weight_velocity = 1. / 160;
	}

	KAL_DATA KalmanFilter::initiate(const DETECTBOX &measurement)
	{
		DETECTBOX mean_pos = measurement;
		DETECTBOX mean_vel;
		for (int i = 0; i < 4; i++) mean_vel(i) = 0;

		KAL_MEAN mean;
		for (int i = 0; i < 8; i++) {
			if (i < 4) mean(i) = mean_pos(i);
			else mean(i) = mean_vel(i - 4);
		}

		KAL_MEAN std;
		std(0) = 2 * _std_weight_position * measurement[3];
		std(1) = 2 * _std_weight_position * measurement[3];
		std(2) = 1e-2;
		std(3) = 2 * _std_weight_position * measurement[3];
		// std(4) = 10 * _std_weight_velocity * measurement[3];
		// std(5) = 10 * _std_weight_velocity * measurement[3];
		std(4) = 1e-5;
		std(5) = 10 * _std_weight_velocity * measurement[3];
		std(6) = 1e-5;
		std(7) = 10 * _std_weight_velocity * measurement[3];

		KAL_MEAN tmp = std.array().square();
		KAL_COVA var = tmp.asDiagonal();
		return std::make_pair(mean, var);
	}

	void KalmanFilter::predict(KAL_MEAN &mean, KAL_COVA &covariance, Eigen::Vector2f& flow, double dt)
	{
		//revise the data;
		// 使用高度？？？

		// DETECTBOX std_pos;
		// std_pos << _std_weight_position * mean(3),
		// 	_std_weight_position * mean(3),
		// 	1e-2,
		// 	_std_weight_position * mean(3);

		// Eigen::Matrix<float, 1, 2> std_vel;
		// std_vel << _std_weight_velocity * mean(3),
		// 	_std_weight_velocity * mean(3);
		// 	// 1e-5,
		// 	// _std_weight_velocity * mean(3);


		DETECTBOX std_pos;
		std_pos << _std_weight_position,
			_std_weight_position,
			_std_weight_position,
			_std_weight_position;

		Eigen::Matrix<float, 1, 4> std_vel;
		std_vel << _std_weight_velocity,
					_std_weight_velocity,
					_std_weight_velocity,
					_std_weight_velocity;

		// if(last_update_stamp < 0){
		// 	dt = 1.0 / 15.0;
		// }
		// else{

		// 	dt = frame_stamp - last_update_stamp;
		// }
		// std::cout << "dt is " << dt << std::endl;
		// std::cout << std::setprecision(15) << "frame_stamp is " << frame_stamp << std::endl;
		// std::cout << std::setprecision(15) << "frame_stamp is " << last_update_stamp << std::endl;
		// std::cout << "mean is " << mean.transpose() << std::endl;
		// last_update_stamp = frame_stamp;
		// std::cout << "flow is " << flow.transpose() << std::endl;
		// std::cout << "vel is " << Eigen::Vector2d(mean[4], mean[5]).transpose() * dt << std::endl;
		// if(flow.norm() > 1.5 * Eigen::Vector2d(mean[4], mean[5]).norm() * dt){

		// 	alpha = 0.1;
		// }
		// else{
		// 	alpha = 0.8;
		// }
		if(flow.norm() == 0){
			alpha = 1;
		}
		else{
			alpha = 0.5;
		}
		
		// std::cout << "alpha is " << alpha << std::endl;

		_motion_mat(0, 4) = alpha * dt;
		_motion_mat(1, 5) = alpha * dt;
		_motion_mat(2, 6) = dt;
		_motion_mat(3, 7) = dt;

		// std::cout << "motion mat is " << _motion_mat << std::endl;

		_control_mat(0, 0) = 1 - alpha;
		_control_mat(1, 1) = 1 - alpha;

		// std::cout << "control mat is " << _control_mat << std::endl;

		
		KAL_MEAN tmp;
		tmp.block<1, 4>(0, 0) = std_pos;
		tmp.block<1, 4>(0, 4) = std_vel;
		tmp = tmp.array().square();
		KAL_COVA motion_cov = tmp.asDiagonal();
		KAL_MEAN mean1 = this->_motion_mat * mean.transpose() + _control_mat * flow;
		// std::cout <<"mean1 is " << mean1 << std::endl;
		// KAL_MEAN mean2 = this->_motion_mat * mean.transpose();
		// std::cout <<"mean2 is " << mean2 << std::endl;
		KAL_COVA covariance1 = this->_motion_mat * covariance *(_motion_mat.transpose());
		covariance1 += motion_cov;

		mean = mean1;
		covariance = covariance1;

		// std::cout << "predict covariance " << covariance << std::endl;
	}

	KAL_HDATA KalmanFilter::project(const KAL_MEAN &mean, const KAL_COVA &covariance)
	{
		DETECTBOX std;
		std << _std_weight_position * 0.01, 
				_std_weight_position * 0.01,
				_std_weight_position * 0.01, 
				_std_weight_position * 0.01;


		// std << _std_weight_position * mean(3) * 0.01, _std_weight_position * mean(3) * 0.01,
		// 			1e-3, _std_weight_position * mean(3) * 0.01;

		KAL_HMEAN mean1 = _update_mat * mean.transpose();
		KAL_HCOVA covariance1 = _update_mat * covariance * (_update_mat.transpose());
		Eigen::Matrix<float, 4, 4> diag = std.asDiagonal();
		diag = diag.array().square().matrix();
		covariance1 += diag;
		//    covariance1.diagonal() << diag;
		return std::make_pair(mean1, covariance1);
	}

	KAL_DATA
		KalmanFilter::update(
			const KAL_MEAN &mean,
			const KAL_COVA &covariance,
			const DETECTBOX &measurement)
	{
		KAL_HDATA pa = project(mean, covariance);
		KAL_HMEAN projected_mean = pa.first;
		KAL_HCOVA projected_cov = pa.second;

		//chol_factor, lower =
		//scipy.linalg.cho_factor(projected_cov, lower=True, check_finite=False)
		//kalmain_gain =
		//scipy.linalg.cho_solve((cho_factor, lower),
		//np.dot(covariance, self._upadte_mat.T).T,
		//check_finite=False).T
		Eigen::Matrix<float, 4, 8> B = (covariance * (_update_mat.transpose())).transpose();
		Eigen::Matrix<float, 8, 4> kalman_gain = (projected_cov.llt().solve(B)).transpose(); // eg.8x4
		Eigen::Matrix<float, 1, 4> innovation = measurement - projected_mean; //eg.1x4
		auto tmp = innovation * (kalman_gain.transpose());
		KAL_MEAN new_mean = (mean.array() + tmp.array()).matrix();
		// std::cout << "meausrements is " << measurement << std::endl;
		// std::cout << "update result is " << new_mean << std::endl;
		KAL_COVA new_covariance = covariance - kalman_gain * projected_cov*(kalman_gain.transpose());
		return std::make_pair(new_mean, new_covariance);
	}

	Eigen::Matrix<float, 1, -1>
		KalmanFilter::gating_distance(
			const KAL_MEAN &mean,
			const KAL_COVA &covariance,
			const std::vector<DETECTBOX> &measurements,
			bool only_position)
	{
		KAL_HDATA pa = this->project(mean, covariance);
		if (only_position) {
			printf("not implement!");
			exit(0);
		}
		KAL_HMEAN mean1 = pa.first;
		KAL_HCOVA covariance1 = pa.second;

		//    Eigen::Matrix<float, -1, 4, Eigen::RowMajor> d(size, 4);
		DETECTBOXSS d(measurements.size(), 4);
		int pos = 0;
		for (DETECTBOX box : measurements) {
			d.row(pos++) = box - mean1;
		}
		Eigen::Matrix<float, -1, -1, Eigen::RowMajor> factor = covariance1.llt().matrixL();
		Eigen::Matrix<float, -1, -1> z = factor.triangularView<Eigen::Lower>().solve<Eigen::OnTheRight>(d).transpose();
		auto zz = ((z.array())*(z.array())).matrix();
		auto square_maha = zz.colwise().sum();
		return square_maha;
	}
}