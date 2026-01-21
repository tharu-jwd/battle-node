#include "fusion/KalmanFilter.h"

namespace SensorFusion
{

    KalmanFilter::KalmanFilter() : initialized_(false)
    {
        state_ = Eigen::VectorXd::Zero(6);
        covariance_ = Eigen::MatrixXd::Identity(6, 6) * 100.0;

        F_ = Eigen::MatrixXd::Identity(6, 6);
        Q_ = Eigen::MatrixXd::Identity(6, 6);

        H_pos_ = Eigen::MatrixXd::Zero(3, 6);
        H_pos_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

        H_full_ = Eigen::MatrixXd::Identity(6, 6);

        setProcessNoise(0.5, 0.5);
    }

    void KalmanFilter::initialize(const Eigen::VectorXd &initialState,
                                  const Eigen::MatrixXd &initialCovariance)
    {
        state_ = initialState;
        covariance_ = initialCovariance;
        initialized_ = true;
    }

    void KalmanFilter::predict(double dt)
    {
        buildTransitionMatrix(dt);
        buildProcessNoise(dt);

        state_ = F_ * state_;
        covariance_ = F_ * covariance_ * F_.transpose() + Q_;
    }

    void KalmanFilter::update(const Eigen::VectorXd &measurement,
                              const Eigen::MatrixXd &measurementCovariance,
                              bool hasVelocity)
    {
        Eigen::MatrixXd H;
        Eigen::VectorXd z;
        Eigen::MatrixXd R;

        if (hasVelocity)
        {
            H = H_full_;
            z = measurement;
            R = measurementCovariance;
        }
        else
        {
            H = H_pos_;
            z = measurement.head(3);
            R = measurementCovariance.block<3, 3>(0, 0);
        }

        Eigen::VectorXd y = z - H * state_;
        Eigen::MatrixXd S = H * covariance_ * H.transpose() + R;
        Eigen::MatrixXd K = covariance_ * H.transpose() * S.inverse();

        state_ = state_ + K * y;

        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
        covariance_ = (I - K * H) * covariance_;
    }

    Position3D KalmanFilter::getPosition() const
    {
        return Position3D(state_(0), state_(1), state_(2));
    }

    Velocity3D KalmanFilter::getVelocity() const
    {
        return Velocity3D(state_(3), state_(4), state_(5));
    }

    void KalmanFilter::setProcessNoise(double posNoise, double velNoise)
    {
        Q_.setZero();
        Q_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * (posNoise * posNoise);
        Q_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * (velNoise * velNoise);
    }

    void KalmanFilter::buildTransitionMatrix(double dt)
    {
        F_.setIdentity();
        F_.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
    }

    void KalmanFilter::buildProcessNoise(double dt)
    {
        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        double dt4 = dt3 * dt;

        double q_pos = 0.5;
        double q_vel = 0.5;

        Q_.setZero();
        Q_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * (dt4 / 4.0) * q_pos;
        Q_.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * (dt3 / 2.0) * q_pos;
        Q_.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity() * (dt3 / 2.0) * q_pos;
        Q_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * dt2 * q_vel;
    }

} // namespace SensorFusion
