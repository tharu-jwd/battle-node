#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>
#include "common/Types.h"

namespace SensorFusion
{

    class KalmanFilter
    {
    public:
        KalmanFilter();

        void initialize(const Eigen::VectorXd &initialState, const Eigen::MatrixXd &initialCovariance);

        void predict(double dt);

        void update(const Eigen::VectorXd &measurement,
                    const Eigen::MatrixXd &measurementCovariance,
                    bool hasVelocity);

        Eigen::VectorXd getState() const { return state_; }
        Eigen::MatrixXd getCovariance() const { return covariance_; }

        Position3D getPosition() const;
        Velocity3D getVelocity() const;

        void setProcessNoise(double posNoise, double velNoise);

    private:
        Eigen::VectorXd state_;
        Eigen::MatrixXd covariance_;

        Eigen::MatrixXd F_;
        Eigen::MatrixXd Q_;
        Eigen::MatrixXd H_pos_;
        Eigen::MatrixXd H_full_;

        bool initialized_;

        void buildTransitionMatrix(double dt);
        void buildProcessNoise(double dt);
    };

} // namespace SensorFusion

#endif // KALMAN_FILTER_H
