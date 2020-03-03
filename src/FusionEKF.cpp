#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF()
{
    is_initialized_ = false;

    previous_timestamp_ = 0;

    // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    Hj_ = MatrixXd(3, 4);
    x_ = VectorXd(4);
    F_ = MatrixXd(4, 4);
    Q_ = MatrixXd(4, 4);
    P_ = MatrixXd(4, 4);

    // measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
                0, 0.0225;

    // measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
                0, 0.0009, 0,
                0, 0, 0.09;

    // Laser Measurement to State
    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;

    // Initialise x vector
    x_ << 1, 1, 1, 1;

    // Initialise the dynamics matrix with a zero time delta to be updated after first iteration
    F_ << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1;

    // Process covariance matrix requires time delta to infer acceleration from pose, so initialise with zeros
    Q_ << 0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0;

    // Initialise state uncertainty covariance matrix
    P_ << 0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0;

    // Initilise the kalman filter class
    ekf_.Init(x_, P_, F_, H_laser_, R_laser_, Q_);
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF()
{
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage& measurement_pack)
{
    /**
     * Initialization
     */
    if (!is_initialized_)
    {
        // first measurement
        cout << "EKF: " << endl;

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
        {
            // Extract components from raw data
            float rho = measurement_pack.raw_measurements_[0];
            float phi = measurement_pack.raw_measurements_[1];
            float rho_dot = measurement_pack.raw_measurements_[2];

            // Convert to cartesian
            float px = cos(phi) * rho;
            float py = sin(phi) * rho;
            float vx = cos(phi) * rho_dot;
            float vy = sin(phi) * rho_dot;

            ekf_.x_ << px, py, vx, vy;

            // As radar also measures velocity, pose and velocity equal initial covariance
            ekf_.P_ << 1, 0, 0, 0,
                       0, 1, 0, 0,
                       0, 0, 1000, 0,
                       0, 0, 0, 1000;

        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
        {
            float px = measurement_pack.raw_measurements_[0];
            float py = measurement_pack.raw_measurements_[1];

            // Radar directly measures pose but velocity in inferred therefore uncertain in first measurment.
            ekf_.P_ <<  1, 0, 0, 0,
                        0, 1, 0, 0,
                        0, 0, 1000, 0,
                        0, 0, 0, 1000;

            ekf_.x_ << px, py, 0, 0;
        }

        // Set first timestamp
        previous_timestamp_ = measurement_pack.timestamp_;

        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }

    /**
     * Prediction
     */

    float delta_t = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;

    // Add time to state transition matrix
    ekf_.F_(0, 2) = delta_t;
    ekf_.F_(1, 3) = delta_t;

    // Calculate our process noise covariance matrix

    // Noise components
    float dt_2 = delta_t * delta_t;
    float dt_3 = dt_2 * delta_t;
    float dt_4 = dt_3 * delta_t;

    ekf_.Q_ << dt_4/4*noise_ax_, 0, dt_3/2*noise_ax_, 0,
        0, dt_4/4*noise_ay_, 0, dt_3/2*noise_ay_,
        dt_3/2*noise_ax_, 0, dt_2*noise_ax_, 0,
        0, dt_3/2*noise_ay_, 0,  dt_2*noise_ay_;

    ekf_.Predict();

    /**
     * Update
     */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
        ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
        ekf_.R_ = R_radar_;

        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    }
    else
    {
        ekf_.H_ = H_laser_;
        ekf_.R_ = R_laser_;

        ekf_.Update(measurement_pack.raw_measurements_);
    }

    // print the output
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}
