#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

class KalmanFilter
{
  public:
    /**
     * @brief Constructor
     */
    KalmanFilter();

    /**
     * @brief Destructor
     */
    virtual ~KalmanFilter();

    /**
     * @brief Init Initializes Kalman filter
     * @param x_in Initial state
     * @param P_in Initial state covariance
     * @param F_in Transition matrix
     * @param H_in Measurement matrix
     * @param R_in Measurement covariance matrix
     * @param Q_in Process covariance matrix
     */
    void Init(Eigen::VectorXd& x_in, Eigen::MatrixXd& P_in, Eigen::MatrixXd& F_in, Eigen::MatrixXd& H_in,
              Eigen::MatrixXd& R_in, Eigen::MatrixXd& Q_in);

    /**
     * @brief Prediction Predicts the state and the state covariance
     *        using the process model
     * @param delta_T Time between k and k+1 in s
     */
    void Predict();

    /**
     * @brief Core update functionality common to EKF and Normal Update.
     * @param y The y matrix calculation based on either EKF or normal Kalman Filter.
     */
    void CoreUpdate(const Eigen::MatrixXd y);

    /**
     * @brief Updates the state by using standard Kalman Filter equations
     * @param z The measurement at k+1
     */
    void Update(const Eigen::VectorXd& z);

    /**
     * @brief Convert Radar Measurement to State Vector and return h(x)
     */

    Eigen::VectorXd RadarMeasurementToState();

    /**
     * @brief Updates the state by using Extended Kalman Filter equations
     * @param z The measurement at k+1
     */
    void UpdateEKF(const Eigen::VectorXd& z);

    // state vector
    Eigen::VectorXd x_;

    // state covariance matrix
    Eigen::MatrixXd P_;

    // state transition matrix
    Eigen::MatrixXd F_;

    // process covariance matrix
    Eigen::MatrixXd Q_;

    // measurement matrix
    Eigen::MatrixXd H_;

    // measurement covariance matrix
    Eigen::MatrixXd R_;
};

#endif  // KALMAN_FILTER_H_