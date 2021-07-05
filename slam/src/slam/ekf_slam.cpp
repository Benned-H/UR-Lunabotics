#include "ekf_slam.h"
#include <cmath>

geometry_msgs::Quaternion yaw_to_quaternion( const double& yaw ) {
    geometry_msgs::Quaternion quaternion;
    quaternion.w = cos(yaw / 2.0);
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(yaw / 2.0);
    return quaternion;
}

// Returns equivalent angle between -pi and pi
double wrap_angle( const double& angle ) {
    while ( angle > M_PI ) {
        angle -= (2.0*M_PI);
    }
    while ( angle < -M_PI ) {
        angle += (2.0*M_PI);
    }
    return angle;
}

bool is_zero( const double& w ) {
    const double eps = 1e-9;
    return ( std::fabs(w) <= eps );
}

EKFSLAM::EKFSLAM() : _u( 0.0, 0.0 ), _mu(3) {
    // State estimate starts with only the robot
    _mu(0) = 0.0;
    _mu(1) = 0.0;
    _mu(2) = 0.0;
}

EKFSLAM::~EKFSLAM() {

}

void EKFSLAM::void handle_command( const geometry_msgs::Twist::ConstPtr& msg ) {
    _u(0) = msg->linear.x; // Grab vx
    _u(1) = msg->angular.z; // Grab wz
}

void EKFSLAM::handle_observations(/* const perception::Observations::ConstPtr& msg*/) {
    // TODO - Actually implement
}

// Step the EKF SLAM belief forward
void EKFSLAM:step( const double& dt ) {

    if ( is_zero( _u(1) ) ) {
        _u(1) = 1e-6; // Accounts for zero angular velocity
    }
    
    int N = _observed_landmarks.size(); // Line 2 of pseudocode
    
    // Motion model step
    
    Eigen::VectorXd _mu_projected = _mu; // Copy values over
    double v_over_w = _u(0) / _u(1);
    _mu_projected(0) = _mu(0) - v_over_w * sin(_mu(2)) + v_over_w * sin( _mu(2) + _u(1)*dt );
    _mu_projected(1) = _mu(1) + v_over_w * cos(_mu(2)) - v_over_w * cos( _mu(2) + _u(1)*dt );
    _mu_projected(2) = _mu(2) + _u(1)*dt; // Line 4 of pseudocode done
    
    Eigen::MatrixXd _sigma_projected = Eigen::MatrixXd::Zero( 3*N+3, 3*N+3 );

    Eigen::MatrixXd G = Eigen::MatrixXd::Identity( 3*N+3, 3*N+3 );
    G(0,2) = -v_over_w * cos(_mu(2)) + v_over_w * cos( _mu(2) + _u(1)*dt );
    G(1,2) = -v_over_w * sin(_mu(2)) + v_over_w * sin( _mu(2) + _u(1)*dt ); // Line 5 of pseudocode done
    
    // TODO - Construct Rt. What is it?
    // TODO - Rest of file is unchecked
    _sigma_projected = G*_sigma*G.transpose() + F.transpose()*R*F;


    // measurement model step
    perception::Observation _z_hat;
    for(unsigned int i = 0; i < _z.observations.size(); i++){
        // j = c_t^i
        /*
         *  if j never seen before
         *      \mu_{j,x}, \mu_{j,y}, \mu_{j,s}
         */

        /*
         *  \delta = [\delta_x, \delta_y]'
         *  q = \delta.T @ \delta
         *  compute _z_hat
         *  Compute innovation and update projections
         */
    }

    return;
}

nav_msgs::Odometry
EKFSLAM::estimated_odometry() const{
    nav_msgs::Odometry out;
    out.pose.pose.position.x = _mu(0);
    out.pose.pose.position.y = _mu(1);
    out.pose.pose.position.z = 0.0;
    out.pose.pose.orientation = yaw_to_quaternion(_mu(2));
    out.twist.twist.linear.x = _u.linear.x;
    out.twist.twist.angular.z = _u.angular.z;
    return out;
}

std::ostream& operator<<( std::ostream& out, const EKFSLAM& other ) {
    // TODO - Implement
}
