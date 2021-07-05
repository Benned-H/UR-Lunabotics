// Implements EKF SLAM with unknown correspondences as presented in Probabilistic Robotics, pg. 321

#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include "ros/ros.h"

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"

class EKFSLAM {
    public:
        EKFSLAM();
        virtual ~EKFSLAM();

        void handle_command( const geometry_msgs::Twist::ConstPtr& msg );
        // void handle_odometry( const nav_msgs::Odometry::ConstPtr& msg ); TODO - Why would we have this?
        void handle_observations(/* const perception::Observations::ConstPtr& msg */);
        void step( const double& dt );

        nav_msgs::Odometry estimated_odometry() const;

    private:
        Eigen::Vector2d _u; // Store motion command (v,w)
        //perception::Observations _z; // Store observations
        std::vector<int> _observed_landmarks; // Store previously seen landmark numbers
        Eigen::VectorXd _mu; // State estimate (x, y, theta)
        Eigen::MatrixXd _sigma; // Covariance matrix
        Eigen::VectorXd _alpha; // TODO - What is this?
        Eigen::MatrixXd _q; // TODO - What is this?
};

std::ostream& operator<<( std::ostream& out, const EKFSLAM& other );
