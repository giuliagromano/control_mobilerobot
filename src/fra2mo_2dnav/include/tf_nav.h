#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include "boost/thread.hpp"
#include "Eigen/Dense"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "utils.h"
#include <kdl/frames.hpp>
#include <tf/transform_broadcaster.h>



class TF_NAV {

    public:
        TF_NAV();
        void run();
        void tf_listener_fun();
        void tf_listener_aruco();
        void position_pub();
        void position_aruco_pub();
        void goal_listener();
        /* 
        void goal1_listener();
        void goal2_listener();
        void goal3_listener();
        void goal4_listener();
        */
        void send_goal();

  private:

        ros::NodeHandle _nh;

        ros::Publisher _position_pub;
        ros::Publisher _position_aruco_pub;

        Eigen::Vector3d _home_pos;
        Eigen::Vector4d _home_or;


        Eigen::Vector3d _cur_pos;
        Eigen::Vector4d _cur_or;
        
        Eigen::Vector3d _camera_pos;
        Eigen::Vector4d _camera_or;

        Eigen::Vector3d _aruco_pos;
        Eigen::Vector4d _aruco_or;
      
        Eigen::Vector3d _goal_pos;
        Eigen::Vector4d _goal_or;
        /*
        Eigen::Vector3d _goal1_pos;
        Eigen::Vector4d _goal1_or;
        Eigen::Vector3d _goal2_pos;
        Eigen::Vector4d _goal2_or;
        Eigen::Vector3d _goal3_pos;
        Eigen::Vector4d _goal3_or;
        Eigen::Vector3d _goal4_pos;
        Eigen::Vector4d _goal4_or;
        */
        typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

        int choice;
};
