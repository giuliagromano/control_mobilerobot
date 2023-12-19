#include "../include/tf_nav.h"

std::vector<double> aruco_pose(7,0.0);
bool aruco_pose_available = false;

void arucoPoseCallback(const geometry_msgs::PoseStamped & msg) {
    aruco_pose_available = true;
    aruco_pose.clear();
    aruco_pose.push_back(msg.pose.position.x);
    aruco_pose.push_back(msg.pose.position.y);
    aruco_pose.push_back(msg.pose.position.z);
    aruco_pose.push_back(msg.pose.orientation.x);
    aruco_pose.push_back(msg.pose.orientation.y);
    aruco_pose.push_back(msg.pose.orientation.z);
    aruco_pose.push_back(msg.pose.orientation.w);
}

void poseCallback(const geometry_msgs::PoseStamped & msg) {
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg.pose.position.x,msg.pose.position.y,msg.pose.position.z));
  tf::Quaternion q(msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "aruco_frame"));
}


TF_NAV::TF_NAV() {
    _position_aruco_pub = _nh.advertise<geometry_msgs::PoseStamped>( "aruco_frame/pose", 1 );
    _position_pub = _nh.advertise<geometry_msgs::PoseStamped>( "/fra2mo/pose", 1 );
    
    _cur_pos << 0.0, 0.0, 0.0;
    _cur_or << 1.0, 0.0, 0.0, 0.0;
    _camera_pos << 0.0, 0.0, 0.0;
    _camera_or << 1.0, 0.0, 0.0, 0.0;
    _aruco_pos << 0.0, 0.0, 0.0;
    _aruco_or << 1.0, 0.0, 0.0, 0.0;
    _goal_pos << 0.0, 0.0, 0.0;
    _goal_or << 1.0, 0.0, 0.0, 0.0;
    _home_pos << -3.0, 5.0, 0.0;
    _home_or << 0.71, 0.0, 0.0, -0.71;
}

void TF_NAV::tf_listener_fun() {
    ros::Rate r( 5 );
    tf::TransformListener listener;
    tf::StampedTransform transform;

    while ( ros::ok() ) {
        try {
            listener.waitForTransform( "map", "base_footprint", ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform( "map", "base_footprint", ros::Time(0), transform );

        }
        catch( tf::TransformException &ex ) {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }
        
        _cur_pos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _cur_or << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();
        position_pub();
        r.sleep();
    }
}


void TF_NAV::tf_listener_aruco() {
    ros::Rate r( 5 );
    tf::TransformListener listener;
    tf::StampedTransform transform;
    
    while ( ros::ok() ) {
        try {
            listener.waitForTransform( "map", "camera_depth_optical_frame", ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform( "map", "camera_depth_optical_frame", ros::Time(0), transform );
        }
        catch( tf::TransformException &ex ) {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }
        
        _camera_pos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _camera_or << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();
    
        if(aruco_pose_available) {
            KDL::Frame cam_T_object(KDL::Rotation::Quaternion(aruco_pose[3], aruco_pose[4], aruco_pose[5], aruco_pose[6]), KDL::Vector(aruco_pose[0], aruco_pose[1], aruco_pose[2]));
            KDL::Frame map_T_cam(KDL::Rotation::Quaternion(_camera_or[1], _camera_or[2], _camera_or[3], _camera_or[0]), KDL::Vector(_camera_pos[0], _camera_pos[1], _camera_pos[2]));
            KDL::Frame map_T_object(KDL::Rotation::Quaternion(_aruco_or[1], _aruco_or[2], _aruco_or[3], _aruco_or[0]), KDL::Vector(_aruco_pos[0], _aruco_pos[1], _aruco_pos[2]));

            map_T_object.p = map_T_cam.p + map_T_cam.M*cam_T_object.p;
            _aruco_pos = toEigen(map_T_object.p);

            map_T_object.M = map_T_cam.M*cam_T_object.M;
            double x,y,z,w;
            map_T_object.M.GetQuaternion(x,y,z,w);
            _aruco_or << x, y, z, w;
            
            std::cout<<"Aruco position wrt map frame:"<<std::endl<< _aruco_pos <<std::endl;
            //std::cout<<"Aruco quaternion wrt map frame:"<< _aruco_or <<std::endl;
            std::cout<<"Camera position wrt map frame:"<<std::endl<< toEigen(map_T_cam.p) <<std::endl;
            //std::cout<<"Aruco position wrt camera frame:"<< toEigen(cam_T_object.p) <<std::endl;
        }
        position_aruco_pub();
        r.sleep();
    }

}


void TF_NAV::position_pub() {

    geometry_msgs::PoseStamped pose;

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";

    pose.pose.position.x = _cur_pos[0];
    pose.pose.position.y = _cur_pos[1];
    pose.pose.position.z = _cur_pos[2];

    pose.pose.orientation.w = _cur_or[0];
    pose.pose.orientation.x = _cur_or[1];
    pose.pose.orientation.y = _cur_or[2];
    pose.pose.orientation.z = _cur_or[3];

    _position_pub.publish(pose);
}

void TF_NAV::position_aruco_pub() {

    geometry_msgs::PoseStamped pose;

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";

    pose.pose.position.x = _aruco_pos[0];
    pose.pose.position.y = _aruco_pos[1];
    pose.pose.position.z = _aruco_pos[2];

    pose.pose.orientation.w = _aruco_or[0];
    pose.pose.orientation.x = _aruco_or[1];
    pose.pose.orientation.y = _aruco_or[2];
    pose.pose.orientation.z = _aruco_or[3];

    _position_aruco_pub.publish(pose);
}

void TF_NAV::goal_listener() {
    ros::Rate r( 1 );
    tf::TransformListener listener;
    tf::StampedTransform transform;

    while ( ros::ok() ) {
        try {
            listener.waitForTransform( "map", "goal_camera", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal_camera", ros::Time( 0 ), transform );
        }
        catch( tf::TransformException &ex ) {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }

        _goal_pos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _goal_or << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();
        r.sleep();
    }    
}

void TF_NAV::send_goal() {
    ros::Rate r( 5 );
    int cmd;
    move_base_msgs::MoveBaseGoal goal;

    while ( ros::ok() ) {
        std::cout<<"\nInsert 1 to send goal from TF "<<std::endl;
        std::cout<<"Insert 2 to send home position goal "<<std::endl;
        std::cout<<"Inser your choice"<<std::endl;
        std::cin>>cmd;

        if ( cmd == 1) {
            MoveBaseClient ac("move_base", true);
            while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();

            goal.target_pose.pose.position.x = _goal_pos[0];
            goal.target_pose.pose.position.y = _goal_pos[1];
            goal.target_pose.pose.position.z = _goal_pos[2];

            goal.target_pose.pose.orientation.w = _goal_or[0];
            goal.target_pose.pose.orientation.x = _goal_or[1];
            goal.target_pose.pose.orientation.y = _goal_or[2];
            goal.target_pose.pose.orientation.z = _goal_or[3];
 
            ROS_INFO("Sending goal to approach Aruco");
            ac.sendGoal(goal);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && aruco_pose_available){
                ROS_INFO("The mobile robot arrived in the TF goal");
                
                while(!ac.waitForServer(ros::Duration(5.0))){
                ROS_INFO("Waiting for the move_base action server to come up");
                }
                goal.target_pose.header.frame_id = "map";
                goal.target_pose.header.stamp = ros::Time::now();
                
                goal.target_pose.pose.position.x = _aruco_pos[0] + 1;
                goal.target_pose.pose.position.y = _aruco_pos[1];
                goal.target_pose.pose.position.z = 0.0;
/*
                goal.target_pose.pose.orientation.w = 0.0;
                goal.target_pose.pose.orientation.x = 0.0;
                goal.target_pose.pose.orientation.y = 0.0;
                goal.target_pose.pose.orientation.z = 1.0;
*/               
                KDL::Rotation Ar_rot=KDL::Rotation::Quaternion(_aruco_or[1], _aruco_or[2], _aruco_or[3], _aruco_or[0]);
                KDL::Rotation Rb_des=Ar_rot*KDL::Rotation::RotX(-1.5708)*KDL::Rotation::RotZ(1.5708);
                double R,P,Y,x,y,z,w;
                Rb_des.GetRPY(R,P,Y);
                Rb_des=KDL::Rotation::RPY(0,0,Y);
                Rb_des.GetQuaternion(x,y,z,w);
                
                goal.target_pose.pose.orientation.w = w;
                goal.target_pose.pose.orientation.x = x;
                goal.target_pose.pose.orientation.y = y;
                goal.target_pose.pose.orientation.z = z;
                
                //std::cout<<"Goal2:"<<std::endl<< goal <<std::endl;
           
                ROS_INFO("Sending goal wrt Aruco pose");
                ac.sendGoal(goal);

                ac.waitForResult();

                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                    ROS_INFO("The mobile robot arrived in the TF goal");
                    ROS_INFO("The sequence of goals has been completed");
                }    
                else
                    ROS_INFO("The base failed to move for some reason");
            }
            else
                ROS_INFO("The base failed to move for some reason");

        }
        else if ( cmd == 2 ) {
            MoveBaseClient ac("move_base", true);
            while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            
            goal.target_pose.pose.position.x = _home_pos[0];
            goal.target_pose.pose.position.y = _home_pos[1];
            goal.target_pose.pose.position.z = _home_pos[2];

            goal.target_pose.pose.orientation.w = _home_or[0];
            goal.target_pose.pose.orientation.x = _home_or[1];
            goal.target_pose.pose.orientation.y = _home_or[2];
            goal.target_pose.pose.orientation.z = _home_or[3];

            ROS_INFO("Sending HOME position as goal");
            ac.sendGoal(goal);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                ROS_INFO("The mobile robot arrived in the HOME position");
            else
                ROS_INFO("The base failed to move for some reason");
        }
         else {
            ROS_INFO("Wrong input!");
        }
        r.sleep();
    }
    
}

void TF_NAV::run() {
    boost::thread tf_listener_fun_t( &TF_NAV::tf_listener_fun, this );
    boost::thread tf_listener_goal_t( &TF_NAV::goal_listener, this );
    boost::thread tf_listener_aruco_t( &TF_NAV::tf_listener_aruco, this );
    boost::thread send_goal_t( &TF_NAV::send_goal, this );
    ros::spin();
}



int main( int argc, char** argv ) {
    ros::init(argc, argv, "tf_navigation");
    ros::NodeHandle n;
    TF_NAV tfnav;
   
    ros::Subscriber aruco_pose_sub = n.subscribe("/aruco_single/pose", 1, arucoPoseCallback);
    ros::Subscriber sub = n.subscribe("aruco_frame/pose", 1, poseCallback); 
    tfnav.run();

    return 0;
}