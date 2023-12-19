#include "../include/tf_nav.h"


TF_NAV::TF_NAV() {

    _position_pub = _nh.advertise<geometry_msgs::PoseStamped>( "/fra2mo/pose", 1 );
    _cur_pos << 0.0, 0.0, 0.0;
    _cur_or << 1.0, 0.0, 0.0, 0.0;
    _goal_pos << 0.0, 0.0, 0.0;
    _goal_or << 1.0, 0.0, 0.0, 0.0;
    _home_pos << -3.0, 5.0, 0.0;
    _home_or << 0.71, 0.0, 0.0, -0.71;    
}

void TF_NAV::tf_listener_fun() {
    ros::Rate r( 5 );
    tf::TransformListener listener;
    tf::StampedTransform transform;
    
    while ( ros::ok() )
    {
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

void TF_NAV::goal_listener() {
    ros::Rate r( 1 );
    tf::TransformListener listener;
    tf::StampedTransform transform;

    while ( ros::ok() )
    {
        switch(choice){
            case 1:
            {
                try{
                listener.waitForTransform( "map", "goal3", ros::Time( 0 ), ros::Duration( 10.0 ) );
                listener.lookupTransform( "map", "goal3", ros::Time( 0 ), transform );
            }
                catch( tf::TransformException &ex ){
                ROS_ERROR("%s", ex.what());
                r.sleep();
                continue;
            }
            break;
            }
            case 2:
            {
                try{
                listener.waitForTransform( "map", "goal4", ros::Time( 0 ), ros::Duration( 10.0 ) );
                listener.lookupTransform( "map", "goal4", ros::Time( 0 ), transform );
            }
                catch( tf::TransformException &ex ){
                ROS_ERROR("%s", ex.what());
                r.sleep();
                continue;
            }   
            break;
            }
            case 3:
            {    
                try
            {
                listener.waitForTransform( "map", "goal2", ros::Time( 0 ), ros::Duration( 10.0 ) );
                listener.lookupTransform( "map", "goal2", ros::Time( 0 ), transform );
            }
                catch( tf::TransformException &ex )
            {
                ROS_ERROR("%s", ex.what());
                r.sleep();
                continue;
            }  
            break;
            }   
            case 4:
            {
                try
            {
                listener.waitForTransform( "map", "goal1", ros::Time( 0 ), ros::Duration( 10.0 ) );
                listener.lookupTransform( "map", "goal1", ros::Time( 0 ), transform );
            }
                catch( tf::TransformException &ex )
            {
                ROS_ERROR("%s", ex.what());
                r.sleep();
                continue;
            }
            break;
            }
                case 5:
            {
                try
            {
                listener.waitForTransform( "map", "goal5", ros::Time( 0 ), ros::Duration( 10.0 ) );
                listener.lookupTransform( "map", "goal5", ros::Time( 0 ), transform );
            }
                catch( tf::TransformException &ex )
            {
                ROS_ERROR("%s", ex.what());
                r.sleep();
                continue;
            }
            break;
            }
        
                case 6:
            {
                try
            {
                listener.waitForTransform( "map", "goal6", ros::Time( 0 ), ros::Duration( 10.0 ) );
                listener.lookupTransform( "map", "goal6", ros::Time( 0 ), transform );
            }
                catch( tf::TransformException &ex )
            {
                ROS_ERROR("%s", ex.what());
                r.sleep();
                continue;
            }
            break;
            }
            default:
                std::cout << "Error goal selection"<<std::endl;
        }
    
        _goal_pos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _goal_or << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();
        // std::cout<<"goal pos: "<<_goal_pos<<std::endl; 
        // std::cout<<"goal or: "<<_goal_or<<std::endl; 
               
        r.sleep();  
    }    
}
void TF_NAV::send_goal() {//1_listener
    ros::Rate r( 5 );
    int cmd;
    move_base_msgs::MoveBaseGoal goal;
    Eigen::Vector3d oldgoalpos; oldgoalpos<<0.0,0.0,0.0;

    while ( ros::ok() )
    {
        std::cout<<"\nInsert 1 to send sequence of goals from TF (3->4->2->1->5->6)"<<std::endl;
        std::cout<<"Insert 2 to send home position goal "<<std::endl;
        std::cout<<"Insert your choice"<<std::endl;
        std::cin>>cmd;

        if ( cmd == 1) {
            
            MoveBaseClient ac("move_base", true);
            
            for (int i=1;i<7;i++){
            
                choice=i;
                std::cout<<"choice "<<choice<<std::endl; 
                
                while(oldgoalpos ==_goal_pos){
                    std::cout<<"WAITING"<<std::endl;
                    r.sleep();
                }
                oldgoalpos=_goal_pos;
               
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
                
               // std::cout<<"goal pos: "<<_goal_pos<<std::endl; 
               // std::cout<<"goal or: "<<_goal_or<<std::endl; 
                std::cout<<"goal: "<<goal<<std::endl; 

                ROS_INFO("Sending goal");
                ac.sendGoal(goal);

                ac.waitForResult();


                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                    ROS_INFO("The mobile robot arrived in the TF goal");
                    if(i == 6) ROS_INFO("The sequence of goals has been completed");
                    r.sleep();
                }
                else{
                    ROS_INFO("The base failed to move for some reason");
                    i=7;
                }
            }
            choice=1;
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
    choice=1;
    boost::thread tf_listener_fun_t( &TF_NAV::tf_listener_fun, this );
    boost::thread tf_listener_goal_t( &TF_NAV::goal_listener, this );
    boost::thread send_goal_t( &TF_NAV::send_goal, this );
    ros::spin();
}



int main( int argc, char** argv ) {
    ros::init(argc, argv, "tf_navigation");
    TF_NAV tfnav;
    tfnav.run();

    return 0;
}