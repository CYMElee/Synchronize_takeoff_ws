#include "takeoff.h"


Takeoff::Takeoff(ros::NodeHandle *nh,int UAV_ID){
    
    MAV_ID = UAV_ID;
    mavlink_topic = std::string("/MAV") + std::to_string(MAV_ID) + std::string("/mavlink/to");
    arm_clint = std::string("/MAV") + std::to_string(MAV_ID) + std::string("/mavros/cmd/arming");
    setmode_clint = std::string("/MAV") + std::to_string(MAV_ID) + std::string("/mavros/set_mode");
    takeoff_clint = std::string("/MAV") + std::to_string(MAV_ID) + std::string("/mavros/cmd/takeoff");
    position_topic = std::string("/MAV") + std::to_string(MAV_ID) + std::string("/mavros/setpoint_position/local");
    state_topic = std::string("/MAV") + std::to_string(MAV_ID) + std::string("/mavros/state");
    signal_topic = std::string("/ground_station/set_mode");

    wake_pub = nh->advertise<geometry_msgs::PoseStamped>(position_topic, 10); 
    takeoff = nh->serviceClient< mavros_msgs::CommandTOL>(takeoff_clint);
    arm = nh -> serviceClient<mavros_msgs::CommandBool>(arm_clint);
    setmode = nh->serviceClient<mavros_msgs::SetMode>(setmode_clint);
    state_sub = nh->subscribe(state_topic, 10, &Takeoff::state_cb,this);
    Takeoff_Signal = nh->subscribe(signal_topic, 10, &Takeoff::Takeoff_Signal_cb,this);

    
}


void Takeoff::MAV_takeoff()
{
    ros::Rate rate_connect(100);
    while(ros::ok() && !current_state.connected){
	  ros::spinOnce();
	  rate_connect.sleep();
	  }

    	
    ros::topic::waitForMessage<mavros_msgs::Mavlink>(mavlink_topic);
    ROS_ERROR("Set Origion Position");

    //send a few setpoints before starting
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    ros::Rate rate_takeoff(35);
   
    for(int i = 100; ros::ok() && i > 0; --i){
        wake_pub.publish(pose);
        ros::spinOnce();
        rate_takeoff.sleep();
    }


    offb_set_mode.request.custom_mode = "GUIDE";
    ros::Rate rate_arm(10);
    while(ros::ok()){
        if(current_state.mode != "GUIDE"){
            if( setmode.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
            ROS_INFO("GUIDE enabled");
            arm_cmd.request.value = true;

            }
        }
        if(!current_state.armed){

            if(arm.call(arm_cmd) && arm_cmd.response.success) {
            ROS_INFO("Vehicle armed");
            }
            }
        if(Change_Mode_Trigger.data == MAV_mod::TAKEOFF)
        {
            break;
        }
        
        ros::spinOnce();
        rate_arm.sleep();
    }



    srv_takeoff.request.altitude = 0.5;
    srv_takeoff.request.latitude = 0;
    srv_takeoff.request.longitude = 0;
    srv_takeoff.request.min_pitch = 0;
    srv_takeoff.request.yaw = 0;
    if (takeoff.call(srv_takeoff)) {
        ROS_INFO("srv_takeoff send ok %d", srv_takeoff.response.success);
    } else {
        ROS_ERROR("Failed Takeoff");
    }

}
void Takeoff::MAV_land(void){
        ros::Rate rate_land(100);
        ROS_WARN("Land!");
        while(ros::ok && Change_Mode_Trigger.data != MAV_mod::LAND)
        {
            ros::spinOnce();
            rate_land.sleep();
        }
        offb_set_mode.request.custom_mode = "LAND";
        setmode.call(offb_set_mode);
        arm_cmd.request.value = false;
        arm.call(arm_cmd);
}



void Takeoff::state_cb(const mavros_msgs::State::ConstPtr& msg){

	current_state = *msg;
}

void Takeoff::Takeoff_Signal_cb(const std_msgs::Int16::ConstPtr& msg){

	Change_Mode_Trigger = *msg;
}
