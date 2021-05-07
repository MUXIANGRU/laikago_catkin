#include <ros/ros.h>
#include "elevation_mapping/ElevationMapping.hpp"
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <cmath>
#include <geometry_msgs/PointStamped.h>
#include <kindr/Core>
#include <kindr_ros/kindr_ros.hpp>
#include <kindr_ros/RosGeometryMsgPhysicalQuantities.hpp>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sim_assiants/FootLocations.h>
#include <limbo/gp.hpp>
#include "gazebo_msgs/ContactsState.h"
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <numeric>
#include <vector>
#include <cmath>

using namespace grid_map;
ros::Subscriber elevation_map_sub,grid_map_sub;
ros::Publisher fused_pub;
std::vector<double>  ave_power;
//foot_position container
std::vector<Eigen::VectorXd> input_samples;
std::vector<Eigen::VectorXd> input_observations;
//vegetable_height container
std::vector<Eigen::VectorXd> input_vegetable_samples;
std::vector<Eigen::VectorXd> vegetable_observations;
//vegetable_height container
std::vector<Eigen::VectorXd> input_center_samples;
std::vector<Eigen::VectorXd> center_observations;
//compliance estimate container
std::vector<Eigen::VectorXd> input_compliance_samples;
std::vector<Eigen::VectorXd> input_compliance_observations;
std::vector<Eigen::VectorXd> process_compliance_samples;
std::vector<Eigen::VectorXd> process_compliance_observations;
double p_torque,p_average,pose_variance,point_variance;
gazebo_msgs::ContactsState contact_state;
sensor_msgs::JointState joint_states;
//map_guassian for foot_layer;map_elevation for vision_layer;map<==>fused map
GridMap map_guassian({"elevation"});
GridMap map_elevation;
GridMap map({"final_fused","elevation_complete","foot_layer","foot_layer_variance","elevation_layer","vegetable_layer","vegetable_plane",
             "vegetable_layer_variance","compliance","cost_of_transport","final_cov"});

Position pos;
boost::recursive_mutex rawMapMutex_;
void initfusedMap(){
    ROS_WARN("Map initialized!!!!!!!!!!!!");
    Eigen::Vector2d posi(0.0,0.0);
    // Create grid map.
    // 地图大小应该与落脚点地图一致
    map.setFrameId("odom");
    map.setGeometry(Length(2.5, 2.5), 0.05,posi);

}

void elevationCB(const grid_map_msgs::GridMap &elevation_map){
    GridMapRosConverter::fromMessage(elevation_map,map_elevation);
//   for(int i=0;i<map_elevation.getLayers().size();i++){
//     std::cout<<map_elevation.getLayers()[i]<<std::endl;
//   }
   map_elevation.setBasicLayers({"elevation", "variance"});
    ROS_WARN("ELEVATION_MAP!!!!!!!!");
//    std::cout<<map_elevation.getSize()<<std::endl;
//    std::cout<<map_elevation.getLength()<<std::endl;
//    std::cout<<map_elevation.getResolution()<<std::endl;
//    for(int i=0;i<map_elevation.getBasicLayers().size();i++){
//      std::cout<<map_elevation.getBasicLayers()[i]<<std::endl;
//    }
    //std::cout<<map_elevation.getPosition().x()<<"   "<<map_elevation.getPosition().y()<<std::endl;
}

void gridmapCB(const grid_map_msgs::GridMap &grid_map){
    GridMapRosConverter::fromMessage(grid_map,map_guassian);
//    ROS_WARN("FOOT_POSITION_MAP!!!!!!!!");
//    std::cout<<map_guassian.getSize()<<std::endl;
//    std::cout<<map_guassian.getLength()<<std::endl;
//    std::cout<<map_guassian.getResolution()<<std::endl;
    //std::cout<<map_guassian.getPosition().x()<<"   "<<map_guassian.getPosition().y()<<std::endl;
}
//for map moving
void poseCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_){
    //std::cout<<"callback!!!!!!!!!!!!!!!!"<<std::endl;
    pos<<pose_->pose.pose.position.x,pose_->pose.pose.position.y;
    //std::cout<<pos<<std::endl;
    double sum=0.0;
    for(int i=0;i<6;i++){
        sum+=pose_->pose.covariance[7*i];
    }
    pose_variance=sum;
    //std::cout<<"pose_variance   "<<pose_variance<<std::endl;
}

void fpCB(const sim_assiants::FootLocationsConstPtr& foot_positions_ptr){
    input_samples.clear();
    input_observations.clear();
    input_vegetable_samples.clear();
    vegetable_observations.clear();
    input_compliance_observations.clear();
    process_compliance_samples.clear();
    process_compliance_observations.clear();
    //std::cout<<"callback!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
    for (int i=0;i<foot_positions_ptr->FootLocations.size();i++) {
        Eigen::VectorXd sample(2);
        Eigen::VectorXd observation(1);
        Eigen::VectorXd compliance_observation(1);
        sample<<foot_positions_ptr->FootLocations[i].vector.x,foot_positions_ptr->FootLocations[i].vector.y;
        observation<<foot_positions_ptr->FootLocations[i].vector.z;
        compliance_observation<<foot_positions_ptr->force.data;
        if(input_samples.size()<60){
                input_samples.push_back(sample);
                input_observations.push_back(observation);
                input_compliance_observations.push_back(compliance_observation);
        }


    }
    for (int k=0;k<input_samples.size();k++) {
        Index my_index;

        if(input_samples[k].x()!=0&&input_samples[k].y()!=0){
            Position position(input_samples[k].x(),input_samples[k].y());
            map.getIndex(position,my_index);
            //std::cout<<my_index<<std::endl;
            if(map_guassian.isValid(my_index)){
                //std::cout<<"coming there!!!!!!"<<std::endl;
                Eigen::VectorXd compliance_sample(2);
                Eigen::VectorXd compliance_observation(1);
                compliance_sample<<input_samples[k].x(),input_samples[k].y();
                if(map_elevation.isValid(my_index)){
                    //compliance_observation<<input_compliance_observations[k]/( map_elevation.at("elevation",my_index) - map_guassian.at("elevation",my_index));
                    compliance_observation<<( map_elevation.at("elevation",my_index) - map_guassian.at("elevation",my_index))/input_compliance_observations[k].x();
                }else{
                    //compliance_observation<<input_compliance_observations[k]/(input_observations[k].x() - map_guassian.at("elevation",my_index));
                    compliance_observation<<(input_observations[k].x() - map_guassian.at("elevation",my_index))/input_compliance_observations[k].x();
                }


                if(process_compliance_samples.size()<60){
                    process_compliance_samples.push_back(compliance_sample);
                    process_compliance_observations.push_back(compliance_observation);
                    //std::cout<<compliance_observation<<std::endl;
                }
            }

    }
    }

//        std::cout<<"******************"<<std::endl;
//        std::cout<<"process_compliance_observations.size():  "<<process_compliance_observations.size()<<std::endl;
//        std::cout<<"******************"<<std::endl;
    //MXR::NOTE save the value for training
    for (int j=0;j<input_samples.size();j++) {
        Index my_index;
        if(input_samples[j].x()!=0&&input_samples[j].y()!=0){
            Position position(input_samples[j].x(),input_samples[j].y());
            map.getIndex(position,my_index);
            //std::cout<<my_index<<std::endl;
            if(map_guassian.isValid(my_index)){
                //std::cout<<"coming there!!!!!!"<<std::endl;
                Eigen::VectorXd vegetable_sample(2);
                Eigen::VectorXd vegetable_observation(1);
                vegetable_sample<<input_samples[j].x(),input_samples[j].y();
                //vegetable_observation<<input_observations[j].x() - map_guassian.at("elevation",my_index);
                vegetable_observation<<map.at("elevation_complete",my_index)-input_observations[j].x();

                if(input_vegetable_samples.size()<60){
                    input_vegetable_samples.push_back(vegetable_sample);
                    vegetable_observations.push_back(vegetable_observation);
                    //std::cout<<vegetable_observation<<std::endl;
                }
            }
        }


    }
//    std::cout<<"******************"<<std::endl;
//    std::cout<<"vegetable_observations.size():  "<<vegetable_observations.size()<<std::endl;
//    std::cout<<"******************"<<std::endl;



}

//void ffCB(const gazebo_msgs::ContactsState::ConstPtr& ffmessage_ptr){
//    contact_state.states = ffmessage_ptr->states;
//}

//void contactCB(const std_msgs::Float64MultiArray::ConstPtr& is_contact){
//    input_compliance_samples.clear();
//    input_compliance_observations.clear();

//}
void jointCB(const sensor_msgs::JointState::ConstPtr &joint_states_){
    joint_states.name = joint_states_->name;
    joint_states.effort = joint_states_->effort;
    joint_states.header = joint_states_->header;
    joint_states.position = joint_states_->position;
    joint_states.velocity = joint_states_->velocity;

//    if(ave_power.size()==0){
//        ave_power.resize(5);
//    }
    //p_torque = 0;
    double torque = 0;
    for(int j=0;j<joint_states.effort.size();j++){
        if(joint_states.effort[j]*joint_states.velocity[j]>0){
             torque+=joint_states.effort[j]*joint_states.velocity[j];
        }

    }
    ave_power.push_back(torque);
    if(ave_power.size()>5){
        p_torque = std::accumulate(ave_power.begin(),ave_power.end(),0.0)/ave_power.size();
        ave_power.clear();
    }

}
bool checkValid(std::vector<Eigen::VectorXd> &warehouse,Eigen::VectorXd &position_){
    for(int i = 0;i<warehouse.size();i++){
        if((std::pow(position_.x()-warehouse[i].x(),2)+std::pow(position_.y()-warehouse[i].y(),2)<0.0025)
                ||std::abs(position_.z()-warehouse[warehouse.size()-1].z())<1){
            return false;
        }
    }
    return true;
}
//MXR::NOTE: limit the length of point
//加引用的作用???
std::vector<Eigen::VectorXd> limitLength(std::vector<Eigen::VectorXd> &vector2d,int length){
    if(vector2d.size()>length){
        for(int i =0;i<vector2d.size()-length;i++){
            std::vector<Eigen::VectorXd>::iterator it = vector2d.begin();
            vector2d.erase(it);
        }

    }
//    std::cout<<"$$$$$$$$$$$$$$$$$$$$$$$$$"<<std::endl;
//    std::cout<<vector2d.size()<<std::endl;
//    std::cout<<"$$$$$$$$$$$$$$$$$$$$$$$$$"<<std::endl;
    return vector2d;
}
Eigen::VectorXd normalize(Eigen::VectorXd& observation){
    Eigen::VectorXd mid = observation;
    if(mid.x()>10){
        mid.x()=p_average;
    }
    mid.x() = mid.x()/50;
    return mid;
}
void odomCB(const nav_msgs::Odometry::ConstPtr &gazebo_odom_){
//    if(input)
//    input_center_samples.clear();
//    center_observations.clear();
    Eigen::VectorXd center_location(3);
    Eigen::VectorXd center_height(1);
    center_height<<p_torque/50*9.8*std::sqrt(std::pow(gazebo_odom_->twist.twist.linear.x,2)+std::pow(gazebo_odom_->twist.twist.linear.y,2)
                             +std::pow(gazebo_odom_->twist.twist.linear.z,2));
    center_location<<gazebo_odom_->pose.pose.position.x,gazebo_odom_->pose.pose.position.y,
            static_cast<double>(gazebo_odom_->header.stamp.sec);
    if(input_center_samples.size()==0){
         input_center_samples.push_back(center_location);
         center_observations.push_back(normalize(center_height));
    }
    double sum=0.0;
    for (int i =0;i<center_observations.size();i++) {
           sum+=center_observations[i].x();
    }
    p_average = sum/center_observations.size();
    if(checkValid(input_center_samples,center_location)){
        input_center_samples.push_back(center_location);
        center_observations.push_back(normalize(center_height));
    }
    limitLength(input_center_samples,60);
    limitLength(center_observations,60);

//    std::cout<<"###########################"<<std::endl;
//    std::cout<<input_center_samples.size()<<std::endl;
//    for (int k=0;k<input_center_samples.size();k++) {
//        std::cout<<input_center_samples[k].x()<<"   "<<input_center_samples[k].y()<<"   "<<center_observations[k].x();
//        std::cout<<std::endl;
//    }
//    std::cout<<center_observations.size()<<std::endl;
//    std::cout<<"##########################"<<std::endl;

}
void covCB(const geometry_msgs::Vector3::ConstPtr& cov){
    point_variance = cov->x;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "fused_map");
  ros::NodeHandle nodeHandle("~");

  elevation_map_sub = nodeHandle.subscribe("/elevation_mapping/elevation_map_raw",1,&elevationCB);
  grid_map_sub = nodeHandle.subscribe("/grid_map_simple_demo/grid_map",1,&gridmapCB);
  fused_pub = nodeHandle.advertise<grid_map_msgs::GridMap>("/fused_map",1,true);
  ros::Subscriber sub = nodeHandle.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/pose_pub_node/base_pose",1,poseCB);
  ros::Subscriber foot_locations_sub = nodeHandle.subscribe<sim_assiants::FootLocations>("/gazebo/foot_location",10,fpCB);
  ros::Subscriber joint_states_sub = nodeHandle.subscribe<sensor_msgs::JointState>("/joint_states",1,&jointCB);
  ros::Subscriber odom_sub = nodeHandle.subscribe<nav_msgs::Odometry>("/gazebo/odom",1,&odomCB);
  ros::Subscriber var_sub = nodeHandle.subscribe<geometry_msgs::Vector3>("/mxr_covariance",1,&covCB);
//  ros::Subscriber foot_force_sub = nodeHandle.subscribe<gazebo_msgs::ContactsState>("/total_contact",10,ffCB);
//  ros::Subscriber foot_contact_sub = nodeHandle.subscribe("/gazebo/foot_contact_state",1,contactCB);
  initfusedMap();


  ros::Rate rate(10.0);
  while (ros::ok()) {
      ros::Time time = ros::Time::now();

      //boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);

//      for (GridMapIterator it(map_elevation); !it.isPastEnd(); ++it) {
//          Position position;
//          grid_map::Index see_index;
//          map_elevation.getPosition(*it, position);
//          map_elevation.getIndex(position,see_index);
//          std::cout<<"see_index  "<<see_index<<std::endl;
//      }
      // Add elevation and surface normal (iterating through grid map and adding data).
      for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
        Position position;
        grid_map::Index my_index,aft_index;
        map.getPosition(*it, position);
        map.getIndex(position,my_index);  //size不同的地图在同一个position具有不同的index,坐标系位于地图右下角
        map_elevation.getIndex(position,aft_index);
        std::vector<double> test_input_x = {position.x()};
        std::vector<double> test_input_y = {position.y()};

        //MXR::NOTE trainning for vegetable height
        if(input_samples.size()>0)
        {
              std::vector<double> height_value=doGaussianProcessRegression(input_vegetable_samples,vegetable_observations,2,1,test_input_x,test_input_y);
              map.at("vegetable_layer",*it) = height_value[0];
              map.at("vegetable_layer_variance",*it) = height_value[1]+point_variance;
        }
        if(input_samples.size()>0)
        {
              std::vector<double> compliance_value=doGaussianProcessRegression(process_compliance_samples,process_compliance_observations,2,1,test_input_x,test_input_y);
              map.at("compliance",*it) = compliance_value[0];

        if(input_center_samples.size()>0)
        {
              std::vector<double> cost_of_transport=doGaussianProcessRegression(input_center_samples,center_observations,2,1,test_input_x,test_input_y);
              map.at("cost_of_transport",*it) = cost_of_transport[0];

        }
                if(map_guassian.isValid(my_index)){
                    //std::cout<<"valid guassian index!!!"<<std::endl;
//                std::cout<<map_guassian.at("elevation",*it)<<std::endl;
                    map.at("foot_layer",*it) = map_guassian.at("elevation",*it);
                    map.at("foot_layer_variance",*it) = map_guassian.at("elevation_variance",*it);
                };
                if(map_elevation.isValid(aft_index)){
                   //std::cout<<"valid elevation index!!!"<<std::endl;
                    //std::cout<<"my_index  "<<my_index<<std::endl;
                    //std::cout<<"aft_index  "<<aft_index<<std::endl;
                    map.at("elevation_layer",my_index) = map_elevation.at("elevation",aft_index);
                  }
                //MXR::NOTE: just complete the elevation_layer because when initialize the map,we cannot have the knowledge of the full map
                if(map_guassian.isValid(*it)&&map_elevation.isValid(aft_index)){
                    map.at("elevation_complete",*it) = map_elevation.at("elevation",aft_index);
                }else if(!map_elevation.isValid(aft_index)&&map_guassian.isValid(*it)){
                    map.at("elevation_complete",*it) = map_guassian.at("elevation",*it);
                }
                //MXR::NOTE:compute the weights to decide the final_fused_map
                double lamda = map.at("vegetable_layer_variance",*it)/(map.at("foot_layer_variance",*it)+map.at("vegetable_layer_variance",*it));
                map.at("vegetable_plane",*it) = map.at("elevation_complete",*it)-map.at("vegetable_layer",*it);
                map.at("final_fused",*it) = lamda*map.at("foot_layer",*it)+(1-lamda)*map.at("vegetable_plane",*it);
                map.at("final_cov",*it) =(map.at("vegetable_layer_variance",*it)*map.at("foot_layer_variance",*it))/
                        (map.at("foot_layer_variance",*it)+map.at("vegetable_layer_variance",*it));
      }
}

      map.move(pos);
      map.setTimestamp(time.toNSec());
      //map.move(Position(0,0));
      grid_map_msgs::GridMap message;
      GridMapRosConverter::toMessage(map, message);
      fused_pub.publish(message);
      ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());


      ros::spinOnce();
      // Wait for next cycle.
      rate.sleep();
    }

  return 0;
}
