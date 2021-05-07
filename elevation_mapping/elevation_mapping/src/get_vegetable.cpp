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

using namespace grid_map;
ros::Subscriber elevation_map_sub,grid_map_sub;
ros::Publisher fused_pub;
std::vector<Eigen::VectorXd> input_samples;
std::vector<Eigen::VectorXd> input_observations;
std::vector<Eigen::VectorXd> vegetable_observations;
GridMap map_fused({"elevation_complete"});
GridMap map_elevation({"elevation"});
GridMap map({"elevation_complete","foot_layer","elevation_layer"});
Position pos;
boost::recursive_mutex rawMapMutex_;
//void initfusedMap(){
//    ROS_WARN("Map initialized!!!!!!!!!!!!");
//    Eigen::Vector2d posi(0.0,0.0);
//    // Create grid map.

//    //map_guassian.setFrameId("odom");
//    //map_guassian.setGeometry(Length(2.0, 2.0), 0.03,posi);

//    //map_elevation.setFrameId("odom");
//    //map_elevation.setGeometry(Length(2.0, 2.0), 0.03,posi);

//    map_fused.setFrameId("odom");
//    map_fused.setGeometry(Length(3.0, 3.0), 0.05,posi);
//    //map_fused.setBasicLayers({"elevation_complete"});
//}


void fusedmapCB(const grid_map_msgs::GridMap &grid_map){
    GridMapRosConverter::fromMessage(grid_map,map_fused,{"elevation_complete"},true,true);
    //std::cout<<map_fused.getBasicLayers()[0]<<std::endl;
    //ROS_WARN("FOOT_POSITION_MAP!!!!!!!!");
    //std::cout<<map_guassian.getPosition().x()<<"   "<<map_guassian.getPosition().y()<<std::endl;
}

void poseCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_){
    //std::cout<<"callback!!!!!!!!!!!!!!!!"<<std::endl;
    pos<<pose_->pose.pose.position.x,pose_->pose.pose.position.y;
    //std::cout<<pos<<std::endl;
}

void fpCB(const sim_assiants::FootLocationsConstPtr& foot_positions_ptr){
    input_samples.clear();
    input_observations.clear();
    //std::cout<<"callback!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
    for (int i=0;i<foot_positions_ptr->FootLocations.size();i++) {
        Eigen::VectorXd sample(2);
        Eigen::VectorXd observation(1);
        sample<<foot_positions_ptr->FootLocations[i].vector.x,foot_positions_ptr->FootLocations[i].vector.y;
        observation<<foot_positions_ptr->FootLocations[i].vector.z;
//        input_samples.push_back(sample);
//        input_observations.push_back(observation);
        if(input_samples.size()<60){
                input_samples.push_back(sample);
                input_observations.push_back(observation);
        }


    }


}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_vegetable");
  ros::NodeHandle nodeHandle("~");
  ros::Subscriber vegetable_sub = nodeHandle.subscribe("/fused_map",1,&fusedmapCB);
          //nodeHandle.advertise<grid_map_msgs::GridMap>("/fused_map",1,true);
  ros::Subscriber sub = nodeHandle.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/pose_pub_node/base_pose",1,poseCB);
  ros::Subscriber foot_locations_sub = nodeHandle.subscribe<sim_assiants::FootLocations>("/gazebo/foot_location",10,fpCB);
  //initfusedMap();


  ros::Rate rate(10.0);
  while (ros::ok()) {
      ros::Time time = ros::Time::now();
      //map.move(pos);

//      for (int j=0;j<input_samples.size();j++) {
//  //        std::cout<<input_samples[j]<<std::endl;
//  //        std::cout<<input_observations[j]<<std::endl;
//          Index my_index,test_index;
//          if(input_samples[j].x()!=0&&input_samples[j].y()!=0){
//              Position position(input_samples[j].x(),input_samples[j].y());
//              //Position position_test(0.2,0.2);
//              //std::cout<<position<<std::endl;
//              //std::cout<<"******************"<<std::endl;
//              map_fused.getIndex(position,my_index);
//              //map_fused.getIndex(position_test,test_index);
//              //std::cout<<my_index<<std::endl;
//              if(map_fused.isValid(my_index)){
//                  std::cout<<"coming there!!!!!!"<<std::endl;
//                  vegetable_observations[j].x() =  map_fused.at("elevation_complete",my_index);
//                  std::cout<<vegetable_observations[j]<<std::endl;
//              }
//          }

    Index my_index,test_index;
    Position position_test(0.2,0.2);
    map_fused.getIndex(position_test,test_index);
    std::cout<<test_index<<std::endl;
                  if(map_fused.isValid(test_index)){
                      std::cout<<"coming there!!!!!!"<<std::endl;
                      std::cout<<map_fused.at("elevation_complete",test_index)<<std::endl;
                  }
//       }
//      map.add("foot_elevation",map_guassian["elevation"]);

//      std::cout<<"programing coming here!!!!!!!"<<std::endl;
//      map.add("camera_elevation",map_elevation["elevation"]);
//      std::cout<<"*****************"<<std::endl;

//      boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);

//      // Add elevation and surface normal (iterating through grid map and adding data).
//      for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
//        Position position;
//        //Index index;
//////        map.getPosition(*it, position);
//////        map.getIndex(position,index);
////        std::cout<<*it<<std::endl;
////        std::cout<<map_guassian.at("elevation",*it)<<std::endl;
//                if(map_guassian.isValid(*it)){
//                    //std::cout<<"valid guassian index!!!"<<std::endl;
//                //std::cout<<map_guassian.at("elevation",*it)<<std::endl;
//                    map.at("foot_layer",*it) = map_guassian.at("elevation",*it);
//                };
//                if(map_elevation.isValid(*it)){
//                    //std::cout<<"valid elevation index!!!"<<std::endl;
//                //std::cout<<map_guassian.at("elevation",*it)<<std::endl;
//                    map.at("elevation_layer",*it) = map_elevation.at("elevation",*it);
//                };
//                //MXR::NOTE: just complete the elevation_layer because when initialize the map,we cannot have the knowledge of the full map
//                if(map_guassian.isValid(*it)&&map_elevation.isValid(*it)){
//                    map.at("elevation_complete",*it) = map_elevation.at("elevation",*it);
//                }else if(!map_elevation.isValid(*it)&&map_guassian.isValid(*it)){
//                    map.at("elevation_complete",*it) = map_guassian.at("elevation",*it);
//                }
//      }

      //    std::cout<<input_samples.size()<<" "<<input_observations.size()<<std::endl;
//          for (int j=0;j<input_samples.size();j++) {
//      //        std::cout<<input_samples[j]<<std::endl;
//      //        std::cout<<input_observations[j]<<std::endl;
//              Index my_index;
//              Position position(input_samples[j].x(),input_samples[j].y());
//              map.getIndex(position,my_index);
//              //std::cout<<my_index<<std::endl;
//              if(map_guassian.isValid(my_index)){
//                  std::cout<<"coming there!!!!!!"<<std::endl;
//                  vegetable_observations[j].x() =  map_guassian.at("elevation",my_index);
//                  std::cout<<vegetable_observations[j]<<std::endl;
//              }

//          }
//        if(map_guassian.getIndex(Position(0.5,0.5),index)){std::cout<<"callback!!!!!!"<<std::endl;
//        std::cout<<index<<std::endl;};

//        std::cout<<map_guassian.getLayers()[0]<<map_guassian.getLayers()[1]<<std::endl;
      //std::cout<<map_guassian.at("elevation",index)<<std::endl;
      //map.add("foot_elevation",map_guassian.get("elevation"));
      //map.add("elevation_height",map_elevation.get("elevation"));
      //map.addDataFrom(map_elevation,false,false,false,{"elevation"});
      //
      //map.move(pos);
//      map.setTimestamp(time.toNSec());
//      //map.move(Position(0,0));
//      grid_map_msgs::GridMap message;
//      GridMapRosConverter::toMessage(map, message);
//      fused_pub.publish(message);
      //ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());


      ros::spinOnce();
      // Wait for next cycle.
      rate.sleep();
    }

  return 0;
}
