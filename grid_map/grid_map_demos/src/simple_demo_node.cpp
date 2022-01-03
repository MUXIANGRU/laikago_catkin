#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
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
using namespace grid_map;
//using namespace tf;

typedef tf2::TransformException TransformException;

kindr::Position3D trackPoint_;
Position pos;
std::array<std::pair<double, double>, 60> mean_value;
std::array<double, 60> height_value;

std::vector<Eigen::VectorXd> input_samples;
std::vector<Eigen::VectorXd> input_observations;
GridMap map_elevation({"elevation"});
double pose_variance;
std::string Pose_topic,Foot_topic;

//MXR::NOTE: move map when robot moving
//don't use
bool updataMaplocation(GridMap map_){
    //MXR::todo
    //write a function to load parameters from parameter_server
    tf::TransformListener transformListener_;
    trackPoint_.x() = 0.0;
    trackPoint_.y() = 0.0;
    trackPoint_.z() = 0.0;
    std::string trackPointId = "/foot_print";
    geometry_msgs::PointStamped trackPoint;
    trackPoint.header.frame_id = trackPointId;
    trackPoint.header.stamp = ros::Time(0);
    kindr_ros::convertToRosGeometryMsg(trackPoint_, trackPoint.point);
    geometry_msgs::PointStamped trackPointTransformed;
    //std::cout<<map_.getFrameId()<<std::endl;
//    std::cout<<trackPoint.point<<std::endl;
//    std::cout<<trackPoint.header.frame_id<<std::endl;
    transformListener_.waitForTransform("/foot_print", "/odom", ros::Time::now(), ros::Duration(0.3));
//    try {
    transformListener_.transformPoint(map_.getFrameId(), trackPoint, trackPointTransformed);
//    } catch (tf::TransformException &ex) {
//      ROS_ERROR("%s", ex.what());
//      return false;
//    }
//    kindr::Position3D position3d;
//    kindr_ros::convertFromRosGeometryMsg(trackPointTransformed.point, position3d);
//    grid_map::Position position = position3d.vector().head(2);
//    map_.move(position);
    return true;
}

void poseCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_){
    //std::cout<<"callback!!!!!!!!!!!!!!!!"<<std::endl;
    pos<<pose_->pose.pose.position.x,pose_->pose.pose.position.y;
//    double sum=0.0;
//    for(int i=0;i<6;i++){
//        sum+=pose_->pose.covariance[7*i];
//    }
//    pose_variance=sum;
//    std::cout<<"pose_variance   "<<pose_variance<<std::endl;
}
void fpCB(const sim_assiants::FootLocationsConstPtr& foot_positions_ptr){
    input_samples.clear();
    input_observations.clear();
    //std::cout<<"callback!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
    //每次拟合之前都对数组重新赋值!!!
    for (int i=0;i<foot_positions_ptr->FootLocations.size();i++) {
        Eigen::VectorXd sample(2);
        Eigen::VectorXd observation(1);
        sample<<foot_positions_ptr->FootLocations[i].vector.x,foot_positions_ptr->FootLocations[i].vector.y;
        observation<<foot_positions_ptr->FootLocations[i].vector.z;

        if(input_samples.size()<60){
                input_samples.push_back(sample);
                input_observations.push_back(observation);
        }


    }
//    std::cout<<input_samples.size()<<" "<<input_observations.size()<<std::endl;
//    for (int j=0;j<input_samples.size();j++) {
//        std::cout<<"_________________"<<std::endl;
////        std::cout<<input_samples[j]<<std::endl;
////        std::cout<<input_samples[j].w()<<" "<<input_samples[j].x()<<" "<<input_samples[j].y()<<input_samples[j].z()<<std::endl;
//        std::cout<<input_observations[j]<<std::endl;
//        std::cout<<input_observations[j].w()<<" "<<input_observations[j].x()<<" "<<input_observations[j].y()<<input_observations[j].z()<<std::endl;
//    }


}


void elevationCB(const grid_map_msgs::GridMap &elevation_map){
//    my_map.data= elevation_map->data;
//    std::cout<<my_map.data.at(1)<<std::endl;
    GridMapRosConverter::fromMessage(elevation_map,map_elevation);
    //MXR::NOTE: the two map have same indices,why error????
//    Position position2 = {1,1};
//    Index index1;
//    map_elevation.getIndex(position2,index1);
//    std::cout<<"--------------------"<<std::endl;
//    std::cout<<"map_elevation index "<<index1<<std::endl;

    //ROS_WARN("ELEVATION_MAP!!!!!!!!");
    //std::cout<<map_elevation.getPosition().x()<<"   "<<map_elevation.getPosition().y()<<std::endl;
}


int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "grid_map_simple_demo");
  ros::NodeHandle nh("~");
  nh.param("/Pose_topic", Pose_topic, std::string("/pose_pub_node/base_pose"));
  nh.param("/Foot_topic", Foot_topic, std::string("/gazebo/foot_location"));
  ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  ros::Subscriber sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(Pose_topic,1,poseCB);
  ros::Subscriber foot_locations_sub = nh.subscribe<sim_assiants::FootLocations>(Foot_topic,10,fpCB);
  //ros::Subscriber  elevation_map_sub = nh.subscribe("/elevation_mapping/elevation_map_raw",1,&elevationCB);




  //MXR::NOTE
  //There is a bug when the tf::TransformListener is named out of main() function
  Eigen::Vector2d posi(0.0,0.0);
  // Create grid map.
  GridMap map({"elevation","elevation_variance"});

  map.setBasicLayers({"elevation","elevation_variance"});

  map.setFrameId("odom");
  map.setGeometry(Length(2.5, 2.5), 0.05,posi);
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1));


  //MXR::NOTE: Work with grid map in a loop.
  ros::Rate rate(50.0);
  while (ros::ok()) {

    // Add data to grid map.
    ros::Time time = ros::Time::now();
//    std::cout<<"**********************"<<std::endl;
//    std::cout<<input_samples.size()<<" "<<input_observations.size()<<std::endl;
//    std::cout<<"**********************"<<std::endl;
    for (GridMapIterator it(map); !it.isPastEnd(); ++it) {

      Index index;
      Position position;
      map.getPosition(*it, position);
//      std::cout<<"--------------------"<<std::endl;
//      std::cout<<"map index "<<index<<std::endl;
      //std::cout<<"map is iterating!!!!!!!!"<<std::endl;
      //std::cout<<position.x()<<" "<<position.y()<<std::endl;
      std::vector<double> test_input_x = {position.x()};
      std::vector<double> test_input_y = {position.y()};
      if(input_samples.size()>0)
      {
            //std::cout<<input_samples[0].x()<<std::endl;
            //每次都相当于重新做一次拟合!!!
            std::vector<double> height_value=doGaussianProcessRegression(input_samples,input_observations,2,1,test_input_x,test_input_y);
            //hyperparam optimization is out of time!!!
            //std::vector<double> height_op=doGaussianProcessRegressionOP(input_samples,input_observations,2,1,test_input_x,test_input_y);
            //std::cout<<"height_value.size(): ";
//            std::cout<<height_value.size()<<std::endl;
//            for (int j =0;j<height_op.size();j++) {
//                    std::cout<<height_op[j]<<std::endl;
//            }
            map.at("elevation",*it) = height_value[0];
            //map.at("elevation_op",*it) = height_op[0];
            map.at("elevation_variance",*it) = height_value[1];
//                    map.at("my_test",index) = map_elevation.at("elevation",index);
                    //std::cout<<" map_elevation "<<map_elevation.at("elevation",index) <<std::endl;
                    //std::cout<<" map "<< map.at("elevation",*it) <<std::endl;
             // }
      }
//

      //else {
//          map.at("elevation",*it) = 0.0;
//      }
      //std::cout<<test_input_x[0]<<" "<<test_input_y[0]<<std::endl;
      //map.getPosition(*it.end(),position1);
      //std::cout<<position<<std::endl;
      //map.at("elevation", *it) = -0.04 + 0.2 * std::sin(3.0 * time.toSec() + 5.0 * position.y()) * position.x();

    }

//    if(updataMaplocation(map)){
//        //ROS_INFO("The map is updating with the robot moving!!!!!!!");
//    }




    //MXR::note
    //move the map
       map.move(pos);



    // Publish grid map.
    map.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(map, message);
    publisher.publish(message);
    ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());

    ros::spinOnce();
    // Wait for next cycle.
    rate.sleep();
  }
  //ros::spin();

  return 0;
}
