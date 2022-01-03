/*
 * get_foot_localization.cpp
 *
 *  Created on: Nov 26, 2020
 *      Author: MXR
 *   Institute: HIT-LRG
 */

#include "ros/ros.h"
#include "gazebo_msgs/ContactsState.h"
#include "geometry_msgs/WrenchStamped.h"
#include "boost/thread.hpp"
#include "std_msgs/Bool.h"
#include "sim_assiants/FootContact.h"
#include "sim_assiants/FootContacts.h"
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "sim_assiants/FootLocations.h"
#include <iterator>
#include "algorithm"

sim_assiants::FootContacts foot_contact;
sim_assiants::FootLocations foot_locations;
gazebo_msgs::ContactState lf_wrench_, rf_wrench_, rh_wrench_, lh_wrench_;
gazebo_msgs::ContactsState total_contact;
//foot_locations
ros::Publisher pose_publisher,foot_location_publisher,pub_lf_force,pub_rf_force,pub_rh_force,pub_lh_force,pub_force;
geometry_msgs::Vector3Stamped position;
//std::vector<std::vector<double>> my_position(60,std::vector<double>(3,0));

//MXR::NOTE: container to reserve foot positions
std::vector<std::vector<double>> my_position;
std::vector<std::vector<double>> foot_location_0;
std::vector<std::vector<double>> foot_location_1;
std::vector<std::vector<double>> foot_location_2;
std::vector<std::vector<double>> foot_location_3;
std::string lf_contact_topic,rf_contact_topic,lh_contact_topic,rh_contact_topic,FootContacts_topic;

//MXR::NOTE: get the model of vector
double getModel(std::vector<double> point_){
    double res=0.0;
    for (int i=0;i<point_.size();i++) {
        res+=point_[i]*point_[i];
    }
    return sqrt(res);
}

//MXR::NOTE: get the distance between two points
double getDistance(std::vector<double> point1_,std::vector<double> point2_){
    double res =0;
//    std::cout<<point1_.size()<<"     **************    "<<point2_.size()<<std::endl;
    for (int i=0;i<point1_.size()-2;i++) {
        res+=pow((point1_[i]-point2_[i]),2);
    }
   //std::cout<<"the distance is"<<sqrt(res)<<std::endl;
    return sqrt(res);
}

//MXR::NOTE: make sure if the point can be accepted
bool isAccept(std::vector<std::vector<double>> myposition,std::vector<double> mypoint){
    bool flag=0;
    for (int i=0;i<myposition.size();i++) {
        if(getDistance(myposition[i],mypoint)>0.05){
            flag=1;
        }
    }
    return flag;
};

bool isAccept(std::vector<double> myposition,std::vector<double> mypoint){
    bool flag=0;
        if(getDistance(myposition,mypoint)>0.05){
            flag=1;
        }
    return flag;
};

//MXR::NOTE: limit the point number(by time)
bool isTimeallowed(std::vector<std::vector<double>> myposition,std::vector<double> mypoint){
     bool flag=0;
     //和前一个位置和时间都要有差别才可以
     if(((mypoint[4]-myposition[myposition.size()-1][4])>=1)&&isAccept(mypoint,myposition[myposition.size()-1])){
         flag = 1;
     }
     return flag;
}

//MXR::NOTE: limit the length of point
//加引用的作用???
std::vector<std::vector<double>> limitLength(std::vector<std::vector<double>> &vector2d,int length){
    if(vector2d.size()>length){
        for(int i =0;i<vector2d.size()-length;i++){
            std::vector<std::vector<double>>::iterator it = vector2d.begin();
            vector2d.erase(it);
        }

    }
//    std::cout<<"$$$$$$$$$$$$$$$$$$$$$$$$$"<<std::endl;
//    std::cout<<vector2d.size()<<std::endl;
//    std::cout<<"$$$$$$$$$$$$$$$$$$$$$$$$$"<<std::endl;
    return vector2d;
}
void contactCB(const sim_assiants::FootContacts::ConstPtr &contacts_msgs){


    //MXR::NOTE:
    //这种方法有一种缺陷，就是当接触消失的时候还有接触位置(todo)
    lf_wrench_.contact_positions.resize(1);
    lf_wrench_.contact_positions[0].x=contacts_msgs->foot_contacts[0].contact_position.vector.x;
    lf_wrench_.contact_positions[0].y=contacts_msgs->foot_contacts[0].contact_position.vector.y;
    lf_wrench_.contact_positions[0].z=contacts_msgs->foot_contacts[0].contact_position.vector.z;

    rf_wrench_.contact_positions.resize(1);
    rf_wrench_.contact_positions[0].x=contacts_msgs->foot_contacts[1].contact_position.vector.x;
    rf_wrench_.contact_positions[0].y=contacts_msgs->foot_contacts[1].contact_position.vector.y;
    rf_wrench_.contact_positions[0].z=contacts_msgs->foot_contacts[1].contact_position.vector.z;

    rh_wrench_.contact_positions.resize(1);
    rh_wrench_.contact_positions[0].x=contacts_msgs->foot_contacts[2].contact_position.vector.x;
    rh_wrench_.contact_positions[0].y=contacts_msgs->foot_contacts[2].contact_position.vector.y;
    rh_wrench_.contact_positions[0].z=contacts_msgs->foot_contacts[2].contact_position.vector.z;

    lh_wrench_.contact_positions.resize(1);
    lh_wrench_.contact_positions[0].x=contacts_msgs->foot_contacts[3].contact_position.vector.x;
    lh_wrench_.contact_positions[0].y=contacts_msgs->foot_contacts[3].contact_position.vector.y;
    lh_wrench_.contact_positions[0].z=contacts_msgs->foot_contacts[3].contact_position.vector.z;

    total_contact.states.resize(4);
    total_contact.states[0] = lf_wrench_;
    total_contact.states[1] = rf_wrench_;
    total_contact.states[2] = rh_wrench_;
    total_contact.states[3] = lh_wrench_;

    std::cout<<"lf_wrench_.total_wrench.force.z  "<<lf_wrench_.total_wrench.force.z<<std::endl;
    std::cout<<"rf_wrench_.total_wrench.force.z  "<<rf_wrench_.total_wrench.force.z<<std::endl;
    std::cout<<"rh_wrench_.total_wrench.force.z  "<<rh_wrench_.total_wrench.force.z<<std::endl;
    std::cout<<"lh_wrench_.total_wrench.force.z  "<<lh_wrench_.total_wrench.force.z<<std::endl;

    pub_force.publish(total_contact);
//    pub_lf_force.publish(lf_wrench_);
//    pub_lh_force.publish(lh_wrench_);
//    pub_rf_force.publish(rf_wrench_);
//    pub_rh_force.publish(rh_wrench_);

    //MXR::note:
    //we need initialize the 2d_vector first
    //my_position.push_back({0,0,0});

    //std::cout<<"callback!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
    //MXR::NOTE:
    //resize is very important!!!!!!!!!!!!!!!!!!
    foot_locations.FootLocations.resize(60);
    //std::cout<<"first foot################################"<<std::endl;
    if(contacts_msgs->foot_contacts[0].is_contact==1){
        std::vector<double> foot_contacts_0 = {contacts_msgs->foot_contacts[0].contact_position.vector.x,
                contacts_msgs->foot_contacts[0].contact_position.vector.y,
                contacts_msgs->foot_contacts[0].contact_position.vector.z,lf_wrench_.total_wrench.force.z,static_cast<double>(contacts_msgs->foot_contacts[0].contact_position.header.stamp.sec)};
        if(foot_location_0.size()==0&&foot_contacts_0[3]!=0){ //MXR::NOTE: the former contact force is zero(notice!!!!!)
            foot_location_0.push_back(foot_contacts_0);
        }
        else {
            if(isAccept(foot_location_0,foot_contacts_0)&&isTimeallowed(foot_location_0,foot_contacts_0)){
                foot_location_0.push_back(foot_contacts_0);
            }
        }
    }
    //std::cout<<"second foot################################"<<std::endl;
    if(contacts_msgs->foot_contacts[1].is_contact==1){
        std::vector<double> foot_contacts_1 = {contacts_msgs->foot_contacts[1].contact_position.vector.x,
                contacts_msgs->foot_contacts[1].contact_position.vector.y,
                contacts_msgs->foot_contacts[1].contact_position.vector.z,rf_wrench_.total_wrench.force.z,static_cast<double>(contacts_msgs->foot_contacts[1].contact_position.header.stamp.sec)};
        if(foot_location_1.size()==0&&foot_contacts_1[3]!=0){
            foot_location_1.push_back(foot_contacts_1);
        }
        else {
            if(isAccept(foot_location_1,foot_contacts_1)&&isTimeallowed(foot_location_1,foot_contacts_1)){
                foot_location_1.push_back(foot_contacts_1);
            }
        }
    }
    if(contacts_msgs->foot_contacts[2].is_contact==1){
        std::vector<double> foot_contacts_2 = {contacts_msgs->foot_contacts[2].contact_position.vector.x,
                contacts_msgs->foot_contacts[2].contact_position.vector.y,
                contacts_msgs->foot_contacts[2].contact_position.vector.z,rh_wrench_.total_wrench.force.z,static_cast<double>(contacts_msgs->foot_contacts[2].contact_position.header.stamp.sec)};
        if(foot_location_2.size()==0&&foot_contacts_2[3]!=0){
            foot_location_2.push_back(foot_contacts_2);
        }
        else {
            if(isAccept(foot_location_2,foot_contacts_2)&&isTimeallowed(foot_location_2,foot_contacts_2)){
                foot_location_2.push_back(foot_contacts_2);
            }
        }
    }
    if(contacts_msgs->foot_contacts[3].is_contact==1){
        std::vector<double> foot_contacts_3 = {contacts_msgs->foot_contacts[3].contact_position.vector.x,
                contacts_msgs->foot_contacts[3].contact_position.vector.y,
                contacts_msgs->foot_contacts[3].contact_position.vector.z,lh_wrench_.total_wrench.force.z,static_cast<double>(contacts_msgs->foot_contacts[2].contact_position.header.stamp.sec)};
        if(foot_location_3.size()==0&&foot_contacts_3[3]!=0){
            foot_location_3.push_back(foot_contacts_3);
        }
        else {
            if(isAccept(foot_location_3,foot_contacts_3)&&isTimeallowed(foot_location_3,foot_contacts_3)){
                foot_location_3.push_back(foot_contacts_3);
            }
        }

    }

    //MXR::NOTE
    //confine the length of foot_location
//    std::cout<<"###############before#############"<<std::endl;
//    std::cout<<foot_location_0[0][3]<<std::endl;
//    std::cout<<"############################"<<std::endl;
    limitLength(foot_location_0,15);
//    std::cout<<"###############after#############"<<std::endl;
//    std::cout<<foot_location_0[0][3]<<std::endl;
//    std::cout<<"############################"<<std::endl;
    limitLength(foot_location_1,15);
    limitLength(foot_location_2,15);
    limitLength(foot_location_3,15);
//    for(int j=0;j<foot_location_0.size();j++){
//        for (int m=0;m<foot_location_0[0].size();m++) {
//            std::cout<<foot_location_0[j][m]<<" ";

//        }
//        std::cout<<std::endl;
//    }

//    if(foot_location_0.size()>10){
//        for(int i =0;i<foot_location_0.size()-10;i++){
//            std::vector<std::vector<double>>::iterator it = foot_location_0.begin();
//            foot_location_0.erase(it);
//        }

//    }
//    std::cout<<"############################"<<std::endl;
//    std::cout<<000<<" "<<foot_location_0.size()<<std::endl;
//    std::cout<<"############################"<<std::endl;
//    std::cout<<"############################"<<std::endl;
//    std::cout<<111<<" "<<foot_location_1.size()<<std::endl;
//    std::cout<<"############################"<<std::endl;
//    std::cout<<"############################"<<std::endl;
//    std::cout<<222<<" "<<foot_location_2.size()<<std::endl;
//    std::cout<<"############################"<<std::endl;
//    std::cout<<"############################"<<std::endl;
//    std::cout<<333<<" "<<foot_location_3.size()<<std::endl;
//    std::cout<<"############################"<<std::endl;


    my_position.assign(foot_location_0.begin(),foot_location_0.begin()+foot_location_0.size());
    for (int ii=0;ii<foot_location_1.size();ii++) {
        my_position.push_back(foot_location_1[ii]);
    }
    for (int jj=0;jj<foot_location_2.size();jj++) {
        my_position.push_back(foot_location_2[jj]);
    }
    for (int kk=0;kk<foot_location_3.size();kk++){
        my_position.push_back(foot_location_3[kk]);
    }

    std::cout<<"vector size before limited"<<std::endl;
    std::cout<<my_position.size()<<std::endl;
    std::cout<<"$$$$$$$$$$$$$$$$"<<std::endl;
    //c++向vector插入元素的方法
    //assign 用于初始化
    //copy 好像必须要覆盖
    //copy(foot_location_0.begin(),foot_location_0.begin()+foot_location_0.size(),my_position.begin());
    //copy(foot_location_1.begin(),foot_location_1.begin()+foot_location_1.size(),my_position.begin()+my_position.size());
   // my_position.insert(my_position.begin()+15,foot_location_1.begin(),foot_location_1.begin()+foot_location_1.size());

    //MXR::note
    //just for  60 foot_positions
    if(my_position.size()>60){
        for(int i =0;i<my_position.size()-60;i++){
            std::vector<std::vector<double>>::iterator it = my_position.begin();
            my_position.erase(it);
        }

    }
    std::cout<<"vector size after limited"<<std::endl;
    std::cout<<my_position.size()<<std::endl;
    std::cout<<"$$$$$$$$$$$$$$$$"<<std::endl;
    //MXR::NOTE:
    //size=50，为啥我j<50就会报错呢？
    for (int j=0;j<my_position.size();j++) {
        foot_locations.FootLocations[j].vector.x = my_position[j][0];
        foot_locations.FootLocations[j].vector.y = my_position[j][1];
        foot_locations.FootLocations[j].vector.z = my_position[j][2];
        foot_locations.force.data = my_position[j][3];
//        std::cout<<"&&&&&&&&&&&&&&&&&&&&"<<std::endl;
//        std::cout<<my_position[j][3]<<std::endl;
//        std::cout<<"&&&&&&&&&&&&&&&&&&&&"<<std::endl;
    }
    foot_location_publisher.publish(foot_locations);

}


void lfCB(const geometry_msgs::WrenchStamped::ConstPtr &contacts_msgs){
    lf_wrench_.total_wrench.force = contacts_msgs->wrench.force;
    //lf_wrench_.contact_positions.push_back();
}
void rfCB(const geometry_msgs::WrenchStamped::ConstPtr &contacts_msgs){
    rf_wrench_.total_wrench.force = contacts_msgs->wrench.force;
}
void lhCB(const geometry_msgs::WrenchStamped::ConstPtr &contacts_msgs){
    lh_wrench_.total_wrench.force = contacts_msgs->wrench.force;
}
void rhCB(const geometry_msgs::WrenchStamped::ConstPtr &contacts_msgs){
    rh_wrench_.total_wrench.force = contacts_msgs->wrench.force;
}

int main(int argc, char **argv){
  ros::init (argc, argv, "get_foot_localization");

  ros::NodeHandle nh;
  nh.param("/lf_contact_topic", lf_contact_topic, std::string("/estimate_torque_lf"));
  nh.param("/rf_contact_topic", rf_contact_topic, std::string("/estimate_torque_rf"));
  nh.param("/lh_contact_topic", lh_contact_topic, std::string("/estimate_torque_lh"));
  nh.param("/rh_contact_topic", rh_contact_topic, std::string("/estimate_torque_rh"));
  nh.param("/FootContacts", FootContacts_topic, std::string("/bumper_sensor_filter_node/foot_contacts"));


  ros::Subscriber sub_contact = nh.subscribe<sim_assiants::FootContacts>(FootContacts_topic,100, contactCB);
  ros::Subscriber sub_lf_force = nh.subscribe<geometry_msgs::WrenchStamped>(lf_contact_topic,100,lfCB);
  ros::Subscriber sub_rf_force = nh.subscribe<geometry_msgs::WrenchStamped>(rf_contact_topic,100,rfCB);
  ros::Subscriber sub_rh_force = nh.subscribe<geometry_msgs::WrenchStamped>(rh_contact_topic,100,rhCB);
  ros::Subscriber sub_lh_force = nh.subscribe<geometry_msgs::WrenchStamped>(lh_contact_topic,100,lhCB);
//  pub_lf_force = nh.advertise<gazebo_msgs::ContactState>("/real_lf_contact_force",100);
//  pub_rf_force = nh.advertise<gazebo_msgs::ContactState>("/real_rf_contact_force",100);
//  pub_rh_force = nh.advertise<gazebo_msgs::ContactState>("/real_rh_contact_force",100);
//  pub_lh_force = nh.advertise<gazebo_msgs::ContactState>("/real_lh_contact_force",100);
  //pub_force = nh.advertise<gazebo_msgs::ContactsState>("/total_contact",100);

  foot_location_publisher=nh.advertise<sim_assiants::FootLocations>("/gazebo/foot_location",100);
  //foot_locations.FootLocations.resize(50);
  ros::Rate loop_rate(5);
  while (ros::ok())
  {
      for(int j=0;j<my_position.size();j++){
          for (int m=0;m<my_position[0].size();m++) {
              std::cout<<my_position[j][m]<<" ";

          }
          std::cout<<std::endl;
      }

//      std::cout<<"*********************"<<std::endl;
      ros::spinOnce();
      loop_rate.sleep();
  }


  //ros::spin();

  return 0;
}
