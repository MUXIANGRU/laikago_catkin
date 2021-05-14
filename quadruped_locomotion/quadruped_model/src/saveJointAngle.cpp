/*
 *  kinematicsTest.cpp
 *  Descriotion:
 *
 *  Created on: date, 2019
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */

#include "quadruped_model/quadrupedkinematics.h"
#include "rbdl/Model.h"
#include "rbdl/addons/urdfreader/urdfreader.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
//#include "curves/ScalarCurveConfig.hpp"
//#include "curves/PolynomialSplineContainer.hpp"
//#include "curves/polynomial_splines_containers.hpp"
//#include "curves/PolynomialSplineScalarCurve.hpp"
//#include "curves/PolynomialSplineVectorSpaceCurve.hpp"

#include "iostream"
#include "fstream"
#include "sstream"


using namespace std;
using namespace quadruped_model;
ros::Subscriber js_sub;
quadruped_model::JointPositions joint_positions;
std::vector<JointPositionsLimb> lf_joint_positions;
std::vector<JointPositionsLimb> rf_joint_positions;
std::vector<JointPositionsLimb> lh_joint_positions;
std::vector<JointPositionsLimb> rh_joint_positions;
std::vector<quadruped_model::JointPositions> joint_positions_total;

void SaveAsFile(const std::vector<JointPositions>& joint_position_vector)
{
    std::ofstream invFile;
    std::cout << "Saving the data" << std::endl;
    std::string file_name;
    file_name = "calculated_crawl.txt";
    invFile.open(file_name);
    if(invFile.fail())
    {
        std::cerr << "The file cannot be opened!";
    }

    double time_ = 0;
    for (unsigned int i = 0; i < joint_position_vector.size(); i++) {
        invFile <<time_ << " " << joint_position_vector[i] << " ";
        invFile << "\r\n";
        time_ = time_ + 0.0025;
    }
    invFile.close();
    file_name = "";

    std::cout << "success store the file " << std::endl;
}

void jsCB(const sensor_msgs::JointState::ConstPtr& jsmsg){

//    for(unsigned int i = 0; i < lf_joint_positions.size(); i++)
//      {
        joint_positions(0) = jsmsg->position[0];//LF
        joint_positions(1) = jsmsg->position[1];
        joint_positions(2) = jsmsg->position[2];

        joint_positions(6) = jsmsg->position[9];//RH
        joint_positions(7) = jsmsg->position[10];
        joint_positions(8) = jsmsg->position[11];

        joint_positions(3) = jsmsg->position[6];//RF
        joint_positions(4) = jsmsg->position[7];
        joint_positions(5) = jsmsg->position[8];

        joint_positions(9) = jsmsg->position[3];//LH
        joint_positions(10) = jsmsg->position[4];
        joint_positions(11) = jsmsg->position[5];

        joint_positions_total.push_back(joint_positions);
        std::cout << "size is " << joint_positions_total.size() << std::endl;
        SaveAsFile(joint_positions_total);

//      }
}
int main(int argc, char **argv)
{

    ros::init(argc, argv, "saveJointAngle");
    ros::NodeHandle nh;

    js_sub = nh.subscribe<sensor_msgs::JointState>("/joint_states",1,&jsCB);


//  std::cout << "let us begin" << std::endl;

    while(ros::ok())
    {
        ros::spin();
    }
    return 0;




//    std::ifstream readfile;
//    readfile.open("calculated.txt");
//    quadruped_model::JointPositions joint_position_file;
//    std::vector<quadruped_model::JointPositions> joint_position_collection;
//    double time;
//    for(int i = 0; !readfile.eof(); i++)
//      {
//         readfile >> time >> joint_position_file(0) >> joint_position_file(1)>> joint_position_file(2)
//                                                       >> joint_position_file(3)>> joint_position_file(4)
//                                                          >> joint_position_file(5)>> joint_position_file(6)
//                                                             >> joint_position_file(7)>> joint_position_file(8)
//                                                                >> joint_position_file(9)>> joint_position_file(10)
//                                                                   >> joint_position_file(11);
//         joint_position_collection.push_back(joint_position_file);
////         std::cout << time <<" " << joint_position_file << std::endl;
//      }
//    readfile.close();
//    joint_position_collection.pop_back();


}
