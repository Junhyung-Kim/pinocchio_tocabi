#include <pinocchio_tocabi/pinocchio_node.h>
#include <ros/ros.h>
#include <iostream>

pinocchio::JointModelFreeFlyer root_joint;

ros::Publisher modelpin_pub;
ros::Subscriber joint_sub;
Eigen::VectorQVQd q; 
Eigen::VectorVQd qdot, qddot, qdot_, qddot_;
Eigen::MatrixXd CMM;
pinocchio_tocabi::model model_msg;
bool first_sub = false;


void JointCallback(const sensor_msgs::JointState &msg)
{
  first_sub = true;
  for(int i = 0; i<40; i++)
  {
      q(i) = msg.position[i];
  }
  for(int i = 0; i<39; i++)
  {
      qdot(i) = msg.velocity[i];
  }
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "pinocchio_node");
    ros::NodeHandle n;
    using namespace pinocchio;
    
    joint_sub = n.subscribe("/tocabi/pinocchio/jointstates", 1, JointCallback);
    modelpin_pub = n.advertise<pinocchio_tocabi::model>("/tocabi/pinocchio",1);

    pinocchio::Model model;
    pinocchio::urdf::buildModel("/home/jhk/catkin_ws/src/dyros_tocabi/tocabi_description/robots/dyros_tocabi.urdf", root_joint, model);

    pinocchio::Data data(model);
   
    q = randomConfiguration(model);
    qdot = Eigen::VectorXd::Zero(model.nv);
    qddot = Eigen::VectorXd::Zero(model.nv);
    qdot_ = Eigen::VectorXd::Zero(model.nv);
    qddot_ = Eigen::VectorXd::Zero(model.nv);
    
    CMM = pinocchio::computeCentroidalMap(model, data, q);
    pinocchio::crba(model, data, q);
    pinocchio::computeCoriolisMatrix(model, data, q, qdot);
    pinocchio::rnea(model, data, q, qdot_, qddot_);

    for(int i = 0; i<6; i++)
    {
      for(int j=0; j<33; j++)
      {
        model_msg.CMM.push_back(0.0);
      }
    }
    
    for(int i = 0; i<33; i++)
    {
      for(int j=0; j<33; j++)
      {
        model_msg.COR.push_back(0.0);
        model_msg.M.push_back(0.0);
      }
    }
    for(int i = 0; i<33; i++)
    {
      model_msg.g.push_back(0.0);//[i] = data.tau(i);
    }

    ros::Rate loop_rate(1000);
 
    while(ros::ok())
    {
      CMM = pinocchio::computeCentroidalMap(model, data, q);
      pinocchio::crba(model, data, q);
      pinocchio::computeCoriolisMatrix(model, data, q, qdot);
      pinocchio::rnea(model, data, q, qdot_, qddot_);

      for(int i = 0; i<6; i++)
      {
        for(int j=0; j<33; j++)
        {
          model_msg.CMM[33*i+j] = CMM(i,j+6);
        }
      }

      for(int i = 0; i<33; i++)
      {
        for(int j=0; j<33; j++)
        {
          model_msg.COR[33*i+j] = data.C(i,j+6);
        }
      }

      for(int i = 0; i<33; i++)
      {
        for(int j=0; j<33; j++)
        {
          model_msg.M[33*i+j] = data.M(i,j+6);
        }
      }

      for(int i = 0; i<33; i++)
      {
        model_msg.g[i] = data.tau(i);
      }
      if(first_sub == true)
        modelpin_pub.publish(model_msg);

      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
}

