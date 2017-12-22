#ifndef _JOINT_CONTROLLER_HH_
#define _JOINT_CONTROLLER_HH_

#include <thread>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "ros/callback_queue.h"

namespace gazebo
{
  class JointController : public ModelPlugin
  {
    public: JointController() {}
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    private: ros::Subscriber rosSub;
    private: ros::CallbackQueue rosQueue;
    private: std::thread rosQueueThread;

    private: physics::ModelPtr model;
    private: physics::JointPtr piston[6];
    private: common::PID pid;

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      int i;

      if (_model->GetJointCount() == 0)
      {
        std::cerr << "Invalid joint count, JointController plugin not loaded\n";
        return;
      }
      else
      {
        std::cout << "\nThe JointController plugin is attach to model[" << _model->GetName() << "]\n";
      }

      this->model = _model;
      this->pid = common::PID(1000.0, 0.1, 100.0);
      for(i = 0; i < 6; i++)
      {
        this->piston[i] = _model->GetJoints()[i];
        this->model->GetJointController()->SetPositionPID(this->piston[i]->GetScopedName(), this->pid);
      }

      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
            ros::init_options::NoSigintHandler);
      }

      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
      ros::SubscribeOptions so =ros::SubscribeOptions::create<std_msgs::Float32MultiArray>("/" + this->model->GetName() + "/piston_cmd", 
                                                                                          100, 
                                                                                          boost::bind(&JointController::SetPosition, this, _1),
                                                                                          ros::VoidPtr(), &this->rosQueue);
      
      this->rosSub = this->rosNode->subscribe(so);
      this->rosQueueThread = std::thread(std::bind(&JointController::QueueThread, this));
    }

    public: void SetPosition(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
      for(int i = 0; i < 6; i++)
      {
        this->model->GetJointController()->SetPositionTarget(this->piston[i]->GetScopedName(), msg->data[i]);
      }
    }

    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

  };

  GZ_REGISTER_MODEL_PLUGIN(JointController)
}
#endif