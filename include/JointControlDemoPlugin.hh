#pragma once
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

namespace gazebo
{
    class JointControlPluginPrivate;
    class JointControlPlugin : public ModelPlugin
    {
    public:
        JointControlPlugin();
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        

    private:
        void OnRequestMsg(ConstGzStringPtr &_msg);
        transport::NodePtr node;
        transport::PublisherPtr pub;
        transport::SubscriberPtr sub;
        sensors::ImuSensorPtr imuSensor;
        physics::JointPtr joint;
        std::shared_ptr<JointControlPluginPrivate> dataPtr;
         void OnUpdate();
    };
}