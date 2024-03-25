#pragma once
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

namespace gazebo
{
    class LinkControlPluginPrivate;
    class LinkControlPlugin : public ModelPlugin
    {
    public:
        LinkControlPlugin();
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        virtual void OnUpdate();

    private:
        void OnRequestMsg(ConstGzStringPtr &_msg);
        std::shared_ptr<LinkControlPluginPrivate> dataPtr;
    };
}