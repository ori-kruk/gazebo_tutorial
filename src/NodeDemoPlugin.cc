
#include <iostream>
#include "gazebo/transport/transport.hh"
#include "SimpleModelPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(NodeDemoPlugin)

NodeDemoPlugin::NodeDemoPlugin()
{
}

void NodeDemoPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(_model->GetWorld()->Name());

    auto topic = std::string("~/") +  _model->GetName() + "/command";
    this->sub = this->node->Subscribe(topic,
      &NodeDemoPlugin::OnRequestMsg, this);

    auto pub_topic = std::string("~/") +  _model->GetName() + "/state";
    this->pub = this->node->Advertise<gazebo::msgs::GzString>(pub_topic);
}

void NodeDemoPlugin::OnRequestMsg(ConstGzStringPtr &_msg)
{
    gzmsg << _msg->data() << std::endl;
    gazebo::msgs::GzString m;
    m.set_data(_msg->data() + "_echo");
    this->pub->Publish(m);
}