

/*
https://github.com/gazebosim/gazebo-classic/tree/gazebo11/examples/plugins/model_move
*/
#include <iostream>
#include "gazebo/transport/transport.hh"
#include "LinkControlPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(LinkControlPlugin)

class gazebo::LinkControlPluginPrivate
{
public:
  physics::ModelPtr model_ptr;
  physics::LinkPtr link_ptr;
  gazebo::event::ConnectionPtr on_update_handler;
};

LinkControlPlugin::LinkControlPlugin() : dataPtr(new LinkControlPluginPrivate)
{
}

void LinkControlPlugin::OnUpdate(){
  this->dataPtr->link_ptr->SetLinearVel({0,1,0});
}

void LinkControlPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  gzmsg << "Link control 0.1" << std::endl;
  this->dataPtr->model_ptr = _model;
  this->dataPtr->link_ptr = _model->GetLink("cam_link");

this->dataPtr->on_update_handler = event::Events::ConnectWorldUpdateBegin(
      std::bind(&LinkControlPlugin::OnUpdate, this));
}
