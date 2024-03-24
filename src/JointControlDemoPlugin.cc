
#include <iostream>
#include "gazebo/transport/transport.hh"
#include "JointControlDemoPlugin.hh"
#include <gazebo/sensors/sensors.hh>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(JointControlPlugin)

JointControlPlugin::JointControlPlugin()
{
}

void JointControlPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(_model->GetWorld()->Name());

  auto topic = std::string("~/") + _model->GetName() + "/command";
  this->sub = this->node->Subscribe(topic,
                                    &JointControlPlugin::OnRequestMsg, this);

  auto pub_topic = std::string("~/") + _model->GetName() + "/state";
  this->pub = this->node->Advertise<gazebo::msgs::GzString>(pub_topic);

  std::string jointName = "tilt_joint";
  if (_sdf->HasElement("joint"))
  {
    jointName = _sdf->Get<std::string>("joint");
  }
  this->joint = _model->GetJoint(jointName);
  gzmsg << "joint name: " << this->joint->GetName() << std::endl;
  gzmsg << "joint name: " << this->joint->GetScopedName() << std::endl;

  std::string imuSensorName = "roll_imu";
  if (_sdf->HasElement("imu_sensor"))
  {
    imuSensorName = _sdf->Get<std::string>("imu_sensor");
  }

  this->imuSensor = std::static_pointer_cast<sensors::ImuSensor>(
      sensors::SensorManager::Instance()->GetSensor(_model->SensorScopedName(imuSensorName)[0]));

  gzmsg << "Sensor name:" << this->imuSensor->Name() << std::endl;
}

void JointControlPlugin::OnRequestMsg(ConstGzStringPtr &_msg)
{
  auto command = atof(_msg->data().c_str());
  this->joint->SetPosition(0, command, false);
  auto roll = this->imuSensor->Orientation().Euler().X(); 
  gzmsg << "roll" << std::endl;

  gzmsg << _msg->data() << std::endl;
  gazebo::msgs::GzString m;
  m.set_data(std::to_string(roll));
  this->pub->Publish(m);
}


// /gazebo/default/stand_with_gimbal/command
// 