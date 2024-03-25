
#include <iostream>
#include "gazebo/transport/transport.hh"
#include "JointControlDemoPlugin.hh"
#include <gazebo/sensors/sensors.hh>
#include <vector>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(JointControlPlugin)

class gazebo::JointControlPluginPrivate
{
public:
  common::PID roll_pid;
  physics::JointPtr roll_joint;
  physics::ModelPtr model;
  std::vector<event::ConnectionPtr> connections;
  common::Time lastUpdateTime;
  double roll_command;
};

JointControlPlugin::JointControlPlugin()
    : dataPtr(new JointControlPluginPrivate)
{
    this->dataPtr->roll_pid.Init(10, 0.001, -1, 0, 0, 100.0, -100.0);

  this->dataPtr->connections.push_back(event::Events::ConnectWorldUpdateBegin(
      std::bind(&JointControlPlugin::OnUpdate, this)));
}

void JointControlPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->dataPtr->model = _model;
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
  this->dataPtr->roll_joint = _model->GetJoint(jointName);
  gzmsg << "joint name: " << this->dataPtr->roll_joint->GetName() << std::endl;
  gzmsg << "joint name: " << this->dataPtr->roll_joint->GetScopedName() << std::endl;

  // std::string imuSensorName = "roll_imu";
  // if (_sdf->HasElement("imu_sensor"))
  // {
  //   imuSensorName = _sdf->Get<std::string>("imu_sensor");
  // }

  // this->imuSensor = std::static_pointer_cast<sensors::ImuSensor>(
  //     sensors::SensorManager::Instance()->GetSensor(_model->SensorScopedName(imuSensorName)[0]));

  // gzmsg << "Sensor name:" << this->imuSensor->Name() << std::endl;
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

void JointControlPlugin::OnUpdate()
{
  if (!this->dataPtr->roll_joint)
  {
    gzerr << "joitn not found" << std::endl;
    return;
  }
  common::Time time = this->dataPtr->model->GetWorld()->SimTime();
  if (time < this->dataPtr->lastUpdateTime)
  {
    this->dataPtr->lastUpdateTime = time;
    return;
  }
  double dt = (this->dataPtr->lastUpdateTime - time).Double();
  double roll_angle = this->dataPtr->roll_joint->Position(0);
  double roll_error = roll_angle - this->dataPtr->roll_command;
  double roll_force = this->dataPtr->roll_pid.Update(roll_error, dt);
  this->dataPtr->roll_joint->SetForce(0, roll_force);

  this->dataPtr->lastUpdateTime = time;
}

// /gazebo/default/stand_with_gimbal/command
//