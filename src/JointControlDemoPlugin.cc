
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
  physics::JointPtr base_joint;
  physics::ModelPtr model;
  std::vector<event::ConnectionPtr> connections;
  common::Time lastUpdateTime;
  double roll_command=0.0;

};

JointControlPlugin::JointControlPlugin()
    : dataPtr(new JointControlPluginPrivate)
{
    this->dataPtr->roll_pid.Init(30.0, 3.0, 15.0, 1, -1, 10.0, -10.0);

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

  sdf::ElementPtr controlSDF;
  if (_sdf->HasElement("control"))
  {
    controlSDF = _sdf->GetElement("control");
  }

  if (controlSDF){
      this->dataPtr->roll_pid.SetPGain(controlSDF->Get<double>("p"));
      this->dataPtr->roll_pid.SetIGain(controlSDF->Get<double>("i"));
      this->dataPtr->roll_pid.SetDGain(controlSDF->Get<double>("d"));

      gzmsg << this->dataPtr->roll_pid.GetPGain() << std::endl;
  }

  std::string jointName = "tilt_joint";
  if (_sdf->HasElement("joint"))
  {
    jointName = _sdf->Get<std::string>("joint");
  }
  this->dataPtr->roll_joint = _model->GetJoint(jointName);
  gzmsg << "joint name: " << this->dataPtr->roll_joint->GetName() << std::endl;
  gzmsg << "joint name: " << this->dataPtr->roll_joint->GetScopedName() << std::endl;

  // this->dataPtr->base_joint = _model->GetJoint("joint1");
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
  this->dataPtr->roll_command = atof(_msg->data().c_str());
  gzmsg << "set command: " << this->dataPtr->roll_command << std::endl;
}

void JointControlPlugin::OnUpdate()
{
  if (!this->dataPtr->roll_joint)
  {
    gzerr << "joint not found" << std::endl;
    return;
  }
  // this->dataPtr->base_joint->SetPosition(0,0);
  common::Time time = this->dataPtr->model->GetWorld()->SimTime();
  if (time < this->dataPtr->lastUpdateTime)
  {
    this->dataPtr->lastUpdateTime = time;
    return;
  }
  double dt = (time - this->dataPtr->lastUpdateTime).Double();
  double roll_angle = this->dataPtr->roll_joint->Position(0);
  double roll_error = roll_angle - this->dataPtr->roll_command;
  double roll_force = this->dataPtr->roll_pid.Update(roll_error, dt);
  this->dataPtr->roll_joint->SetForce(0, roll_force);
  // gzmsg << "force: " << roll_force << std::endl;
  this->dataPtr->lastUpdateTime = time;
}

// /gazebo/default/stand_with_gimbal/command
//