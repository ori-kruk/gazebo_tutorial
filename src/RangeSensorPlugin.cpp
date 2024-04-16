#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

namespace gazebo
{
 class RangeSensorPlugin : public SensorPlugin
 {
    public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
    {
      // Initialize the sensor
      this->parentSensor = std::dynamic_pointer_cast<sensors::RaySensor>(_sensor);
      if (!this->parentSensor)
      {
        gzerr << "RangeSensorPlugin requires a RaySensor.\n";
        return;
      }

      // Connect to the sensor's update event
      this->updateConnection = this->parentSensor->ConnectUpdated(
          std::bind(&RangeSensorPlugin::OnUpdate, this));
    }

    public: void OnUpdate()
    {
      // Get the range data
      auto rangeData = this->parentSensor->Range(0);

      // Print the range data
      gzmsg << "Range: " << rangeData << std::endl;
    }

    private: sensors::RaySensorPtr parentSensor;
    private: event::ConnectionPtr updateConnection;
 };

 GZ_REGISTER_SENSOR_PLUGIN(RangeSensorPlugin)
}
