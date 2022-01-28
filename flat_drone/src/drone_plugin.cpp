#include <iostream>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

class DronePlugin : public gazebo::ModelPlugin {
public:
  DronePlugin() : gazebo::ModelPlugin() {
    std::cout << "Starting drone_plugin" << std::endl;
  }
 
  virtual ~DronePlugin() {
    std::cout << "Closing drone_plugin" << std::endl;
  }

  void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf) {
    _model = parent;
  }
 
private:
  gazebo::physics::ModelPtr _model;

};
 
GZ_REGISTER_MODEL_PLUGIN(DronePlugin)