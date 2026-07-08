#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/plugin/Register.hh>

using namespace gz;
using namespace sim;
using namespace systems;

class MimicJointPlugin :
  public System,
  public ISystemConfigure,
  public ISystemPreUpdate
{
  Model model;

  Joint masterJoint;
  Joint slaveJoint;

  double multiplier{1.0};
  double offset{0.0};

public:
  void Configure(const Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 EntityComponentManager &ecm,
                 EventManager &) override
  {
    this->model = Model(entity);

    std::string masterName = sdf->Get<std::string>("master_joint");
    std::string slaveName  = sdf->Get<std::string>("slave_joint");

    if (sdf->HasElement("multiplier"))
      multiplier = sdf->Get<double>("multiplier");

    if (sdf->HasElement("offset"))
      offset = sdf->Get<double>("offset");

    this->masterJoint = Joint(this->model.JointByName(ecm, masterName));
    this->slaveJoint  = Joint(this->model.JointByName(ecm, slaveName));
  }

  void PreUpdate(const UpdateInfo &,
                 EntityComponentManager &ecm) override
  {
    if (!masterJoint.Valid(ecm) || !slaveJoint.Valid(ecm))
      return;

    double masterPos = masterJoint.Position(ecm).value_or(0.0);

    double target = masterPos * multiplier + offset;

    slaveJoint.SetPosition(ecm, 0, target);
  }
};

GZ_ADD_PLUGIN(MimicJointPlugin,
              System,
              MimicJointPlugin::ISystemConfigure,
              MimicJointPlugin::ISystemPreUpdate)