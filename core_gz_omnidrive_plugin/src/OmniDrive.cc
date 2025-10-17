// https://qiita.com/srs/items/2675e9dcbb942c979c3c
// https://gazebosim.org/api/gazebo/6/createsystemplugins.html

#include "OmniDrive.hh"

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/time.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>

#include <limits>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <gz/math/Quaternion.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Velocity command.
struct Commands
{
    /// \brief Linear velocity.
    double lin;

    /// \brief Angular velocity.
    double ang;

    Commands() : lin(0.0), ang(0.0) {}
};

class OmniDrivePrivate
{
    /// \brief Callback for velocity subscription
    /// \param[in] _msg Velocity message
public:
    void OnCmdVel(const msgs::Twist &_msg);

    /// \brief Callback for enable/disable subscription
    /// \param[in] _msg Boolean message
public:
    void OnEnable(const msgs::Boolean &_msg);

    /// \brief Update odometry and publish an odometry message.
    /// \param[in] _info System update information.
    /// \param[in] _ecm The EntityComponentManager of the given simulation
    /// instance.
public:
    void UpdateOdometry(const UpdateInfo &_info,
                        const EntityComponentManager &_ecm);

    /// \brief Update the linear and angular velocities.
    /// \param[in] _info System update information.
    /// \param[in] _ecm The EntityComponentManager of the given simulation
    /// instance.
public:
    void UpdateVelocity(const UpdateInfo &_info,
                        const EntityComponentManager &_ecm);

    /// \brief Gazebo communication node.
public:
    transport::Node node;

    /// \brief Model interface
public:
    Model model{kNullEntity};

    /// \brief Last target velocity requested.
public:
    msgs::Twist targetVel;

    /// \brief Enable/disable state of the controller.
public:
    bool enabled;

    /// \brief A mutex to protect the target velocity command.
public:
    std::mutex mutex;

};

//////////////////////////////////////////////////
OmniDrive::OmniDrive()
    : dataPtr(std::make_unique<OmniDrivePrivate>())
{
}

//////////////////////////////////////////////////
void OmniDrive::Configure(const Entity &_entity,
                          const std::shared_ptr<const sdf::Element> &_sdf,
                          EntityComponentManager &_ecm,
                          EventManager & eventMgr)
{
    this->dataPtr->model = Model(_entity);

    // Subscribe to commands
    std::vector<std::string> topics;
    if (_sdf->HasElement("topic"))
    {
        topics.push_back(_sdf->Get<std::string>("topic"));
    }
    topics.push_back("/cmd_vel");
    auto topic = validTopic(topics);

    this->dataPtr->node.Subscribe(topic, &OmniDrivePrivate::OnCmdVel, this->dataPtr.get());

    // Subscribe to enable/disable
    std::vector<std::string> enableTopics;
    enableTopics.push_back("/enable");
    auto enableTopic = validTopic(enableTopics);

    if (!enableTopic.empty())
    {
        this->dataPtr->node.Subscribe(enableTopic, &OmniDrivePrivate::OnEnable,this->dataPtr.get());
    }
    this->dataPtr->enabled = true;
}

//////////////////////////////////////////////////
void OmniDrive::PreUpdate(const UpdateInfo &_info,
                          EntityComponentManager &_ecm)
{
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    if (!this->dataPtr->enabled)
        return;

    auto linear = math::Vector3d(
        this->dataPtr->targetVel.linear().x(),
        this->dataPtr->targetVel.linear().y(),
        this->dataPtr->targetVel.linear().z());

    auto angular = math::Vector3d(
        this->dataPtr->targetVel.angular().x(),
        this->dataPtr->targetVel.angular().y(),
        this->dataPtr->targetVel.angular().z());

    auto modelEntity = this->dataPtr->model.Entity();
    auto baseLinkEntity = this->dataPtr->model.LinkByName(_ecm, "base_link");

    auto linCmd = _ecm.Component<gz::sim::components::LinearVelocityCmd>(baseLinkEntity);
    if (linCmd == nullptr)
        _ecm.CreateComponent(baseLinkEntity, gz::sim::components::LinearVelocityCmd(linear));
    else
        *linCmd = gz::sim::components::LinearVelocityCmd(linear);

    // Angular velocity command
    auto angCmd = _ecm.Component<gz::sim::components::AngularVelocityCmd>(baseLinkEntity);
    if (angCmd == nullptr)
        _ecm.CreateComponent(baseLinkEntity, gz::sim::components::AngularVelocityCmd(angular));
    else
        *angCmd = gz::sim::components::AngularVelocityCmd(angular);
}

//////////////////////////////////////////////////
void OmniDrive::PostUpdate(const UpdateInfo &_info,
                           const EntityComponentManager &_ecm)
{

}

//////////////////////////////////////////////////
void OmniDrivePrivate::OnCmdVel(const msgs::Twist &_msg)
{
    std::lock_guard<std::mutex> lock(this->mutex);
    if (this->enabled)
    {
        this->targetVel = _msg;
        printf("On cmd_vel: linear=%.2f, angular=%.2f\n",
               _msg.linear().x(), _msg.angular().z());
    }
}

//////////////////////////////////////////////////
void OmniDrivePrivate::OnEnable(const msgs::Boolean &_msg)
{
    std::lock_guard<std::mutex> lock(this->mutex);
    this->enabled = _msg.data();
    if (!this->enabled)
    {
        math::Vector3d zeroVector{0, 0, 0};
        msgs::Set(this->targetVel.mutable_linear(), zeroVector);
        msgs::Set(this->targetVel.mutable_angular(), zeroVector);
    }
}

IGNITION_ADD_PLUGIN(OmniDrive,
              gz::sim::System,
              OmniDrive::ISystemConfigure,
              OmniDrive::ISystemPreUpdate,
              OmniDrive::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(OmniDrive, "CoreOmniDrive")