#ifndef __GZ_OMNIDRIVE_HH__
#define __GZ_OMNIDRIVE_HH__
#include <memory>
#include <gz/sim/System.hh>

class OmniDrivePrivate;

class OmniDrive: public gz::sim::System, public gz::sim::ISystemConfigure,
public gz::sim::ISystemPreUpdate, public gz::sim::ISystemPostUpdate
{
public:
  OmniDrive();

  ~OmniDrive() override = default;

  void Configure(
    const gz::sim::Entity & _entity,
    const std::shared_ptr < const sdf::Element > & _sdf,
    gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager & _eventMgr) override;

  void PreUpdate(
    const gz::sim::UpdateInfo & _info,
    gz::sim::EntityComponentManager & _ecm) override;

  void PostUpdate(
    const gz::sim::UpdateInfo & _info,
    const gz::sim::EntityComponentManager & _ecm) override;

private:
  std::unique_ptr < OmniDrivePrivate > dataPtr;
};

#endif
