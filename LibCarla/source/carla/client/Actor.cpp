// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/client/Actor.h"

#include "carla/Logging.h"
#include "carla/client/detail/Simulator.h"

namespace carla {
namespace client {
//获取演员（Actor）的位置
  geom::Location Actor::GetLocation() const {
    //通过锁定当前情节（Episode）并获取当前演员的位置
    return GetEpisode().Lock()->GetActorLocation(*this);

  }

  geom::Transform Actor::GetTransform() const {
    //获取演员（Actor）的变换（包括位置和朝向）
    return GetEpisode().Lock()->GetActorTransform(*this);
       //获取演员（Actor）的速度
  }

  geom::Vector3D Actor::GetVelocity() const {
    return GetEpisode().Lock()->GetActorVelocity(*this);
       //获取演员（Actor）的角速度
  }

  geom::Vector3D Actor::GetAngularVelocity() const {
    return GetEpisode().Lock()->GetActorAngularVelocity(*this);
       //获取演员（Actor）的加速度
  }
   //获取演员（Actor）特定组件在世界坐标系中的变换
  geom::Vector3D Actor::GetAcceleration() const {
    return GetEpisode().Lock()->GetActorAcceleration(*this);
  }
//获取演员（Actor）特定组件在相对坐标系中的变换
  geom::Transform Actor::GetComponentWorldTransform(const std::string componentName) const {
    return GetEpisode().Lock()->GetActorComponentWorldTransform(*this, componentName);
  }
 //获取演员（Actor）所有骨骼在世界坐标系中的变换
  geom::Transform Actor::GetComponentRelativeTransform(const std::string componentName) const {
    return GetEpisode().Lock()->GetActorComponentRelativeTransform(*this, componentName);
  }
 //获取演员（Actor）所有骨骼在相对坐标系中的变换
  std::vector<geom::Transform> Actor::GetBoneWorldTransforms() const {
    return GetEpisode().Lock()->GetActorBoneWorldTransforms(*this);
  }
 //获取演员（Actor）所有组件的名称
  std::vector<geom::Transform> Actor::GetBoneRelativeTransforms() const {
    return GetEpisode().Lock()->GetActorBoneRelativeTransforms(*this);
  }
 //获取演员（Actor）所有骨骼的名称
  std::vector<std::string> Actor::GetComponentNames() const {
    return GetEpisode().Lock()->GetActorComponentNames(*this);
  }
// 获取演员（Actor）所有套接字（Socket）在世界坐标系中的变换
  std::vector<std::string> Actor::GetBoneNames() const {
    return GetEpisode().Lock()->GetActorBoneNames(*this);
  } 
  // 获取演员（Actor）所有套接字（Socket）在相对坐标系中的变换
  std::vector<geom::Transform> Actor::GetSocketWorldTransforms() const {
    return GetEpisode().Lock()->GetActorSocketWorldTransforms(*this);
  }
// 获取演员（Actor）所有套接字（Socket）的名称
  std::vector<geom::Transform> Actor::GetSocketRelativeTransforms() const {
    return GetEpisode().Lock()->GetActorSocketRelativeTransforms(*this);
  }

  std::vector<std::string> Actor::GetSocketNames() const {
    return GetEpisode().Lock()->GetActorSocketNames(*this);
  }  

  void Actor::SetLocation(const geom::Location &location) {
    GetEpisode().Lock()->SetActorLocation(*this, location);
  }

  void Actor::SetTransform(const geom::Transform &transform) {
    GetEpisode().Lock()->SetActorTransform(*this, transform);
  }

  void Actor::SetTargetVelocity(const geom::Vector3D &vector) {
    GetEpisode().Lock()->SetActorTargetVelocity(*this, vector);
  }

  void Actor::SetTargetAngularVelocity(const geom::Vector3D &vector) {
    GetEpisode().Lock()->SetActorTargetAngularVelocity(*this, vector);
  }

  void Actor::EnableConstantVelocity(const geom::Vector3D &vector) {
    GetEpisode().Lock()->EnableActorConstantVelocity(*this, vector);
  }

  void Actor::DisableConstantVelocity() {
    GetEpisode().Lock()->DisableActorConstantVelocity(*this);
  }

  void Actor::AddImpulse(const geom::Vector3D &impulse) {
    GetEpisode().Lock()->AddActorImpulse(*this, impulse);
  }

  void Actor::AddImpulse(const geom::Vector3D &impulse, const geom::Vector3D &location) {
    GetEpisode().Lock()->AddActorImpulse(*this, impulse, location);
  }

  void Actor::AddForce(const geom::Vector3D &force) {
    GetEpisode().Lock()->AddActorForce(*this, force);
  }

  void Actor::AddForce(const geom::Vector3D &force, const geom::Vector3D &location) {
    GetEpisode().Lock()->AddActorForce(*this, force, location);
  }

  void Actor::AddAngularImpulse(const geom::Vector3D &vector) {
    GetEpisode().Lock()->AddActorAngularImpulse(*this, vector);
  }

  void Actor::AddTorque(const geom::Vector3D &torque) {
    GetEpisode().Lock()->AddActorTorque(*this, torque);
  }

  void Actor::SetSimulatePhysics(const bool enabled) {
    GetEpisode().Lock()->SetActorSimulatePhysics(*this, enabled);
  }

  void Actor::SetCollisions(const bool enabled) {
    GetEpisode().Lock()->SetActorCollisions(*this, enabled);
  }

  void Actor::SetActorDead() {
    GetEpisode().Lock()->SetActorDead(*this);
  }

  void Actor::SetEnableGravity(const bool enabled) {
    GetEpisode().Lock()->SetActorEnableGravity(*this, enabled);
  }

  rpc::ActorState Actor::GetActorState() const {
    return GetEpisode().Lock()->GetActorState(*this);
  }

  bool Actor::Destroy() {
    rpc::ActorState actor_state = GetActorState();
    bool result = false;
    if (actor_state != rpc::ActorState::Invalid) {
      result = GetEpisode().Lock()->DestroyActor(*this);
    } else {
      log_warning(
          "attempting to destroy an actor that is already dead:",
          GetDisplayId());
    }
    return result;
  }

} // 命名空间client方便区分函数
} // 命名空间carla方便区分函数
