#include "TeamBehavior.h"

#include <zero/behavior/BehaviorBuilder.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/behavior/nodes/AimNode.h>
#include <zero/behavior/nodes/BlackboardNode.h>
#include <zero/behavior/nodes/InputActionNode.h>
#include <zero/behavior/nodes/MapNode.h>
#include <zero/behavior/nodes/MathNode.h>
#include <zero/behavior/nodes/MoveNode.h>
#include <zero/behavior/nodes/PlayerNode.h>
#include <zero/behavior/nodes/RenderNode.h>
#include <zero/behavior/nodes/ShipNode.h>
#include <zero/behavior/nodes/TargetNode.h>
#include <zero/behavior/nodes/ThreatNode.h>
#include <zero/behavior/nodes/TimerNode.h>
#include <zero/behavior/nodes/WaypointNode.h>
#include <zero/zones/svs/nodes/DynamicPlayerBoundingBoxQueryNode.h>
#include <zero/zones/svs/nodes/IncomingDamageQueryNode.h>
#include <zero/zones/trenchwars/TrenchWars.h>
#include <zero/zones/trenchwars/nodes/MoveNode.h>

namespace zero {
namespace tw {

constexpr float kTeamLeashDistance = 30.0f;
constexpr float kAvoidTeamDistance = 2.0f;
constexpr float kFarEnemyDistance = 35.0f;
constexpr float kSpawnAreaRadius = 80.0f; // Distance from spawn before engaging enemies - increased more
constexpr float kMaxChaseDistance = 40.0f; // Break off chase if enemy is further than this - reduced
constexpr u32 kPostSpawnExploreTime = 1500; // Ticks (15 seconds) to explore before engaging - increased

// Defensive behavior - dodge incoming damage, warp if about to die
static std::unique_ptr<behavior::BehaviorNode> CreateDefensiveTree() {
  using namespace behavior;

  constexpr float kDangerDistance = 6.0f;
  constexpr float kRepelDistance = 16.0f;

  BehaviorBuilder builder;

  // clang-format off
  builder
    .Sequence()
        .Sequence(CompositeDecorator::Success) // Check incoming damage and our energy
            .Child<svs::IncomingDamageQueryNode>(kDangerDistance, "incoming_damage")
            .Child<PlayerCurrentEnergyQueryNode>("self_energy")
            .End()
        .Sequence(CompositeDecorator::Success) // Warp if we're about to die
            .Child<ScalarThresholdNode<float>>("incoming_damage", "self_energy")
            .Child<InputActionNode>(InputAction::Warp)
            .End()
        .Child<DodgeIncomingDamage>(0.5f, kRepelDistance, 0.0f)
        .Child<InputActionNode>(InputAction::Afterburner) // Afterburner to escape
        .End();
  // clang-format on

  return builder.Build();
}

// Offensive behavior - aim and shoot at enemies
static std::unique_ptr<behavior::BehaviorNode> CreateOffensiveTree() {
  using namespace behavior;

  constexpr float kBulletEnergyCost = 0.9f;
  constexpr float kLowEnergyThreshold = 0.35f;
  constexpr float kNearDistance = 20.0f;

  BehaviorBuilder builder;

  // clang-format off
  builder
    .Sequence()
        .Child<PlayerPositionQueryNode>("self_position")
        .Child<PlayerEnergyQueryNode>("self_energy")
        .Child<AimNode>(WeaponType::Bullet, "nearest_target", "aimshot")
        .Parallel() // Do movement and shooting in parallel
            .Selector() // Choose movement strategy
                .Sequence() // Rush if we have more energy than target
                    .Child<PlayerEnergyQueryNode>("nearest_target", "target_energy")
                    .Child<ScalarThresholdNode<float>>("self_energy", "target_energy")
                    .Child<PlayerEnergyPercentThresholdNode>(0.4f)
                    .Child<SeekNode>("aimshot", 3.0f, SeekNode::DistanceResolveType::Zero)
                    .End()
                .Sequence() // Back off if low energy
                    .InvertChild<PlayerEnergyPercentThresholdNode>(kLowEnergyThreshold)
                    .Child<SeekNode>("aimshot", kTeamLeashDistance, SeekNode::DistanceResolveType::Dynamic)
                    .End()
                .Sequence() // Use afterburner to chase if far
                    .Child<DistanceThresholdNode>("nearest_target_position", kFarEnemyDistance)
                    .Child<AfterburnerThresholdNode>()
                    .Child<SeekNode>("aimshot", 10.0f, SeekNode::DistanceResolveType::Zero)
                    .End()
                .Child<SeekNode>("aimshot", 8.0f, SeekNode::DistanceResolveType::Zero) // Default: maintain medium distance
                .End()
            .Child<AvoidTeamNode>(kAvoidTeamDistance) // Don't bump teammates
            .Child<FaceNode>("aimshot") // Face target
            .Sequence(CompositeDecorator::Success) // Shoot if we have energy and good shot
                .Child<PlayerEnergyPercentThresholdNode>(kBulletEnergyCost)
                .InvertChild<ShipWeaponCooldownQueryNode>(WeaponType::Bullet)
                .Child<ShotVelocityQueryNode>(WeaponType::Bullet, "bullet_velocity")
                .Child<RayNode>("self_position", "bullet_velocity", "bullet_ray")
                .Child<svs::DynamicPlayerBoundingBoxQueryNode>("nearest_target", "target_bounds", 3.5f)
                .Child<MoveRectangleNode>("target_bounds", "aimshot", "target_bounds")
                .Child<RayRectangleInterceptNode>("bullet_ray", "target_bounds")
                .Child<InputActionNode>(InputAction::Bullet)
                .End()
            .End()
        .End();
  // clang-format on

  return builder.Build();
}

// Chase behavior - path to enemy if not directly visible
static std::unique_ptr<behavior::BehaviorNode> CreateChaseTree() {
  using namespace behavior;

  BehaviorBuilder builder;

  // clang-format off
  builder
    .Sequence()
        .InvertChild<ShipTraverseQueryNode>("nearest_target_position")
        .Child<GoToNode>("nearest_target_position")
        .Child<RenderPathNode>(Vector3f(1, 0.5f, 0))
        .End();
  // clang-format on

  return builder.Build();
}

// Patrol behavior - follow waypoints when no enemies nearby
static std::unique_ptr<behavior::BehaviorNode> CreatePatrolTree() {
  using namespace behavior;

  BehaviorBuilder builder;

  // clang-format off
  builder
    .Sequence()
        .Child<WaypointNode>("waypoints", "waypoint_index", "waypoint_position", 15.0f)
        .Selector()
            .Sequence() // Direct path if traversable
                .Child<ShipTraverseQueryNode>("waypoint_position")
                .Child<FaceNode>("waypoint_position")
                .Child<ArriveNode>("waypoint_position", 1.25f)
                .End()
            .Sequence() // Path around obstacles
                .Child<GoToNode>("waypoint_position")
                .Child<RenderPathNode>(Vector3f(0.0f, 0.8f, 1.0f))
                .End()
            .End()
        .End();
  // clang-format on

  return builder.Build();
}

std::unique_ptr<behavior::BehaviorNode> TeamBehavior::CreateTree(behavior::ExecuteContext& ctx) {
  using namespace behavior;

  BehaviorBuilder builder;

  // clang-format off

  builder
    .Selector()
        .Sequence() // Enter the specified ship if not already in it.
            .InvertChild<ShipQueryNode>("request_ship")
            .Child<ShipRequestNode>("request_ship")
            .Child<TimerSetNode>("explore_timer", kPostSpawnExploreTime) // Set explore timer on ship entry
            .End()
        .Sequence() // Do nothing while waiting for spawn cooldown, then reset explore timer
            .InvertChild<TimerExpiredNode>(TrenchWars::SpawnExecuteCooldownKey())
            .Child<TimerSetNode>("explore_timer", kPostSpawnExploreTime) // Reset timer on respawn
            .End()
        .Sequence() // Join requested freq
            .Child<PlayerFrequencyQueryNode>("self_freq")
            .InvertChild<EqualityNode<u16>>("self_freq", "request_freq")
            .Child<TimerExpiredNode>("next_freq_change_tick")
            .Child<TimerSetNode>("next_freq_change_tick", 300)
            .Child<PlayerChangeFrequencyNode>("request_freq")
            .End()
        .Sequence() // If we are in spec, do nothing
            .Child<ShipQueryNode>(8)
            .End()
        .Sequence() // Force exploration for first 15 seconds after spawn
            .InvertChild<ShipQueryNode>(8)
            .InvertChild<TimerExpiredNode>("explore_timer") // Still in explore mode
            .Composite(CreatePatrolTree()) // Just patrol, no combat
            .End()
        .Sequence() // Leave spawn area before engaging enemies
            .InvertChild<ShipQueryNode>(8) // Make sure we're not in spec
            .Child<PlayerPositionQueryNode>("self_position")
            .InvertChild<DistanceThresholdNode>("self_position", "spawn_position", kSpawnAreaRadius)
            .Sequence(CompositeDecorator::Success) // Pick random direction to leave spawn
                .Child<ExecuteNode>([](behavior::ExecuteContext& ctx) {
                  // Generate random exit point 30% from edges
                  constexpr float kMapSize = 1024.0f;
                  constexpr float kEdgeMargin = 0.30f;
                  float min_coord = kMapSize * kEdgeMargin;
                  float max_coord = kMapSize * (1.0f - kEdgeMargin);
                  float x = min_coord + (rand() % (int)(max_coord - min_coord));
                  float y = min_coord + (rand() % (int)(max_coord - min_coord));
                  ctx.blackboard.Set("leave_spawn_target", Vector2f(x, y));
                  return behavior::ExecuteResult::Success;
                })
                .End()
            .Sequence(CompositeDecorator::Success) // Use afterburners to leave spawn faster
                .Child<AfterburnerThresholdNode>()
                .End()
            .Selector() // Navigate to waypoint
                .Sequence()
                    .Child<ShipTraverseQueryNode>("leave_spawn_target")
                    .Child<FaceNode>("leave_spawn_target")
                    .Child<ArriveNode>("leave_spawn_target", 1.25f)
                    .End()
                .Child<GoToNode>("leave_spawn_target")
                .End()
            .End()
        .Sequence() // Main behavior for all ships (only when outside spawn area)
            .InvertChild<ShipQueryNode>(8) // Make sure we're not in spec
            .Selector()
                .Composite(CreateDefensiveTree()) // Priority 1: Defend if under attack
                .Sequence() // Priority 2: Fight enemies if nearby (and within chase range)
                    .Child<PlayerPositionQueryNode>("self_position")
                    .Child<NearestTargetNode>("nearest_target", true)
                    .Child<PlayerPositionQueryNode>("nearest_target", "nearest_target_position")
                    .InvertChild<DistanceThresholdNode>("nearest_target_position", kMaxChaseDistance) // Break off if too far
                    .Selector()
                        .Composite(CreateChaseTree()) // Path to enemy if not visible
                        .Composite(CreateOffensiveTree()) // Attack if visible
                        .End()
                    .End()
                .Composite(CreatePatrolTree()) // Priority 3: Patrol when no enemies (or enemy too far)
                .End()
            .End()
        .Sequence() // Warp out if all above sequences failed
            .Child<WarpNode>()
            .End()
        .End();
  // clang-format on

  return builder.Build();
}

}  // namespace tw
}  // namespace zero
