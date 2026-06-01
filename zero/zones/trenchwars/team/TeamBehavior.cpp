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

#include <random>

namespace zero {
namespace tw {

constexpr u32 kTileIdWormhole = 220;
constexpr float kTeamLeashDistance = 30.0f;
constexpr float kAvoidTeamDistance = 2.0f;
constexpr float kWormholeDetectionRadius = 5.0f;
constexpr float kAimJitterAmount = 1.5f;
constexpr u32 kSpawnProtectionTime = 1500; // Ticks (15 seconds) to spread out before engaging

// Defensive behavior - dodge incoming damage (based on basing defensive tree)
static std::unique_ptr<behavior::BehaviorNode> CreateDefensiveTree() {
  using namespace behavior;

  constexpr float kRepelDistance = 16.0f;
  constexpr float kLowEnergyThreshold = 450.0f;
  constexpr float kNearbyEnemyThreshold = 20.0f;

  BehaviorBuilder builder;

  // clang-format off
  builder
    .Sequence() // Attempt to dodge incoming damage
        .Sequence(CompositeDecorator::Success) // Always check incoming damage
            .Child<svs::IncomingDamageQueryNode>(kRepelDistance, "incoming_damage")
            .Child<PlayerCurrentEnergyQueryNode>("self_energy")
            .End()
        .Sequence(CompositeDecorator::Invert) // Don't dodge if enemy is rushing with low energy
            .Child<PlayerEnergyQueryNode>("nearest_target", "nearest_target_energy")
            .InvertChild<ScalarThresholdNode<float>>("nearest_target_energy", kLowEnergyThreshold)
            .InvertChild<DistanceThresholdNode>("nearest_target_position", "self_position", kNearbyEnemyThreshold)
            .End()
        .Child<DodgeIncomingDamage>(0.4f, 16.0f, 0.0f)
        .End();
  // clang-format on

  return builder.Build();
}

// Offensive behavior - aim and shoot at enemies (based on basing offensive tree)
static std::unique_ptr<behavior::BehaviorNode> CreateOffensiveTree(const char* nearest_target_key,
                                                                   const char* nearest_target_position_key) {
  using namespace behavior;

  constexpr float kLowEnergyThreshold = 450.0f;

  BehaviorBuilder builder;

  // clang-format off
  builder
    .Sequence() // Aim at target and shoot while seeking them
        .Child<AimNode>(WeaponType::Bullet, nearest_target_key, "aimshot")
        .Sequence(CompositeDecorator::Success) // Add random aim jitter for medium difficulty
            .Child<ExecuteNode>([](behavior::ExecuteContext& ctx) {
              auto opt_aimshot = ctx.blackboard.Value<Vector2f>("aimshot");
              if (opt_aimshot) {
                static std::random_device rd;
                static std::mt19937 gen(rd());
                std::uniform_real_distribution<float> dis(-kAimJitterAmount, kAimJitterAmount);
                Vector2f jittered = *opt_aimshot + Vector2f(dis(gen), dis(gen));
                ctx.blackboard.Set("aimshot", jittered);
              }
              return behavior::ExecuteResult::Success;
            })
            .End()
        .Parallel()
            .Child<FaceNode>("aimshot")
            .Child<BlackboardEraseNode>("rushing")
            .Selector()
                .Sequence() // If target is very low energy, rush at them
                    .Child<PlayerEnergyQueryNode>(nearest_target_key, "nearest_target_energy")
                    .InvertChild<ScalarThresholdNode<float>>("nearest_target_energy", kLowEnergyThreshold)
                    .Child<SeekNode>("aimshot", 0.0f, SeekNode::DistanceResolveType::Static)
                    .Child<ScalarNode>(1.0f, "rushing")
                    .End()
                .Sequence() // Move away if our energy is low
                    .InvertChild<PlayerEnergyPercentThresholdNode>(0.3f)
                    .Child<SeekNode>("aimshot", kTeamLeashDistance, SeekNode::DistanceResolveType::Dynamic)
                    .End()
                .Child<SeekNode>("aimshot", 0.0f, SeekNode::DistanceResolveType::Zero)
                .End()
            .Child<AvoidTeamNode>(kAvoidTeamDistance)
            .Sequence(CompositeDecorator::Success) // Shoot if we have energy and weapon is ready
                .Selector()
                    .Child<BlackboardSetQueryNode>("rushing")
                    .Child<PlayerEnergyPercentThresholdNode>(0.3f)
                    .End()
                .InvertChild<ShipWeaponCooldownQueryNode>(WeaponType::Bullet)
                .Child<ShotVelocityQueryNode>(WeaponType::Bullet, "bullet_velocity")
                .Child<RayNode>("self_position", "bullet_velocity", "bullet_ray")
                .Child<svs::DynamicPlayerBoundingBoxQueryNode>(nearest_target_key, "target_bounds", 3.5f)
                .Child<MoveRectangleNode>("target_bounds", "aimshot", "target_bounds")
                .Child<RayRectangleInterceptNode>("bullet_ray", "target_bounds")
                .Child<InputActionNode>(InputAction::Bullet)
                .End()
            .End()
        .End();
  // clang-format on

  return builder.Build();
}

// Patrol and fight behavior - primary goal is moving to waypoints, fight enemies encountered along the way
static std::unique_ptr<behavior::BehaviorNode> CreatePatrolAndFightTree() {
  using namespace behavior;

  BehaviorBuilder builder;

  // clang-format off
  builder
    .Sequence()
        .Child<PlayerPositionQueryNode>("self_position")
        .Sequence(CompositeDecorator::Success) // Generate or update random waypoint
            .Child<ExecuteNode>([](behavior::ExecuteContext& ctx) {
              auto opt_waypoint = ctx.blackboard.Value<Vector2f>("waypoint_position");
              auto opt_self_pos = ctx.blackboard.Value<Vector2f>("self_position");
              
              bool need_new_waypoint = !opt_waypoint;
              
              // Check if we reached the waypoint
              if (opt_waypoint && opt_self_pos) {
                float dist_sq = (*opt_self_pos - *opt_waypoint).LengthSq();
                if (dist_sq < 20.0f * 20.0f) {
                  need_new_waypoint = true;
                }
              }
              
              // Generate new waypoint if needed
              if (need_new_waypoint) {
                constexpr float kMapSize = 1024.0f;
                constexpr float kEdgeMargin = 0.10f; // 10% margin = waypoints from 102 to 921 (80% of map)
                float min_coord = kMapSize * kEdgeMargin;
                float max_coord = kMapSize * (1.0f - kEdgeMargin);
                
                static std::random_device rd;
                static std::mt19937 gen(rd());
                std::uniform_real_distribution<float> dis(min_coord, max_coord);
                
                Vector2f new_waypoint(dis(gen), dis(gen));
                ctx.blackboard.Set("waypoint_position", new_waypoint);
              }
              return behavior::ExecuteResult::Success;
            })
            .End()
        .Selector()
            .Sequence() // Travel to waypoint (primary goal, ALWAYS happens first 10s after spawn)
                .InvertChild<TimerExpiredNode>("spawn_protection")
                .Child<AvoidTeamNode>(kAvoidTeamDistance)
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
                .End()
            .Sequence() // After spawn protection expires, can engage enemies while traveling
                .Child<NearestTargetNode>("nearest_target", true)
                .Child<PlayerPositionQueryNode>("nearest_target", "nearest_target_position")
                .Child<AvoidTeamNode>(kAvoidTeamDistance)
                .Selector()
                    .Composite(CreateDefensiveTree())
                    .Sequence() // If can't see target, path to them
                        .InvertChild<ShipTraverseQueryNode>("nearest_target_position")
                        .Child<GoToNode>("nearest_target_position")
                        .End()
                    .Composite(CreateOffensiveTree("nearest_target", "nearest_target_position"))
                    .End()
                .End()
            .Sequence() // Default: just travel to waypoint (no enemies or can't reach them)
                .Child<AvoidTeamNode>(kAvoidTeamDistance)
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
        .Sequence() // Enter the specified ship if not already in it
            .InvertChild<ShipQueryNode>("request_ship")
            .Child<ShipRequestNode>("request_ship")
            .Child<TimerSetNode>("spawn_protection", kSpawnProtectionTime)
            .End()
        .Sequence() // Do nothing while waiting for spawn cooldown
            .InvertChild<TimerExpiredNode>(TrenchWars::SpawnExecuteCooldownKey())
            .End()
        .Sequence() // Join requested freq
            .Child<PlayerFrequencyQueryNode>("self_freq")
            .InvertChild<EqualityNode<u16>>("self_freq", "request_freq")
            .Child<TimerExpiredNode>("next_freq_change_tick")
            .Child<TimerSetNode>("next_freq_change_tick", 300)
            .Child<PlayerChangeFrequencyNode>("request_freq")
            .End()
        .Sequence() // If in spec, do nothing
            .Child<ShipQueryNode>(8)
            .End()
        .Sequence() // Main TDM behavior - patrol and fight like basing travels and fights
            .Sequence(CompositeDecorator::Success) // Reset spawn protection timer if it expired (respawn detection)
                .Child<TimerExpiredNode>("spawn_protection")
                .Child<TimerSetNode>("spawn_protection", kSpawnProtectionTime)
                .End()
            .Selector()
                .Sequence() // Priority 1: Check for wormhole and boost through
                    .Child<PlayerPositionQueryNode>("self_position")
                    .Child<ExecuteNode>([](behavior::ExecuteContext& ctx) {
                      auto opt_pos = ctx.blackboard.Value<Vector2f>("self_position");
                      if (!opt_pos) return behavior::ExecuteResult::Failure;
                      
                      Vector2f pos = *opt_pos;
                      auto& map = ctx.bot->game->GetMap();
                      
                      // Check for wormhole in nearby tiles
                      for (int dy = -5; dy <= 5; ++dy) {
                        for (int dx = -5; dx <= 5; ++dx) {
                          int tx = (int)pos.x + dx;
                          int ty = (int)pos.y + dy;
                          if (tx >= 0 && tx < 1024 && ty >= 0 && ty < 1024) {
                            if (map.GetTileId((u16)tx, (u16)ty) == kTileIdWormhole) {
                              float dist_sq = (float)(dx * dx + dy * dy);
                              if (dist_sq < kWormholeDetectionRadius * kWormholeDetectionRadius) {
                                return behavior::ExecuteResult::Success;
                              }
                            }
                          }
                        }
                      }
                      return behavior::ExecuteResult::Failure;
                    })
                    .Child<AfterburnerThresholdNode>()
                    .End()
                .Composite(CreatePatrolAndFightTree()) // Priority 2: Main behavior - patrol with combat
                .End()
            .End()
        .End();
  // clang-format on

  return builder.Build();
}

}  // namespace tw
}  // namespace zero
