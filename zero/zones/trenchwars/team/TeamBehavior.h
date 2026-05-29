#pragma once

#include <zero/Math.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/Behavior.h>
#include <zero/behavior/BehaviorBuilder.h>

namespace zero {
namespace tw {

struct TeamBehavior : public behavior::Behavior {
  void OnInitialize(behavior::ExecuteContext& ctx) override {
    // Setup blackboard here for this specific behavior
    ctx.blackboard.Set("request_ship", 0);
    ctx.blackboard.Set("leash_distance", 30.0f);

    // Spawn position for detecting if bot needs to leave initial area
    ctx.blackboard.Set("spawn_position", Vector2f(512, 512));

    // Generate random waypoints within safe zone (30% from edges)
    constexpr float kMapSize = 1024.0f;
    constexpr float kEdgeMargin = 0.30f; // 30% from edges
    float min_coord = kMapSize * kEdgeMargin;
    float max_coord = kMapSize * (1.0f - kEdgeMargin);
    
    std::vector<Vector2f> waypoints;
    for (int i = 0; i < 10; ++i) {
      float x = min_coord + (rand() % (int)(max_coord - min_coord));
      float y = min_coord + (rand() % (int)(max_coord - min_coord));
      waypoints.push_back(Vector2f(x, y));
    }

    ctx.blackboard.Set("waypoints", waypoints);

    ctx.blackboard.Set<u16>("request_freq", (rand() % 9898) + 100);

    auto opt_freq = ctx.bot->config->GetInt("TrenchWars", "Freq");
    if (opt_freq && *opt_freq >= 0 && *opt_freq <= 9998) {
      ctx.blackboard.Set<u16>("request_freq", *opt_freq);
    }
  }

  std::unique_ptr<behavior::BehaviorNode> CreateTree(behavior::ExecuteContext& ctx) override;
};

}  // namespace tw
}  // namespace zero
