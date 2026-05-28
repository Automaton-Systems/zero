#pragma once

#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>

namespace zero {
namespace behavior {

// Returns success if the provided position is within a provided view angle
struct HeadingPositionViewNode : public behavior::BehaviorNode {
  HeadingPositionViewNode(const char* position_key, float view_radians)
      : position_key(position_key), view_radians(view_radians) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    auto self = ctx.bot->game->player_manager.GetSelf();

    if (!self) return ExecuteResult::Failure;

    auto opt_position = ctx.blackboard.Value<Vector2f>(position_key);
    if (!opt_position.has_value()) return ExecuteResult::Failure;

    Vector2f& position = opt_position.value();
    Vector2f direction = Normalize(position - self->position);

    float cos_angle = direction.Dot(self->GetHeading());
    float angle = acosf(cos_angle);

    return angle <= view_radians ? ExecuteResult::Success : ExecuteResult::Failure;
  }

  const char* position_key;
  float view_radians;
};

struct HeadingDirectionViewNode : public behavior::BehaviorNode {
  HeadingDirectionViewNode(const char* direction_key, float view_radians)
      : direction_key(direction_key), view_radians(view_radians) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    auto self = ctx.bot->game->player_manager.GetSelf();

    if (!self) return ExecuteResult::Failure;

    auto opt_direction = ctx.blackboard.Value<Vector2f>(direction_key);
    if (!opt_direction.has_value()) return ExecuteResult::Failure;

    Vector2f& direction = opt_direction.value();

    float cos_angle = direction.Dot(self->GetHeading());
    float angle = acosf(cos_angle);

    return angle <= view_radians ? ExecuteResult::Success : ExecuteResult::Failure;
  }

  const char* direction_key;
  float view_radians;
};

// If obey_stealth is true, then we will ignore players that we can't see.
struct NearestTargetNode : public behavior::BehaviorNode {
  NearestTargetNode(const char* player_key, bool obey_stealth = false)
      : player_key(player_key), obey_stealth(obey_stealth) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self) return behavior::ExecuteResult::Failure;

    Player* nearest = GetNearestTarget(ctx, *self);

    if (!nearest) {
      ctx.blackboard.Erase(player_key);
      return behavior::ExecuteResult::Failure;
    }

    ctx.blackboard.Set(player_key, nearest);

    return behavior::ExecuteResult::Success;
  }

  inline static bool IsVisible(ArenaSettings& settings, const Player& self, const Player& target) {
    constexpr Vector2f kViewDim(1920.0f, 1080.0f);

    // XRadar can see no matter what.
    if (self.togglables & Status_XRadar) return true;

    // We can always see them if they don't have stealth on.
    if (!(target.togglables & Status_Stealth)) return true;

    const Vector2f half_view_dim = kViewDim * 0.5f;

    Rectangle view_rect(self.position - half_view_dim, self.position + half_view_dim);
    Rectangle target_rect =
        Rectangle::FromPositionRadius(target.position, settings.ShipSettings[target.ship].GetRadius());

    // Target has stealth on and is off screen. We cannot see them.
    if (!BoxBoxIntersect(view_rect.min, view_rect.max, target_rect.min, target_rect.max)) {
      return false;
    }

    // We can see them if they don't have cloak on since they are on our screen.
    return !(target.togglables & Status_Cloak);
  }

 private:
  Player* GetNearestTarget(behavior::ExecuteContext& ctx, Player& self) {
    Game& game = *ctx.bot->game;
    RegionRegistry& region_registry = *ctx.bot->bot_controller->region_registry;

    Player* best_target = nullptr;
    float closest_dist_sq = std::numeric_limits<float>::max();

    for (size_t i = 0; i < game.player_manager.player_count; ++i) {
      Player* player = game.player_manager.players + i;

      if (player->ship >= 8) continue;
      if (player->frequency == self.frequency) continue;
      if (player->IsRespawning()) continue;
      if (player->position == Vector2f(0, 0)) continue;
      if (!game.player_manager.IsSynchronized(*player)) continue;
      if (!region_registry.IsConnected(self.position, player->position)) continue;

      bool in_safe = game.connection.map.GetTileId(player->position) == kTileIdSafe;
      if (in_safe) continue;

      if (obey_stealth && !IsVisible(ctx.bot->game->connection.settings, self, *player)) continue;

      float dist_sq = player->position.DistanceSq(self.position);
      if (dist_sq < closest_dist_sq) {
        closest_dist_sq = dist_sq;
        best_target = player;
      }
    }

    return best_target;
  }

  bool obey_stealth = false;
  const char* player_key;
};

// DecoyAwareTargetNode: Like NearestTargetNode but can be fooled by decoys
// Bots have a configurable chance to target a decoy instead of a real player
struct DecoyAwareTargetNode : public behavior::BehaviorNode {
  DecoyAwareTargetNode(const char* player_key, const char* position_key, bool obey_stealth = true, float decoy_confusion_chance = 0.30f)
      : player_key(player_key), position_key(position_key), obey_stealth(obey_stealth), decoy_confusion_chance(decoy_confusion_chance) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self) return behavior::ExecuteResult::Failure;

    Game& game = *ctx.bot->game;
    
    // Collect active decoys
    constexpr size_t kMaxDecoys = 64;
    DecoyInfo decoys[kMaxDecoys];
    size_t decoy_count = game.weapon_manager.GetActiveDecoys(decoys, kMaxDecoys);

    // Random chance to be confused by a decoy
    bool consider_decoys = decoy_count > 0 && ((rand() % 100) < (int)(decoy_confusion_chance * 100.0f));

    if (consider_decoys) {
      // Find nearest decoy
      DecoyInfo* nearest_decoy = nullptr;
      float closest_dist_sq = std::numeric_limits<float>::max();

      for (size_t i = 0; i < decoy_count; ++i) {
        DecoyInfo* decoy = &decoys[i];
        
        // Don't target own team's decoys
        Player* thrower = game.player_manager.GetPlayerById(decoy->player_id);
        if (thrower && thrower->frequency == self->frequency) continue;

        // Don't target decoys in safe zones
        if (game.connection.map.GetTileId(decoy->position) == kTileIdSafe) continue;

        float dist_sq = decoy->position.DistanceSq(self->position);
        if (dist_sq < closest_dist_sq) {
          closest_dist_sq = dist_sq;
          nearest_decoy = decoy;
        }
      }

      if (nearest_decoy) {
        // Bot is confused! Target the decoy position
        ctx.blackboard.Erase(player_key);  // No real player target
        ctx.blackboard.Set(position_key, nearest_decoy->position);
        return behavior::ExecuteResult::Success;
      }
    }

    // Fall back to normal nearest target logic
    Player* nearest = GetNearestTarget(ctx, *self);

    if (!nearest) {
      ctx.blackboard.Erase(player_key);
      ctx.blackboard.Erase(position_key);
      return behavior::ExecuteResult::Failure;
    }

    ctx.blackboard.Set(player_key, nearest);
    ctx.blackboard.Set(position_key, nearest->position);

    return behavior::ExecuteResult::Success;
  }

 private:
  Player* GetNearestTarget(behavior::ExecuteContext& ctx, Player& self) {
    Game& game = *ctx.bot->game;
    RegionRegistry& region_registry = *ctx.bot->bot_controller->region_registry;

    Player* best_target = nullptr;
    float closest_dist_sq = std::numeric_limits<float>::max();

    for (size_t i = 0; i < game.player_manager.player_count; ++i) {
      Player* player = game.player_manager.players + i;

      if (player->ship >= 8) continue;
      if (player->frequency == self.frequency) continue;
      if (player->IsRespawning()) continue;
      if (player->position == Vector2f(0, 0)) continue;
      if (!game.player_manager.IsSynchronized(*player)) continue;
      if (!region_registry.IsConnected(self.position, player->position)) continue;

      bool in_safe = game.connection.map.GetTileId(player->position) == kTileIdSafe;
      if (in_safe) continue;

      if (obey_stealth && !NearestTargetNode::IsVisible(ctx.bot->game->connection.settings, self, *player)) continue;

      float dist_sq = player->position.DistanceSq(self.position);
      if (dist_sq < closest_dist_sq) {
        closest_dist_sq = dist_sq;
        best_target = player;
      }
    }

    return best_target;
  }

  bool obey_stealth = true;
  float decoy_confusion_chance = 0.30f;  // 30% chance to be fooled by decoy
  const char* player_key;
  const char* position_key;
};

}  // namespace behavior
}  // namespace zero
