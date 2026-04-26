#pragma once

#include <string>
namespace decision {
struct Point {
  double x;
  double y;
  double yaw;
  Point(double x, double y, double yaw) : x(x), y(y), yaw(yaw){};
};
struct StateIsGoHome{
  int go_home_hp=150;
  int go_home_projectile_allowance=20;
  int become_home_hp=300;
  int become_home_projectile_allowance=200;
};
struct GoalPoint {
  Point Patrol1 = Point(1, 1, 1);
  Point Patrol2 = Point(2, 2, 2);
  Point Patrol3 = Point(3, 3, 3);
  Point Patrol4 = Point(4, 4, 4);
  Point HitOutpost = Point(0, 0, 0);
  Point home = Point(0.229, 0.807, 0.0);
};
struct FSMConfig {
  std::string map_tf_name_ = "map";
};
struct GameStatus {
  int is_game;
  int current_hp;
  int projectile_allowance;
  int is_enemy_outpost_destroyed;
};
enum RobotState {
  need2home,
  OkBecomeHome,
  Normal,
};
enum EnemyOutpostState {
  destroyed,
  not_destroyed,
};
enum class GameTask {
  Gohome,
  HitEnemyOutpost,
  PatrolA,
  PatrolB,
  Free,
};
enum class Nav2State {
  unkown, //一般未开始
  aborted,
  succeeded,
  running,
};
// enum class RobotState {
//     supply,
//     defense,
//     attack,
//     move,
// };
} // namespace decision