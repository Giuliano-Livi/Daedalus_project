// Trajectory generator: A* on the known map of the warehouse through the target points, then one
// smooth trajectory (rounded corners, speed profile, rate-limited yaw) streamed on /drone/setpoint.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Dense>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <daedalus_interfaces/msg/flight_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>

using daedalus_interfaces::msg::FlightSetpoint;
using Vec3 = Eigen::Vector3d;

namespace
{
constexpr double NaN = std::numeric_limits<double>::quiet_NaN();
constexpr double INF = std::numeric_limits<double>::infinity();

using Cell = std::pair<int, int>;                 // (row, col) = (i, j)

// first index where s[idx] >= value (np.searchsorted, side='left')
size_t searchsorted(const std::vector<double> & s, double value)
{
  return std::lower_bound(s.begin(), s.end(), value) - s.begin();
}
}  // namespace

class TrajectoryGenerator : public rclcpp::Node
{
public:
  TrajectoryGenerator()
  : Node("trajectory_generator")
  {
    // 2D occupancy grid from scripts/gen_occupancy_grid.py: 1 = occupied
    load_grid(ament_index_cpp::get_package_share_directory("daedalus_trajectories") + "/maps/warehouse_occupancy.csv");

    // clearance: distance (m) from every cell to the nearest obstacle cell of the raw map
    std::vector<Cell> obstacles;
    for (int i = 0; i < rows_; ++i) {
      for (int j = 0; j < cols_; ++j) {
        if (grid_[i * cols_ + j]) {obstacles.emplace_back(i, j);}
      }
    }
    clearance_.assign(rows_ * cols_, INF);
    for (int i = 0; i < rows_; ++i) {
      for (int j = 0; j < cols_; ++j) {
        double best = INF;
        for (const auto & [oi, oj] : obstacles) {best = std::min(best, std::hypot(i - oi, j - oj));}
        clearance_[i * cols_ + j] = best * grid_res_;
      }
    }

    // inflate: a cell is occupied if any cell within `inflation` cells of it is
    std::vector<uint8_t> inflated(rows_ * cols_, 0);
    const int r = inflation_;
    for (int i = 0; i < rows_; ++i) {
      for (int j = 0; j < cols_; ++j) {
        for (int di = -r; di <= r && !inflated[i * cols_ + j]; ++di) {
          for (int dj = -r; dj <= r; ++dj) {
            const int ni = i + di, nj = j + dj;
            if (ni >= 0 && ni < rows_ && nj >= 0 && nj < cols_ && grid_[ni * cols_ + nj]) {
              inflated[i * cols_ + j] = 1;
              break;
            }
          }
        }
      }
    }
    grid_ = inflated;

    setpoint_pub_ = create_publisher<FlightSetpoint>("/drone/setpoint", 10);

    // 10Hz loop
    timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(100), [this]() {publish();});
  }

private:
  Vec3 current_position_ = Vec3::Zero();
  std::vector<Vec3> target_pos_ = {
    {0, 54.5, 3}, {7.6, 54.5, 3}, {7.6, -4.5, 3}, {16.2, -4.5, 3}, {16.2, 54.5, 3}, {7.6, 54.5, 3},
    {7.6, -4.5, 3}, {0, 0, 3}};
  double des_yaw_ = M_PI / 2;

  std::vector<uint8_t> grid_;
  int rows_ = 0, cols_ = 0;
  std::vector<double> clearance_;
  const double grid_origin_[2] = {-15.0, -35.0};   // world x, y of the grid's south-west corner
  const double grid_res_ = 0.5;                    // cell side (m)
  const double spawn_xy_[2] = {-10.2, -25.0};      // drone spawn in world
  const double turn_cost_ = 2.0;                   // A* extra cost (m) for every change of direction, 0 = shortest path
  const int inflation_ = 4;                        // safety margin: cells added around every obstacle, 0 = none
  const double center_dist_ = 4.5;                 // m: closer than this to an obstacle, a step costs extra (>= half the widest lane)
  const double center_weight_ = 0.3;               // extra cost per step for each m closer than center_dist, 0 = off

  // smooth trajectory: corners rounded instead of stopping at every waypoint
  const double blend_radius_ = 1.5;                // m: a corner is rounded from up to this far before to after it
  const double stop_angle_ = 120.0 * M_PI / 180;   // turns sharper than this (path reversals) stop at the corner
  const double yaw_lead_ = 1.0;                    // m: face the path direction this far ahead (turn before the corner)
  const double yaw_rate_max_ = 25.0 * M_PI / 180;  // below PX4's MC_YAWRATE_MAX (30 deg/s, docker-compose.yml)
  const double ds_ = 0.02;                         // m: resolution of the path used to build the speed profile

  std::vector<Vec3> pos_traj_, vel_traj_, acc_traj_;
  std::vector<double> yaw_traj_, yaw_rate_traj_;
  size_t step_of_traj_ = 0;
  bool trajectory_generated_ = false;

  rclcpp::Publisher<FlightSetpoint>::SharedPtr setpoint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void load_grid(const std::string & path)
  {
    std::ifstream file(path);
    if (!file) {throw std::runtime_error("cannot open " + path);}
    std::string line;
    while (std::getline(file, line)) {
      if (line.empty()) {continue;}
      std::stringstream ss(line);
      std::string value;
      int n = 0;
      while (std::getline(ss, value, ',')) {
        grid_.push_back(static_cast<uint8_t>(std::stoi(value)));
        ++n;
      }
      cols_ = n;
      ++rows_;
    }
  }

  // callbacks
  void publish()
  {
    FlightSetpoint setpoint;
    if (!trajectory_generated_) {generate_trajectory();}
    const bool running = step_of_traj_ < pos_traj_.size();
    const size_t k = running ? step_of_traj_ : pos_traj_.size() - 1;
    ++step_of_traj_;
    setpoint.position.x = pos_traj_[k].x();
    setpoint.position.y = pos_traj_[k].y();
    setpoint.position.z = pos_traj_[k].z();
    setpoint.velocity.x = vel_traj_[k].x();
    setpoint.velocity.y = vel_traj_[k].y();
    setpoint.velocity.z = vel_traj_[k].z();
    setpoint.acceleration.x = acc_traj_[k].x();
    setpoint.acceleration.y = acc_traj_[k].y();
    setpoint.acceleration.z = acc_traj_[k].z();
    setpoint.yaw = yaw_traj_[k];
    setpoint.yaw_rate = running ? yaw_rate_traj_[k] : 0.0;
    setpoint_pub_->publish(setpoint);
  }

  // functions
  // coefficients of the quintic from (x0, v0, a0) to (xf, vf, af) in T
  Eigen::Matrix<double, 6, 1> quintic(double x0, double v0, double a0, double xf, double vf, double af, double T)
  {
    const double c0 = x0, c1 = v0, c2 = a0 / 2;
    Eigen::Matrix3d A;
    A << std::pow(T, 3), std::pow(T, 4), std::pow(T, 5),
      3 * T * T, 4 * std::pow(T, 3), 5 * std::pow(T, 4),
      6 * T, 12 * T * T, 20 * std::pow(T, 3);
    const Eigen::Vector3d b(xf - (c0 + c1 * T + c2 * T * T), vf - (c1 + 2 * c2 * T), af - 2 * c2);
    const Eigen::Vector3d c = A.colPivHouseholderQr().solve(b);
    Eigen::Matrix<double, 6, 1> out;
    out << c0, c1, c2, c[0], c[1], c[2];
    return out;
  }

  // position, velocity, acceleration of the quintic at t
  std::array<double, 3> sample(const Eigen::Matrix<double, 6, 1> & c, double t)
  {
    double x = 0, v = 0, a = 0;
    for (int i = 0; i < 6; ++i) {x += c[i] * std::pow(t, i);}
    for (int i = 1; i < 6; ++i) {v += i * c[i] * std::pow(t, i - 1);}
    for (int i = 2; i < 6; ++i) {a += i * (i - 1) * c[i] * std::pow(t, i - 2);}
    return {x, v, a};
  }

  // converts the target in grid dimensions and in drone's world coordinates
  Cell setpoint_to_cell(const Vec3 & p) const
  {
    const double x = p.x() + spawn_xy_[0], y = p.y() + spawn_xy_[1];
    return {static_cast<int>(std::floor((y - grid_origin_[1]) / grid_res_)),
      static_cast<int>(std::floor((x - grid_origin_[0]) / grid_res_))};
  }

  // grid cell (i, j) -> setpoint at the cell center
  Vec3 cell_to_setpoint(const Cell & c, double z) const
  {
    return {grid_origin_[0] + (c.second + 0.5) * grid_res_ - spawn_xy_[0],
      grid_origin_[1] + (c.first + 0.5) * grid_res_ - spawn_xy_[1], z};
  }

  bool is_free(const Cell & c) const
  {
    return c.first >= 0 && c.first < rows_ && c.second >= 0 && c.second < cols_ && !grid_[c.first * cols_ + c.second];
  }

  // cheapest 4-connected path of cells from start to goal, empty if unreachable.
  // cost = length + turn_cost for every change of direction + a penalty for cells close
  // to obstacles (keeps the path in the middle of the lanes), so the state is
  // (cell, direction of the move that reached it); (0, 0) = no move yet
  std::optional<std::vector<Cell>> astar(const Cell & start, const Cell & goal) const
  {
    if (!(is_free(start) && is_free(goal))) {return std::nullopt;}
    const int moves[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    // h guesses how far a cell is from the goal
    auto h = [&goal](int i, int j) {return std::hypot(i - goal.first, j - goal.second);};
    // state = (i, j, di, dj); ordered like the Python tuples ((i, j), (di, dj)), so that equal
    // costs are taken from the heap in the same order
    using State = std::tuple<int, int, int, int>;
    using Item = std::pair<double, State>;
    const State s0{start.first, start.second, 0, 0};
    std::map<State, double> g{{s0, 0.0}};   // notebook with the best cost found at the moment to reach the state
    std::map<State, State> parent;          // annotates the state you came from to form the final path
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;   // states to be explored
    open.emplace(h(start.first, start.second), s0);
    std::set<State> closed;                 // states already explored
    while (!open.empty()) {
      State s = open.top().second;
      open.pop();
      const auto [ci, cj, di0, dj0] = s;
      // if the goal is reached follow the parent notebook and append the setpoints
      if (ci == goal.first && cj == goal.second) {
        std::vector<Cell> path{{ci, cj}};
        while (parent.count(s)) {
          s = parent.at(s);
          path.emplace_back(std::get<0>(s), std::get<1>(s));
        }
        std::reverse(path.begin(), path.end());
        return path;
      }
      // if the state is already explored skip it, happens when a state is added to the heap twice
      if (closed.count(s)) {continue;}
      closed.insert(s);
      // try the moves
      for (const auto & m : moves) {
        const Cell n{ci + m[0], cj + m[1]};
        if (!is_free(n)) {continue;}
        const double turn = (di0 != 0 || dj0 != 0) && (m[0] != di0 || m[1] != dj0) ? turn_cost_ : 0.0;
        const double near = std::max(0.0, center_dist_ - clearance_[n.first * cols_ + n.second]);   // 0 when far enough
        const double ng = g.at(s) + std::hypot(m[0], m[1]) + turn + center_weight_ * near;
        const State ns{n.first, n.second, m[0], m[1]};
        const auto it = g.find(ns);
        if (it == g.end() || ng < it->second) {
          g[ns] = ng;
          parent[ns] = s;
          open.emplace(ng + h(n.first, n.second), ns);
        }
      }
    }
    return std::nullopt;
  }

  // keep only the cells where the direction changes: one segment per straight line
  std::vector<Cell> compact(const std::vector<Cell> & path) const
  {
    std::vector<Cell> out(path.begin(), path.begin() + std::min<size_t>(1, path.size()));
    for (size_t k = 0; k + 2 < path.size(); ++k) {
      const auto & a = path[k], & b = path[k + 1], & c = path[k + 2];
      if (b.first - a.first != c.first - b.first || b.second - a.second != c.second - b.second) {out.push_back(b);}
    }
    if (path.size() > 1) {out.push_back(path.back());}
    return out;
  }

  // A* from setpoint p0 to p1: exact p0, the corners of the path at p1's height, exact p1
  std::vector<Vec3> plan_path(const Vec3 & p0, const Vec3 & p1) const
  {
    const auto cells = astar(setpoint_to_cell(p0), setpoint_to_cell(p1));
    if (!cells) {
      std::ostringstream msg;
      msg << "A*: no path from " << p0.transpose() << " to " << p1.transpose() << " (occupied or outside the map?)";
      throw std::runtime_error(msg.str());
    }
    const auto corners = compact(*cells);
    std::vector<Vec3> out{p0};
    for (size_t k = 1; k + 1 < corners.size(); ++k) {out.push_back(cell_to_setpoint(corners[k], p1.z()));}
    out.push_back(p1);
    return out;
  }

  // dense path (one point every ~ds m) through the waypoints, each corner replaced by a
  // quadratic Bezier from blend_radius before to blend_radius after it (control point = the
  // corner, so the path stays tangent to both segments and passes ~0.35 * radius from a 90 deg
  // corner); returns points, curvature and stop flags (start, end, `stops` indices of
  // waypoints, reversals sharper than stop_angle)
  void smooth_path(
    const std::vector<Vec3> & waypoints, const std::set<size_t> & stops,
    std::vector<Vec3> & P, std::vector<double> & K, std::vector<uint8_t> & S) const
  {
    std::vector<Vec3> pts{waypoints[0]};
    std::vector<uint8_t> stp{1};
    for (size_t k = 1; k < waypoints.size(); ++k) {
      if ((waypoints[k] - pts.back()).norm() > 1e-6) {        // skip repeated waypoints
        pts.push_back(waypoints[k]);
        stp.push_back(stops.count(k));
      } else {
        stp.back() = stp.back() || stops.count(k);
      }
    }
    stp.back() = 1;
    const size_t n = pts.size();
    std::vector<double> r(n, 0.0);                             // blend radius of every corner, 0 = sharp
    for (size_t i = 1; i + 1 < n; ++i) {
      const Vec3 d_in = pts[i] - pts[i - 1], d_out = pts[i + 1] - pts[i];
      const double l_in = d_in.norm(), l_out = d_out.norm();
      const double angle = std::acos(std::clamp(d_in.dot(d_out) / (l_in * l_out), -1.0, 1.0));
      if (stp[i] || angle > stop_angle_) {
        stp[i] = 1;
      } else if (angle > 1e-3) {
        r[i] = std::min({blend_radius_, 0.45 * l_in, 0.45 * l_out});   // blends never overlap
      }
    }

    P = {pts[0]};
    K = {0.0};
    S = {1};
    for (size_t i = 1; i < n; ++i) {
      const Vec3 d = (pts[i] - pts[i - 1]).normalized();
      const Vec3 start = pts[i - 1] + d * r[i - 1], end = pts[i] - d * r[i];   // straight part
      int m = std::max(1, static_cast<int>(std::ceil((end - start).norm() / ds_)));
      for (int j = 1; j <= m; ++j) {
        P.push_back(start + (end - start) * j / m);
        K.push_back(0.0);
        S.push_back(0);
      }
      if (r[i] == 0.0) {                                       // sharp: stop at the corner (or the end)
        S.back() = stp[i];
        continue;
      }
      const Vec3 A = end, C = pts[i];
      const Vec3 B = C + (pts[i + 1] - C).normalized() * r[i];
      m = std::max(2, static_cast<int>(std::ceil(2 * r[i] / ds_)));
      for (int j = 1; j <= m; ++j) {
        const double t = static_cast<double>(j) / m;
        const Vec3 d1 = 2 * (1 - t) * (C - A) + 2 * t * (B - C);
        const Vec3 d2 = 2 * (A - 2 * C + B);
        P.push_back((1 - t) * (1 - t) * A + 2 * (1 - t) * t * C + t * t * B);
        K.push_back(d1.cross(d2).norm() / std::pow(d1.norm(), 3));
        S.push_back(0);
      }
    }
  }

  // speed along the dense path: v_max on straights, limited on curves by the lateral
  // acceleration (v^2 k <= a_max) and the yaw rate (v k <= yaw_rate_max), 0 at the stops;
  // then forward / backward passes so |dv/dt| <= a_max
  void speed_profile(
    const std::vector<Vec3> & P, const std::vector<double> & K, const std::vector<uint8_t> & S,
    double v_max, double a_max, std::vector<double> & v, std::vector<double> & ds) const
  {
    const size_t n = P.size();
    ds.resize(n - 1);
    for (size_t i = 0; i + 1 < n; ++i) {ds[i] = (P[i + 1] - P[i]).norm();}
    v.resize(n);
    for (size_t i = 0; i < n; ++i) {
      v[i] = K[i] > 0 ? std::min(v_max, std::min(std::sqrt(a_max / K[i]), yaw_rate_max_ / K[i])) : v_max;
      if (S[i]) {v[i] = 0.0;}
    }
    for (size_t i = 1; i < n; ++i) {v[i] = std::min(v[i], std::sqrt(v[i - 1] * v[i - 1] + 2 * a_max * ds[i - 1]));}
    for (size_t i = n - 1; i-- > 0;) {v[i] = std::min(v[i], std::sqrt(v[i + 1] * v[i + 1] + 2 * a_max * ds[i]));}
  }

  // one rest-to-rest piece sampled every dt: positions, velocities, accelerations and the
  // yaw to face (direction of the path yaw_lead m ahead, NaN where it is vertical)
  void time_piece(
    const std::vector<Vec3> & P, const std::vector<double> & v, const std::vector<double> & ds,
    double a_max, double dt, std::vector<Vec3> & pos, std::vector<Vec3> & vel, std::vector<Vec3> & acc,
    std::vector<double> & yaw) const
  {
    const size_t n = P.size();
    std::vector<double> t(n, 0.0), s(n, 0.0);
    for (size_t i = 0; i + 1 < n; ++i) {
      const double vs = v[i] + v[i + 1];
      const double dti = vs > 1e-6 ? 2 * ds[i] / std::max(vs, 1e-6) : std::sqrt(4 * ds[i] / a_max);
      t[i + 1] = t[i] + dti;
      s[i + 1] = s[i] + ds[i];
    }
    std::vector<Vec3> tang(n);
    for (size_t i = 0; i + 1 < n; ++i) {tang[i] = (P[i + 1] - P[i]) / ds[i];}
    tang[n - 1] = tang[n - 2];
    std::vector<double> head(n);
    for (size_t i = 0; i < n; ++i) {
      head[i] = std::hypot(tang[i].x(), tang[i].y()) > 0.1 ? std::atan2(tang[i].y(), tang[i].x()) : NaN;
    }
    std::vector<double> ahead(n);
    for (size_t i = 0; i < n; ++i) {ahead[i] = head[std::min(searchsorted(s, s[i] + yaw_lead_), n - 1)];}
    for (size_t i = 0; i < n; ++i) {
      if (!std::isnan(ahead[i])) {head[i] = ahead[i];}
    }
    std::vector<size_t> valid;
    for (size_t i = 0; i < n; ++i) {
      if (!std::isnan(head[i])) {valid.push_back(i);}
    }
    if (!valid.empty()) {                                      // fill vertical parts, then unwrap
      std::vector<double> uw(valid.size());
      uw[0] = head[valid[0]];
      for (size_t k = 1; k < valid.size(); ++k) {
        const double d = head[valid[k]] - head[valid[k - 1]];
        uw[k] = uw[k - 1] + std::atan2(std::sin(d), std::cos(d));
      }
      for (size_t i = 0; i < n; ++i) {                         // np.interp over the valid samples
        if (i <= valid.front()) {
          head[i] = uw.front();
        } else if (i >= valid.back()) {
          head[i] = uw.back();
        } else {
          const size_t k = std::upper_bound(valid.begin(), valid.end(), i) - valid.begin();
          const double f = static_cast<double>(i - valid[k - 1]) / (valid[k] - valid[k - 1]);
          head[i] = uw[k - 1] * (1 - f) + uw[k] * f;
        }
      }
    }
    // sample times: dt, 2 dt, ... (np.arange up to t[-1] - 1e-9), then the end
    std::vector<double> tt;
    const double stop = t.back() - 1e-9;
    const long count = stop > dt ? static_cast<long>(std::ceil((stop - dt) / dt)) : 0;
    for (long i = 0; i < count; ++i) {tt.push_back(dt + i * dt);}
    tt.push_back(t.back());
    const size_t m = tt.size();
    pos.resize(m);
    vel.resize(m);
    acc.resize(m);
    yaw.resize(m);
    for (size_t q = 0; q < m; ++q) {
      const long k_raw = static_cast<long>(searchsorted(t, tt[q])) - 1;
      const size_t k = static_cast<size_t>(std::clamp<long>(k_raw, 0, static_cast<long>(n) - 2));
      const double f = (tt[q] - t[k]) / std::max(t[k + 1] - t[k], 1e-9);
      pos[q] = P[k] * (1 - f) + P[k + 1] * f;
      vel[q] = tang[k] * (v[k] * (1 - f) + v[k + 1] * f);
      yaw[q] = valid.empty() ? NaN : head[k] * (1 - f) + head[k + 1] * f;
    }
    // acceleration: np.gradient (central differences, one-sided at the ends)
    if (m > 1) {
      for (size_t q = 0; q < m; ++q) {
        if (q == 0) {
          acc[q] = (vel[1] - vel[0]) / dt;
        } else if (q == m - 1) {
          acc[q] = (vel[m - 1] - vel[m - 2]) / dt;
        } else {
          acc[q] = (vel[q + 1] - vel[q - 1]) / (2 * dt);
        }
      }
    } else {
      acc[0] = Vec3::Zero();
    }
  }

  void append_setpoint(const Vec3 & p, const Vec3 & v, const Vec3 & a, double yaw, double yaw_rate)
  {
    pos_traj_.push_back(p);
    vel_traj_.push_back(v);
    acc_traj_.push_back(a);
    yaw_traj_.push_back(std::atan2(std::sin(yaw), std::cos(yaw)));
    yaw_rate_traj_.push_back(yaw_rate);
  }

  void generate_trajectory(double v_max = 1.0, double a_max = 1.0, double dt = 0.1)
  {
    // vertical takeoff to the first target's height, then A* through every target
    const Vec3 takeoff(0, 0, target_pos_[0].z());
    std::vector<Vec3> targets{takeoff};
    targets.insert(targets.end(), target_pos_.begin(), target_pos_.end());
    std::vector<Vec3> waypoints{current_position_, takeoff};
    // pairs the targets in couples (T0,T1), (T1,T2) ...
    for (size_t k = 0; k + 1 < targets.size(); ++k) {
      const auto segment = plan_path(targets[k], targets[k + 1]);
      waypoints.insert(waypoints.end(), segment.begin() + 1, segment.end());   // +1: no repeated point
    }
    std::ostringstream wp;
    for (const auto & p : waypoints) {wp << "[" << std::round(p.x() * 100) / 100 << ", " <<
        std::round(p.y() * 100) / 100 << ", " << std::round(p.z() * 100) / 100 << "] ";}
    RCLCPP_INFO(get_logger(), "A*: %zu waypoints: %s", waypoints.size(), wp.str().c_str());

    // one smooth trajectory instead of stopping at every waypoint: rounded corners, speed kept
    // through them; it stops only after the vertical takeoff, at path reversals and at the end
    std::vector<Vec3> P;
    std::vector<double> K;
    std::vector<uint8_t> S;
    smooth_path(waypoints, {1}, P, K, S);
    std::vector<double> v, ds;
    speed_profile(P, K, S, v_max, a_max, v, ds);
    std::vector<size_t> stops;
    for (size_t i = 0; i < S.size(); ++i) {
      if (S[i]) {stops.push_back(i);}
    }
    double yaw = des_yaw_;
    const double step = yaw_rate_max_ * dt;
    for (size_t q = 0; q + 1 < stops.size(); ++q) {
      const size_t i0 = stops[q], i1 = stops[q + 1];
      const std::vector<Vec3> Pp(P.begin() + i0, P.begin() + i1 + 1);
      const std::vector<double> vp(v.begin() + i0, v.begin() + i1 + 1);
      const std::vector<double> dsp(ds.begin() + i0, ds.begin() + i1);
      std::vector<Vec3> pos, vel, acc;
      std::vector<double> head;
      time_piece(Pp, vp, dsp, a_max, dt, pos, vel, acc, head);
      const auto first = std::find_if(head.begin(), head.end(), [](double h) {return !std::isnan(h);});
      if (first != head.end()) {                               // at a stop: turn in place to the new direction
        const double turn = std::atan2(std::sin(*first - yaw), std::cos(*first - yaw));
        const int nturn = static_cast<int>(std::ceil(std::abs(turn) / (0.6 * step)));   // cosine profile: peak ~0.94 * yaw_rate_max
        for (int j = 1; j <= nturn; ++j) {
          const double y = yaw + turn * (1 - std::cos(M_PI * j / nturn)) / 2;
          const double rate = turn * M_PI / (2 * nturn * dt) * std::sin(M_PI * j / nturn);
          append_setpoint(P[i0], Vec3::Zero(), Vec3::Zero(), y, rate);
        }
        yaw += turn;
      }
      for (size_t k = 0; k < pos.size(); ++k) {               // fly, facing yaw_lead m ahead (rate limited)
        double d = 0.0;
        if (!std::isnan(head[k])) {
          const double err = std::atan2(std::sin(head[k] - yaw), std::cos(head[k] - yaw));
          d = std::max(-step, std::min(step, err));
        }
        yaw += d;
        append_setpoint(pos[k], vel[k], acc[k], yaw, d / dt);
      }
    }
    des_yaw_ = yaw;
    double length = 0.0;
    for (double d : ds) {length += d;}
    RCLCPP_INFO(get_logger(), "smooth trajectory: %.1f s, %ld stops (takeoff + reversals), %.1f m",
      pos_traj_.size() * dt, static_cast<long>(stops.size()) - 2, length);
    trajectory_generated_ = true;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryGenerator>());
  rclcpp::shutdown();
  return 0;
}
