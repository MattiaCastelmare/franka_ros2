// SIM. ScenarioObstacles: kinematic obstacles that follow a scenario law at the physics rate.
//
// docs/gz_cbf_experiments_plan.md §5.4. A world system for Ignition Fortress (gazebo 6).
// In every PreUpdate it writes a WorldPoseCmd for each obstacle model. The pose
// comes from ONE law function evaluated for the end of the step, so reset and
// step cannot disagree (franka_rl's _obstacle_at rule). Laws:
//
//   static     p(t) = p0
//   linear     p(t) = p0 + v0 t
//   ballistic  p(t) = p0 + v0 t + 1/2 g t^2                 (exact, no integrator)
//   pursuit    p <- p + speed * (target - p)/|target - p| * dt
//              with target = the TRUE flange pose read from the ECM in the same
//              step, i.e. cbf_scenarios.Sphere.step(dt, flange)
//   oscillate  p(t) = p0 + axis * amplitude * sin(2 pi t / period + phase)  (exact)
//              A ball sweeping back and forth across the robot's path: it passes
//              the plane through p0 twice per period, at peak speed
//              amplitude * 2 pi / period. <axis> is normalised here, so the
//              amplitude is always metres along it.
//              <phase_deg> (degrees, default 0) shifts the sine's argument, which
//              is what decides WHERE THE BALL IS BORN: at 0 it starts at p0, the
//              CENTRE of the swing, which for a centre near the arm means it
//              spawns next to the robot. At -90 the law is -cos and it starts at
//              p0 - axis * amplitude, the far end, and sweeps in. simexp's
//              obstacle_laws.position/velocity carry the same term; the two are
//              the sim and the truth of one motion and must not disagree.
//   wander     p_i(t) = p0_i + box_i * sin(2 pi ratio_i t / period + phase_i)
//              A ball touring a BOX quasi-randomly: one independent sinusoid
//              per world axis, with INCOMMENSURATE frequency ratios (default
//              1, golden ratio, silver ratio), so the path never closes and
//              never repeats inside a run while staying C-infinity, bounded by
//              p0 +/- box and exactly reproducible. It is deliberately NOT a
//              seeded random walk: the truth side (simexp.obstacle_laws) must
//              evaluate the SAME motion, and no RNG is bit-identical across
//              C++ and numpy. Per-seed variety comes from the catalog's
//              jitter_m on p0, which translates the whole box.
//              <box> half-extents [m], <ratios> frequency multipliers,
//              <period> [s] the base period, <phase_deg_xyz> [deg] the three
//              phases, which decide where the ball is BORN. Peak speed on axis
//              i is box_i * 2 pi ratio_i / period.
//
// Before the episode starts the obstacles are PARKED out of view (unless
// <park>false</park>). The start time arrives as a sim time on a gz topic
// (gz.msgs.Double, bridged from the ROS episode supervisor). From that
// instant t = sim_time - t_start, so the first step moves an obstacle from
// p(0) to p(dt): a birth, never a teleport.
//
// Truth: every <truth_period> the system publishes the obstacles' world
// positions (gz.msgs.Pose_V on <truth_topic>) and their law velocities (the
// same message type on <truth_topic>_vel, velocity stored in `position`), both
// stamped with the sim time of the pose.
//
// The models must be visual-only (no <collision>, gravity off): the depth camera
// sees them, the arm cannot touch them (contact is measured geometrically).

#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/msgs/double.pb.h>
#include <ignition/msgs/pose_v.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace franka_simulation
{
using ignition::math::Pose3d;
using ignition::math::Vector3d;

namespace
{
Vector3d ParseVec(const std::string & s, const Vector3d & dflt)
{
  std::istringstream is(s);
  double x, y, z;
  if (is >> x >> y >> z) {
    return {x, y, z};
  }
  return dflt;
}

double ToSeconds(const std::chrono::steady_clock::duration & d)
{
  return std::chrono::duration<double>(d).count();
}
}  // namespace

struct Obstacle
{
  std::string model;
  std::string law{"static"};
  Vector3d p0, v0, gravity{0, 0, -9.81}, park{0, 0, -20}, axis{1, 0, 0};
  double speed{0.0};
  double amplitude{0.0}, period{0.0}, phase{0.0};   // phase in RADIANS
  Vector3d box{0, 0, 0}, ratios{1.0, 1.6180339887498949, 2.4142135623730951};
  Vector3d phase_xyz{0, 0, 0};                     // wander phases, RADIANS
  bool use_park{true};
  ignition::gazebo::Entity entity{ignition::gazebo::kNullEntity};
  Vector3d p, v;   // pursuit state, and the last commanded position / velocity
  Vector3d p_cmd;
};

class ScenarioObstacles : public ignition::gazebo::System,
  public ignition::gazebo::ISystemConfigure,
  public ignition::gazebo::ISystemPreUpdate,
  public ignition::gazebo::ISystemPostUpdate
{
public:
  void Configure(
    const ignition::gazebo::Entity &, const std::shared_ptr<const sdf::Element> & sdf,
    ignition::gazebo::EntityComponentManager &, ignition::gazebo::EventManager &) override
  {
    auto el = std::const_pointer_cast<sdf::Element>(sdf);
    start_topic_ = el->Get<std::string>("start_topic", "/sim/episode_start").first;
    truth_topic_ = el->Get<std::string>("truth_topic", "/sim/truth/obstacle_poses").first;
    truth_period_ = el->Get<double>("truth_period", 0.001).first;
    target_model_ = el->Get<std::string>("target_model", "fr3").first;
    target_link_ = el->Get<std::string>("target_link", "fr3_link7").first;
    target_offset_ = ParseVec(el->Get<std::string>("target_offset", "0 0 0.107").first,
        {0, 0, 0.107});
    for (auto o = el->GetElement("obstacle"); o; o = o->GetNextElement("obstacle")) {
      Obstacle ob;
      ob.model = o->Get<std::string>("model");
      ob.law = o->Get<std::string>("law", "static").first;
      ob.p0 = ParseVec(o->Get<std::string>("p0", "0 0 0").first, {0, 0, 0});
      ob.v0 = ParseVec(o->Get<std::string>("v0", "0 0 0").first, {0, 0, 0});
      ob.gravity = ParseVec(o->Get<std::string>("gravity", "0 0 -9.81").first, {0, 0, -9.81});
      ob.park = ParseVec(o->Get<std::string>("park_at", "0 0 -20").first, {0, 0, -20});
      ob.use_park = o->Get<bool>("park", true).first;
      ob.speed = o->Get<double>("speed", 0.0).first;
      ob.axis = ParseVec(o->Get<std::string>("axis", "1 0 0").first, {1, 0, 0});
      ob.amplitude = o->Get<double>("amplitude", 0.0).first;
      ob.period = o->Get<double>("period", 0.0).first;
      ob.phase = o->Get<double>("phase_deg", 0.0).first * M_PI / 180.0;
      // wander: one sinusoid per world axis. The defaults are the golden and
      // silver ratios, which is the whole point -- rational ratios would close
      // the curve into a repeating Lissajous figure.
      ob.box = ParseVec(o->Get<std::string>("box", "0 0 0").first, {0, 0, 0});
      ob.ratios = ParseVec(o->Get<std::string>("ratios",
          "1 1.6180339887498949 2.4142135623730951").first,
          {1.0, 1.6180339887498949, 2.4142135623730951});
      const Vector3d ph_deg = ParseVec(
          o->Get<std::string>("phase_deg_xyz", "0 0 0").first, {0, 0, 0});
      ob.phase_xyz = ph_deg * (M_PI / 180.0);
      ob.p = ob.p0;
      ob.v = Vector3d::Zero;
      if (ob.law != "static" && ob.law != "linear" && ob.law != "ballistic" &&
        ob.law != "pursuit" && ob.law != "oscillate" && ob.law != "wander")
      {
        ignerr << "[ScenarioObstacles] unknown law '" << ob.law << "' for " << ob.model << "\n";
        continue;
      }
      if (ob.law == "oscillate") {
        const double n = ob.axis.Length();
        if (n < 1e-9 || ob.period <= 0.0) {
          ignerr << "[ScenarioObstacles] oscillate needs a non-zero <axis> and a positive "
                 << "<period> for " << ob.model << "\n";
          continue;
        }
        ob.axis /= n;
      }
      if (ob.law == "wander" && ob.period <= 0.0) {
        ignerr << "[ScenarioObstacles] wander needs a positive <period> for "
               << ob.model << "\n";
        continue;
      }
      obstacles_.push_back(ob);
    }
    node_.Subscribe(start_topic_, &ScenarioObstacles::OnStart, this);
    pose_pub_ = node_.Advertise<ignition::msgs::Pose_V>(truth_topic_);
    vel_pub_ = node_.Advertise<ignition::msgs::Pose_V>(truth_topic_ + "_vel");
    ignmsg << "[ScenarioObstacles] SIM: " << obstacles_.size() << " obstacle(s), start on "
           << start_topic_ << ", truth on " << truth_topic_ << "\n";
  }

  void PreUpdate(const ignition::gazebo::UpdateInfo & info,
    ignition::gazebo::EntityComponentManager & ecm) override
  {
    if (info.paused) {
      return;
    }
    // gz-sim advances simTime BEFORE calling the systems, so in PreUpdate it is
    // already the time at the END of the step being computed: the pose for
    // this step is p(simTime), and PostUpdate stamps it with the same time.
    // (Measured: evaluating at simTime + dt put every law exactly one step ahead.)
    const double t_end = ToSeconds(info.simTime);
    const double dt = ToSeconds(info.dt);
    std::optional<double> t0;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      t0 = t_start_;
    }
    // 1 ns of tolerance: a start time on the step grid (3.53 s) arrives as the
    // double 3.5300000000000002 and must still start ON that step, at p(0).
    const bool running = t0.has_value() && t_end >= *t0 - 1e-9;
    std::optional<Vector3d> target;
    for (auto & ob : obstacles_) {
      if (ob.entity == ignition::gazebo::kNullEntity) {
        ob.entity = ecm.EntityByComponents(ignition::gazebo::components::Model(),
            ignition::gazebo::components::Name(ob.model));
        if (ob.entity == ignition::gazebo::kNullEntity) {
          continue;
        }
      }
      Vector3d p;
      if (!running) {
        p = ob.use_park ? ob.park : ob.p0;
        ob.v = Vector3d::Zero;
        ob.p = ob.p0;
      } else {
        const double t = std::max(0.0, t_end - *t0);
        if (ob.law == "static") {
          p = ob.p0;
          ob.v = Vector3d::Zero;
        } else if (ob.law == "linear") {
          p = ob.p0 + ob.v0 * t;
          ob.v = ob.v0;
        } else if (ob.law == "ballistic") {
          p = ob.p0 + ob.v0 * t + 0.5 * ob.gravity * t * t;
          ob.v = ob.v0 + ob.gravity * t;
        } else if (ob.law == "oscillate") {
          const double w = 2.0 * M_PI / ob.period;
          p = ob.p0 + ob.axis * (ob.amplitude * std::sin(w * t + ob.phase));
          ob.v = ob.axis * (ob.amplitude * w * std::cos(w * t + ob.phase));
        } else if (ob.law == "wander") {
          // Per-axis, so the three frequencies are independent: that is what
          // makes the tour quasi-random instead of a line segment.
          const double w0 = 2.0 * M_PI / ob.period;
          for (int i = 0; i < 3; ++i) {
            const double wi = w0 * ob.ratios[i];
            const double a = wi * t + ob.phase_xyz[i];
            p[i] = ob.p0[i] + ob.box[i] * std::sin(a);
            ob.v[i] = ob.box[i] * wi * std::cos(a);
          }
        } else {   // pursuit: the same time origin as the analytic laws
          if (!target) {
            target = Target(ecm);
          }
          // Integrate over the part of this step after t_start: on the first
          // running step that is t_end - t0, never a full dt before t0.
          const double h = std::min(dt, t);
          Vector3d d = target ? (*target - ob.p) : Vector3d::Zero;
          const double n = d.Length();
          ob.v = n > 1e-9 ? d * (ob.speed / n) : Vector3d::Zero;
          ob.p += ob.v * h;
          p = ob.p;
        }
      }
      ignition::gazebo::Model(ob.entity).SetWorldPoseCmd(ecm, Pose3d(p, {1, 0, 0, 0}));
      ob.p_cmd = p;
    }
  }

  void PostUpdate(const ignition::gazebo::UpdateInfo & info,
    const ignition::gazebo::EntityComponentManager & ecm) override
  {
    if (info.paused || obstacles_.empty()) {
      return;
    }
    const double t = ToSeconds(info.simTime);
    if (t + 1e-9 < next_truth_) {
      return;
    }
    next_truth_ = t + truth_period_;
    ignition::msgs::Pose_V poses, vels;
    const auto sec = static_cast<int64_t>(std::floor(t));
    const auto nsec = static_cast<int32_t>(std::llround((t - sec) * 1e9) % 1000000000LL);
    for (auto * m : {&poses, &vels}) {
      m->mutable_header()->mutable_stamp()->set_sec(sec);
      m->mutable_header()->mutable_stamp()->set_nsec(nsec);
    }
    for (const auto & ob : obstacles_) {
      if (ob.entity == ignition::gazebo::kNullEntity) {
        continue;
      }
      const auto pose = ignition::gazebo::worldPose(ob.entity, ecm);
      auto * pp = poses.add_pose();
      pp->set_name(ob.model);
      pp->mutable_position()->set_x(pose.Pos().X());
      pp->mutable_position()->set_y(pose.Pos().Y());
      pp->mutable_position()->set_z(pose.Pos().Z());
      pp->mutable_orientation()->set_w(1.0);
      auto * vv = vels.add_pose();
      vv->set_name(ob.model);
      vv->mutable_position()->set_x(ob.v.X());
      vv->mutable_position()->set_y(ob.v.Y());
      vv->mutable_position()->set_z(ob.v.Z());
      vv->mutable_orientation()->set_w(1.0);
    }
    pose_pub_.Publish(poses);
    vel_pub_.Publish(vels);
  }

private:
  void OnStart(const ignition::msgs::Double & msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!t_start_) {
      t_start_ = msg.data();
      ignmsg << "[ScenarioObstacles] episode start at t=" << msg.data() << " s (sim)\n";
    }
  }

  std::optional<Vector3d> Target(const ignition::gazebo::EntityComponentManager & ecm)
  {
    if (target_entity_ == ignition::gazebo::kNullEntity) {
      auto model = ecm.EntityByComponents(ignition::gazebo::components::Model(),
          ignition::gazebo::components::Name(target_model_));
      if (model == ignition::gazebo::kNullEntity) {
        return std::nullopt;
      }
      target_entity_ = ignition::gazebo::Model(model).LinkByName(ecm, target_link_);
      if (target_entity_ == ignition::gazebo::kNullEntity) {
        return std::nullopt;
      }
    }
    const auto pose = ignition::gazebo::worldPose(target_entity_, ecm);
    return pose.Pos() + pose.Rot().RotateVector(target_offset_);
  }

  std::vector<Obstacle> obstacles_;
  std::string start_topic_, truth_topic_, target_model_, target_link_;
  Vector3d target_offset_;
  double truth_period_{0.001};
  double next_truth_{0.0};
  ignition::gazebo::Entity target_entity_{ignition::gazebo::kNullEntity};
  std::mutex mutex_;
  std::optional<double> t_start_;
  ignition::transport::Node node_;
  ignition::transport::Node::Publisher pose_pub_, vel_pub_;
};

}  // namespace franka_simulation

IGNITION_ADD_PLUGIN(franka_simulation::ScenarioObstacles, ignition::gazebo::System,
  franka_simulation::ScenarioObstacles::ISystemConfigure,
  franka_simulation::ScenarioObstacles::ISystemPreUpdate,
  franka_simulation::ScenarioObstacles::ISystemPostUpdate)
IGNITION_ADD_PLUGIN_ALIAS(franka_simulation::ScenarioObstacles,
  "franka_simulation::ScenarioObstacles")
