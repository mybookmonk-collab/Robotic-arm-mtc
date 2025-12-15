#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/marker_tools.h>

#include <rviz_marker_tools/marker_creation.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/attached_body.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <fmt/format.h>
#include "std_msgs/msg/string.hpp"
#include <geometric_shapes/shape_operations.h>   // createMeshFromResource
#include <geometric_shapes/mesh_operations.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mutex>

#include <Eigen/Dense>
#include <cmath>
#include <nlohmann/json.hpp>
#include <functional>

using namespace moveit::task_constructor::stages;
using json = nlohmann::json;

std::vector<geometry_msgs::msg::Pose> ptube_world;
std::vector<geometry_msgs::msg::Pose> place_poses;
double t_z = 0.0;
double t_y = 0.35;
int i = 0;
double xs = 0.036;
std::string tube = "tube_";
geometry_msgs::msg::PoseStamped grasp_pose_msg_qj;
geometry_msgs::msg::Pose pose;

geometry_msgs::msg::Pose tube_rack_pose = []{
  geometry_msgs::msg::Pose p;
  p.position.x = -0.25;
  p.position.y = 0.62;
  p.position.z = t_z + 0.045;
  // p.orientation.x = 0.0;
  // p.orientation.y = 0.0;
  // p.orientation.z = 0.7071;
  // p.orientation.w = 0.7071;
  p.orientation.w = 1;
  return p;
}();

geometry_msgs::msg::Pose tube_rack_pose2 = []{
  geometry_msgs::msg::Pose p;
  p.position.x = 0.25;
  p.position.y = 0.62;
  p.position.z = t_z + 0.045;
  // p.orientation.x = 0.0;
  // p.orientation.y = 0.0;
  // p.orientation.z = 0.7071;
  // p.orientation.w = 0.7071;
  p.orientation.w = 1;
  return p;
}();

geometry_msgs::msg::Pose tube_rack_pose3 = []{
  geometry_msgs::msg::Pose p;
  p.position.x = -0.40;
  p.position.y = 0.62;
  p.position.z = t_z + 0.045;
  // p.orientation.x = 0.0;
  // p.orientation.y = 0.0;
  // p.orientation.z = 0.7071;
  // p.orientation.w = 0.7071;
  p.orientation.w = 1;
  return p;
}();

geometry_msgs::msg::Pose tube_rack_pose4 = []{
  geometry_msgs::msg::Pose p;
  p.position.x = 0.40;
  p.position.y = 0.62;
  p.position.z = t_z + 0.045;
  // p.orientation.x = 0.0;
  // p.orientation.y = 0.0;
  // p.orientation.z = 0.7071;
  // p.orientation.w = 0.7071;
  p.orientation.w = 1;
  return p;
}();

geometry_msgs::msg::Pose box_beaker = []{
  geometry_msgs::msg::Pose p;
  p.position.x = 0.8;
  p.position.y = 0.0;
  p.position.z = t_z - 0.0007;
  p.orientation.x = 0.0;
  p.orientation.y = 0.0;
  p.orientation.z = 1;
  p.orientation.w = 0.0;
  return p;
}();

geometry_msgs::msg::Pose beaker = []{
  geometry_msgs::msg::Pose p;
  p.position.x = 0.0;
  p.position.y = 0.70;
  p.position.z = t_z + 0.05;
  p.orientation.w = 1;
  return p;
}();

geometry_msgs::msg::Pose target_beaker = []{
  geometry_msgs::msg::Pose p;
  p.position.x = 0.7;
  p.position.y = 0.0;
  p.position.z = t_z + + 0.04;
  p.orientation.w = 1;
  return p;
}();

geometry_msgs::msg::Pose glove_boxp = []{
  geometry_msgs::msg::Pose p;
  p.position.x = 0.0;
  p.position.y = 0.0;
  p.position.z = 0.0;
  p.orientation.w = 1;
  return p;
}();

Eigen::Vector3d computeDirectionFromOrigin(double x, double y)
{
  Eigen::Vector3d dir(0, 0, 0);
  double norm = std::sqrt(x * x + y * y);
  if (norm < 1e-9) {
    // 原点与目标点重合，返回默认方向 (0,1,0)
    return Eigen::Vector3d(0, 1, 0);
  }
  dir << x / norm, y / norm, 0.0;
  return dir;
}

double alignZAxis(const Eigen::Quaterniond& q_end, const Eigen::Vector3d& z_world) {
    // 1. 提取末端 Z 轴方向
    Eigen::Vector3d z_end = q_end * Eigen::Vector3d::UnitZ();

    // 2. 投影到 XY 平面并归一化
    Eigen::Vector3d z_end_xy(z_end.x(), z_end.y(), 0.0);
    Eigen::Vector3d z_world_xy(z_world.x(), z_world.y(), 0.0);

    if (z_end_xy.norm() < 1e-6 || z_world_xy.norm() < 1e-6) {
        std::cerr << "Warning: Zero-length vector in XY plane!" << std::endl;
        return 0;
    }

    z_end_xy.normalize();
    z_world_xy.normalize();

    // 3. 计算旋转角度（弧度）
    double angle = std::atan2(z_world_xy.y(), z_world_xy.x()) - std::atan2(z_end_xy.y(), z_end_xy.x());

    return angle;
}

double computeBaseRotation(
    double x_c, double y_c,
    double x_t, double y_t
) {
    auto normalize = [](double a) {
        while (a > M_PI)  a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    };

    double theta_c = std::atan2(y_c, x_c); // 当前末端方向角
    double theta_t = std::atan2(y_t, x_t); // 目标点方向角

    double delta = normalize(theta_t - theta_c);

    return delta;
}


class MoveRelativeRefactor : public moveit::task_constructor::stages::MoveRelative {
public:
  using PreComputeCallback =
      std::function<void(MoveRelativeRefactor*,
                         const moveit::task_constructor::InterfaceState&)>;

  MoveRelativeRefactor(const std::string& name,
                        const moveit::task_constructor::solvers::PlannerInterfacePtr& planner)
      : moveit::task_constructor::stages::MoveRelative(name, planner) {}

  void setPreComputeCallback(PreComputeCallback cb) {
    pre_compute_cb_ = cb;
  }

protected:
  bool compute(const moveit::task_constructor::InterfaceState& state,
               planning_scene::PlanningScenePtr& scene,
               moveit::task_constructor::SubTrajectory& solution,
               moveit::task_constructor::Interface::Direction dir) override {
    if (pre_compute_cb_)
      pre_compute_cb_(this, state);

    return MoveRelative::compute(state, scene, solution, dir);
  }

private:
  PreComputeCallback pre_compute_cb_;
};


std::vector<geometry_msgs::msg::Pose> getTubeWorldPosesSorted(
    const geometry_msgs::msg::Pose& rack_pose,
    bool nearest_first = true  // true = 近→远, false = 远→近
) {
    double dx = 0.072;
    double dy = 0.036;
    // double z  = 0.024;
    double z  = 0.07;

    // 每组局部坐标
    std::array<std::array<tf2::Vector3, 3>, 2> tube_groups = {{
        { tf2::Vector3(-dy, -dx-0.018, z), tf2::Vector3(-dy, -0.018, z), tf2::Vector3(-dy, dx-0.018, z) },
        // { tf2::Vector3(0.0, -dx-0.018, z), tf2::Vector3(0.0, -0.018, z), tf2::Vector3(0.0, dx-0.018, z) },
        { tf2::Vector3(dy, -dx-0.018, z), tf2::Vector3(dy, -0.018, z), tf2::Vector3(dy, dx-0.018, z) }
    }};

    // rack -> world
    tf2::Quaternion q_rack(rack_pose.orientation.x, rack_pose.orientation.y, rack_pose.orientation.z, rack_pose.orientation.w);
    tf2::Transform rack_tf(tf2::Transform(q_rack, tf2::Vector3(rack_pose.position.x, rack_pose.position.y, rack_pose.position.z)));

    // 转换到世界坐标
    std::vector<std::vector<geometry_msgs::msg::Pose>> world_groups(tube_groups.size());
    for (size_t i = 0; i < tube_groups.size(); ++i) {
        for (const auto& local : tube_groups[i]) {
            tf2::Vector3 global = rack_tf * local;
            geometry_msgs::msg::Pose p;
            p.position.x = global.x();
            p.position.y = global.y();
            p.position.z = global.z();
            p.orientation = rack_pose.orientation;
            world_groups[i].push_back(p);
        }
    }

    // 每组内部排序：按试管到原点距离
    for (auto& group : world_groups) {
        std::sort(group.begin(), group.end(),
                  [](const geometry_msgs::msg::Pose& a, const geometry_msgs::msg::Pose& b) {
                      tf2::Vector3 va(a.position.x, a.position.y, a.position.z);
                      tf2::Vector3 vb(b.position.x, b.position.y, b.position.z);
                      return va.length2() < vb.length2();
                  });
    }

    // 组排序：按组中心到原点距离
    std::sort(world_groups.begin(), world_groups.end(),
              [](const std::vector<geometry_msgs::msg::Pose>& a, const std::vector<geometry_msgs::msg::Pose>& b) {
                  tf2::Vector3 center_a(0,0,0), center_b(0,0,0);
                  for (const auto& p : a) center_a += tf2::Vector3(p.position.x, p.position.y, p.position.z);
                  for (const auto& p : b) center_b += tf2::Vector3(p.position.x, p.position.y, p.position.z);
                  center_a /= a.size();
                  center_b /= b.size();
                  return center_a.length2() < center_b.length2();
              });

    if (!nearest_first) {
        std::reverse(world_groups.begin(), world_groups.end());
        for (auto& group : world_groups) {
            std::reverse(group.begin(), group.end());
        }
    }

    // 展开成单个 vector
    std::vector<geometry_msgs::msg::Pose> tube_world;
    for (const auto& group : world_groups) {
        tube_world.insert(tube_world.end(), group.begin(), group.end());
    }

    return tube_world;
}

struct IKSolution
{
    std::vector<double> joint_positions;
    collision_detection::Contact contact;
    bool collision_free;
    bool satisfies_constraints;
};

using IKSolutions = std::vector<IKSolution>;


static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  bool doTask(int typeint);

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask(int typeint);
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}

void MTCTaskNode::setupPlanningScene()
{
  moveit::planning_interface::PlanningSceneInterface psi;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

  const std::string tube_mesh_path = "package://crt_ctag2f90c_gripper_visualization/meshes/sg_exp.stl";
  const std::string rack_mesh_path = "package://crt_ctag2f90c_gripper_visualization/meshes/xsgj_exp.stl";
  const std::string beaker_mesh_path = "package://crt_ctag2f90c_gripper_visualization/meshes/sb_exp.stl";
  const std::string box_mesh_path = "package://crt_ctag2f90c_gripper_visualization/meshes/box.stl";
  const std::string glove_box_path = "package://crt_ctag2f90c_gripper_visualization/meshes/glove_box.STL";
  // === 生成 18 个试管 ===
  {
    ptube_world = getTubeWorldPosesSorted(tube_rack_pose,true);
    place_poses = getTubeWorldPosesSorted(tube_rack_pose4,false);
    std::vector<geometry_msgs::msg::Pose> end_tube = getTubeWorldPosesSorted(tube_rack_pose3,true);
    ptube_world.insert(ptube_world.end(), end_tube.begin(), end_tube.end());
    end_tube = getTubeWorldPosesSorted(tube_rack_pose2,false);
    place_poses.insert(place_poses.end(), end_tube.begin(), end_tube.end());
    int id_counter = 0;
    std::vector<geometry_msgs::msg::Pose> result;
    result.insert(result.end(), place_poses.begin(), place_poses.begin() + std::min(i, static_cast<int>(place_poses.size())));
    result.insert(result.end(), ptube_world.begin() + i, ptube_world.end());

    for (const auto& tp : result) {
      moveit_msgs::msg::CollisionObject tube;
      tube.id = "tube_" + std::to_string(id_counter++);
      tube.header.frame_id = "world";

      tube.primitive_poses.resize(1);
      tube.primitive_poses[0] = tp;  // 直接赋值姿态

      tube.primitives.resize(1);
      tube.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;  // 更像试管
      tube.primitives[0].dimensions = {0.12, 0.007};  // 高度=0.1, 半径=0.01

      tube.operation = tube.ADD;
      collision_objects.push_back(tube);
    }
  }

  // === 添加试管架 ===
  {
    moveit_msgs::msg::CollisionObject rack;
    rack.id = "tube_rack";
    rack.header.frame_id = "world";

    shapes::Mesh* m = shapes::createMeshFromResource(rack_mesh_path);
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(m, mesh_msg);
    shape_msgs::msg::Mesh mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

    rack.meshes.push_back(mesh);
    rack.mesh_poses.push_back(tube_rack_pose);  // 用同一个 rack_pose

    rack.operation = rack.ADD;
    collision_objects.push_back(rack);
  }

  {
    moveit_msgs::msg::CollisionObject rack;
    rack.id = "tube_rack2";
    rack.header.frame_id = "world";

    shapes::Mesh* m = shapes::createMeshFromResource(rack_mesh_path);
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(m, mesh_msg);
    shape_msgs::msg::Mesh mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

    rack.meshes.push_back(mesh);
    rack.mesh_poses.push_back(tube_rack_pose2);  // 用同一个 rack_pose

    rack.operation = rack.ADD;
    collision_objects.push_back(rack);
  }

  {
    moveit_msgs::msg::CollisionObject rack;
    rack.id = "tube_rack3";
    rack.header.frame_id = "world";

    shapes::Mesh* m = shapes::createMeshFromResource(rack_mesh_path);
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(m, mesh_msg);
    shape_msgs::msg::Mesh mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

    rack.meshes.push_back(mesh);
    rack.mesh_poses.push_back(tube_rack_pose3);  // 用同一个 rack_pose

    rack.operation = rack.ADD;
    collision_objects.push_back(rack);
  }

  {
    moveit_msgs::msg::CollisionObject rack;
    rack.id = "tube_rack4";
    rack.header.frame_id = "world";

    shapes::Mesh* m = shapes::createMeshFromResource(rack_mesh_path);
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(m, mesh_msg);
    shape_msgs::msg::Mesh mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

    rack.meshes.push_back(mesh);
    rack.mesh_poses.push_back(tube_rack_pose4);  // 用同一个 rack_pose

    rack.operation = rack.ADD;
    collision_objects.push_back(rack);
  }

  {
    moveit_msgs::msg::CollisionObject rack;
    rack.id = "box_rack";
    rack.header.frame_id = "world";

    shapes::Mesh* m = shapes::createMeshFromResource(box_mesh_path);
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(m, mesh_msg);
    shape_msgs::msg::Mesh mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

    rack.meshes.push_back(mesh);
    rack.mesh_poses.push_back(box_beaker);  // 用同一个 rack_pose

    rack.operation = rack.ADD;
    collision_objects.push_back(rack);
  }

  {
    moveit_msgs::msg::CollisionObject rack;
    rack.id = "beaker_rack";
    rack.header.frame_id = "world";

    shapes::Mesh* m = shapes::createMeshFromResource(beaker_mesh_path);
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(m, mesh_msg);
    shape_msgs::msg::Mesh mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

    rack.meshes.push_back(mesh);
    rack.mesh_poses.push_back(beaker);

    rack.operation = rack.ADD;
    collision_objects.push_back(rack);
  }

  {
    moveit_msgs::msg::CollisionObject rack;
    rack.id = "glove_box";
    rack.header.frame_id = "world";

    shapes::Mesh* m = shapes::createMeshFromResource(glove_box_path);
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(m, mesh_msg);
    shape_msgs::msg::Mesh mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

    rack.meshes.push_back(mesh);
    rack.mesh_poses.push_back(glove_boxp);

    rack.operation = rack.ADD;
    collision_objects.push_back(rack);
  }

  psi.applyCollisionObjects(collision_objects);
}

bool MTCTaskNode::doTask(int typeint)
{
  task_ = createTask(typeint);

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return false;
  }

  try
  {
    if (!task_.plan(5))
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
      return false;
    }
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return false;
  }

  // 执行任务
  // task_.introspection().publishSolution(*task_.solutions().front());
  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return false;
  }

  task_.clear();
  return true;
}

mtc::Task MTCTaskNode::createTask(int typeint)
{
  mtc::Task task;
  // 设置任务名称和加载机器人模型
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "elfin_arm";
  const auto& hand_group_name = "gripper";
  const auto& hand_frame = "elfin_end_link";

  // 这些属性可以在任务的各个阶段中共享和使用
  // 设置机械臂的关节组名称
  task.setProperty("group", arm_group_name);
  // 设置末端执行器的关节组名称
  task.setProperty("eef", hand_group_name);
  // 会以这个坐标系为参考来求解关节角度
  task.setProperty("ik_frame", hand_frame);

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop
  // 添加一个CurrentState阶段
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  // 获取当前机器人的状态
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  // 创建采样规划器、插值规划器和笛卡尔路径规划器
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>( node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  auto cartesian_planner2 = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner2->setMaxVelocityScalingFactor(1.0);
  cartesian_planner2->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner2->setStepSize(0.1);
  cartesian_planner2->setMinFraction(0);

  auto cartesian_planner3 = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner3->setMaxVelocityScalingFactor(1.0);
  cartesian_planner3->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner3->setJumpThreshold(0.0);  // 关闭跳跃检测
  cartesian_planner3->setStepSize(0.01);

  auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("one open hand" + std::to_string(typeint), interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal(typeint == 1 ? "open" : "open_cup");
  task.add(std::move(stage_open_hand));

  //添加一个Connect阶段，将机械臂从当前位置移动到抓取位置
  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  switch (typeint)
  {
      case 1:
          // 使用SerialContainer创建一个抓取物体的子任务，包含接近物体、生成抓取姿态、碰撞、关闭手爪、抓取物体和提升物体等阶段
          {
            auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
            task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
            grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                                  { "eef", "group", "ik_frame" });
            {
              auto stage =
                  std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
              stage->properties().set("marker_ns", "approach_object");
              stage->properties().set("link", hand_frame);
              stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
              stage->setMinMaxDistance(0.1, 0.15);

              // Set hand forward direction
              geometry_msgs::msg::Vector3Stamped vec;
              vec.header.frame_id = hand_frame;
              vec.vector.z = 1;
              stage->setDirection(vec);
              grasp->insert(std::move(stage));
            }

            {
              // Sample grasp pose
              auto stage = std::make_unique<moveit::task_constructor::stages::GenerateGraspPose>("generate grasp pose");
              stage->properties().configureInitFrom(mtc::Stage::PARENT);
              stage->properties().set("marker_ns", "grasp_pose");
              stage->setPreGraspPose("open");
              stage->setObject(tube);
              stage->setAngleDelta(M_PI / 24);
              stage->setRotationAxis(Eigen::Vector3d(0, 0, 1)); // 默认是Z轴
              stage->setMonitoredStage(current_state_ptr);  // Hook into current state

              Eigen::Isometry3d grasp_frame_transform;
              Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                                    Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
              // Eigen::Quaterniond q(0.5, -0.5, -0.5, -0.5);
              pose.orientation.x = q.x();
              pose.orientation.y = q.y();
              pose.orientation.z = q.z();
              pose.orientation.w = q.w();
              
              grasp_frame_transform.linear() = q.matrix();
              Eigen::Quaterniond quat(grasp_frame_transform.rotation());
              grasp_frame_transform.translation().z() = 0.213;  // 目标在 jaw_link，jaw 比 wrist 再向下 15cm
              grasp_frame_transform.translation().x() = -0.03;  // 目标在 jaw_link，jaw 比 wrist 再向下 15cm
              geometry_msgs::msg::PoseStamped target_pose;
              target_pose.header.frame_id = "world";
              auto wrapper =
                  std::make_unique<moveit::task_constructor::stages::ComputeIK>("grasp pose IK", std::move(stage));
              wrapper->setMaxIKSolutions(8);
              wrapper->setMinSolutionDistance(1.0);
              wrapper->setIKFrame(grasp_frame_transform, hand_frame);
              wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
              wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });

              grasp->insert(std::move(wrapper));
            }

            {
              auto stage =
                  std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
              stage->allowCollisions(tube,
                                    task.getRobotModel()
                                        ->getJointModelGroup(hand_group_name)
                                        ->getLinkModelNamesWithCollisionGeometry(),
                                    true);
              grasp->insert(std::move(stage));
            }

            {
              auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
              stage->setGroup(hand_group_name);
              stage->setGoal("closed");
              grasp->insert(std::move(stage));
            }

            {
              auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
              stage->attachObject(tube, hand_frame);
              grasp->insert(std::move(stage));
            }

            {
              auto stage =
                  std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
              stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
              stage->setMinMaxDistance(0.1, 0.15);
              stage->setIKFrame(hand_frame);
              stage->properties().set("marker_ns", "lift_object");

              // Set upward direction
              geometry_msgs::msg::Vector3Stamped vec;
              vec.header.frame_id = "world";
              vec.vector.z = 1.0;
              stage->setDirection(vec);
              grasp->insert(std::move(stage));
            }

            {
              auto pour_motion = std::make_unique<MoveRelativeRefactor>("dynamic diagonal", interpolation_planner);
              pour_motion->setGroup(arm_group_name);
              pour_motion->setPreComputeCallback(
                [](MoveRelativeRefactor* self,const moveit::task_constructor::InterfaceState& state) {
                  planning_scene::PlanningScenePtr scene_p = state.scene()->diff();
                  Eigen::Vector3d position = scene_p->getFrameTransform("elfin_end_link").translation();
                  double fd = computeBaseRotation(position.x(), position.y(), beaker.position.x, beaker.position.y);
                  std::map<std::string, double> joint_target;
                  joint_target["elfin_joint1"] = fd;
                  self->setProperty("direction", joint_target);
                }
              );
              grasp->insert(std::move(pour_motion));
            }

            {

              auto move_relative = std::make_unique<MoveRelativeRefactor>("posture correction", cartesian_planner);
              move_relative->properties().configureInitFrom(mtc::Stage::PARENT, {"group","ik_frame"});
              move_relative->setGroup(arm_group_name);
              move_relative->setIKFrame(hand_frame);
              move_relative->setPreComputeCallback(
                [](MoveRelativeRefactor* self,const moveit::task_constructor::InterfaceState& state) {
                  planning_scene::PlanningScenePtr scene_p = state.scene()->diff();
                  const auto& robot_state = scene_p->getCurrentState();
                  Eigen::Isometry3d ee_pose = robot_state.getGlobalLinkTransform(hand_frame);
                  Eigen::Quaterniond orientation(ee_pose.rotation());

                  Eigen::Vector3d z_world = computeDirectionFromOrigin(beaker.position.x, beaker.position.y);

                  double theta_rad = alignZAxis(orientation, z_world);

                  geometry_msgs::msg::TwistStamped twist;
                  twist.header.frame_id = "world";  // 相对于夹爪局部坐标系
                  twist.twist.linear.x = 0.0;
                  twist.twist.linear.y = 0.0;
                  twist.twist.linear.z = 0.0;

                  // 绕局部 Y 轴旋转 90°（单位：弧度）
                  twist.twist.angular.x = 0.0;
                  twist.twist.angular.y = 0.0;  // pitch
                  twist.twist.angular.z = theta_rad;
                  self->setProperty("direction", twist);
                }
              );
              move_relative->setMinDistance(0.1);
              grasp->insert(std::move(move_relative));
            }

            {
              auto stage = std::make_unique<MoveRelativeRefactor>("move horizontally",interpolation_planner);
              stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group", "ik_frame" });
              stage->setGroup(arm_group_name);
              stage->setIKFrame(hand_frame);
              stage->setPreComputeCallback(
                [](MoveRelativeRefactor* self,const moveit::task_constructor::InterfaceState& state) {
                  planning_scene::PlanningScenePtr scene_p = state.scene()->diff();
                  const auto& robot_state = scene_p->getCurrentState();
                  Eigen::Vector3d position = scene_p->getFrameTransform(tube).translation();
                  auto positions = robot_state.getJointPositions("elfin_joint6");
                  int direction = 1;
                  if (positions[0] > 0)
                    direction = -1;
                  geometry_msgs::msg::Vector3Stamped metry;
                  metry.header.frame_id = "world";
                  metry.vector.x = beaker.position.x - 0.05 * direction - position.x();
                  metry.vector.y = beaker.position.y - position.y();
                  metry.vector.z = beaker.position.z+0.15 - position.z();
                  self->setProperty("direction", metry);
                }
              );
              grasp->insert(std::move(stage));
            }

            {
              auto pour_motion = std::make_unique<MoveRelativeRefactor>("dynamic water pouring", interpolation_planner);
              pour_motion->setGroup(arm_group_name);
              pour_motion->setPreComputeCallback(
                [](MoveRelativeRefactor* self,const moveit::task_constructor::InterfaceState& state) {
                  planning_scene::PlanningScenePtr scene_p = state.scene()->diff();
                  Eigen::Vector3d position = scene_p->getFrameTransform(tube).translation();
                  int direction = 1;
                  if (position.x() > beaker.position.x)
                    direction = -1;
                  std::map<std::string, double> joint_target;
                  joint_target["elfin_joint6"] = direction * 100.0 * M_PI / 180.0;
                  self->setProperty("direction", joint_target);
                }
              );
              grasp->insert(std::move(pour_motion));
            }

            {
              auto stage = std::make_unique<mtc::stages::MoveRelative>("stay up", cartesian_planner);
              stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
              stage->setMinMaxDistance(0, 0.03);
              stage->setIKFrame(hand_frame);
              geometry_msgs::msg::Vector3Stamped vec;
              vec.header.frame_id = "world";
              vec.vector.z = 1.0;
              stage->setDirection(vec);
              grasp->insert(std::move(stage));
            }

            {
              auto stage = std::make_unique<mtc::stages::MoveRelative>("stay down", cartesian_planner);
              stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
              stage->setMinMaxDistance(0, 0.03);
              stage->setIKFrame(hand_frame);
              geometry_msgs::msg::Vector3Stamped vec;
              vec.header.frame_id = "world";
              vec.vector.z = -1.0;
              stage->setDirection(vec);
              grasp->insert(std::move(stage));
            }

            {
              auto pour_motion = std::make_unique<MoveRelativeRefactor>("dynamic return to normal", cartesian_planner);
              pour_motion->setGroup(arm_group_name);
              pour_motion->setPreComputeCallback(
                [](MoveRelativeRefactor* self,const moveit::task_constructor::InterfaceState& state) {
                  planning_scene::PlanningScenePtr scene_p = state.scene()->diff();
                  Eigen::Vector3d position = scene_p->getFrameTransform(tube).translation();
                  int direction = -1;
                  if (position.x() > beaker.position.x)
                    direction = 1;
                  std::map<std::string, double> joint_target;
                  joint_target["elfin_joint6"] = direction * 100.0 * M_PI / 180.0;
                  self->setProperty("direction", joint_target);
                }
              );
              grasp->insert(std::move(pour_motion));
            }
            {
              auto stage = std::make_unique<mtc::stages::MoveRelative>("stay up", cartesian_planner3);
              stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
              stage->setMinMaxDistance(0.05, 0.1);
              stage->setIKFrame(hand_frame);
              geometry_msgs::msg::Vector3Stamped vec;
              vec.header.frame_id = "world";
              vec.vector.z = 1.0;
              stage->setDirection(vec);

              grasp->insert(std::move(stage));
            }

            {
              auto pour_motion = std::make_unique<MoveRelativeRefactor>("dynamic diagonal", interpolation_planner);
              pour_motion->setGroup(arm_group_name);
              pour_motion->setPreComputeCallback(
                [](MoveRelativeRefactor* self,const moveit::task_constructor::InterfaceState& state) {
                  planning_scene::PlanningScenePtr scene_p = state.scene()->diff();
                  Eigen::Vector3d position = scene_p->getFrameTransform(tube).translation();
                  double fd = computeBaseRotation(position.x(), position.y(), place_poses[i].position.x, place_poses[i].position.y);
                  std::map<std::string, double> joint_target;
                  joint_target["elfin_joint1"] = fd;
                  self->setProperty("direction", joint_target);
                }
              );
              grasp->insert(std::move(pour_motion));
            }

            {
              auto stage = std::make_unique<MoveRelativeRefactor>("move horizontally",cartesian_planner3);
              stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group", "ik_frame" });
              stage->setGroup(arm_group_name);
              stage->setIKFrame(hand_frame);
              stage->setPreComputeCallback(
                [](MoveRelativeRefactor* self,const moveit::task_constructor::InterfaceState& state) {
                  planning_scene::PlanningScenePtr scene_p = state.scene()->diff();
                  Eigen::Vector3d position = scene_p->getFrameTransform(tube).translation();
                  geometry_msgs::msg::Vector3Stamped metry;
                  metry.header.frame_id = "world";
                  metry.vector.x = place_poses[i].position.x - position.x();
                  metry.vector.y = place_poses[i].position.y - position.y();
                  metry.vector.z = place_poses[i].position.z+0.12 - position.z();
                  self->setProperty("direction", metry);
                }
              );
              grasp->insert(std::move(stage));
            }

            {
              auto stage = std::make_unique<MoveRelativeRefactor>("f4", cartesian_planner);
              stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
              stage->setIKFrame(hand_frame);
              stage->properties().set("marker_ns", "f4");
              stage->setPreComputeCallback(
                [](MoveRelativeRefactor* self,const moveit::task_constructor::InterfaceState& state) {
                  planning_scene::PlanningScenePtr scene_p = state.scene()->diff();
                  Eigen::Vector3d position = scene_p->getFrameTransform(tube).translation();
                  geometry_msgs::msg::Vector3Stamped vec;
                  vec.header.frame_id = "world";
                  vec.vector.z = -1.0;
                  self->setProperty("direction", vec);
                  self->setMinMaxDistance(position.z() - place_poses[i].position.z - 0.01, position.z() - place_poses[i].position.z  - 0.01);
                }
              );
              grasp->insert(std::move(stage));
            }


            // 添加一个MoveTo阶段，打开夹爪
            {
              auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
              stage->setGroup(hand_group_name);
              stage->setGoal("open");
              grasp->insert(std::move(stage));
            }

            // 防止碰撞
            {
              auto stage =
                  std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
              stage->allowCollisions(tube,
                                    task.getRobotModel()
                                        ->getJointModelGroup(hand_group_name)
                                        ->getLinkModelNamesWithCollisionGeometry(),
                                    false);
              grasp->insert(std::move(stage));
            }

            // 分离物体
            {
              auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
              stage->detachObject(tube, hand_frame);
              grasp->insert(std::move(stage));
            }

            // 回撤
            {
              auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
              stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
                    stage->setMinMaxDistance(0.1, 0.2);
              stage->setIKFrame(hand_frame);
              stage->properties().set("marker_ns", "retreat");

              geometry_msgs::msg::Vector3Stamped vec;
              vec.header.frame_id = "world";
              vec.vector.z = 1.0;
              stage->setDirection(vec);
              grasp->insert(std::move(stage));
            }

            task.add(std::move(grasp));
          }
          break;

      case 2:
          // 执行任务2
          {
            auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
            task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
            grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                                  { "eef", "group", "ik_frame" });
            {
              auto stage =
                  std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
              stage->properties().set("marker_ns", "approach_object");
              stage->properties().set("link", hand_frame);
              stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
              stage->setMinMaxDistance(0.1, 0.15);

              // Set hand forward direction
              geometry_msgs::msg::Vector3Stamped vec;
              vec.header.frame_id = hand_frame;
              vec.vector.z = 1;
              stage->setDirection(vec);
              grasp->insert(std::move(stage));
            }

            {
              // Sample grasp pose
              auto stage = std::make_unique<moveit::task_constructor::stages::GenerateGraspPose>("generate grasp pose");
              stage->properties().configureInitFrom(mtc::Stage::PARENT);
              stage->properties().set("marker_ns", "grasp_pose");
              stage->setPreGraspPose("open_cup");
              stage->setObject(tube);
              stage->setAngleDelta(M_PI / 24);
              stage->setRotationAxis(Eigen::Vector3d(0, 0, 1)); // 默认是Z轴
              stage->setMonitoredStage(current_state_ptr);  // Hook into current state

              Eigen::Isometry3d grasp_frame_transform;
              Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                                    Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
              // Eigen::Quaterniond q(0.5, -0.5, -0.5, -0.5);
              pose.orientation.x = q.x();
              pose.orientation.y = q.y();
              pose.orientation.z = q.z();
              pose.orientation.w = q.w();
              
              grasp_frame_transform.linear() = q.matrix();
              Eigen::Quaterniond quat(grasp_frame_transform.rotation());
              grasp_frame_transform.translation().z() = 0.213;  // 目标在 jaw_link，jaw 比 wrist 再向下 15cm
              grasp_frame_transform.translation().x() = -0.05;  // 目标在 jaw_link，jaw 比 wrist 再向下 15cm
              geometry_msgs::msg::PoseStamped target_pose;
              target_pose.header.frame_id = "world";
              auto wrapper =
                  std::make_unique<moveit::task_constructor::stages::ComputeIK>("grasp pose IK", std::move(stage));
              wrapper->setMaxIKSolutions(8);
              wrapper->setMinSolutionDistance(1.0);
              wrapper->setIKFrame(grasp_frame_transform, hand_frame);
              wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
              wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
              grasp->insert(std::move(wrapper));
            }

            {
              auto stage =
                  std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
              stage->allowCollisions(tube,
                                    task.getRobotModel()
                                        ->getJointModelGroup(hand_group_name)
                                        ->getLinkModelNamesWithCollisionGeometry(),
                                    true);
              grasp->insert(std::move(stage));
            }

            {
              auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
              stage->setGroup(hand_group_name);
              stage->setGoal("closed_cup");
              grasp->insert(std::move(stage));
            }

            {
              auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
              stage->attachObject(tube, hand_frame);
              grasp->insert(std::move(stage));
            }

            {
              auto stage =
                  std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
              stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
              stage->setMinMaxDistance(0.1, 0.15);
              stage->setIKFrame(hand_frame);
              stage->properties().set("marker_ns", "lift_object");

              // Set upward direction
              geometry_msgs::msg::Vector3Stamped vec;
              vec.header.frame_id = "world";
              vec.vector.z = 1.0;
              stage->setDirection(vec);
              grasp->insert(std::move(stage));
            }

            {
              auto stage = std::make_unique<MoveRelativeRefactor>("Intelligent advancement", cartesian_planner);
              stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
              stage->setIKFrame(hand_frame);
              stage->properties().set("marker_ns", "Intelligent advancement");
              stage->setPreComputeCallback(
                [](MoveRelativeRefactor* self,const moveit::task_constructor::InterfaceState& state) {
                  planning_scene::PlanningScenePtr scene_p = state.scene()->diff();
                  Eigen::Vector3d position = scene_p->getFrameTransform(tube).translation();
                  geometry_msgs::msg::Vector3Stamped vec;
                  vec.header.frame_id = hand_frame;
                  vec.vector.z = -1.0;
                  self->setProperty("direction", vec);
                  self->setMinMaxDistance(position.y() - 0.55, position.y() - 0.45);
                }
              );
              grasp->insert(std::move(stage));
            }

            {
              auto pour_motion = std::make_unique<MoveRelativeRefactor>("dynamic diagonal", interpolation_planner);
              pour_motion->setGroup(arm_group_name);
              pour_motion->setPreComputeCallback(
                [](MoveRelativeRefactor* self,const moveit::task_constructor::InterfaceState& state) {
                  planning_scene::PlanningScenePtr scene_p = state.scene()->diff();
                  Eigen::Vector3d position = scene_p->getFrameTransform(tube).translation();
                  double fd = computeBaseRotation(position.x(), position.y(), target_beaker.position.x, target_beaker.position.y);
                  std::map<std::string, double> joint_target;
                  joint_target["elfin_joint1"] = fd;
                  self->setProperty("direction", joint_target);
                }
              );
              grasp->insert(std::move(pour_motion));
            }

            {

              auto move_relative = std::make_unique<MoveRelativeRefactor>("posture correction", cartesian_planner);
              move_relative->properties().configureInitFrom(mtc::Stage::PARENT, {"group","ik_frame"});
              move_relative->setGroup(arm_group_name);
              move_relative->setIKFrame(hand_frame);
              move_relative->setPreComputeCallback(
                [](MoveRelativeRefactor* self,const moveit::task_constructor::InterfaceState& state) {
                  planning_scene::PlanningScenePtr scene_p = state.scene()->diff();
                  const auto& robot_state = scene_p->getCurrentState();
                  Eigen::Isometry3d ee_pose = robot_state.getGlobalLinkTransform(hand_frame);
                  Eigen::Quaterniond orientation(ee_pose.rotation());

                  Eigen::Vector3d z_world = computeDirectionFromOrigin(target_beaker.position.x, target_beaker.position.y);
                  double theta_rad = alignZAxis(orientation, z_world);;

                  geometry_msgs::msg::TwistStamped twist;
                  twist.header.frame_id = "world";  // 相对于夹爪局部坐标系
                  twist.twist.linear.x = 0.0;
                  twist.twist.linear.y = 0.0;
                  twist.twist.linear.z = 0.0;

                  // 绕局部 Y 轴旋转 90°（单位：弧度）
                  twist.twist.angular.x = 0.0;
                  twist.twist.angular.y = 0.0;  // pitch
                  twist.twist.angular.z = theta_rad;
                  self->setProperty("direction", twist);
                }
              );
              move_relative->setMinDistance(0.1);
              grasp->insert(std::move(move_relative));
            }

            {
              auto pour_motion = std::make_unique<MoveRelativeRefactor>("dynamic diagonal", interpolation_planner);
              pour_motion->setGroup(arm_group_name);
              pour_motion->setPreComputeCallback(
                [](MoveRelativeRefactor* self,const moveit::task_constructor::InterfaceState& state) {
                  planning_scene::PlanningScenePtr scene_p = state.scene()->diff();
                  Eigen::Vector3d position = scene_p->getFrameTransform(tube).translation();
                  double fd = computeBaseRotation(position.x(), position.y(), target_beaker.position.x, target_beaker.position.y);
                  std::map<std::string, double> joint_target;
                  joint_target["elfin_joint1"] = fd;
                  self->setProperty("direction", joint_target);
                }
              );
              grasp->insert(std::move(pour_motion));
            }

            {
              auto stage = std::make_unique<MoveRelativeRefactor>("Intelligent advancement", cartesian_planner);
              stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
              stage->setIKFrame(hand_frame);
              stage->properties().set("marker_ns", "Intelligent advancement");
              stage->setPreComputeCallback(
                [](MoveRelativeRefactor* self,const moveit::task_constructor::InterfaceState& state) { 
                  planning_scene::PlanningScenePtr scene_p = state.scene()->diff();
                  Eigen::Vector3d position = scene_p->getFrameTransform(tube).translation();
                  geometry_msgs::msg::Vector3Stamped vec;
                  vec.header.frame_id = hand_frame;
                  vec.vector.z = 1.0;
                  self->setProperty("direction", vec);
                  self->setMinMaxDistance(target_beaker.position.x - position.x() - 0.05, target_beaker.position.x - position.x() + 0.05);
                }
              );
              grasp->insert(std::move(stage));
            }

            {
              auto stage = std::make_unique<MoveRelativeRefactor>("f4", cartesian_planner);
              stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
              stage->setIKFrame(hand_frame);
              stage->properties().set("marker_ns", "f4");
              stage->setPreComputeCallback(
                [](MoveRelativeRefactor* self,const moveit::task_constructor::InterfaceState& state) {
                  planning_scene::PlanningScenePtr scene_p = state.scene()->diff();
                  Eigen::Vector3d position = scene_p->getFrameTransform(tube).translation();
                  geometry_msgs::msg::Vector3Stamped vec;
                  vec.header.frame_id = "world";
                  vec.vector.z = -1.0;
                  self->setProperty("direction", vec);
                  self->setMinMaxDistance(position.z() - target_beaker.position.z - 0.005, position.z() - target_beaker.position.z  - 0.005);
                }
              );
              grasp->insert(std::move(stage));
            }

            // 添加一个MoveTo阶段，打开夹爪
            {
              auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
              stage->setGroup(hand_group_name);
              stage->setGoal("open_cup");
              grasp->insert(std::move(stage));
            }

            // 防止碰撞
            {
              auto stage =
                  std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
              stage->allowCollisions(tube,
                                    task.getRobotModel()
                                        ->getJointModelGroup(hand_group_name)
                                        ->getLinkModelNamesWithCollisionGeometry(),
                                    false);
              grasp->insert(std::move(stage));
            }

            // 分离物体
            {
              auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
              stage->detachObject(tube, hand_frame);
              grasp->insert(std::move(stage));
            }

            // 回撤
            {
              auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
              stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
                    stage->setMinMaxDistance(0.15, 0.3);
              stage->setIKFrame(hand_frame);
              stage->properties().set("marker_ns", "retreat");

              // Set retreat direction
              geometry_msgs::msg::Vector3Stamped vec;
              vec.header.frame_id = hand_frame;
              vec.vector.z = -1.0;
              stage->setDirection(vec);
              grasp->insert(std::move(stage));
            }

            {
              auto stage = std::make_unique<mtc::stages::MoveTo>("closed hand", interpolation_planner);
              stage->setGroup(hand_group_name);
              stage->setGoal("closed");
              grasp->insert(std::move(stage));
            }

            task.add(std::move(grasp));
          }
          break;

      case 3:
          // 执行任务3
          {
            auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
            task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
            grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                                  { "eef", "group", "ik_frame" });
            {
              auto stage =
                  std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
              stage->properties().set("marker_ns", "approach_object");
              stage->properties().set("link", hand_frame);
              stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
              stage->setMinMaxDistance(0.1, 0.15);

              // Set hand forward direction
              geometry_msgs::msg::Vector3Stamped vec;
              vec.header.frame_id = hand_frame;
              vec.vector.z = 1;
              stage->setDirection(vec);
              grasp->insert(std::move(stage));
            }

            {
              // Sample grasp pose
              auto stage = std::make_unique<moveit::task_constructor::stages::GenerateGraspPose>("generate grasp pose");
              stage->properties().configureInitFrom(mtc::Stage::PARENT);
              stage->properties().set("marker_ns", "grasp_pose");
              stage->setPreGraspPose("open_cup");
              stage->setObject(tube);
              stage->setAngleDelta(M_PI / 24);
              stage->setRotationAxis(Eigen::Vector3d(0, 0, 1)); // 默认是Z轴
              stage->setMonitoredStage(current_state_ptr);  // Hook into current state

              Eigen::Isometry3d grasp_frame_transform;
              Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                                    Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
              // Eigen::Quaterniond q(0.5, -0.5, -0.5, -0.5);
              pose.orientation.x = q.x();
              pose.orientation.y = q.y();
              pose.orientation.z = q.z();
              pose.orientation.w = q.w();
              
              grasp_frame_transform.linear() = q.matrix();
              Eigen::Quaterniond quat(grasp_frame_transform.rotation());
              grasp_frame_transform.translation().z() = 0.213;  // 目标在 jaw_link，jaw 比 wrist 再向下 15cm
              grasp_frame_transform.translation().x() = -0.05;  // 目标在 jaw_link，jaw 比 wrist 再向下 15cm
              geometry_msgs::msg::PoseStamped target_pose;
              target_pose.header.frame_id = "world";
              auto wrapper =
                  std::make_unique<moveit::task_constructor::stages::ComputeIK>("grasp pose IK", std::move(stage));
              wrapper->setMaxIKSolutions(8);
              wrapper->setMinSolutionDistance(1.0);
              wrapper->setIKFrame(grasp_frame_transform, hand_frame);
              wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
              wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
              grasp->insert(std::move(wrapper));
            }

            {
              auto stage =
                  std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
              stage->allowCollisions(tube,
                                    task.getRobotModel()
                                        ->getJointModelGroup(hand_group_name)
                                        ->getLinkModelNamesWithCollisionGeometry(),
                                    true);
              grasp->insert(std::move(stage));
            }

            {
              auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
              stage->setGroup(hand_group_name);
              stage->setGoal("closed_cup");
              grasp->insert(std::move(stage));
            }

            {
              auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
              stage->attachObject(tube, hand_frame);
              grasp->insert(std::move(stage));
            }

            {
              auto stage =
                  std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
              stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
              stage->setMinMaxDistance(0.02, 0.05);
              stage->setIKFrame(hand_frame);
              stage->properties().set("marker_ns", "lift_object");

              // Set upward direction
              geometry_msgs::msg::Vector3Stamped vec;
              vec.header.frame_id = "world";
              vec.vector.z = 1.0;
              stage->setDirection(vec);
              grasp->insert(std::move(stage));
            }

            {
              auto stage = std::make_unique<MoveRelativeRefactor>("Intelligent advancement", cartesian_planner);
              stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
              stage->setIKFrame(hand_frame);
              stage->properties().set("marker_ns", "Intelligent advancement");
              stage->setPreComputeCallback(
                [](MoveRelativeRefactor* self,const moveit::task_constructor::InterfaceState& state) {
                  planning_scene::PlanningScenePtr scene_p = state.scene()->diff();
                  Eigen::Vector3d position = scene_p->getFrameTransform(tube).translation();
                  geometry_msgs::msg::Vector3Stamped vec;
                  vec.header.frame_id = hand_frame;
                  vec.vector.z = -1.0;
                  self->setProperty("direction", vec);
                  self->setMinMaxDistance(position.x() - 0.55, position.x() - 0.45);
                }
              );
              grasp->insert(std::move(stage));
            }

            {
              auto pour_motion = std::make_unique<MoveRelativeRefactor>("dynamic diagonal", interpolation_planner);
              pour_motion->setGroup(arm_group_name);
              pour_motion->setPreComputeCallback(
                [](MoveRelativeRefactor* self,const moveit::task_constructor::InterfaceState& state) {
                  planning_scene::PlanningScenePtr scene_p = state.scene()->diff();
                  Eigen::Vector3d position = scene_p->getFrameTransform(tube).translation();
                  double fd = computeBaseRotation(position.x(), position.y(), beaker.position.x, beaker.position.y);
                  std::map<std::string, double> joint_target;
                  joint_target["elfin_joint1"] = fd;
                  self->setProperty("direction", joint_target);
                }
              );
              grasp->insert(std::move(pour_motion));
            }

            {

              auto move_relative = std::make_unique<MoveRelativeRefactor>("posture correction", cartesian_planner);
              move_relative->properties().configureInitFrom(mtc::Stage::PARENT, {"group","ik_frame"});
              move_relative->setGroup(arm_group_name);
              move_relative->setIKFrame(hand_frame);
              move_relative->setPreComputeCallback(
                [](MoveRelativeRefactor* self,const moveit::task_constructor::InterfaceState& state) {
                  planning_scene::PlanningScenePtr scene_p = state.scene()->diff();
                  const auto& robot_state = scene_p->getCurrentState();
                  Eigen::Isometry3d ee_pose = robot_state.getGlobalLinkTransform(hand_frame);
                  Eigen::Quaterniond orientation(ee_pose.rotation());

                  Eigen::Vector3d z_world = computeDirectionFromOrigin(beaker.position.x, beaker.position.y);
                  double theta_rad = alignZAxis(orientation, z_world);;

                  geometry_msgs::msg::TwistStamped twist;
                  twist.header.frame_id = "world";  // 相对于夹爪局部坐标系
                  twist.twist.linear.x = 0.0;
                  twist.twist.linear.y = 0.0;
                  twist.twist.linear.z = 0.0;

                  // 绕局部 Y 轴旋转 90°（单位：弧度）
                  twist.twist.angular.x = 0.0;
                  twist.twist.angular.y = 0.0;  // pitch
                  twist.twist.angular.z = theta_rad;
                  self->setProperty("direction", twist);
                }
              );
              move_relative->setMinDistance(0.1);
              grasp->insert(std::move(move_relative));
            }

            {
              auto stage = std::make_unique<MoveRelativeRefactor>("move horizontally",interpolation_planner);
              stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
              stage->setGroup(arm_group_name);
              stage->setIKFrame(hand_frame);
              stage->setPreComputeCallback(
                [](MoveRelativeRefactor* self,const moveit::task_constructor::InterfaceState& state) {
                  planning_scene::PlanningScenePtr scene_p = state.scene()->diff();
                  Eigen::Vector3d position = scene_p->getFrameTransform(tube).translation();
                  geometry_msgs::msg::Vector3Stamped metry;
                  metry.header.frame_id = "world";
                  metry.vector.x = beaker.position.x - position.x();
                  metry.vector.y = beaker.position.y - position.y() - 0.005;
                  metry.vector.z = 0;
                  self->setProperty("direction", metry);
                }
              );
              grasp->insert(std::move(stage));
            }

            {
              auto stage = std::make_unique<MoveRelativeRefactor>("f4", cartesian_planner);
              stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
              stage->setIKFrame(hand_frame);
              stage->properties().set("marker_ns", "f4");
              stage->setPreComputeCallback(
                [](MoveRelativeRefactor* self,const moveit::task_constructor::InterfaceState& state) {
                  planning_scene::PlanningScenePtr scene_p = state.scene()->diff();
                  Eigen::Vector3d position = scene_p->getFrameTransform(tube).translation();
                  geometry_msgs::msg::Vector3Stamped vec;
                  vec.header.frame_id = "world";
                  vec.vector.z = -1.0;
                  self->setProperty("direction", vec);
                  self->setMinMaxDistance(position.z() - beaker.position.z + 0.007, position.z() - beaker.position.z  + 0.007);
                }
              );
              grasp->insert(std::move(stage));
            }

            // 添加一个MoveTo阶段，打开夹爪
            {
              auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
              stage->setGroup(hand_group_name);
              stage->setGoal("open_cup");
              grasp->insert(std::move(stage));
            }

            // 防止碰撞
            {
              auto stage =
                  std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
              stage->allowCollisions(tube,
                                    task.getRobotModel()
                                        ->getJointModelGroup(hand_group_name)
                                        ->getLinkModelNamesWithCollisionGeometry(),
                                    false);
              grasp->insert(std::move(stage));
            }

            // 分离物体
            {
              auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
              stage->detachObject(tube, hand_frame);
              grasp->insert(std::move(stage));
            }

            // 回撤
            {
              auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
              stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
                    stage->setMinMaxDistance(0.08, 0.15);
              stage->setIKFrame(hand_frame);
              stage->properties().set("marker_ns", "retreat");

              // Set retreat direction
              geometry_msgs::msg::Vector3Stamped vec;
              vec.header.frame_id = hand_frame;
              vec.vector.z = -1.0;
              stage->setDirection(vec);
              grasp->insert(std::move(stage));
            }

            {
              auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat z", cartesian_planner);
              stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
                    stage->setMinMaxDistance(0.1, 0.2);
              stage->setIKFrame(hand_frame);
              stage->properties().set("marker_ns", "retreat");

              geometry_msgs::msg::Vector3Stamped vec;
              vec.header.frame_id = "world";
              vec.vector.z = 1.0;
              stage->setDirection(vec);
              grasp->insert(std::move(stage));
            }
            
            task.add(std::move(grasp));
          }

          {
            auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
            stage->setGroup(hand_group_name);
            stage->setGoal("closed");
            task.add(std::move(stage));
          }

          break;

      default:
          std::cerr << "未知任务类型: " << typeint << std::endl;
          break;
  }

  return task;
}

int main(int argc, char** argv)
{
  // --- 初始化 ROS2 节点 ---
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  // --- 创建任务节点与执行器 ---
  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  // spin线程
  std::thread spin_thread([&]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface()); 
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  // --- 初始化任务参数 ---
  geometry_msgs::msg::Pose temp;
  int n = 11;
  int po = 0;
  i = n;

  // 交换初始位置
  if (po == 1) {
    std::swap(tube_rack_pose, tube_rack_pose2);
    std::swap(tube_rack_pose3, tube_rack_pose4);
  }

  // --- 主循环 ---
  while (rclcpp::ok()) {
    mtc_task_node->setupPlanningScene();

    while (true) {
      tube = "tube_" + std::to_string(i);
      RCLCPP_INFO_STREAM(LOGGER, "\n========== 执行任务：" << tube << " ==========");
      std::this_thread::sleep_for(std::chrono::seconds(2));

      // 执行抓取任务
      if (!mtc_task_node->doTask(1)) {
        std::cerr << "[ERROR] Task 1 failed.\n";
        goto CLEANUP;
      }

      // 判断是否到达末尾
      if (static_cast<size_t>(i + 1) == ptube_world.size()) {
        tube = "beaker_rack";
        RCLCPP_INFO_STREAM(LOGGER, "\n========== 执行任务：" << tube << " ==========");

        if (!mtc_task_node->doTask(2)) { RCLCPP_ERROR_STREAM(LOGGER, "[ERROR] Task 2 failed."); goto CLEANUP; }
        if (!mtc_task_node->doTask(3)) { RCLCPP_ERROR_STREAM(LOGGER, "[ERROR] Task 3 failed."); goto CLEANUP; }

        break;
      }

      i++;
    }

    // --- 交换管架位置以准备下一轮 ---
    std::swap(tube_rack_pose, tube_rack_pose2);
    std::swap(tube_rack_pose3, tube_rack_pose4);

    i = 0;
    std::this_thread::sleep_for(std::chrono::seconds(5));
  }

CLEANUP:
  std::this_thread::sleep_for(std::chrono::seconds(500));
  rclcpp::shutdown();
  if (spin_thread.joinable()) spin_thread.join();
  return 0;
}
