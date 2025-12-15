
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
using namespace moveit::task_constructor::stages;
using json = nlohmann::json;
geometry_msgs::msg::Pose node_target_pose;
std::vector<geometry_msgs::msg::Pose> ptube_world;
std::vector<geometry_msgs::msg::Pose> place_poses;
double t_z = 0.15;
double t_y = 0.35;
int i = 0;
double xs = 0.036;
std::string tube = "tube_";
geometry_msgs::msg::PoseStamped grasp_pose_msg_qj;
geometry_msgs::msg::Pose pose;

geometry_msgs::msg::Pose tube_rack_pose = []{
  geometry_msgs::msg::Pose p;
  p.position.x = -0.29;
  p.position.y = 0.63;
  p.position.z = t_z + 0.035;
  // p.orientation.x = 0.0;
  // p.orientation.y = 0.0;
  // p.orientation.z = 0.7071;
  // p.orientation.w = 0.7071;
  p.orientation.w = 1;
  return p;
}();

geometry_msgs::msg::Pose tube_rack_pose2 = []{
  geometry_msgs::msg::Pose p;
  p.position.x = 0.402+0.018;  // 你根据实际情况调整
  p.position.y = 0.616;
  p.position.z = t_z + 0.035;
  p.orientation.x = 0.0;
  p.orientation.y = 0.0;
  p.orientation.z = 0.7071;
  p.orientation.w = 0.7071;
  return p;
}();

std::array<double, 8> beaker = {0.22,0.75,t_z,1,0,0,0,1};

class MoveRelativeRefactor : public moveit::task_constructor::stages::MoveRelative {
public:
    MoveRelativeRefactor(const std::string& name,
                         const moveit::task_constructor::solvers::PlannerInterfacePtr& planner)
        : moveit::task_constructor::stages::MoveRelative(name, planner) {}

protected:
    // 重写 compute 方法
    bool compute(const moveit::task_constructor::InterfaceState& state,
                 planning_scene::PlanningScenePtr& scene,
                 moveit::task_constructor::SubTrajectory& solution,
                 moveit::task_constructor::Interface::Direction dir) override {
        return MoveRelative::compute(state, scene, solution, dir);
    }
};


std::vector<geometry_msgs::msg::Pose> getTubeWorldPosesSorted(
    const geometry_msgs::msg::Pose& rack_pose,
    bool nearest_first = true  // true = 近→远, false = 远→近
) {
    double dx = 0.072;
    double dy = 0.036;
    double z  = 0.025;
  
    // 每组局部坐标
    std::array<std::array<tf2::Vector3, 3>, 2> tube_groups = {{
        { tf2::Vector3(-dy, -dx-0.018, z), tf2::Vector3(-dy, -0.018, z), tf2::Vector3(-dy, dx-0.018, z) },
        // { tf2::Vector3(0.0, -dx-0.018, z), tf2::Vector3(0.0, -0.018, z), tf2::Vector3(0.0, dx-0.018, z) },
        { tf2::Vector3(dy, -dx-0.018, z), tf2::Vector3(dy, -0.018, z), tf2::Vector3(dy, dx-0.018, z) }
    }};

    tf2::Quaternion q_rack(rack_pose.orientation.x, rack_pose.orientation.y, rack_pose.orientation.z, rack_pose.orientation.w);
    tf2::Transform rack_tf;
    rack_tf.setOrigin(tf2::Vector3(rack_pose.position.x, rack_pose.position.y, rack_pose.position.z));
    rack_tf.setRotation(q_rack);

    // 先把每组转换为世界坐标
    std::vector<std::vector<geometry_msgs::msg::Pose>> world_groups(tube_groups.size());
    for (size_t i = 0; i < tube_groups.size(); ++i) {
        for (const auto& local : tube_groups[i]) {
            tf2::Vector3 global = rack_tf * local;
            geometry_msgs::msg::Pose p;
            p.position.x = global.x();
            p.position.y = global.y();
            p.position.z = global.z();
            p.orientation.x = 0.0;
            p.orientation.y = 0.0;
            p.orientation.z = 0.0;
            p.orientation.w = 1.0;
            world_groups[i].push_back(p);
        }
    }

    // 按每组中心点到原点距离排序
    std::sort(world_groups.begin(), world_groups.end(),
        [](const std::vector<geometry_msgs::msg::Pose>& a, const std::vector<geometry_msgs::msg::Pose>& b) {
            // 组中心点
            tf2::Vector3 center_a(0,0,0), center_b(0,0,0);
            for (const auto& p : a) center_a += tf2::Vector3(p.position.x, p.position.y, p.position.z);
            for (const auto& p : b) center_b += tf2::Vector3(p.position.x, p.position.y, p.position.z);
            center_a /= a.size();
            center_b /= b.size();

            double da2 = center_a.length2();
            double db2 = center_b.length2();
            return da2 < db2;  // 默认近→远
        });

    if (!nearest_first) {
        std::reverse(world_groups.begin(), world_groups.end());  // 远→近
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

  bool doTask();

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
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
  // === 生成 18 个试管 ===
  // {
  //   ptube_world = getTubeWorldPosesSorted(tube_rack_pose,true);
  //   place_poses = getTubeWorldPosesSorted(tube_rack_pose2,false);
  //   int id_counter = 0;

  //   for (const auto& tp : ptube_world) {
  //     moveit_msgs::msg::CollisionObject tube;
  //     tube.id = "tube_" + std::to_string(id_counter++);
  //     tube.header.frame_id = "world";

  //     tube.primitive_poses.resize(1);
  //     tube.primitive_poses[0] = tp;  // 直接赋值姿态

  //     tube.primitives.resize(1);
  //     tube.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;  // 更像试管
  //     tube.primitives[0].dimensions = {0.1, 0.007};  // 高度=0.1, 半径=0.01

  //     tube.operation = tube.ADD;
  //     collision_objects.push_back(tube);
  //   }
  // }

  // // === 添加试管架 ===
  // {
  //   moveit_msgs::msg::CollisionObject rack;
  //   rack.id = "tube_rack";
  //   rack.header.frame_id = "world";

  //   shapes::Mesh* m = shapes::createMeshFromResource(rack_mesh_path);
  //   shapes::ShapeMsg mesh_msg;
  //   shapes::constructMsgFromShape(m, mesh_msg);
  //   shape_msgs::msg::Mesh mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

  //   rack.meshes.push_back(mesh);
  //   rack.mesh_poses.push_back(tube_rack_pose);  // 用同一个 rack_pose

  //   rack.operation = rack.ADD;
  //   collision_objects.push_back(rack);
  // }

  // {
  //   moveit_msgs::msg::CollisionObject rack;
  //   rack.id = "tube_rack2";
  //   rack.header.frame_id = "world";

  //   shapes::Mesh* m = shapes::createMeshFromResource(rack_mesh_path);
  //   shapes::ShapeMsg mesh_msg;
  //   shapes::constructMsgFromShape(m, mesh_msg);
  //   shape_msgs::msg::Mesh mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

  //   rack.meshes.push_back(mesh);
  //   rack.mesh_poses.push_back(tube_rack_pose2);  // 用同一个 rack_pose

  //   rack.operation = rack.ADD;
  //   collision_objects.push_back(rack);
  // }

  // {
  //   moveit_msgs::msg::CollisionObject rack;
  //   rack.id = "beaker_rack";
  //   rack.header.frame_id = "world";

  //   shapes::Mesh* m = shapes::createMeshFromResource(beaker_mesh_path);
  //   shapes::ShapeMsg mesh_msg;
  //   shapes::constructMsgFromShape(m, mesh_msg);
  //   shape_msgs::msg::Mesh mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

  //   rack.meshes.push_back(mesh);
  //   geometry_msgs::msg::Pose pose;
  //   pose.position.x = beaker[0];
  //   pose.position.y = beaker[1];
  //   pose.position.z = beaker[2];
  //   pose.orientation.x = beaker[4];
  //   pose.orientation.y = beaker[5];
  //   pose.orientation.z = beaker[6];
  //   pose.orientation.w = beaker[7];

  //   rack.mesh_poses.push_back(pose);

  //   rack.operation = rack.ADD;
  //   collision_objects.push_back(rack);
  // }

  // === 第二个障碍物（大盒子） ===
  moveit_msgs::msg::CollisionObject obj2;
  obj2.id = "object_2";
  obj2.header.frame_id = "world";
  obj2.primitive_poses.resize(1);
  obj2.primitive_poses[0].position.x = 0;
  obj2.primitive_poses[0].position.y = t_y + 0.6;
  obj2.primitive_poses[0].position.z = t_z / 2;
  obj2.primitive_poses[0].orientation.w = 1.0;
  obj2.primitives.resize(1);
  obj2.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  obj2.primitives[0].dimensions = {2, 1.2, t_z};
  obj2.operation = obj2.ADD;

  collision_objects.push_back(obj2);

  // === 应用到 PlanningScene ===
  psi.applyCollisionObjects(collision_objects);
}

bool MTCTaskNode::doTask()
{
  task_ = createTask();

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

  task_.introspection().publishSolution(*task_.solutions().front());
  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return false;
  }

  return true;
}

mtc::Task MTCTaskNode::createTask()
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

  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  // 使用SerialContainer创建一个放置物体的子任务，包含生成放置姿态、打开手爪、碰撞、分离物体和回撤等阶段
  {
    auto place = std::make_unique<mtc::SerialContainer>("pouring position");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });

    {
      // Sample place pose
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("pouring pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject(hand_frame);

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = "world";
      target_pose_msg.pose = node_target_pose; 
      stage->setPose(target_pose_msg);

      stage->setMonitoredStage(current_state_ptr);

      // Compute IK
      auto wrapper =
          std::make_unique<moveit::task_constructor::stages::ComputeIK>("pouring pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(16);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    task.add(std::move(place));
  }

  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // 创建节点
  auto node = std::make_shared<rclcpp::Node>("mtc_task_node");

  // 声明参数（如果 launch 文件没有传，会使用默认值）
  node->declare_parameter<double>("target_x", 0.4);
  node->declare_parameter<double>("target_y", 0.5);
  node->declare_parameter<double>("target_z", 0.3);
  node->declare_parameter<double>("target_qx", -0.5);
  node->declare_parameter<double>("target_qy", -0.5);
  node->declare_parameter<double>("target_qz", -0.5);
  node->declare_parameter<double>("target_qw", 0.5);

  // 读取参数
  node->get_parameter("target_x", node_target_pose.position.x);
  node->get_parameter("target_y", node_target_pose.position.y);
  node->get_parameter("target_z", node_target_pose.position.z);
  node->get_parameter("target_qx", node_target_pose.orientation.x);
  node->get_parameter("target_qy", node_target_pose.orientation.y);
  node->get_parameter("target_qz", node_target_pose.orientation.z);
  node->get_parameter("target_qw", node_target_pose.orientation.w);

  RCLCPP_INFO(node->get_logger(),
              "Target Pose: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
              node_target_pose.position.x,
              node_target_pose.position.y,
              node_target_pose.position.z,
              node_target_pose.orientation.x,
              node_target_pose.orientation.y,
              node_target_pose.orientation.z,
              node_target_pose.orientation.w);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  // 创建MTCTaskNode对象和多线程执行器
  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}


