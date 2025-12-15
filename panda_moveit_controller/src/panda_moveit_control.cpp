
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
 
static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;
 
class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);
 
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
 
  void doTask();
 
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
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  // 参考坐标系 world
  object.header.frame_id = "base";

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions = {0.10, 0.02, 0.02};  // 长宽高

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0;
  pose.position.y = -0.21869;
  pose.position.z = 0.0;
  pose.orientation.w = 1.0;

  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  object.operation = object.ADD;
 
  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}
 
void MTCTaskNode::doTask()
{
  task_ = createTask();
 
  try
  {
    RCLCPP_ERROR_STREAM(LOGGER, "================== 1");
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "================== 2");
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }
  try
  {
    RCLCPP_ERROR_STREAM(LOGGER, "================== 3");
    if (!task_.plan(5))
    {
      RCLCPP_ERROR_STREAM(LOGGER, "================== 4");
      RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
      return;
    }
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "================== 2");
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  RCLCPP_ERROR_STREAM(LOGGER, "================== 5");
  task_.introspection().publishSolution(*task_.solutions().front());
  RCLCPP_ERROR_STREAM(LOGGER, "================== 6");
  auto result = task_.execute(*task_.solutions().front());
  RCLCPP_ERROR_STREAM(LOGGER, "================== 7");
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }
 
  return;
}
 
mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  // 设置任务名称和加载机器人模型
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);
 
  const auto& arm_group_name = "manipulator";
  const auto& hand_group_name = "gripper";
  const auto& hand_frame = "wrist_2_link";
 
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
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);
 
  auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));
 
  //添加一个Connect阶段，将机械臂从当前位置移动到抓取位置
  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));
  
  // 使用SerialContainer创建一个抓取物体的子任务，包含接近物体、生成抓取姿态、碰撞、关闭手爪、抓取物体和提升物体等阶段
  mtc::Stage* attach_object_stage = nullptr;  // Forward attach_object_stage to place pose generator
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
      stage->setMinMaxDistance(0.01, 0.05);
 
      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.y = -1;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }
 
    {
      // Sample grasp pose
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject("object");
      stage->setAngleDelta(M_PI / 12);
      stage->setMonitoredStage(current_state_ptr);  // Hook into current state
 
      Eigen::Isometry3d grasp_frame_transform;
      // Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
      //                       Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
      //                       Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
      // grasp_frame_transform.linear() = q.matrix();
      Eigen::Quaterniond q;
      q = Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY());
      grasp_frame_transform.linear() = q.toRotationMatrix();
      std::cout << "Transform matrix:\n" << grasp_frame_transform.matrix() << std::endl;
      grasp_frame_transform.translation().y() = 0.13;  // 目标在 jaw_link，jaw 比 wrist 再向下 15cm
        // Compute IK
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
            wrapper->addSolutionCallback([this](const moveit::task_constructor::SolutionBase& s) {

        // 遍历各个阶段以找到执行抓取的阶段
        // 您应该知道您的抓取/拾取阶段的名称（例如："pick object"）
        const std::string GRASP_STAGE_NAME = "grasp pose IK"; // <<< 重要：替换为您实际的抓取阶段名称

        const auto& stage = s.creator();
        if (true) { // 使用 find 进行部分匹配
            const auto* grasp_end_state = s.end(); // 获取抓取阶段的结束状态

            if (grasp_end_state) {
                const moveit::core::RobotState& robot_state_at_grasp = grasp_end_state->scene()->getCurrentState();

                // 在抓取点获取末端执行器连杆的姿态
                // 将 "your_gripper_link_name" 替换为您夹具连杆的实际名称
                // 这通常是直接连接到物体或夹具手指基座的连杆。
                const std::string end_effector_link_name = "manipulator"; // <<< IMPORTANT: CHANGE THIS!
                const std::string end_effector_link_name2 = "wrist_2_link"; // <<< IMPORTANT: CHANGE THIS!

                const Eigen::Isometry3d& grasp_end_effector_pose = robot_state_at_grasp.getGlobalLinkTransform(end_effector_link_name2);

                RCLCPP_INFO(rclcpp::get_logger("mtc"), "--- 在阶段 '%s' 中找到抓取姿态 ---", stage->name().c_str());
                RCLCPP_INFO(rclcpp::get_logger("mtc"), "位置 (x, y, z): %f, %f, %f",
                            grasp_end_effector_pose.translation().x(),
                            grasp_end_effector_pose.translation().y(),
                            grasp_end_effector_pose.translation().z());

                Eigen::Quaterniond q(grasp_end_effector_pose.rotation());
                RCLCPP_INFO(rclcpp::get_logger("mtc"), "姿态 (qx, qy, qz, qw): %f, %f, %f, %f",
                            q.x(), q.y(), q.z(), q.w());

                // 您已经找到了，您可以中断循环，或者如果有多个抓取则继续
                // 如果您特别想要抓取应用之前的状态（即接近姿态），
                // 您可以查看抓取阶段的 stage->start()，
                // 或者如果您明确定义了抓取前阶段，则查看其 end()。
                return; // 找到第一个抓取后退
            }
        }
        RCLCPP_INFO(rclcpp::get_logger("mtc"), "未在解决方案中找到名为 '%s' 的抓取阶段。", GRASP_STAGE_NAME.c_str());
      });
      grasp->insert(std::move(wrapper));
    }
 
    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions("object",
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
      stage->attachObject("object", hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }
 
    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.01, 0.05);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");
 
      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "base";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }
 
    task.add(std::move(grasp));
  }
 
  // // 添加一个Connect阶段，将机械臂从抓取位置移动到放置位置
  // {
  //   auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
  //       "move to place",
  //       mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
  //                                                 { hand_group_name, sampling_planner } });
  //   stage_move_to_place->setTimeout(5.0);
  //   stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
  //   task.add(std::move(stage_move_to_place));
  // }
 
  // // 使用SerialContainer创建一个放置物体的子任务，包含生成放置姿态、打开手爪、碰撞、分离物体和回撤等阶段
  // {
  //   auto place = std::make_unique<mtc::SerialContainer>("place object");
  //   task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
  //   place->properties().configureInitFrom(mtc::Stage::PARENT,
  //                                         { "eef", "group", "ik_frame" });
  //   {
  //     // Sample place pose
  //     auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
  //     stage->properties().configureInitFrom(mtc::Stage::PARENT);
  //     stage->properties().set("marker_ns", "place_pose");
  //     stage->setObject("object");
 
  //     geometry_msgs::msg::PoseStamped target_pose_msg;
  //     target_pose_msg.header.frame_id = "object";
  //     target_pose_msg.pose.position.y = 0.5;
  //     target_pose_msg.pose.orientation.w = 1.0;
  //     stage->setPose(target_pose_msg);
  //     stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage
 
  //     // Compute IK
  //     auto wrapper =
  //         std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
  //     wrapper->setMaxIKSolutions(2);
  //     wrapper->setMinSolutionDistance(1.0);
  //     wrapper->setIKFrame("object");
  //     wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
  //     wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
  //     place->insert(std::move(wrapper));
  //   }
 
  //   // 添加一个MoveTo阶段，打开夹爪
  //   {
  //     auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  //     stage->setGroup(hand_group_name);
  //     stage->setGoal("open");
  //     place->insert(std::move(stage));
  //   }
 
  //   // 防止碰撞
  //   {
  //     auto stage =
  //         std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
  //     stage->allowCollisions("object",
  //                           task.getRobotModel()
  //                               ->getJointModelGroup(hand_group_name)
  //                               ->getLinkModelNamesWithCollisionGeometry(),
  //                           false);
  //     place->insert(std::move(stage));
  //   }
 
  //   // 分离物体
  //   {
  //     auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
  //     stage->detachObject("object", hand_frame);
  //     place->insert(std::move(stage));
  //   }
 
  //   // 回撤
  //   {
  //     auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
  //     stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  //     stage->setMinMaxDistance(0.1, 0.3);
  //     stage->setIKFrame(hand_frame);
  //     stage->properties().set("marker_ns", "retreat");
 
  //     // Set retreat direction
  //     geometry_msgs::msg::Vector3Stamped vec;
  //     vec.header.frame_id = "base";
  //     vec.vector.x = -0.5;
  //     stage->setDirection(vec);
  //     place->insert(std::move(stage));
  //   }
 
  //   task.add(std::move(place));
  // }
 
  // // 添加一个MoveTo阶段，将机械臂移动到初始位置
  // {
  //   auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
  //   stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  //   stage->setGoal("ready");
  //   task.add(std::move(stage));
  // }
 
  return task;
}
 
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
 
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
  while(true){
    mtc_task_node->doTask();
    usleep(1000000);
  }
  // mtc_task_node->doTask();
 
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
