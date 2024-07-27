#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit_msgs/msg/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit/task_constructor/cost_terms.h>
// #include <moveit_msgs/AllowedCollisionMatrix.h>
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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("my_sample");
namespace mtc = moveit::task_constructor;

class My_tutorial
{
public:
  My_tutorial(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();
  mtc::Task pickPlaceTask();
  mtc::Task setSceneTask();
  mtc::Task boxTask();
  mtc::Task cylinderTask();
  mtc::Task mergerTask();
  mtc::Task alternativesTask();
  
private:
  // Compose an MTC task from a series of stages.
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

My_tutorial::My_tutorial(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("my_tutorial_node", options) }
{
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr My_tutorial::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void My_tutorial::setupPlanningScene()
{
    // Initialize the Planning Scene Interface
    moveit::planning_interface::PlanningSceneInterface psi;
    // Add the objects to the scene
    // Cylinder1
    moveit_msgs::msg::CollisionObject object1;
    object1.id = "object";
    object1.header.frame_id = "world";
    object1.primitives.resize(1);
    object1.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    object1.primitives[0].dimensions = { 0.3, 0.02 };

    geometry_msgs::msg::Pose pose1;
    pose1.position.x = 0.5;
    pose1.position.y = -0.25;
    pose1.position.z = 0.3;
    pose1.orientation.w = 1.0;
    object1.pose = pose1;
    psi.applyCollisionObject(object1);



    moveit_msgs::msg::CollisionObject object4;
    object4.id = "cylinder2";
    object4.header.frame_id = "world";
    object4.primitives.resize(1);
    object4.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    object4.primitives[0].dimensions = { 0.6, 0.02 };

    geometry_msgs::msg::Pose pose4;
    pose4.position.x = 0.4;
    pose4.position.y = -0.3;
    pose4.position.z = 0.3;
    pose4.orientation.w = 1.0;
    object4.pose = pose4;
    psi.applyCollisionObject(object4);



    // BOX
    moveit_msgs::msg::CollisionObject object2;
    object2.id = "box";
    object2.header.frame_id = "world";
    object2.primitives.resize(1);

    // Define the size of the box in meters
    object2.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;//object2.primitives[0].BOX;
    object2.primitives[0].dimensions.resize(3);
    object2.primitives[0].dimensions[object2.primitives[0].BOX_X] = 0.05;
    object2.primitives[0].dimensions[object2.primitives[0].BOX_Y] = 0.05;
    object2.primitives[0].dimensions[object2.primitives[0].BOX_Z] = 0.05;

    object2.primitive_poses.resize(1);
    object2.primitive_poses[0].position.x = 0.5;
    object2.primitive_poses[0].position.y = -0.2;
    object2.primitive_poses[0].position.z = 0.2;//-0.026;
    object2.primitive_poses[0].orientation.w = 1.0;
    // object2.pose = pose2;
    object2.operation = object2.ADD;

    // Apply the table collision object
    psi.applyCollisionObject(object2);



    // Table
    moveit_msgs::msg::CollisionObject object3;
    object3.id = "table";
    object3.header.frame_id = "world";
    shape_msgs::msg::SolidPrimitive primitive3;

    // Define the size of the box in meters
    primitive3.type = primitive3.BOX;
    primitive3.dimensions.resize(3);
    primitive3.dimensions[primitive3.BOX_X] = 2;
    primitive3.dimensions[primitive3.BOX_Y] = 2;
    primitive3.dimensions[primitive3.BOX_Z] = 0.1;

    geometry_msgs::msg::Pose pose3;
    pose3.position.x = 0;
    pose3.position.y = 0;
    pose3.position.z = -0.051;
    pose3.orientation.w = 1.0;

    object3.primitives.push_back(primitive3);
    object3.primitive_poses.push_back(pose3);
    object3.operation = object3.ADD;

    // Apply the table collision object
    psi.applyCollisionObject(object3);
  
}   

void My_tutorial::doTask()
{
  task_ = setSceneTask();//task;
  try{
    task_.init();
  }
  catch (mtc::InitStageException& e){
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }
  if (!task_.plan(5)){
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());
  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS){
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  task_ = alternativesTask();//mergerTask();//cylinderTask();//task;
  try{
    task_.init();
  }
  catch (mtc::InitStageException& e){
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }
  if (!task_.plan(5)){
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());
  auto result1 = task_.execute(*task_.solutions().front());
  if (result1.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS){
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  // task_ = boxTask();//cylinderTask();//task;
  // try{
  //   task_.init();
  // }
  // catch (mtc::InitStageException& e){
  //   RCLCPP_ERROR_STREAM(LOGGER, e);
  //   return;
  // }
  // if (!task_.plan(5)){
  //   RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
  //   return;
  // }
  // task_.introspection().publishSolution(*task_.solutions().front());
  // auto result2 = task_.execute(*task_.solutions().front());
  // if (result2.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS){
  //   RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
  //   return;
  // }
  return;
}

mtc::Task My_tutorial::pickPlaceTask()
{
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel( node_ );

  const auto& arm_group_name = "panda_arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "panda_hand";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

  auto stage_allow_base_collision =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (robot,table)");
      stage_allow_base_collision->allowCollisions("table","panda_link0",false);
  task.add(std::move(stage_allow_base_collision));
    
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  // sampling_planner->setProperty("goal_joint_tolerance", 0.01); // Adjust tolerance
  // sampling_planner->setProperty("goal_position_tolerance", 0.01); // Adjust tolerance
  // sampling_planner->setProperty("goal_orientation_tolerance", 0.01); // Adjust tolerance
  // sampling_planner->setProperty("planning_time", 10.0); // Increase planning time
	sampling_planner->setProperty("goal_joint_tolerance", 1e-5); // If this is large then we get an error - that planned trajectory is too far from the goal etc


  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.005);



  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
    "move to pick",
    mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  stage_move_to_pick->setTimeout(15.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  mtc::Stage* attach_object_stage =
      nullptr;  // Forward attach_object_stage to place pose generator



  /** SERIAL CONTAINER TO Pick object up
   * 1. approach the object from a given orientation (using carteisan planner)
   * 2. Generate grasp poses around the object
   * 3. Wrap the grasp generator in a IK module to provide a robot pose for that grasp orientation
   * 4. Allow collision between hand and object
   * 5. close hand
   * 6. attach object to hand frame
   */
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });
    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner );
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.15);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.x = -1.0;
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
      Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().z() = 0.1;

      // Compute IK
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
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
      stage->allowCollisions("object",
                            task.getRobotModel()
                                ->getJointModelGroup(hand_group_name)
                                ->getLinkModelNamesWithCollisionGeometry(),
                            true);
    //   stage->allowCollisions("table","panda_link0",true);
      grasp->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("close");
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
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));

  }

  {
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
                                                  { hand_group_name, sampling_planner } });
    stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));
  }
  /** SERIAL CONTAINER TO LIFT AND PLACE OBJECTS
   * 1.
   * 2.
   * 3.
   */
  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });

    {
      // Sample place pose
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject("object");

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = "object";
      target_pose_msg.pose.position.y = 0.5;
      target_pose_msg.pose.orientation.w = 1.0;
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

      // Compute IK
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame("object");
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));

    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }
    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions("object",
                            task.getRobotModel()
                                ->getJointModelGroup(hand_group_name)
                                ->getLinkModelNamesWithCollisionGeometry(),
                            false);
      place->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("object", hand_frame);
      place->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");

      // Set retreat direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.x = -0.5;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }
    
    task.add(std::move(place));
  }
  return task;
}

mtc::Task My_tutorial::setSceneTask()
{
  mtc::Task task;
  task.stages()->setName("setup task");
  task.loadRobotModel( node_ );

  const auto& arm_group_name = "panda_arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "panda_hand";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

  
    
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto stage_allow_base_collision =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (robot,table)");
      stage_allow_base_collision->allowCollisions("table","panda_link0",true);
  task.add(std::move(stage_allow_base_collision));
  return task;
}

mtc::Task My_tutorial::cylinderTask()
{
  mtc::Task task;
  task.stages()->setName("cylinder task");
  task.loadRobotModel( node_ );

  const auto& arm_group_name = "panda_arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "panda_hand";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop
    
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  // sampling_planner->setProperty("goal_joint_tolerance", 0.01); // Adjust tolerance
  // sampling_planner->setProperty("goal_position_tolerance", 0.01); // Adjust tolerance
  // sampling_planner->setProperty("goal_orientation_tolerance", 0.01); // Adjust tolerance
  // sampling_planner->setProperty("planning_time", 10.0); // Increase planning time
	sampling_planner->setProperty("goal_joint_tolerance", 1e-5); // If this is large then we get an error - that planned trajectory is too far from the goal etc


  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.005);



  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
    "move to pick",
    mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  stage_move_to_pick->setTimeout(15.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  mtc::Stage* attach_object_stage =
      nullptr;  // Forward attach_object_stage to place pose generator



  /** SERIAL CONTAINER TO Pick object up
   * 1. approach the object from a given orientation (using carteisan planner)
   * 2. Generate grasp poses around the object
   * 3. Wrap the grasp generator in a IK module to provide a robot pose for that grasp orientation
   * 4. Allow collision between hand and object
   * 5. close hand
   * 6. attach object to hand frame
   */
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });
    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner );
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.15);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.x = -1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    } 
    {
      // Sample grasp pose
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject("cylinder2");
      stage->setAngleDelta(M_PI / 4);
      stage->setMonitoredStage(current_state_ptr);  // Hook into current state
      
      Eigen::Isometry3d grasp_frame_transform;
      float angle = M_PI/2;
      Eigen::Quaterniond q = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().z() = 0.1;

      // Compute IK
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
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
      stage->allowCollisions("cylinder2",
                            task.getRobotModel()
                                ->getJointModelGroup(hand_group_name)
                                ->getLinkModelNamesWithCollisionGeometry(),
                            true);
      grasp->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("close");
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("cylinder2", hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }
    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));

  }

  {
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
                                                  { hand_group_name, sampling_planner } });
    stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));
  }
  /** SERIAL CONTAINER TO LIFT AND PLACE OBJECTS
   * 1.
   * 2.
   * 3.
   */
  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });

    {
      // Sample place pose
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject("cylinder2");

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = "cylinder2";
      target_pose_msg.pose.position.y = 0.5;
      target_pose_msg.pose.orientation.w = 1.0;
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

      // Compute IK
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame("cylinder2");
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));

    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }
    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions("cylinder2",
                            task.getRobotModel()
                                ->getJointModelGroup(hand_group_name)
                                ->getLinkModelNamesWithCollisionGeometry(),
                            false);
      place->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("cylinder2", hand_frame);
      place->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");

      // Set retreat direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.x = -0.5;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }
    
    task.add(std::move(place));
  }
  return task;
}

mtc::Task My_tutorial::mergerTask()
{
  mtc::Task task;
  task.stages()->setName("merger task");
  task.loadRobotModel( node_ );

  const auto& arm_group_name = "panda_arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "panda_hand";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  // Disable warnings for this line, as it's a variable that's set but not used in this example
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  #pragma GCC diagnostic pop
    
  //Current stage : defines the current position of the robot
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));


  // Define planners
    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    sampling_planner->setProperty("goal_joint_tolerance", 1e-5); // If this is large then we get an error - that planned trajectory is too far from the goal etc

    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(.005);

  // MoveTo stage : This moves the the planning group to the specified predefined joined state defined in the SRDF file
    auto stage_open_hand =  std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    stage_open_hand->setGroup(hand_group_name);
    stage_open_hand->setGoal("open");
    task.add(std::move(stage_open_hand));

  mtc::Stage* attach_object_stage = nullptr;  // Forward attach_object_stage to place pose generator

  // Merger Block example : Merger is a parallel block that allows seperate planning for two independent planning groups simultaneously
    auto merger = std::make_unique<mtc::Merger>("move arm and close gripper");
    {
      task.properties().exposeTo(merger->properties(), { "eef", "group", "ik_frame" });
      // merger->properties().configureInitFrom(mtc::Stage::PARENT,
      //                                       { "eef", "group", "ik_frame" });


      // MoveRelative stage moves the robot in the direction specified using a specified planner
        auto back_approach = std::make_unique<mtc::stages::MoveRelative>("Approach", cartesian_planner);

        back_approach->properties().set("marker_ns", "approach_object");
        back_approach->properties().set("link", hand_frame);
        back_approach->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        back_approach->setMinMaxDistance(0.1, 0.15);

        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = hand_frame;
        vec.vector.x = -1.5;
        back_approach->setDirection(vec);
        merger->add(std::move(back_approach));

      // MoveTo stage to move the gripper to open position
        auto open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
        open_hand->setGroup(hand_group_name);
        open_hand->setGoal("open");
        merger->add(std::move(open_hand));
    }
    task.add(std::move(merger));


  //Now moving each planning group independenatly to see the diffeence without parallel execution
  
  // MoveRelatvie stage
    auto front_approach = std::make_unique<mtc::stages::MoveRelative>("Approach", cartesian_planner);
    front_approach->properties().set("marker_ns", "approach_object");
    front_approach->properties().set("link", hand_frame);
    front_approach->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    front_approach->setMinMaxDistance(0.1, 0.15);

  // Set hand forward direction
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = hand_frame;
  vec.vector.x = 1.5;
  front_approach->setDirection(vec);
  task.add(std::move(front_approach));

  auto open_hand = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
  open_hand->setGroup(hand_group_name);
  open_hand->setGoal("close");
  task.add(std::move(open_hand));



  //Falbacks examplme not working
  // auto ptp = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "pilz_industrial_motion_planner", "PTP");
  // auto rrtconnect = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl", "RRTConnectkConfigDefault");

  // fallbacks to reach target_state
  // std::cout << "Adding fallback\n\n";
  // auto fallbacks = std::make_unique<mtc::Fallbacks>("move to packing");
  // {
  //   task.properties().exposeTo(fallbacks->properties(), { "eef", "group", "ik_frame" });
  //   auto add_to_fallbacks{ [&](auto& solver, auto& name) {
  //       auto move_to = std::make_unique<mtc::stages::MoveTo>(name, solver);
  //       // move_to->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  //       move_to->setGroup("panda_arm");
  //       move_to->setGoal("other");
  //       fallbacks->add(std::move(move_to));
  //   } };
  //   add_to_fallbacks(cartesian_planner, "Cartesian path");
  //   add_to_fallbacks(sampling_planner, "sampling path");
  //   // add_to_fallbacks(rrtconnect, "RRT path");
  //   task.add(std::move(fallbacks));
  // }

  return task;
}

mtc::Task My_tutorial::alternativesTask()
{
  mtc::Task task;
  task.stages()->setName("Alternatives task");
  task.loadRobotModel( node_ );

  const auto& arm_group_name = "panda_arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "panda_hand";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  // Disable warnings for this line, as it's a variable that's set but not used in this example
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  #pragma GCC diagnostic pop


  // Current stage to represent the current robot state   
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));


  // Set up planners
    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    sampling_planner->setProperty("goal_joint_tolerance", 1e-5); // If this is large then we get an error - that planned trajectory is too far from the goal etc
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(.005);

  // MoveTo stage to move gripper to open pose
    auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    stage_open_hand->setGroup(hand_group_name);
    stage_open_hand->setGoal("open");
    task.add(std::move(stage_open_hand));

  // Connect stage without alternatives - does only connect using default cost - not sure what that is yet 
    // auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
    //   "move to pick",
    //   mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
    // stage_move_to_pick->setTimeout(15.0);
    // stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
    // task.add(std::move(stage_move_to_pick));


  // Alternatives stage : A parallel stage that takes mutliple stages and computes them in parallel
  // can select a solution based on the different computation of cost here
  auto alternatives{ std::make_unique<mtc::Alternatives>("connect") };
  {
      //Connect stage that finds a path between the robot states on either side of the pipeline
      auto connect{ std::make_unique<mtc::stages::Connect>(
          "path length", mtc::stages::Connect::GroupPlannerVector{ { "panda_arm", sampling_planner } }) };
      connect->setCostTerm(std::make_unique<mtc::cost::PathLength>());
      alternatives->add(std::move(connect));
  }
  {
      //Connect stage that finds a path between the robot states on either side of the pipeline
      auto connect{ std::make_unique<mtc::stages::Connect>(
          "trajectory duration", mtc::stages::Connect::GroupPlannerVector{ { "panda_arm", sampling_planner } }) };
      connect->setCostTerm(std::make_unique<mtc::cost::TrajectoryDuration>());
      alternatives->add(std::move(connect));
  }
  task.add(std::move(alternatives));

  mtc::Stage* attach_object_stage =  nullptr;  // Forward attach_object_stage to place pose generator

  /** SERIAL CONTAINER TO Pick object up
   * 1. approach the object from a given orientation (using carteisan planner)
   * 2. Generate grasp poses around the object
   * 3. Wrap the grasp generator in a IK module to provide a robot pose for that grasp orientation
   * 4. Allow collision between hand and object
   * 5. close hand
   * 6. attach object to hand frame
   */
  {
    // A serial container that contains a series of stages as a subtask
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });
    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner );
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.15);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.x = -1.0;
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
      Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().z() = 0.1;

      // Compute IK
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }
  

    //Modify planning scene stage allows us to allow/disallow collisions
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
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
      stage->setGoal("close");
      grasp->insert(std::move(stage));
    }

    //Modify planning scene stage allows us to make changes to objects in the planning scene
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
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));

  }
  return task;
}

mtc::Task My_tutorial::boxTask()
{
  mtc::Task task;
  task.stages()->setName("box task");
  task.loadRobotModel( node_ );

  const auto& arm_group_name = "panda_arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "panda_hand";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop
    
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  // sampling_planner->setProperty("goal_joint_tolerance", 0.01); // Adjust tolerance
  // sampling_planner->setProperty("goal_position_tolerance", 0.01); // Adjust tolerance
  // sampling_planner->setProperty("goal_orientation_tolerance", 0.01); // Adjust tolerance
  // sampling_planner->setProperty("planning_time", 10.0); // Increase planning time
	sampling_planner->setProperty("goal_joint_tolerance", 1e-5); // If this is large then we get an error - that planned trajectory is too far from the goal etc


  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.005);



  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
    "move to pick",
    mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  stage_move_to_pick->setTimeout(15.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  mtc::Stage* attach_object_stage =
      nullptr;  // Forward attach_object_stage to place pose generator



  /** SERIAL CONTAINER TO Pick object up
   * 1. approach the object from a given orientation (using carteisan planner)
   * 2. Generate grasp poses around the object
   * 3. Wrap the grasp generator in a IK module to provide a robot pose for that grasp orientation
   * 4. Allow collision between hand and object
   * 5. close hand
   * 6. attach object to hand frame
   */
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });
    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner );
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.15);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.x = -1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    } 
    {
      // Sample grasp pose
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject("box");
      stage->setAngleDelta(M_PI / 4);
      stage->setMonitoredStage(current_state_ptr);  // Hook into current state
      
      Eigen::Isometry3d grasp_frame_transform;
      float angle = M_PI/2;
      Eigen::Quaterniond q = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().z() = 0.1;

      // Compute IK
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }
  

    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,box)");
      stage->allowCollisions("box",
                            task.getRobotModel()
                                ->getJointModelGroup(hand_group_name)
                                ->getLinkModelNamesWithCollisionGeometry(),
                            true);
      grasp->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("close");
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("box", hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }
    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));

  }

  {
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
                                                  { hand_group_name, sampling_planner } });
    stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));
  }
  /** SERIAL CONTAINER TO LIFT AND PLACE OBJECTS
   * 1.
   * 2.
   * 3.
   */
  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });

    {
      // Sample place pose
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject("box");

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = "box";
      target_pose_msg.pose.position.y = 0.5;
      target_pose_msg.pose.orientation.w = 1.0;
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

      // Compute IK
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame("box");
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));

    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }
    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,box)");
      stage->allowCollisions("box",
                            task.getRobotModel()
                                ->getJointModelGroup(hand_group_name)
                                ->getLinkModelNamesWithCollisionGeometry(),
                            false);
      place->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("box", hand_frame);
      place->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");

      // Set retreat direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 0.1;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }
    
    task.add(std::move(place));
  }
  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<My_tutorial>(options);
  mtc_task_node->setupPlanningScene();
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  std::cout << "STARTING TASKS\n\n";

//   mtc_task_node->setSceneTask();
  mtc_task_node->doTask();
//   mtc_task_node->doTask(mtc_task_node->pickPlaceTask());

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}