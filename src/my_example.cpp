#include <rclcpp/rclcpp.hpp>
//Headers for the planning scene interface, collision objects, primitives etc
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/cost_terms.h>
#include <moveit/planning_scene/planning_scene.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("my_example");

class CustomMtcPipeline : public rclcpp::Node {

    public:
    CustomMtcPipeline(const rclcpp::NodeOptions& options)
    : Node ("my_example",options){

    }

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface()
    {
    return this->get_node_base_interface();
    }

    void addTable(){

        moveit::planning_interface::PlanningSceneInterface psi;
        
        moveit_msgs::msg::CollisionObject table;
        table.id = "table";
        table.header.frame_id = "world";
        shape_msgs::msg::SolidPrimitive cuboid;

        // Define the size of the box in meters
        cuboid.type = cuboid.BOX;
        cuboid.dimensions.resize(3);
        cuboid.dimensions[cuboid.BOX_X] = 2;
        cuboid.dimensions[cuboid.BOX_Y] = 2;
        cuboid.dimensions[cuboid.BOX_Z] = 0.1;

        geometry_msgs::msg::Pose pose;
        pose.position.x = 0;
        pose.position.y = 0;
        pose.position.z = -0.051;
        pose.orientation.w = 1.0;

        table.primitives.push_back(cuboid);
        table.primitive_poses.push_back(pose);
        table.operation = table.ADD;

        // Apply the table collision object
        psi.applyCollisionObject(table);
    
    }

    void addCylinder(){
        // Initialize the Planning Scene Interface
        moveit::planning_interface::PlanningSceneInterface psi;
    
        moveit_msgs::msg::CollisionObject cylinder;
        cylinder.id = "cylinder";
        cylinder.header.frame_id = "world";
        cylinder.primitives.resize(1);
        cylinder.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
        cylinder.primitives[0].dimensions = { 0.3, 0.02 };

        geometry_msgs::msg::Pose pose4;
        pose4.position.x = 0.4;
        pose4.position.y = -0.3;
        pose4.position.z = 0.3;
        pose4.orientation.w = 1.0;
        cylinder.pose = pose4;

        // Apply the table collision object
        psi.applyCollisionObject(cylinder);

    }

    void addBox(){
        // Initialize the Planning Scene Interface
    moveit::planning_interface::PlanningSceneInterface psi;
  
    // BOX
    moveit_msgs::msg::CollisionObject object;
    object.id = "box";
    object.header.frame_id = "world";
    object.primitives.resize(1);

    // Define the size of the box in meters
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;//object.primitives[0].BOX;
    object.primitives[0].dimensions.resize(3);
    object.primitives[0].dimensions[object.primitives[0].BOX_X] = 0.05;
    object.primitives[0].dimensions[object.primitives[0].BOX_Y] = 0.05;
    object.primitives[0].dimensions[object.primitives[0].BOX_Z] = 0.05;

    object.primitive_poses.resize(1);
    object.primitive_poses[0].position.x = 0.5;
    object.primitive_poses[0].position.y = -0.2;
    object.primitive_poses[0].position.z = 0.2;//-0.026;
    object.primitive_poses[0].orientation.w = 1.0;
    object.operation = object.ADD;

    // Apply the table collision object
    psi.applyCollisionObject(object);

    }

// TASKS DEFINITION

    /**  Task flow :
    *     1. Current stage
    *     2. MoveTo ready position
    *     3. MoveRelative with Cartesian planner : end eff moves back on it's x axis by 2 units
    *     4. MoveTo ready position
    *     5. MoveRelative with Sampling planner : end eff moves back on it's x axis by 2 units
    *     6. MoveTo ready position
    * 
    *   Observations :
    *      Notice how the sampling planner (using RRT* from OMPL backend) behaves compared to the cartesian planner
    *      Samping planner : samples a route such that the end effector position is at the desired position of relative offset from start
    *      Cartesian Planner : generates a path where the end effector is constrained to move along cartesian axes to reach the desired position
    */
    moveit::task_constructor::Task moveRelativeStageTask(){
        
        // Relevant groups and frames
        const auto& arm_group_name = "panda_arm";
        const auto& hand_frame = "panda_hand";
        const auto& hand_group_name = "hand";


        // Initialize the task
        moveit::task_constructor::Task task;
        task.setName("Sample MoveTo task");
        
        
        // Add propoerties : These are like variables we can add to the task, can be anything. Some stages expect some properties to be given, such as 'group' for moveRelative
        // In this example, we specify ik Frame and hand group specifically in the stage and not inherit it. But the 'group' for move relative is specified to be taken from task as an example
        task.setProperty("group", arm_group_name);
        task.setProperty("eef", hand_group_name);
        task.setProperty("ik_frame", hand_frame);

        // Expects a shared pointer to the node class - 'this' is not a shared pointer, so within a Node subclass we can get the shared reference using this function 'shared_from_this()'
        // Needs to be loaded before task.init() is called to initialize the task later
        task.loadRobotModel(shared_from_this());

        // Sampling planner and a cartesian planner example - can try out either
        auto cartesian_planner = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();
        cartesian_planner->setMaxVelocityScalingFactor(1.0);
        cartesian_planner->setMaxAccelerationScalingFactor(1.0);
        cartesian_planner->setStepSize(.005);

        auto sampling_planner = std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(shared_from_this());
        sampling_planner->setProperty("goal_joint_tolerance", 1e-5); // If this is large then we get an error - that planned trajectory is too far from the goal etc



        // 1. Current stage : This is the current state of the robot and scene - this is a generator stage
        auto current_stage = std::make_unique<moveit::task_constructor::stages::CurrentState>("current");
        task.add(std::move(current_stage));

        // 2. MoveTo Stage allows us to move a planning group to a pre-defined or specified RobotState : pre-defined states can be found in the SRDF
        auto move_to_ready_stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("MoveTo predefined position (ready pose)", sampling_planner);
            move_to_ready_stage->setGroup(arm_group_name);
            move_to_ready_stage->setGoal("ready"); // SRDF has a 'ready' state defined
            move_to_ready_stage->setTimeout(10);   // Default timeout is 1 for most stages, but we can adjust it if needed
            // move_to_ready_stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT,{ "eef", "group", "ik_frame" });
        task.add(std::move(move_to_ready_stage));


        // 3. MoveRelative stage : moves the robot relative to the specified frame by a transformation, in a manner computed by the chosen planner
        auto move_relative_with_cartesian_planner_stage =  std::make_unique<moveit::task_constructor::stages::MoveRelative>("MoveRelative with cartesian planner", cartesian_planner );
            //set properties
            move_relative_with_cartesian_planner_stage->properties().set("marker_ns", "approach_object"); // namespace for marker arrow in Rviz - viz only
            move_relative_with_cartesian_planner_stage->properties().set("link", hand_frame);   // The moverelative is performed on this frame/link
             // Only Move Relative in specified direction given by the vector - by as much as it can move within the min/max range
             // If it cannot complete max distance, it will do the most it can within the min limit or throw an error if cannot even move the min value
            move_relative_with_cartesian_planner_stage->setMinMaxDistance(0.01, 0.3);          
            
            // Inherit required properties from parent stage/task
            move_relative_with_cartesian_planner_stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "group" });
            // move_relative_with_cartesian_planner_stage->setGroup(arm_group_name); - this also works if we do not want to inherit the 'group' property from parent stage and set it directly

            // move in a specified transformation wrt to a given frame
            geometry_msgs::msg::Vector3Stamped vec_cartesian;
            vec_cartesian.header.frame_id = hand_frame;
            vec_cartesian.vector.x = -2.0;
            move_relative_with_cartesian_planner_stage->setDirection(vec_cartesian);
        task.add(std::move(move_relative_with_cartesian_planner_stage));


        // 4. Move To allows us to move a planning group to a pre-defined or specified RobotState : pre-defined states can be found in the SRDF
        move_to_ready_stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("MoveTo predefined position (ready pose)", sampling_planner);
            move_to_ready_stage->setGroup(arm_group_name);
            move_to_ready_stage->setGoal("ready");
            move_to_ready_stage->setIKFrame(hand_frame);
            // move_to_ready_stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT,{ "eef", "group", "ik_frame" });
        task.add(std::move(move_to_ready_stage));


        // 5. Move Relative stage but with a different planner
        auto move_relative_with_sampling_planner_stage =  std::make_unique<moveit::task_constructor::stages::MoveRelative>("MoveRelative with sampling planner", sampling_planner );
            //set properties
            move_relative_with_sampling_planner_stage->properties().set("marker_ns", "approach_object");
            move_relative_with_sampling_planner_stage->properties().set("link", hand_frame);
            move_relative_with_sampling_planner_stage->setMinMaxDistance(0.01, 0.3);
            // Inherent properties of chioce from parent stage/task
            move_relative_with_sampling_planner_stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "group" });

            // move in a specified transformation wrt to a given frame
            geometry_msgs::msg::Vector3Stamped vec_sampling;
            vec_sampling.header.frame_id = hand_frame;
            vec_sampling.vector.x = -2.0;
            move_relative_with_sampling_planner_stage->setDirection(vec_sampling);
        task.add(std::move(move_relative_with_sampling_planner_stage));

        // 6. Move To allows us to move a planning group to a pre-defined or specified RobotState : pre-defined states can be found in the SRDF
        move_to_ready_stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("MoveTo predefined position (ready pose)", sampling_planner);
            move_to_ready_stage->setGroup(arm_group_name);
            move_to_ready_stage->setGoal("ready");
        task.add(std::move(move_to_ready_stage));


        return task;
    }

    /**  Task flow :
    *     1. Current stage
    *     2. MoveTo arm to ready position
    *     3. MoveTo gripper to open position
    *     4. Merger container
    *           a. MoveRelative arm back
    *           b. MoveTo gripper to close position
    *     5. MoveTo gripper to open position
    *     6. MoveRelative arm back
    *     7. MoveTo gripper close position
    * 
    *   Observations :
    *      Here we show that merger allows you to plan the motion of two independent planning groups simultaneously
    *      We move the arm back, and close the gripper -> first using the merger and the second time serially - and we can see the difference
    */
    moveit::task_constructor::Task mergerTask(){
        //Relevant groups and frames
        const auto& arm_group_name = "panda_arm";
        const auto& hand_frame = "panda_hand";
        const auto& hand_group_name = "hand";

        // Initialize the task
        moveit::task_constructor::Task task;
        task.setName("Sample Merger task");
        
        //Add propoerties if needed
        task.setProperty("group", arm_group_name);
        task.setProperty("eef", hand_group_name);
        task.setProperty("ik_frame", hand_frame);

        //Load robot model
        task.loadRobotModel(shared_from_this());

        // Define Planners
        auto cartesian_planner = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();
        // cartesian_planner->setMaxVelocityScalingFactor(1.0);
        // cartesian_planner->setMaxAccelerationScalingFactor(1.0);
        // cartesian_planner->setStepSize(.005);

        auto interpolation_planner = std::make_shared<moveit::task_constructor::solvers::JointInterpolationPlanner>();
        auto sampling_planner = std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(shared_from_this());
        sampling_planner->setProperty("goal_joint_tolerance", 1e-5); // If this is large then we get an error - that planned trajectory is too far from the goal etc

        // Current stage : This is the current state of the robot and scene - this is a generator stage
        auto current_stage = std::make_unique<moveit::task_constructor::stages::CurrentState>("current");
        task.add(std::move(current_stage));
       
        // Move to Ready position
        auto move_to_ready_stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("MoveTo predefined position (ready pose)", sampling_planner);
            move_to_ready_stage->setGroup(arm_group_name);
            move_to_ready_stage->setGoal("ready"); 
        task.add(std::move(move_to_ready_stage));


        // MoveTo stage : This moves the the planning group to the specified predefined joined state defined in the SRDF file
        auto stage_open_hand =  std::make_unique<moveit::task_constructor::stages::MoveTo>("open hand", interpolation_planner);
            stage_open_hand->setGroup(hand_group_name);
            stage_open_hand->setGoal("open");
        task.add(std::move(stage_open_hand));


        // Merger is an example of a parallel container : Where we can have simultaneous planning of two independent planning groups (Gripper, Robot Arm)
        auto merger = std::make_unique<moveit::task_constructor::Merger>("move arm and close gripper");
        {
                // Expose properties of chioce to the container and the stages within it
                task.properties().exposeTo(merger->properties(), { "eef", "group", "ik_frame" });


                // MoveRelative stage moves the robot in the direction specified using a specified planner
                auto move_back_stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>("back merger", cartesian_planner);
                    move_back_stage->properties().set("marker_ns", "approach_object");
                    move_back_stage->properties().set("link", hand_frame);
                    move_back_stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "group" });
                    move_back_stage->setMinMaxDistance(0.01, 0.15);

                    geometry_msgs::msg::Vector3Stamped vec;
                    vec.header.frame_id = hand_frame;
                    vec.vector.x = -1;
                    move_back_stage->setDirection(vec);
                merger->add(std::move(move_back_stage));

                // MoveTo stage to move the gripper to open position
                auto close_hand = std::make_unique<moveit::task_constructor::stages::MoveTo>("close hand merger", interpolation_planner);
                    close_hand->setGroup(hand_group_name);
                    close_hand->setGoal("close");
                merger->add(std::move(close_hand));
        }
        task.add(std::move(merger));

        // MoveTo stage : This moves the the planning group to the specified predefined joined state defined in the SRDF file
        stage_open_hand =  std::make_unique<moveit::task_constructor::stages::MoveTo>("open hand", interpolation_planner);
            stage_open_hand->setGroup(hand_group_name);
            stage_open_hand->setGoal("open");
        task.add(std::move(stage_open_hand));

        // Now moving each planning group independently to see the difference without parallel execution

        // MoveRelatvie stage
        auto back_move_stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>("back without merger", cartesian_planner);
            back_move_stage->properties().set("marker_ns", "approach_object");
            back_move_stage->properties().set("link", hand_frame);
            back_move_stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "group" });
            back_move_stage->setMinMaxDistance(0.01, 0.15);

            // Set hand forward direction
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = hand_frame;
            vec.vector.x = -1;
            back_move_stage->setDirection(vec);
        task.add(std::move(back_move_stage));

        //MoveTo stage
        auto open_hand = std::make_unique<moveit::task_constructor::stages::MoveTo>("close hand", interpolation_planner);
            open_hand->setGroup(hand_group_name);
            open_hand->setGoal("close");
        task.add(std::move(open_hand));

        return task;
    }   

    /**  Task flow :
    *     1. Current stage
    *     2. MoveTo arm to ready position
    *     3. MoveRelative arm backward
    *     4. Alternatvies container
    *           a. MoveTo arm to extended position
    *           b. MoveTo arm to ready position
    * 
    *   Observations :
    *      Here we show that alternatives allows you to plan two different plans - maybe to different locations, or different planners or different costs
    */
    moveit::task_constructor::Task alternativesTask(){

        //Relevant groups and frames
        const auto& arm_group_name = "panda_arm";
        const auto& hand_frame = "panda_hand";

        // Initialize the task
        moveit::task_constructor::Task task;
        task.setName("Sample alternatvies task");
        
        //Add propoerties if needed
        task.setProperty("group", arm_group_name);

        //Load robot model
        task.loadRobotModel(shared_from_this());

        // Define Planners
        auto cartesian_planner = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();
        auto interpolation_planner = std::make_shared<moveit::task_constructor::solvers::JointInterpolationPlanner>();
        auto sampling_planner = std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(shared_from_this());
        sampling_planner->setProperty("goal_joint_tolerance", 1e-5); // If this is large then we get an error - that planned trajectory is too far from the goal etc

        // Current stage : This is the current state of the robot and scene - this is a generator stage
        auto current_stage = std::make_unique<moveit::task_constructor::stages::CurrentState>("current");
        task.add(std::move(current_stage));

        // Move to Ready position
        auto move_to_ready_stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("MoveTo predefined position (ready pose)", sampling_planner);
            move_to_ready_stage->setGroup(arm_group_name);
            move_to_ready_stage->setGoal("ready"); 
        task.add(std::move(move_to_ready_stage));

        //MoveRelative back and to the side by a little
        auto move_relative_with_sampling_planner_stage =  std::make_unique<moveit::task_constructor::stages::MoveRelative>("MoveRelative with sampling planner", cartesian_planner );
            //set properties
            move_relative_with_sampling_planner_stage->properties().set("marker_ns", "approach_object");
            move_relative_with_sampling_planner_stage->properties().set("link", hand_frame);
            move_relative_with_sampling_planner_stage->setMinMaxDistance(0.01, 0.3);
            move_relative_with_sampling_planner_stage->setTimeout(10);
            // Inherent properties of chioce from parent stage/task
            move_relative_with_sampling_planner_stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "group" });

            // move in a specified transformation wrt to a given frame
            geometry_msgs::msg::Vector3Stamped vec_sampling;
            vec_sampling.header.frame_id = hand_frame;
            vec_sampling.vector.x = -1;
            vec_sampling.vector.y = 1;
            move_relative_with_sampling_planner_stage->setDirection(vec_sampling);
        task.add(std::move(move_relative_with_sampling_planner_stage));


        // Alternatives : A parallel container that takes mutliple stages and computes them in parallel
        // can select a solution based on the different computation of cost here
        auto alternatives{ std::make_unique<moveit::task_constructor::Alternatives>("move back to ready pose") };
        {
            auto move_to_extended_stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("MoveTo (extended pose)", sampling_planner);
                move_to_extended_stage->setGroup(arm_group_name);
                move_to_extended_stage->setGoal("extended");
                // move_to_extended_stage->setCostTerm(std::make_unique<moveit::task_constructor::cost::PathLength>()); // Can add different cost terms
            alternatives->add(std::move(move_to_extended_stage));
        }
        {
           auto move_to_ready_stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("MoveTo (Ready pose)", sampling_planner);
                move_to_ready_stage->setGroup(arm_group_name);
                move_to_ready_stage->setGoal("ready");
                // move_to_ready_stage->setCostTerm(std::make_unique<moveit::task_constructor::cost::TrajectoryDuration>());
            alternatives->add(std::move(move_to_ready_stage));
        }
        task.add(std::move(alternatives));
        return task;
    }

    /**  Task flow :
    *     1. Current stage
    *     2. MoveTo arm to ready position
    *     3. MoveTo open gripper
    *     4. Connect
    *     5. Serial container : Grasp and lift object
    *           a. MoveRelative (pre-grasp approach)
    *               b. Generate Grasp Pose
    *               c. ComputeIK wrapper
    *           d. ModifyPlanningScene to allow collision with object
    *           e. MoveTo to close gripper
    *           f. ModifyPlanningScene to attach object frame to hand frame
    *           g. MoveRelative to lift robot arm  + object upwards
    *     6. Connect
    *     7. Serial Container : Place object and retreat
    *               a. GeneratePlacePose at desired pose
    *               b. ComputeIK wrapper
    *           c. MoveTo open gripper
    *           d. ModifyPlanningScene to forbid collision with object and gripper
    *           e. ModifyPlanningScene to detach frame of object with gripper
    *           f. MoveRelative to retreat from the object
    *     8. MoveTo arm to ready pose
    *   
    *   Observations :
    *      Notice how the planning happens from the generators - not in linear order. Within a Serial container, the planning begins at the Generate(Grasp/place) stage - and outside it begins at current stage
    *      and since each generate provides a specific robot location/configuration, the connect stage is used to plan the path of a robot between two defined configurations
    */
    moveit::task_constructor::Task pickObjectTask(std::string object, geometry_msgs::msg::PoseStamped target_pose_msg){
        // Relevant groups and frames
        const auto& arm_group_name = "panda_arm";
        const auto& hand_frame = "panda_hand";
        const auto& hand_group_name = "hand";


        // Initialize the task
        moveit::task_constructor::Task task;
        task.setName("pick object task : " + object);
        
        
        // Add propoerties : These are like variables we can add to the task, can be anything. Some stages expect some properties to be given, such as 'group' for moveRelative
        // In this example, we specify ik Frame and hand group specifically in the stage and not inherit it. But the 'group' for move relative is specified to be taken from task as an example
        task.setProperty("group", arm_group_name);
        task.setProperty("eef", hand_group_name);
        task.setProperty("ik_frame", hand_frame);

        //Load Robot Model
        task.loadRobotModel(shared_from_this());

        // Initialize planners
        auto interpolation_planner = std::make_shared<moveit::task_constructor::solvers::JointInterpolationPlanner>();
        auto cartesian_planner = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();
        cartesian_planner->setMaxVelocityScalingFactor(1.0);
        cartesian_planner->setMaxAccelerationScalingFactor(1.0);
        cartesian_planner->setStepSize(.005);

        auto sampling_planner = std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(shared_from_this());
        sampling_planner->setProperty("goal_joint_tolerance", 1e-5); // If this is large then we get an error - that planned trajectory is too far from the goal etc

        //Stage pointers
        moveit::task_constructor::Stage* attach_object_stage_pointer = nullptr;
        moveit::task_constructor::Stage* current_state_ptr = nullptr; 

        // 1. Current stage
        auto current_stage = std::make_unique<moveit::task_constructor::stages::CurrentState>("current");
          current_state_ptr = current_stage.get();
        task.add(std::move(current_stage));

        // 2. MoveTo arm to ready position
        auto move_to_ready_stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("MoveTo predefined position (ready pose)", sampling_planner);
            move_to_ready_stage->setGroup(arm_group_name);
            move_to_ready_stage->setGoal("ready");
        task.add(std::move(move_to_ready_stage));
            
        // 3. MoveTo stage : open gripper
        auto stage_open_hand =  std::make_unique<moveit::task_constructor::stages::MoveTo>("open hand", interpolation_planner);
            stage_open_hand->setGroup(hand_group_name);
            stage_open_hand->setGoal("open");
        task.add(std::move(stage_open_hand));

        // 4. Connect stage : This stage moves the arm from the configuration in the above stage to the configuration in the below stage 
        auto stage_move_to_pick = std::make_unique<moveit::task_constructor::stages::Connect>("move to pick",moveit::task_constructor::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
            stage_move_to_pick->setTimeout(15.0);
            stage_move_to_pick->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);
        task.add(std::move(stage_move_to_pick));

        // 5. Serial Container for picking object : This is almost like a task within a task
        auto grasp = std::make_unique<moveit::task_constructor::SerialContainer>("pick object");
        {
            task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
            grasp->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT,{ "eef", "group", "ik_frame" });

            // 5a. MoveRelatvie stage
            auto pre_grasp_approach_stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>("pre grasp approach", cartesian_planner);
                pre_grasp_approach_stage->properties().set("marker_ns", "approach_object");
                pre_grasp_approach_stage->properties().set("link", hand_frame);
                pre_grasp_approach_stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "group" });
                pre_grasp_approach_stage->setMinMaxDistance(0.01, 0.15);

                geometry_msgs::msg::Vector3Stamped vec;
                vec.header.frame_id = hand_frame;
                vec.vector.x = -1;
                pre_grasp_approach_stage->setDirection(vec);
            grasp->insert(std::move(pre_grasp_approach_stage));

            
            // 5b. GenerateGraspPose stage : This samples a series of grasps about the given transform/sampling  !!!!<Need to understand this better>
            auto grasp_generator_stage = std::make_unique<moveit::task_constructor::stages::GenerateGraspPose>("generate grasp pose");
                grasp_generator_stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);
                grasp_generator_stage->properties().set("marker_ns", "grasp_pose");
                grasp_generator_stage->setPreGraspPose("open");
                grasp_generator_stage->setObject(object);
                grasp_generator_stage->setAngleDelta(M_PI / 12);
                grasp_generator_stage->setMonitoredStage(current_state_ptr);  // Hook into current state
                
                Eigen::Isometry3d grasp_frame_transform;
                float angle = M_PI/2;
                Eigen::Quaterniond q =  Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()) *
                                        Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()) *
                                        Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());
                grasp_frame_transform.linear() = q.matrix();
                grasp_frame_transform.translation().z() = 0.1;

            // 5c. ComputeIK stage : This creates the robot configuration for the sampled poses from the grasp generator : Thus this stage provides a set of fixed robot configurations
             auto wrapper = std::make_unique<moveit::task_constructor::stages::ComputeIK>("grasp pose IK", std::move(grasp_generator_stage));
                wrapper->setMaxIKSolutions(8);
                wrapper->setMinSolutionDistance(1.0);
                wrapper->setIKFrame(grasp_frame_transform, hand_frame);
                wrapper->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "eef", "group" });
                wrapper->properties().configureInitFrom(moveit::task_constructor::Stage::INTERFACE, { "target_pose" });
            grasp->insert(std::move(wrapper));

            // 5d. ModifyPlanningScene stage allows us to modify collisions, remove objects, move objects, attach frames etc - anything to do with the current planning scene
            auto allow_collision_stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("allow collision (object,box)");
                allow_collision_stage->allowCollisions(object,task.getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(),true);
            grasp->insert(std::move(allow_collision_stage));

            // 5e. MoveTo stage : close gripper
            auto stage_open_hand =  std::make_unique<moveit::task_constructor::stages::MoveTo>("open hand", interpolation_planner);
                stage_open_hand->setGroup(hand_group_name);
                stage_open_hand->setGoal("open");
            task.add(std::move(stage_open_hand));

            // 5f. ModifyPlanningScene stage to attach the object frame to the hand frame to show us the visualization as if the object is being picked up
            auto attach_object_stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("attach object");
                attach_object_stage->attachObject(object, hand_frame);
                attach_object_stage_pointer = attach_object_stage.get();
            grasp->insert(std::move(attach_object_stage));

            // 5g. MoveRelative to lift the object upward      
            auto lift_object_stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>("lift object", cartesian_planner);
                lift_object_stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "group" });
                lift_object_stage->setMinMaxDistance(0.1, 0.3);
                lift_object_stage->setIKFrame(hand_frame);
                lift_object_stage->properties().set("marker_ns", "lift_object");

                // Set upward direction
                geometry_msgs::msg::Vector3Stamped vec1;
                vec1.header.frame_id = "world";
                vec1.vector.z = 1.0;
                lift_object_stage->setDirection(vec1);
            grasp->insert(std::move(lift_object_stage));
            
        }
        task.add(std::move(grasp));


        // 6. Connect stage 
        stage_move_to_pick = std::make_unique<moveit::task_constructor::stages::Connect>("move to pick",moveit::task_constructor::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
            stage_move_to_pick->setTimeout(15.0);
            stage_move_to_pick->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);
        task.add(std::move(stage_move_to_pick));

        // 7. Serial container to place
        auto place = std::make_unique<moveit::task_constructor::SerialContainer>("place object");
        {
            task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
            place->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT,{ "eef", "group", "ik_frame" });


            // 7a. GeneratePlacePose stage : !!!<Need to check what this does>
            auto place_pose_stage = std::make_unique<moveit::task_constructor::stages::GeneratePlacePose>("generate place pose");
                place_pose_stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);
                place_pose_stage->properties().set("marker_ns", "place_pose");
                place_pose_stage->setObject(object);

                // geometry_msgs::msg::PoseStamped target_pose_msg;
                // target_pose_msg.header.frame_id = object;
                // target_pose_msg.pose.position.y = 0.5;
                // target_pose_msg.pose.orientation.w = 1.0;
                place_pose_stage->setPose(target_pose_msg);
                place_pose_stage->setMonitoredStage(attach_object_stage_pointer);  // Hook into attach_object_stage

            // 7b. Compute IK
            auto wrapper = std::make_unique<moveit::task_constructor::stages::ComputeIK>("place pose IK", std::move(place_pose_stage));
                wrapper->setMaxIKSolutions(5);
                wrapper->setMinSolutionDistance(1.0);
                wrapper->setIKFrame(object);
                wrapper->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "eef", "group" });
                wrapper->properties().configureInitFrom(moveit::task_constructor::Stage::INTERFACE, { "target_pose" });
            place->insert(std::move(wrapper));

            // 7c. MoveTo stage : open gripper
            auto stage_open_hand =  std::make_unique<moveit::task_constructor::stages::MoveTo>("open hand", interpolation_planner);
                stage_open_hand->setGroup(hand_group_name);
                stage_open_hand->setGoal("open");
            place->insert(std::move(stage_open_hand));

            // 7d. ModifyPlanningScene stage allows us to modify collisions, remove objects, move objects, attach frames etc - anything to do with the current planning scene
            auto allow_collision_stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("forbid collision (object,box)");
                allow_collision_stage->allowCollisions(object,task.getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(),false);
            place->insert(std::move(allow_collision_stage));

            // 7e. ModifyPlanningScene stage to attach the object frame to the hand frame to show us the visualization as if the object is being picked up
            auto detach_object_stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("detach object");
                detach_object_stage->detachObject(object, hand_frame);
            place->insert(std::move(detach_object_stage));


            // 7f. MoveRelatvie stage
            auto post_place_retreat_stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>("post place retreat", cartesian_planner);
                post_place_retreat_stage->properties().set("marker_ns", "approach_object");
                post_place_retreat_stage->properties().set("link", hand_frame);
                post_place_retreat_stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "group" });
                post_place_retreat_stage->setMinMaxDistance(0.01, 0.15);

                geometry_msgs::msg::Vector3Stamped vec;
                vec.header.frame_id = hand_frame;
                vec.vector.z = -1;
                post_place_retreat_stage->setDirection(vec);
            place->insert(std::move(post_place_retreat_stage));
        
        task.add(std::move(place));
        }

        // 6. MoveTo arm to ready position
        move_to_ready_stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("MoveTo predefined position (ready pose)", sampling_planner);
            move_to_ready_stage->setGroup(arm_group_name);
            move_to_ready_stage->setGoal("ready");
        task.add(std::move(move_to_ready_stage));

        return task;
    }



    void doTask(moveit::task_constructor::Task task){

        try{
            task.init();
        }
        catch (moveit::task_constructor::InitStageException& e){
            RCLCPP_ERROR_STREAM(LOGGER, e);
            return;
        }
        if (!task.plan(10)){
            RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
            return;
        }
        
        task.introspection().publishSolution(*task.solutions().front());
        // task.introspection().publishAllSolutions(false);
        
        auto result = task.execute(*task.solutions().front());
        if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS){
            RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
            return;
        }
    }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);


    std::shared_ptr<CustomMtcPipeline> mtc_node = std::make_shared<CustomMtcPipeline>(options);
    mtc_node->addTable();
    mtc_node->addCylinder();
    mtc_node->addBox();
    rclcpp::executors::MultiThreadedExecutor executor;
    auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_node]() {
        executor.add_node(mtc_node->getNodeBaseInterface());
        executor.spin();
        executor.remove_node(mtc_node->getNodeBaseInterface());
    });

    // mtc_node->doTask(mtc_node->moveRelativeStageTask());
    // mtc_node->doTask(mtc_node->alternativesTask());
    // mtc_node->doTask(mtc_node->mergerTask());


    std::string cylinder="cylinder";
    geometry_msgs::msg::PoseStamped target_cylinder_pose;
    target_cylinder_pose.header.frame_id = cylinder;
    target_cylinder_pose.pose.position.y = 0.5;
    target_cylinder_pose.pose.position.z = 0.2;
    target_cylinder_pose.pose.orientation.w = 1.0;
    mtc_node->doTask(mtc_node->pickObjectTask(cylinder,target_cylinder_pose));


    std::string box="box";
    geometry_msgs::msg::PoseStamped target_box_pose;
    target_box_pose.header.frame_id = box;
    target_box_pose.pose.position.y = 0.5;
    target_box_pose.pose.position.z =  -0.1;
    target_box_pose.pose.orientation.w = 1.0;
    mtc_node->doTask(mtc_node->pickObjectTask(box,target_box_pose));


    
    spin_thread->join();
    rclcpp::shutdown();

}