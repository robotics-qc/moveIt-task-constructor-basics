#include <rclcpp/rclcpp.hpp>


//Headers for the planning scene interface, collision objects, primitives etc
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>

#include <moveit/planning_scene/planning_scene.h>



class CustomMtcPipeline : public rclcpp::Node {

    public:
        CustomMtcPipeline()
        : Node ("my_example"){

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
        cylinder.id = "cylinder2";
        cylinder.header.frame_id = "world";
        cylinder.primitives.resize(1);
        cylinder.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
        cylinder.primitives[0].dimensions = { 0.6, 0.02 };

        geometry_msgs::msg::Pose pose4;
        pose4.position.x = 0.4;
        pose4.position.y = -0.3;
        pose4.position.z = 0.3;
        pose4.orientation.w = 1.0;
        cylinder.pose = pose4;
        psi.applyCollisionObject(cylinder);


        // Apply the table collision object
        psi.applyCollisionObject(cylinder);

    }


    /**  Task flow :
    *      Current stage
    *      MoveTo ready position
    *      MoveRelative with Cartesian planner : end eff moves back on it's x axis by 2 units
    *      MoveTo ready position
    *      MoveRelative with Sampling planner : end eff moves back on it's x axis by 2 units
    *      MoveTo ready position
    * 
    *   Observations :
    *      Notice how the sampling planner (using RRT* from OMPL backend) behaves compared to the cartesian planner
    *      Samping planner : samples a route such that the end effector position is at the desired position of relative offset from start
    *      Cartesian Planner : generates a path where the end effector is constrained to move along cartesian axes to reach the desired position
    */
    moveit::task_constructor::Task moveRelativeStageTask(){
        
        //Relevant groups and frames
        const auto& arm_group_name = "panda_arm";
        const auto& hand_frame = "panda_hand";
        const auto& hand_group_name = "hand";


        // Initialize the task
        moveit::task_constructor::Task task;
        task.setName("Sample MoveTo task");
        
        
        //add propoerties : These are like variables we can add to the task, can be anything. Some stages expect some properties to be given, such as 'group' for moveRelative
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



        // Current stage : This is the current state of the robot and scene - this is a generator stage
        auto current_stage = std::make_unique<moveit::task_constructor::stages::CurrentState>("current");
            task.add(std::move(current_stage));
            // Pointer to the currrent state
            moveit::task_constructor::Stage* current_state_ptr = current_stage.get(); 



        // MoveTo Stage allows us to move a planning group to a pre-defined or specified RobotState : pre-defined states can be found in the SRDF
        auto move_to_ready_stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("MoveTo predefined position (ready pose)", sampling_planner);
            move_to_ready_stage->setGroup(arm_group_name);
            move_to_ready_stage->setGoal("ready"); // SRDF has a 'ready' state defined
            move_to_ready_stage->setTimeout(10);
            // move_to_ready_stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT,{ "eef", "group", "ik_frame" });
            task.add(std::move(move_to_ready_stage));


        // MoveRelative stage : is a propogator stage that passes on the information to adjecent stages
        // Also moves the robot frame sepcified relative to it's current position/frame by a given value and orientation
        auto move_relative_with_cartesian_planner_stage =  std::make_unique<moveit::task_constructor::stages::MoveRelative>("MoveRelative with cartesian planner", cartesian_planner );
            //set properties
            move_relative_with_cartesian_planner_stage->properties().set("marker_ns", "approach_object");
            move_relative_with_cartesian_planner_stage->properties().set("link", hand_frame);
            move_relative_with_cartesian_planner_stage->setMinMaxDistance(0.1, 0.15);

            
            // Inherent properties of chioce from parent stage/task
            // move_relative_with_cartesian_planner_stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT,{ "eef", "group", "ik_frame" });
            move_relative_with_cartesian_planner_stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "group" });
            // move_relative_with_cartesian_planner_stage->setGroup(arm_group_name); - this also works if we do not want to inherit the 'group' property from parent stage


            // move in a specified transformation wrt to a given frame
            geometry_msgs::msg::Vector3Stamped vec_cartesian;
            vec_cartesian.header.frame_id = hand_frame;
            vec_cartesian.vector.x = -2.0;
            move_relative_with_cartesian_planner_stage->setDirection(vec_cartesian);
            task.add(std::move(move_relative_with_cartesian_planner_stage));


        // // Move To allows us to move a planning group to a pre-defined or specified RobotState : pre-defined states can be found in the SRDF
        // move_to_ready_stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("MoveTo predefined position (ready pose)", sampling_planner);
        //     move_to_ready_stage->setGroup(arm_group_name);
        //     move_to_ready_stage->setGoal("ready");
        //     move_to_ready_stage->setIKFrame(hand_frame);
        //     // move_to_ready_stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT,{ "eef", "group", "ik_frame" });

        //     task.add(std::move(move_to_ready_stage));



        // // Move Relative stage but with a different planner
        // auto move_relative_with_sampling_planner_stage =  std::make_unique<moveit::task_constructor::stages::MoveRelative>("MoveRelative with sampling planner", cartesian_planner );
        //     //set properties
        //     move_relative_with_sampling_planner_stage->properties().set("marker_ns", "approach_object");
        //     move_relative_with_sampling_planner_stage->properties().set("link", hand_frame);
        //     move_relative_with_sampling_planner_stage->setMinMaxDistance(0.1, 0.15);
            
        //     // Inherent properties of chioce from parent stage/task
        //     move_relative_with_sampling_planner_stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "group" });


        //     // move in a specified transformation wrt to a given frame
        //     geometry_msgs::msg::Vector3Stamped vec_sampling;
        //     vec_sampling.header.frame_id = hand_frame;
        //     vec_sampling.vector.x = -2.0;
        //     move_relative_with_sampling_planner_stage->setDirection(vec_sampling);
        //     task.add(std::move(move_relative_with_sampling_planner_stage));


        // // Move To allows us to move a planning group to a pre-defined or specified RobotState : pre-defined states can be found in the SRDF
        // move_to_ready_stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("MoveTo predefined position (ready pose)", cartesian_planner);
        //     move_to_ready_stage->setGroup(arm_group_name);
        //     move_to_ready_stage->setGoal("ready");
        //     task.add(std::move(move_to_ready_stage));

        // // Move back to initial state
        // auto return_stage = std::make_unique<moveit::task_constructor::stages::Connect>("move to start",
        //         moveit::task_constructor::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
        //     return_stage->setTimeout(15.0);
        //     return_stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);
        //     task.add(std::move(return_stage));

        
        // //Final fixed State
        // auto fixed = std::make_unique<moveit::task_constructor::stages::FixedState>("final state");
        //     auto scene = std::make_shared<planning_scene::PlanningScene>(task.getRobotModel());
        //     auto& state = scene->getCurrentStateNonConst();
        //     // state.setToDefaultValues();  // initialize state
        //     state.setToDefaultValues(state.getJointModelGroup(arm_group_name), "ready");
        //     // state.setToDefaultValues(state.getJointModelGroup("right_arm"), "home");
        //     state.update();
        //     fixed->setState(scene);
        //     task.add(std::move(fixed));

        return task;
    }



    moveit::task_constructor::Task mergerTask(){
            //Relevant groups and frames
        const auto& arm_group_name = "panda_arm";
        const auto& hand_frame = "panda_hand";
        const auto& hand_group_name = "hand";


        // Initialize the task
        moveit::task_constructor::Task task;
        task.setName("Sample MoveTo task");
        
        
        //add propoerties : These are like variables we can add to the task, can be anything. Some stages expect some properties to be given, such as 'group' for moveRelative
        task.setProperty("group", arm_group_name);
        task.setProperty("eef", hand_group_name);
        task.setProperty("ik_frame", hand_frame);

    
        task.loadRobotModel(shared_from_this());

        // Define Planners
        auto cartesian_planner = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();
        cartesian_planner->setMaxVelocityScalingFactor(1.0);
        cartesian_planner->setMaxAccelerationScalingFactor(1.0);
        cartesian_planner->setStepSize(.005);

        auto interpolation_planner = std::make_shared<moveit::task_constructor::solvers::JointInterpolationPlanner>();
        auto sampling_planner = std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(shared_from_this());
        sampling_planner->setProperty("goal_joint_tolerance", 1e-5); // If this is large then we get an error - that planned trajectory is too far from the goal etc



        // Current stage : This is the current state of the robot and scene - this is a generator stage
        auto current_stage = std::make_unique<moveit::task_constructor::stages::CurrentState>("current");
            task.add(std::move(current_stage));
            // Pointer to the currrent state
            moveit::task_constructor::Stage* current_state_ptr = current_stage.get(); 


        // MoveTo stage : This moves the the planning group to the specified predefined joined state defined in the SRDF file
            auto stage_open_hand =  std::make_unique<moveit::task_constructor::stages::MoveTo>("open hand", interpolation_planner);
            stage_open_hand->setGroup(hand_group_name);
            stage_open_hand->setGoal("open");
            task.add(std::move(stage_open_hand));

        moveit::task_constructor::Stage* attach_object_stage = nullptr;  // Forward attach_object_stage to place pose generator

        // Merger Block example : Merger is a parallel block that allows seperate planning for two independent planning groups simultaneously
            auto merger = std::make_unique<moveit::task_constructor::Merger>("move arm and close gripper");
            {
                task.properties().exposeTo(merger->properties(), { "eef", "group", "ik_frame" });
                // merger->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT,{ "eef", "group", "ik_frame" });


                // MoveRelative stage moves the robot in the direction specified using a specified planner
                auto back_approach = std::make_unique<moveit::task_constructor::stages::MoveRelative>("Approach", cartesian_planner);

                    back_approach->properties().set("marker_ns", "approach_object");
                    back_approach->properties().set("link", hand_frame);
                    back_approach->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "group" });
                    back_approach->setMinMaxDistance(0.1, 0.15);

                    geometry_msgs::msg::Vector3Stamped vec;
                    vec.header.frame_id = hand_frame;
                    vec.vector.x = -1.5;
                    back_approach->setDirection(vec);
                    merger->add(std::move(back_approach));

                // MoveTo stage to move the gripper to open position
                auto open_hand = std::make_unique<moveit::task_constructor::stages::MoveTo>("open hand", interpolation_planner);
                    open_hand->setGroup(hand_group_name);
                    open_hand->setGoal("open");
                    merger->add(std::move(open_hand));
            }
            task.add(std::move(merger));


        //Now moving each planning group independenatly to see the diffeence without parallel execution
        
        // MoveRelatvie stage
            auto front_approach = std::make_unique<moveit::task_constructor::stages::MoveRelative>("Approach", cartesian_planner);
            front_approach->properties().set("marker_ns", "approach_object");
            front_approach->properties().set("link", hand_frame);
            front_approach->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "group" });
            front_approach->setMinMaxDistance(0.1, 0.15);

        // Set hand forward direction
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = hand_frame;
        vec.vector.x = 1.5;
        front_approach->setDirection(vec);
        task.add(std::move(front_approach));

        auto open_hand = std::make_unique<moveit::task_constructor::stages::MoveTo>("close hand", interpolation_planner);
        open_hand->setGroup(hand_group_name);
        open_hand->setGoal("close");
        task.add(std::move(open_hand));

        return task;
    }   



    void doTask(moveit::task_constructor::Task task){
        // moveit::task_constructor::Task task = moveToTask();
        task.init();
        task.plan(10);
        task.introspection().publishSolution(*task.solutions().front());
        auto result = task.execute(*task.solutions().front());
    }

};


int main(int argc, char** argv){
    rclcpp::init(argc, argv);

    std::shared_ptr<CustomMtcPipeline> mtc_node = std::make_shared<CustomMtcPipeline>();
    mtc_node->addTable();
    mtc_node->addCylinder();
    // mtc_node->doTask(mtc_node->moveRelativeStageTask());
    mtc_node->doTask(mtc_node->mergerTask());

    rclcpp::spin(mtc_node);
    rclcpp::shutdown();

}