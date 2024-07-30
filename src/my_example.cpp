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

    moveit::task_constructor::Task moveRelativeStageTask(){
        
        //Relevant groups and frames
        const auto& arm_group_name = "panda_arm";
        const auto& hand_frame = "panda_hand";


        // Initialize the task
        moveit::task_constructor::Task task;
        task.setName("Sample MoveTo task");
        
        //add propoerties : These are like variables we can add to the task, can be anything. Some stages expect some properties to be given, such as 'group' for moveRelative
        task.setProperty("group", arm_group_name);

        // Expects a shared pointer to the node class - 'this' is not a shared pointer, so within a Node subclass we can get the shared reference using this function 'shared_from_this()'
        // Needs to be loaded before task.init() is called to initialize the task later
        task.loadRobotModel(shared_from_this());

        // Sampling planner and a cartesian planner example - can try out either
        auto cartesian_planner = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();
        auto sampling_planner = std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(shared_from_this());
        sampling_planner->setProperty("goal_joint_tolerance", 1e-5); // If this is large then we get an error - that planned trajectory is too far from the goal etc
        
        //  Add stages to task 

        // Current stage : This is the current state of the robot and scene - this is a generator stage
        auto current_stage = std::make_unique<moveit::task_constructor::stages::CurrentState>("current");
            task.add(std::move(current_stage));
        
            // Pointer to the currrent state
            moveit::task_constructor::Stage* current_state_ptr = current_stage.get(); 


        // Move Relative stage : is a propogator stage that passes on the information to adjecent stages as well as moves the robot relative to it's current position/frame by a given value and orientation
        auto move_stage =  std::make_unique<moveit::task_constructor::stages::MoveRelative>("move a little", sampling_planner );
            //set properties
            move_stage->properties().set("marker_ns", "approach_object");
            move_stage->properties().set("link", hand_frame);
            move_stage->setMinMaxDistance(0.1, 0.15);
            
            // Inherent properties of chioce from parent stage/task
            move_stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "group" });
            // move_stage->setGroup(arm_group_name); - this also works if we do not want to inherit the 'group' property from parent stage


            // move in a specified transformation wrt to a given frame
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = hand_frame;
            vec.vector.x = -1.0;
            move_stage->setDirection(vec);
            task.add(std::move(move_stage));


        // Move To allows us to move a planning group to a pre-defined or specified RobotState : pre-defined states can be found in the SRDF
        auto stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("close hand", sampling_planner);
            stage->setGroup(arm_group_name);
            stage->setGoal("ready");
            task.add(std::move(stage));

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

    void doTask(moveit::task_constructor::Task task){
        // moveit::task_constructor::Task task = moveToTask();
        task.init();
        task.plan(2);
        task.introspection().publishSolution(*task.solutions().front());
        auto result = task.execute(*task.solutions().front());
    }

};


int main(int argc, char** argv){
    rclcpp::init(argc, argv);

    std::shared_ptr<CustomMtcPipeline> mtc_node = std::make_shared<CustomMtcPipeline>();
    mtc_node->addTable();
    mtc_node->addCylinder();
    mtc_node->doTask(mtc_node->moveRelativeStageTask());

    rclcpp::spin(mtc_node);
    rclcpp::shutdown();

}