#include <hiqp/task.h>
#include <hiqp_ros/hiqp_client.h>
#include <std_msgs/msg/float64_multi_array.hpp>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  hiqp_ros::HiQPClient client("","hiqp_controller");
  client.run();
  sleep(1); //wait for client to start up
   
  auto publisher_ = client.getHandle()->create_publisher<std_msgs::msg::Float64MultiArray>
    ("/gripper_controller/commands", 10);
  auto gripper_command = std_msgs::msg::Float64MultiArray();
  gripper_command.data.push_back(0.1);
  gripper_command.data.push_back(0.1);


  std::vector<hiqp_msgs::msg::Primitive> primitives;
  hiqp_msgs::msg::Primitive p1, p2, p3, p4, p5;
  p1.name = "tool";
  p1.type = "frame";
  p1.frame_id = "panda_link7";
  p1.visible = true;
  p1.color = {0.0, 0.0, 1.0, 1.0};
  p1.parameters = {0.0, 0.0, 0.1, 0.0, 0.0, 0.0};

  p2.name = "target";
  p2.type = "frame";
  p2.frame_id = "panda_link0";
  p2.visible = true;
  p2.color = {0.0, 0.0, 1.0, 1.0};
  p2.parameters = {-1.5, 0.0, 0.5, 0, 3.1415, 0};

  p3.name = "tool_point";
  p3.type = "point";
  p3.frame_id = "panda_link7";
  p3.visible = true;
  p3.color = {1.0, 0.0, 0.0, 1.0};
  p3.parameters = {0.0, 0.0, 0.15};

  p4.name = "target_sphere";
  p4.type = "sphere";
  p4.frame_id = "panda_link0";
  p4.visible = true;
  p4.color = {0.0, 0.0, 1.0, 0.5};
  p4.parameters = {0.6, 0.25, 0.1, 0.05};
  
  //test nullspace
  p5.name = "second_point";
  p5.type = "point";
  p5.frame_id = "panda_link4";
  p5.visible = true;
  p5.color = {1.0, 0.0, 0.0, 1.0};
  p5.parameters = {0.0, 0.0, 0.1};

  primitives.push_back(p1);
  primitives.push_back(p2);
  primitives.push_back(p3);
  primitives.push_back(p4);
  primitives.push_back(p5);

  publisher_->publish(gripper_command);
  client.setPrimitives(primitives);
  //std::vector<double> config1 {0.0, -0.5, 0.0, -2.3, 0.0, 1.9, 0.8};
  //std::vector<double> config2 {0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.5};
  std::vector<double> config1 {1.51, -1.17, 0.003, -2.89, -0.0, 1.82, 0.84};
  std::vector<double> config2 {-0.51, -1.17, 0.003, -2.89, -0.0, 1.82, 1.84};

#if 0
  std::vector<std::string> tdef_params{"TDefFullPose"};
  for (auto jointValue : config1) {
    tdef_params.push_back(std::to_string(jointValue));
  }
  bool ret = client.setTask("joint_configuration", 3, true, true, true, tdef_params,
      {"TDynLinear", "0.95"});
  double tol = 0.05; //error tolerance when task is considered complete
  if (ret) {
      std::cerr<<"setting tolerance of "<<tol<<std::endl;
      client.waitForCompletion({"joint_configuration"}, {hiqp_ros::TaskDoneReaction::REMOVE},
          {tol});
  }
#endif

  client.setJointAngles(config1);
  
  gripper_command.data.clear();
  gripper_command.data.push_back(-0.1);
  gripper_command.data.push_back(-0.1);
  publisher_->publish(gripper_command);

  //test frame-frame alignment
  double tol = 0.01; //error tolerance when task is considered complete
  std::string tname = "frame2frame";
  std::vector<std::string> def_params{"TDefGeomAlign", "frame", "frame", "tool = target", "1.5"};
  bool success = client.setTask(tname, 0, true, true, true,
                 def_params, {"TDynLinear", "0.75"});

  if (success) {
    std::cerr<<"Waiting for completion of task"<<std::endl;
    //char c;
    //std::cin>>c;
    client.waitForCompletion({"frame2frame"}, {hiqp_ros::TaskDoneReaction::REMOVE},
        {tol});
  }
  //move to other side 
  
  tol = 0.02;
  client.setJointAngles(config2,false,tol); //set joint angles and do not remove task when done

  //test point in sphere
  tol = 0.01;
  tname = "point_in_sphere";
  def_params = {"TDefGeomProj", "point", "sphere", "tool_point < target_sphere"};
  success = client.setTask(tname, 1, true, true, true,
                 def_params, {"TDynLinear", "0.95"});

  if (success) {
    std::cerr<<"Waiting for completion of task"<<std::endl;
    client.waitForCompletion({"point_in_sphere"}, {hiqp_ros::TaskDoneReaction::NONE},
        {tol});
  }

  //remove the joint task
  //client.removeTask("joint_configuration");
  

  p4.name = "link2_target1";
  p4.type = "sphere";
  p4.frame_id = "panda_link0";
  p4.visible = true;
  p4.color = {0.0, 0.5, 0.5, 0.5};
  p4.parameters = {0.4, 0.2, 0.6, 0.2};
  
  p1.name = "link2_target2";
  p1.type = "sphere";
  p1.frame_id = "panda_link0";
  p1.visible = true;
  p1.color = {0.0, 0.5, 0.5, 0.5};
  p1.parameters = {0.2, -0.2, 0.5, 0.2};

  primitives.clear();
  primitives.push_back(p1);
  primitives.push_back(p4);
  client.setPrimitives(primitives);

  tname = "nullspace_sphere";
  def_params = {"TDefGeomProj", "point", "sphere", "second_point < link2_target1"};
  success = client.setTask(tname, 2, true, true, true,
                 def_params, {"TDynLinear", "0.75"});

  if (success) {
    std::cerr<<"Waiting for completion of task "<<tname<<std::endl;
    client.waitForCompletion({"nullspace_sphere","point_in_sphere"}, {hiqp_ros::TaskDoneReaction::REMOVE, hiqp_ros::TaskDoneReaction::NONE},
        {tol, tol});
  }

  tname = "nullspace_sphere2";
  def_params = {"TDefGeomProj", "point", "sphere", "second_point < link2_target2"};
  success = client.setTask(tname, 2, true, true, true,
                 def_params, {"TDynLinear", "0.75"});

  if (success) {
    std::cerr<<"Waiting for completion of task "<<tname<<std::endl;
    client.waitForCompletion({"nullspace_sphere2","point_in_sphere"}, {hiqp_ros::TaskDoneReaction::REMOVE, hiqp_ros::TaskDoneReaction::NONE},
        {tol, tol});
  }

  //cleanup
  client.removeAllTasks();
  client.removeAllPrimitives(); 
  client.quit();

  return 0;
}
