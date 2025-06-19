// modbus_nodes.cpp
// BehaviorTree.CPP action nodes for Modbus register writes with delay

#include <iostream>
#include <chrono>
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include <vector>
#include <thread>


using namespace BT;

// Base class for write-and-delay actions
template<int REG_ADDR>
class WriteRegisterDelay : public SyncActionNode {
public:
  explicit WriteRegisterDelay(const std::string& name, const NodeConfiguration& config)
    : SyncActionNode(name, config) {}

  static PortsList providedPorts() {
    return { InputPort<int>("target", "Value to write to register") };
  }

  NodeStatus tick() override {
    int target;
    if (!getInput("target", target)) {
      throw RuntimeError("Missing required input [target]");
    }

    std::cout<<"Moving to Pos: "<<target<<std::endl;
    // Write target value
    // ModbusComm::instance().writeRegister(REG_ADDR, target);

    // Delay (simulate movement time)
    std::this_thread::sleep_for(std::chrono::seconds(5));

    std::cout<<"Moving to Pos: "<<0<<std::endl;
    // Optionally, write zero or confirmation value
    return NodeStatus::SUCCESS;
  }
};

// Define specific nodes with register addresses
using MoveFork = WriteRegisterDelay<0x1001>;
using MoveTelescope = WriteRegisterDelay<0x1002>;
using MoveTurntable = WriteRegisterDelay<0x1003>;
using MoveLift = WriteRegisterDelay<0x1004>;

int main() {
  BehaviorTreeFactory factory;
  // Register custom nodes
  factory.registerNodeType<MoveFork>("MoveFork");
  factory.registerNodeType<MoveTelescope>("MoveTelescope");
  factory.registerNodeType<MoveTurntable>("MoveTurntable");
  factory.registerNodeType<MoveLift>("MoveLift");
  
  // Load tree from XML
  auto tree = factory.createTreeFromFile("./../basic_movement.xml");

  // Tick the root until it finishes
  while (tree.rootNode()->executeTick() == NodeStatus::RUNNING) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return 0;
}



// ---------------- Dummy Conditions ----------------

BT::NodeStatus BinClearSensors()
{
std::cout << "[Condition] BinClearSensors\n";
return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus BinPresenceSensors()
{
std::cout << "[Condition] BinPresenceSensors\n";
return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus TelescopeHomeSensors()
{
std::cout << "[Condition] TelescopeHomeSensors\n";
return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus RackSensors()
{
std::cout << "[Condition] RackSensors\n";
return BT::NodeStatus::SUCCESS;
}

// Example of condition with a blackboard input/output port
class Lift_Pos : public BT::ConditionNode
{
public:
Lift_Pos(const std::string& name, const BT::NodeConfig& config)
: BT::ConditionNode(name, config) {}

static BT::PortsList providedPorts()
{
return { BT::InputPortstd::string("lift_act_pos") };
}

BT::NodeStatus tick() override
{
std::string val;
getInput("lift_act_pos", val);
std::cout << "[Condition] Lift_Pos with lift_act_pos = " << val << "\n";
return BT::NodeStatus::SUCCESS;
}
};

// ---------------- Dummy Actions ----------------

class moveLift : public BT::SyncActionNode
{
public:
moveLift(const std::string& name, const BT::NodeConfig& config)
: BT::SyncActionNode(name, config) {}

static BT::PortsList providedPorts()
{
return { BT::InputPortstd::string("lift_target_pos") };
}

BT::NodeStatus tick() override
{
std::string pos;
getInput("lift_target_pos", pos);
std::cout << "[Action] moveLift to " << pos << "\n";
return BT::NodeStatus::SUCCESS;
}
};

class moveTelescope : public BT::SyncActionNode
{
public:
moveTelescope(const std::string& name, const BT::NodeConfig& config)
: BT::SyncActionNode(name, config) {}

static BT::PortsList providedPorts()
{
return {
BT::InputPortstd::string("telescope_target_pos"),
BT::InputPortstd::string("control_mode")
};
}

BT::NodeStatus tick() override
{
std::string target, mode;
getInput("telescope_target_pos", target);
getInput("control_mode", mode);
std::cout << "[Action] moveTelescope to " << target << " in mode " << mode << "\n";
return BT::NodeStatus::SUCCESS;
}
};

// ---------------- Main ----------------

int main()
{
BT::BehaviorTreeFactory factory;

// Register all actions and conditions here
factory.registerSimpleCondition("BinClearSensors", BinClearSensors);
factory.registerSimpleCondition("BinPresenceSensors", BinPresenceSensors);
factory.registerSimpleCondition("RackSensors", RackSensors);
factory.registerSimpleCondition("TelescopeHomeSensors", TelescopeHomeSensors);
factory.registerNodeType<Lift_Pos>("Lift_Pos");
factory.registerNodeType<moveLift>("moveLift");
factory.registerNodeType<moveTelescope>("moveTelescope");

// Load tree from XML
auto tree = factory.createTreeFromFile("behavior_tree.xml");

// Connect the Groot2Publisher. This will allow Groot2 to
// get the tree and poll status updates.
BT::Groot2Publisher publisher(tree);

std::cout << "\nStarting tree tick...\n";
BT::NodeStatus status = tree.tickWhileRunning();
std::cout << "Tree finished with status: " << toStr(status) << "\n";

return 0;
}