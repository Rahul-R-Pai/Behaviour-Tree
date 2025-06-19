// modbus_nodes.cpp
// BehaviorTree.CPP action nodes for Modbus register writes with delay

#include <iostream>
#include <chrono>
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
// #include "behaviortree_cpp/loggers/bt_zmq_publisher.h"
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

  // Connect the Groot2Publisher. This will allow Groot2 to
  // get the tree and poll status updates.
  BT::Groot2Publisher publisher(tree);

  // Tick the root until it finishes
  while (tree.rootNode()->executeTick() == NodeStatus::RUNNING) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return 0;
}
