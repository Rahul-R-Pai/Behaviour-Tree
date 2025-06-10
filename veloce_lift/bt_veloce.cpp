// modbus_nodes.cpp
// BehaviorTree.CPP action nodes for Modbus register writes with delay

#include <behaviortree_cpp_v3/bt_factory.h>
#include <thread>
#include <chrono>

using namespace BT;

// Base class for write-and-delay actions
template<int REG_ADDR>
class WriteRegisterDelay : public SyncActionNode {
public:
  WriteRegisterDelay(const std::string& name, const NodeConfiguration& config)
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
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout<<"Moving to Pos: "<<0<<std::endl;
    // Optionally, write zero or confirmation value
    // ModbusComm::instance().writeRegister(REG_ADDR, 0);
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
