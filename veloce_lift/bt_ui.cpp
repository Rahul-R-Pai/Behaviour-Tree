#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <ncurses.h>
#include <thread>
#include <chrono>
#include <map>
#include <string>
#include <cstdlib>

struct BBValue {
    std::string key;
    std::string type; // "bool", "string", "double", or "int"
    std::string value;
};

// Simulated current positions
std::map<std::string, std::string> current_positions = {
    {"current_lift_pos", "0.0"},
    {"current_telescope_pos", "0"},
    {"current_fork_pos", "0.0"},
    {"current_turntable_pos", "home"}
};

// Custom StatefulActionNode for MoveLift
class MoveLiftNode : public BT::StatefulActionNode
{
public:
    MoveLiftNode(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config), start_time_(std::chrono::steady_clock::time_point())
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("lift_target_position_m"),
            BT::OutputPort<double>("current_lift_position_m"),
            BT::OutputPort<int>("error_code"),
            BT::OutputPort<std::string>("error_msg")
        };
    }

    BT::NodeStatus onStart() override
    {
        int error_code = 0;
        std::string error_msg = "Success";
        if (!getInput("lift_target_position_m", target_pos_))
        {
            error_code = 1;
            error_msg = "Missing lift_target_position_m";
            setOutput("error_code", error_code);
            setOutput("error_msg", error_msg);
            std::cout << "[Action] MoveLift failed: " << error_msg << std::endl;
            return BT::NodeStatus::FAILURE;
        }
        start_time_ = std::chrono::steady_clock::now();
        std::cout << "[Action] MoveLift started to " << target_pos_ << " m" << std::endl;
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time_);
        if (elapsed >= std::chrono::seconds(5))
        {
            setOutput("current_lift_position_m", target_pos_);
            setOutput("error_code", 0);
            setOutput("error_msg", "Success");
            current_positions["current_lift_pos"] = std::to_string(target_pos_);
            std::cout << "[Action] MoveLift reached target: " << target_pos_ << " m" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << "[Action] MoveLift halted" << std::endl;
    }

private:
    double target_pos_;
    std::chrono::steady_clock::time_point start_time_;
};

// Custom StatefulActionNode for MoveTelescope
class MoveTelescopeNode : public BT::StatefulActionNode
{
public:
    MoveTelescopeNode(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config), start_time_(std::chrono::steady_clock::time_point())
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<int>("telescope_target_position_mm"),
            BT::InputPort<std::string>("control_mode"),
            BT::OutputPort<int>("current_telescope_position_mm"),
            BT::OutputPort<int>("error_code"),
            BT::OutputPort<std::string>("error_msg")
        };
    }

    BT::NodeStatus onStart() override
    {
        int error_code = 0;
        std::string error_msg = "Success";
        if (!getInput("telescope_target_position_mm", target_pos_))
        {
            error_code = 1;
            error_msg = "Missing telescope_target_position_mm";
            setOutput("error_code", error_code);
            setOutput("error_msg", error_msg);
            std::cout << "[Action] MoveTelescope failed: " << error_msg << std::endl;
            return BT::NodeStatus::FAILURE;
        }
        getInput("control_mode", mode_); // Optional
        start_time_ = std::chrono::steady_clock::now();
        std::cout << "[Action] MoveTelescope started to " << target_pos_ << " mm (" << mode_ << ")" << std::endl;
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time_);
        if (elapsed >= std::chrono::seconds(5))
        {
            setOutput("current_telescope_position_mm", target_pos_);
            setOutput("error_code", 0);
            setOutput("error_msg", "Success");
            current_positions["current_telescope_pos"] = std::to_string(target_pos_);
            std::cout << "[Action] MoveTelescope reached target: " << target_pos_ << " mm (" << mode_ << ")" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << "[Action] MoveTelescope halted" << std::endl;
    }

private:
    int target_pos_;
    std::string mode_ = "position";
    std::chrono::steady_clock::time_point start_time_;
};

// Custom StatefulActionNode for MoveTurntable
class MoveTurntableNode : public BT::StatefulActionNode
{
public:
    MoveTurntableNode(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config), start_time_(std::chrono::steady_clock::time_point())
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("turntable_target_position"),
            BT::OutputPort<std::string>("current_turntable_position"),
            BT::OutputPort<int>("error_code"),
            BT::OutputPort<std::string>("error_msg")
        };
    }

    BT::NodeStatus onStart() override
    {
        int error_code = 0;
        std::string error_msg = "Success";
        if (!getInput("turntable_target_position", target_pos_))
        {
            error_code = 1;
            error_msg = "Missing turntable_target_position";
            setOutput("error_code", error_code);
            setOutput("error_msg", error_msg);
            std::cout << "[Action] MoveTurntable failed: " << error_msg << std::endl;
            return BT::NodeStatus::FAILURE;
        }
        start_time_ = std::chrono::steady_clock::now();
        std::cout << "[Action] MoveTurntable started to " << target_pos_ << std::endl;
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time_);
        if (elapsed >= std::chrono::seconds(5))
        {
            setOutput("current_turntable_position", target_pos_);
            setOutput("error_code", 0);
            setOutput("error_msg", "Success");
            current_positions["current_turntable_pos"] = target_pos_;
            std::cout << "[Action] MoveTurntable reached target: " << target_pos_ << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << "[Action] MoveTurntable halted" << std::endl;
    }

private:
    std::string target_pos_;
    std::chrono::steady_clock::time_point start_time_;
};

// Custom StatefulActionNode for MoveForks
class MoveForksNode : public BT::StatefulActionNode
{
public:
    MoveForksNode(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config), start_time_(std::chrono::steady_clock::time_point())
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("fork_position_deg"),
            BT::OutputPort<double>("current_fork_position_deg"),
            BT::OutputPort<int>("error_code"),
            BT::OutputPort<std::string>("error_msg")
        };
    }

    BT::NodeStatus onStart() override
    {
        int error_code = 0;
        std::string error_msg = "Success";
        if (!getInput("fork_position_deg", fork_pos_))
        {
            error_code = 1;
            error_msg = "Missing fork_position_deg";
            setOutput("error_code", error_code);
            setOutput("error_msg", error_msg);
            std::cout << "[Action] MoveForks failed: " << error_msg << std::endl;
            return BT::NodeStatus::FAILURE;
        }
        start_time_ = std::chrono::steady_clock::now();
        std::cout << "[Action] MoveForks started to " << fork_pos_ << " deg" << std::endl;
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time_);
        if (elapsed >= std::chrono::seconds(5))
        {
            setOutput("current_fork_position_deg", fork_pos_);
            setOutput("error_code", 0);
            setOutput("error_msg", "Success");
            current_positions["current_fork_pos"] = std::to_string(fork_pos_);
            std::cout << "[Action] MoveForks reached target: " << fork_pos_ << " deg" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << "[Action] MoveForks halted" << std::endl;
    }

private:
    double fork_pos_ = 0.0;
    std::chrono::steady_clock::time_point start_time_;
};

int main()
{
    BT::BehaviorTreeFactory factory;

    // Dummy conditions that read from global state
    std::map<std::string, BT::NodeStatus> conditions;

    // Sensor names
    std::vector<std::string> sensors = {
        "BinClearSensors",
        "BinPresenceSensors",
        "TelescopeHomeSensors",
        "RackSensors",
        "ToteOutOfReach",
        "TurntablePosition",
        "LiftPosition",
        "ForkPosition",
        "TelescopePosition"
    };

    // Set all sensors to initial FAILURE
    for (const auto& s : sensors)
    {
        conditions[s] = BT::NodeStatus::FAILURE;
    }

    // Register conditions with typed ports
    factory.registerSimpleCondition("BinClearSensors", [&conditions](BT::TreeNode&) {
        return conditions.at("BinClearSensors");
    });

    factory.registerSimpleCondition("BinPresenceSensors", [&conditions](BT::TreeNode&) {
        return conditions.at("BinPresenceSensors");
    });

    factory.registerSimpleCondition("TelescopeHomeSensors", [&conditions](BT::TreeNode&) {
        return conditions.at("TelescopeHomeSensors");
    });

    factory.registerSimpleCondition("RackSensors", [&conditions](BT::TreeNode&) {
        return conditions.at("RackSensors");
    });

    factory.registerSimpleCondition("ToteOutOfReach", [&conditions](BT::TreeNode& node) {
        bool tote_out = false;
        node.setOutput("tote_out_of_reach", tote_out);
        std::cout << "[Condition] ToteOutOfReach setting tote_out_of_reach: " << tote_out << std::endl;
        return conditions.at("ToteOutOfReach");
    }, {BT::OutputPort<bool>("tote_out_of_reach")});

    factory.registerSimpleCondition("TurntablePosition", [&conditions](BT::TreeNode& node) {
        std::string target_pos;
        if (!node.getInput("turntable_position", target_pos)) {
            std::cout << "[Condition] TurntablePosition: Missing turntable_position" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
        auto current_pos = current_positions["current_turntable_pos"];
        bool is_equal = (current_pos == target_pos);
        std::cout << "[Condition] TurntablePosition checking: current=" << current_pos
                  << ", target=" << target_pos << ", equal=" << is_equal << std::endl;
        conditions["TurntablePosition"] = is_equal ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        return conditions["TurntablePosition"];
    }, {BT::InputPort<std::string>("turntable_position")});

    factory.registerSimpleCondition("LiftPosition", [&conditions](BT::TreeNode& node) {
        double target_pos;
        if (!node.getInput("lift_position_m", target_pos)) {
            std::string input;
            if (node.getInput<std::string>("lift_position_m", input)) {
                std::cout << "[Condition] LiftPosition: Invalid lift_position_m value: " << input << std::endl;
            } else {
                std::cout << "[Condition] LiftPosition: Missing lift_position_m" << std::endl;
            }
            return BT::NodeStatus::FAILURE;
        }
        double current_pos = std::stod(current_positions["current_lift_pos"]);
        bool is_equal = std::abs(current_pos - target_pos) < 0.0001; // Floating-point comparison
        std::cout << "[Condition] LiftPosition checking: current=" << current_pos
                  << ", target=" << target_pos << ", equal=" << is_equal << std::endl;
        conditions["LiftPosition"] = is_equal ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        return conditions["LiftPosition"];
    }, {BT::InputPort<double>("lift_position_m")});

    factory.registerSimpleCondition("ForkPosition", [&conditions](BT::TreeNode& node) {
        double target_pos;
        if (!node.getInput("fork_position_deg", target_pos)) {
            std::cout << "[Condition] ForkPosition: Missing fork_position_deg" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
        double current_pos = std::stod(current_positions["current_fork_pos"]);
        bool is_equal = std::abs(current_pos - target_pos) < 0.0001; // Floating-point comparison
        node.setOutput("fork_position_deg", current_pos);
        std::cout << "[Condition] ForkPosition checking: current=" << current_pos
                  << ", target=" << target_pos << ", equal=" << is_equal << std::endl;
        conditions["ForkPosition"] = is_equal ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        return conditions["ForkPosition"];
    }, {BT::InputPort<double>("fork_position_deg"), BT::OutputPort<double>("fork_position_deg")});

    factory.registerSimpleCondition("TelescopePosition", [&conditions](BT::TreeNode& node) {
        int target_pos;
        if (!node.getInput("telescope_position_mm", target_pos)) {
            std::cout << "[Condition] TelescopePosition: Missing telescope_position_mm" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
        int current_pos = std::stoi(current_positions["current_telescope_pos"]);
        bool is_equal = (current_pos == target_pos);
        std::cout << "[Condition] TelescopePosition checking: current=" << current_pos
                  << ", target=" << target_pos << ", equal=" << is_equal << std::endl;
        conditions["TelescopePosition"] = is_equal ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        return conditions["TelescopePosition"];
    }, {BT::InputPort<int>("telescope_position_mm")});

    // Register custom action nodes
    factory.registerNodeType<MoveLiftNode>("MoveLift");
    factory.registerNodeType<MoveTelescopeNode>("MoveTelescope");
    factory.registerNodeType<MoveTurntableNode>("MoveTurntable");
    factory.registerNodeType<MoveForksNode>("MoveForks");

    // Create blackboard explicitly for v3.x
    auto blackboard = BT::Blackboard::create();

    // Initialize blackboard with valid numeric values
    blackboard->set("target_lift_pos", 0.5); // Default lift position (meters)
    blackboard->set("target_telescope_pos", 0); // Default telescope position (mm)
    blackboard->set("target_fork_pos", 0.0); // Default fork position (deg)
    blackboard->set("turntable_dir", std::string("home"));
    blackboard->set("telescope_mode", std::string("position"));
    blackboard->set("deep", std::string("1"));
    blackboard->set("tote_out_of_reach", false);
    blackboard->set("picked", false);

    // Load your tree
    BT::Tree tree = factory.createTreeFromFile("./../veloce_lift_new.xml", blackboard);
    BT::Groot2Publisher publisher(tree); // Use v3.x ZMQ publisher

    // Blackboard keys you want to edit
    std::vector<BBValue> editable = {
        {"deep", "string", "1"},
        {"turntable_dir", "string", "home"},
        {"tote_out_of_reach", "bool", "false"},
        {"picked", "bool", "false"},
        {"target_lift_pos", "double", "0.5"},
        {"target_telescope_pos", "int", "0"},
        {"target_fork_pos", "double", "0.0"},
        {"telescope_mode", "string", "position"}
    };

    // Init ncurses
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    keypad(stdscr, TRUE);

    int selected = 0;
    bool show_tick_message = false;

    while (true)
    {
        if (!show_tick_message) {
            clear();
        }

        mvprintw(0, 2, "BehaviorTree TUI Blackboard Editor - ↑↓ to navigate, Enter to toggle/edit, F5 to tick, q to quit");

        int row = 2;
        for (size_t i = 0; i < sensors.size(); ++i)
        {
            bool active = (selected == (int)i);
            attron(active ? A_REVERSE : A_NORMAL);
            auto status = conditions[sensors[i]] == BT::NodeStatus::SUCCESS ? "[o]" : "[ ]";
            mvprintw(row++, 2, "Sensor %s %s", status, sensors[i].c_str());
            attroff(active ? A_REVERSE : A_NORMAL);
        }

        for (size_t i = 0; i < editable.size(); ++i)
        {
            int j = i + sensors.size();
            bool active = (selected == j);
            attron(active ? A_REVERSE : A_NORMAL);
            mvprintw(row++, 2, "BB %-20s = %s", editable[i].key.c_str(), editable[i].value.c_str());
            attroff(active ? A_REVERSE : A_NORMAL);
        }

        if (show_tick_message) {
            mvprintw(row + 1, 2, "Ticked MainTree!");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            show_tick_message = false;
        }

        refresh();

        int ch = getch();

        if (ch == 'q') break;
        else if (ch == KEY_DOWN && selected < (int)(sensors.size() + editable.size()) - 1) selected++;
        else if (ch == KEY_UP && selected > 0) selected--;
        else if (ch == 10 || ch == KEY_ENTER)
        {
            if (selected < (int)sensors.size())
            {
                auto& s = sensors[selected];
                conditions[s] = (conditions[s] == BT::NodeStatus::SUCCESS) ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
            }
            else
            {
                echo();
                curs_set(1);
                int i = selected - sensors.size();
                mvprintw(row + 1, 2, "Enter new value for %s: ", editable[i].key.c_str());
                char input[64];
                getstr(input);
                // Validate numeric inputs
                if (editable[i].type == "double") {
                    try {
                        std::stod(input);
                        editable[i].value = input;
                    } catch (const std::exception& e) {
                        mvprintw(row + 2, 2, "Invalid double value: %s", input);
                        refresh();
                        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                        continue;
                    }
                } else if (editable[i].type == "int") {
                    try {
                        std::stoi(input);
                        editable[i].value = input;
                    } catch (const std::exception& e) {
                        mvprintw(row + 2, 2, "Invalid int value: %s", input);
                        refresh();
                        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                        continue;
                    }
                } else {
                    editable[i].value = input;
                }
                noecho();
                curs_set(0);
            }
        }
        else if (ch == KEY_F(5))
        {
            for (const auto& val : editable)
            {
                try {
                    if (val.type == "string")
                        blackboard->set(val.key, val.value);
                    else if (val.type == "bool")
                        blackboard->set(val.key, (val.value == "true" || val.value == "1"));
                    else if (val.type == "double")
                        blackboard->set(val.key, std::stod(val.value));
                    else if (val.type == "int")
                        blackboard->set(val.key, std::stoi(val.value));
                } catch (const std::exception& e) {
                    std::cout << "Error setting blackboard key " << val.key << ": " << e.what() << std::endl;
                }
            }

            tree.tickOnce();
            show_tick_message = true;
        }
    }

    endwin();
    return 0;
}