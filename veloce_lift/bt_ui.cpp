#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <ncurses.h>
#include <thread>
#include <chrono>
#include <map>
#include <string>

struct BBValue {
    std::string key;
    std::string type; // "bool", "string", "double", or "int"
    std::string value;
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
    }, {});

    factory.registerSimpleCondition("BinPresenceSensors", [&conditions](BT::TreeNode&) {
        return conditions.at("BinPresenceSensors");
    }, {});

    factory.registerSimpleCondition("TelescopeHomeSensors", [&conditions](BT::TreeNode&) {
        return conditions.at("TelescopeHomeSensors");
    }, {});

    factory.registerSimpleCondition("RackSensors", [&conditions](BT::TreeNode&) {
        return conditions.at("RackSensors");
    }, {});

    factory.registerSimpleCondition("ToteOutOfReach", [&conditions](BT::TreeNode& node) {
        bool tote_out = false;
        node.setOutput("tote_out_of_reach", tote_out);
        std::cout << "[Condition] ToteOutOfReach setting tote_out_of_reach: " << tote_out << std::endl;
        return conditions.at("ToteOutOfReach");
    }, {BT::OutputPort<bool>("tote_out_of_reach")});

    factory.registerSimpleCondition("TurntablePosition", [&conditions](BT::TreeNode& node) {
        std::string turn_pos;
        if (!node.getInput("turntable_position", turn_pos)) {
            std::cout << "[Condition] TurntablePosition: Missing turntable_position" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
        std::cout << "[Condition] TurntablePosition checking: " << turn_pos << std::endl;
        return conditions.at("TurntablePosition");
    }, {BT::InputPort<std::string>("turntable_position")});

    factory.registerSimpleCondition("LiftPosition", [&conditions](BT::TreeNode& node) {
        double lift_pos;
        if (!node.getInput("lift_position_m", lift_pos)) {
            std::cout << "[Condition] LiftPosition: Missing lift_position_m" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
        std::cout << "[Condition] LiftPosition checking: " << lift_pos << " m" << std::endl;
        return conditions.at("LiftPosition");
    }, {BT::InputPort<double>("lift_position_m")});

    factory.registerSimpleCondition("ForkPosition", [&conditions](BT::TreeNode& node) {
        double fork_pos = 0.0; // Default value
        node.setOutput("fork_position_deg", fork_pos);
        std::cout << "[Condition] ForkPosition setting fork_position_deg: " << fork_pos << std::endl;
        return conditions.at("ForkPosition");
    }, {BT::OutputPort<double>("fork_position_deg")});

    factory.registerSimpleCondition("TelescopePosition", [&conditions](BT::TreeNode& node) {
        int telescope_pos;
        if (!node.getInput("telescope_position_mm", telescope_pos)) {
            std::cout << "[Condition] TelescopePosition: Missing telescope_position_mm" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
        std::cout << "[Condition] TelescopePosition checking: " << telescope_pos << " mm" << std::endl;
        return conditions.at("TelescopePosition");
    }, {BT::InputPort<int>("telescope_position_mm")});

    // Register actions with typed ports and error outputs
    factory.registerSimpleAction("MoveLift", [](BT::TreeNode& node) {
        double target_pos;
        int error_code = 0;
        std::string error_msg = "Success";
        if (!node.getInput("lift_target_position_m", target_pos)) {
            error_code = 1;
            error_msg = "Missing lift_target_position_m";
            node.setOutput("error_code", error_code);
            node.setOutput("error_msg", error_msg);
            std::cout << "[Action] MoveLift failed: " << error_msg << std::endl;
            return BT::NodeStatus::FAILURE;
        }
        node.setOutput("current_lift_position_m", target_pos); // Feedback
        node.setOutput("error_code", error_code);
        node.setOutput("error_msg", error_msg);
        std::cout << "[Action] MoveLift to " << target_pos << " m" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }, {BT::InputPort<double>("lift_target_position_m"),
        BT::OutputPort<double>("current_lift_position_m"),
        BT::OutputPort<int>("error_code"),
        BT::OutputPort<std::string>("error_msg")});

    factory.registerSimpleAction("MoveTelescope", [](BT::TreeNode& node) {
        int target_pos;
        std::string mode = "position"; // Default
        int error_code = 0;
        std::string error_msg = "Success";
        if (!node.getInput("telescope_target_position_mm", target_pos)) {
            error_code = 1;
            error_msg = "Missing telescope_target_position_mm";
            node.setOutput("error_code", error_code);
            node.setOutput("error_msg", error_msg);
            std::cout << "[Action] MoveTelescope failed: " << error_msg << std::endl;
            return BT::NodeStatus::FAILURE;
        }
        node.getInput("control_mode", mode); // Optional
        node.setOutput("current_telescope_position_mm", target_pos); // Feedback
        node.setOutput("error_code", error_code);
        node.setOutput("error_msg", error_msg);
        std::cout << "[Action] MoveTelescope to " << target_pos << " mm (" << mode << ")" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }, {BT::InputPort<int>("telescope_target_position_mm"),
        BT::InputPort<std::string>("control_mode", "position"),
        BT::OutputPort<int>("current_telescope_position_mm"),
        BT::OutputPort<int>("error_code"),
        BT::OutputPort<std::string>("error_msg")});

    factory.registerSimpleAction("MoveTurntable", [](BT::TreeNode& node) {
        std::string target_pos;
        int error_code = 0;
        std::string error_msg = "Success";
        if (!node.getInput("turntable_target_position", target_pos)) {
            error_code = 1;
            error_msg = "Missing turntable_target_position";
            node.setOutput("error_code", error_code);
            node.setOutput("error_msg", error_msg);
            std::cout << "[Action] MoveTurntable failed: " << error_msg << std::endl;
            return BT::NodeStatus::FAILURE;
        }
        node.setOutput("current_turntable_position", target_pos); // Feedback
        node.setOutput("error_code", error_code);
        node.setOutput("error_msg", error_msg);
        std::cout << "[Action] MoveTurntable to " << target_pos << std::endl;
        return BT::NodeStatus::SUCCESS;
    }, {BT::InputPort<std::string>("turntable_target_position"),
        BT::OutputPort<std::string>("current_turntable_position"),
        BT::OutputPort<int>("error_code"),
        BT::OutputPort<std::string>("error_msg")});

    factory.registerSimpleAction("MoveForks", [](BT::TreeNode& node) {
        double fork_pos = 0.0; // Default value
        int error_code = 0;
        std::string error_msg = "Success";
        if (!node.getInput("fork_position_deg", fork_pos)) {
            error_code = 1;
            error_msg = "Missing fork_position_deg";
            node.setOutput("error_code", error_code);
            node.setOutput("error_msg", error_msg);
            std::cout << "[Action] MoveForks failed: " << error_msg << std::endl;
            return BT::NodeStatus::FAILURE;
        }
        node.setOutput("current_fork_position_deg", fork_pos); // Feedback
        node.setOutput("error_code", error_code);
        node.setOutput("error_msg", error_msg);
        std::cout << "[Action] MoveForks to " << fork_pos << " deg" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }, {BT::InputPort<double>("fork_position_deg", 0.0),
        BT::OutputPort<double>("current_fork_position_deg"),
        BT::OutputPort<int>("error_code"),
        BT::OutputPort<std::string>("error_msg")});

    // Load your tree (blackboard managed internally in v4.x)
    BT::Tree tree = factory.createTreeFromFile("/workspace/behavior_tree.xml");
    BT::Groot2Publisher publisher(tree);

    // Blackboard keys you want to edit
    std::vector<BBValue> editable = {
        {"deep", "string", "1"},
        {"turntable_dir", "string", "1"},
        {"tote_out_of_reach", "bool", "false"},
        {"picked", "bool", "false"}
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
            auto status = conditions[sensors[i]] == BT::NodeStatus::SUCCESS ? "[✓]" : "[ ]";
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
                editable[i].value = input;
                noecho();
                curs_set(0);
            }
        }
        else if (ch == KEY_F(5))
        {
            auto blackboard = tree.blackboard();
            for (const auto& val : editable)
            {
                if (val.type == "string")
                    blackboard->set(val.key, val.value);
                else if (val.type == "bool")
                    blackboard->set(val.key, (val.value == "true" || val.value == "1"));
                else if (val.type == "double")
                    blackboard->set(val.key, std::stod(val.value));
                else if (val.type == "int")
                    blackboard->set(val.key, std::stoi(val.value));
            }

            tree.tickOnce();
            show_tick_message = true;
        }
    }

    endwin();
    return 0;
}