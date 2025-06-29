#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <ncurses.h>
#include <thread>
#include <chrono>
#include <map>
#include <string>
#include <functional>
#include <yaml-cpp/yaml.h>
#include <stdexcept>

using namespace BT;

// Structure to hold YAML operation settings
struct TelescopeConfig {
    int depth;
    int length_mm;
    double fork_angle_deg;
};

struct TurntableConfig {
    std::string direction;
    double lift_position_mm;
    double angle_deg;
    std::vector<TelescopeConfig> telescope;
};

struct LiftLevelConfig {
    int lift_level;
    std::vector<TurntableConfig> turntable;
};

struct LiftControlConfig {
    std::vector<LiftLevelConfig> operation_settings;

    bool getParams(int lift_level, const std::string& turntable_dir, int depth,
                   double& lift_position_m, std::string& turntable_pos, int& telescope_pos_mm, double& fork_pos_deg) const {
        for (const auto& level : operation_settings) {
            if (level.lift_level == lift_level) {
                for (const auto& tt : level.turntable) {
                    if (tt.direction == turntable_dir) {
                        for (const auto& tel : tt.telescope) {
                            if (tel.depth == depth) {
                                lift_position_m = tt.lift_position_mm / 1000.0;
                                turntable_pos = std::to_string(tt.angle_deg);
                                telescope_pos_mm = tel.length_mm;
                                fork_pos_deg = tel.fork_angle_deg;
                                return true;
                            }
                        }
                    }
                }
            }
        }
        return false;
    }
};

LiftControlConfig loadYaml(const std::string& filename) {
    LiftControlConfig config;
    try {
        YAML::Node yaml = YAML::LoadFile(filename);
        const auto& op_settings = yaml["lift_control"]["operation_settings"];
        for (const auto& level : op_settings) {
            LiftLevelConfig level_config;
            level_config.lift_level = level["lift_level"].as<int>();
            for (const auto& tt : level["turntable"]) {
                TurntableConfig tt_config;
                tt_config.direction = tt["direction"].as<std::string>();
                tt_config.lift_position_mm = tt["lift_position_mm"].as<double>();
                tt_config.angle_deg = tt["angle_deg"].as<double>();
                for (const auto& tel : tt["telescope"]) {
                    TelescopeConfig tel_config;
                    tel_config.depth = tel["depth"].as<int>();
                    tel_config.length_mm = tel["length_mm"].as<int>();
                    tel_config.fork_angle_deg = tel["fork_angle_deg"].as<double>();
                    tt_config.telescope.push_back(tel_config);
                }
                level_config.turntable.push_back(tt_config);
            }
            config.operation_settings.push_back(level_config);
        }
    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading YAML: " << e.what() << std::endl;
        throw;
    }
    return config;
}

struct BBValue {
    std::string key;
    std::string type;
    std::string value;
};

std::map<std::string, std::string> current_positions = {
    {"current_lift_pos", "0.0"},
    {"current_telescope_pos", "0"},
    {"current_fork_pos", "0.0"},
    {"current_turntable_pos", "0"}
};

std::map<std::string, NodeStatus> conditions;

NodeStatus CheckBinClearSensors() {
    if (conditions.at("BinClearSensors") == NodeStatus::FAILURE) {
        std::cout << "[Condition] BinClearSensors: Failure" << std::endl;
    }
    else if (conditions.at("BinClearSensors") == NodeStatus::SUCCESS) {
        std::cout << "[Condition] BinClearSensors: Success" << std::endl;
    }    
    return conditions.at("BinClearSensors");
}

NodeStatus CheckBinPresenceSensors() {
    if (conditions.at("BinPresenceSensors") == NodeStatus::FAILURE) {
        std::cout << "[Condition] BinPresenceSensors: Failure" << std::endl;
    }
    else if (conditions.at("BinPresenceSensors") == NodeStatus::SUCCESS) {
        std::cout << "[Condition] BinPresenceSensors: Success" << std::endl;
    }
    return conditions.at("BinPresenceSensors");
}

NodeStatus CheckTelescopeHomeSensors() {
    if (conditions.at("TelescopeHomeSensors") == NodeStatus::FAILURE) {
        std::cout << "[Condition] TelescopeHomeSensors: Failure" << std::endl;
    }
    else if (conditions.at("TelescopeHomeSensors") == NodeStatus::SUCCESS) {
        std::cout << "[Condition] TelescopeHomeSensors: Success" << std::endl;
    }
    return conditions.at("TelescopeHomeSensors");
}

NodeStatus CheckRackSensors() {
    if (conditions.at("RackSensors") == NodeStatus::FAILURE) {
        std::cout << "[Condition] RackSensors: Failure" << std::endl;
    }
    else if (conditions.at("RackSensors") == NodeStatus::SUCCESS) {
        std::cout << "[Condition] RackSensors: Success" << std::endl;
    }
    return conditions.at("RackSensors");
}

NodeStatus CheckToteOutOfReach(TreeNode& node) {
    bool tote_out = false;
    node.setOutput("tote_out_of_reach", tote_out);
    std::cout << "[Condition] ToteOutOfReach setting tote_out_of_reach: " << tote_out << std::endl;
    return conditions.at("ToteOutOfReach");
}

NodeStatus CheckTurntablePosition(TreeNode& node) {
    std::string target_pos;
    if (!node.getInput("turntable_position", target_pos)) {
        std::cout << "[Condition] TurntablePosition: Missing turntable_position" << std::endl;
        return NodeStatus::FAILURE;
    }
    auto current_pos = current_positions["current_turntable_pos"];
    bool is_equal = (current_pos == target_pos);
    std::cout << "[Condition] TurntablePosition checking: current=" << current_pos
              << ", target=" << target_pos << ", equal=" << is_equal << std::endl;
    conditions["TurntablePosition"] = is_equal ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    return conditions["TurntablePosition"];
}

NodeStatus CheckLiftPosition(TreeNode& node) {
    double target_pos;
    if (!node.getInput("lift_position_m", target_pos)) {
        std::cout << "[Condition] LiftPosition: Missing lift_position_m" << std::endl;
        return NodeStatus::FAILURE;
    }
    double current_pos;
    try {
        current_pos = std::stod(current_positions["current_lift_pos"]);
    } catch (const std::exception& e) {
        std::cout << "[Condition] LiftPosition: Invalid current_lift_pos: " << current_positions["current_lift_pos"] << std::endl;
        return NodeStatus::FAILURE;
    }
    bool is_equal = std::abs(current_pos - target_pos) < 0.0001;
    std::cout << "[Condition] LiftPosition checking: current=" << current_pos
              << ", target=" << target_pos << ", equal=" << is_equal << std::endl;
    conditions["LiftPosition"] = is_equal ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    return conditions["LiftPosition"];
}

NodeStatus CheckForkPosition(TreeNode& node) {
    double target_pos;
    if (!node.getInput("fork_position_deg", target_pos)) {
        std::cout << "[Condition] ForkPosition: Missing fork_position_deg" << std::endl;
        return NodeStatus::FAILURE;
    }
    double current_pos;
    try {
        current_pos = std::stod(current_positions["current_fork_pos"]);
    } catch (const std::exception& e) {
        std::cout << "[Condition] ForkPosition: Invalid current_fork_pos: " << current_positions["current_fork_pos"] << std::endl;
        return NodeStatus::FAILURE;
    }
    bool is_equal = std::abs(current_pos - target_pos) < 0.0001;
    node.setOutput("fork_position_deg", current_pos);
    std::cout << "[Condition] ForkPosition checking: current=" << current_pos
              << ", target=" << target_pos << ", equal=" << is_equal << std::endl;
    conditions["ForkPosition"] = is_equal ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    return conditions["ForkPosition"];
}

NodeStatus CheckTelescopePosition(TreeNode& node) {
    int target_pos;
    if (!node.getInput("telescope_position_mm", target_pos)) {
        std::cout << "[Condition] TelescopePosition: Missing telescope_position_mm" << std::endl;
        return NodeStatus::FAILURE;
    }
    int current_pos;
    try {
        current_pos = std::stoi(current_positions["current_telescope_pos"]);
    } catch (const std::exception& e) {
        std::cout << "[Condition] TelescopePosition: Invalid current_telescope_pos: " << current_positions["current_telescope_pos"] << std::endl;
        return NodeStatus::FAILURE;
    }
    bool is_equal = (current_pos == target_pos);
    std::cout << "[Condition] TelescopePosition checking: current=" << current_pos
              << ", target=" << target_pos << ", equal=" << is_equal << std::endl;
    conditions["TelescopePosition"] = is_equal ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    return conditions["TelescopePosition"];
}

class MoveLiftNode : public StatefulActionNode {
public:
    MoveLiftNode(const std::string& name, const NodeConfig& config)
        : StatefulActionNode(name, config), start_time_(std::chrono::steady_clock::now()) {}

    static PortsList providedPorts() {
        return {
            InputPort<double>("lift_target_position_m"),
            OutputPort<double>("current_lift_position_m"),
            OutputPort<int>("error_code"),
            OutputPort<std::string>("error_msg")
        };
    }

    NodeStatus onStart() override {
        int error_code = 0;
        std::string error_msg = "Success";
        if (!getInput("lift_target_position_m", lift_target_pos_)) {
            error_code = 1;
            error_msg = "Missing lift_target_position_m";
            setOutput("error_code", error_code);
            setOutput("error_msg", error_msg);
            std::cout << "[Action] MoveLift failed: " << error_msg << std::endl;
            return NodeStatus::FAILURE;
        }
        start_time_ = std::chrono::steady_clock::now();
        std::cout << "[Action] MoveLift started to " << lift_target_pos_ << " m" << std::endl;
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time_);
        if (elapsed >= std::chrono::seconds(3)) {
            setOutput("current_lift_position_m", lift_target_pos_);
            setOutput("error_code", 0);
            setOutput("error_msg", "Success");
            current_positions["current_lift_pos"] = std::to_string(lift_target_pos_);
            std::cout << "[Action] MoveLift reached target: " << lift_target_pos_ << " m" << std::endl;
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::RUNNING;
    }

    void onHalted() override {
        std::cout << "[Action] MoveLift halted" << std::endl;
    }

private:
    double lift_target_pos_;
    std::chrono::steady_clock::time_point start_time_;
};

class MoveTelescopeNode : public StatefulActionNode {
public:
    MoveTelescopeNode(const std::string& name, const NodeConfig& config)
        : StatefulActionNode(name, config), start_time_(std::chrono::steady_clock::now()) {}

    static PortsList providedPorts() {
        return {
            InputPort<int>("telescope_target_position_mm"),
            InputPort<std::string>("control_mode"),
            OutputPort<int>("current_telescope_position_mm"),
            OutputPort<int>("error_code"),
            OutputPort<std::string>("error_msg")
        };
    }

    NodeStatus onStart() override {
        int error_code = 0;
        std::string error_msg = "Success";
        if (!getInput("telescope_target_position_mm", telescope_target_pos_)) {
            error_code = 1;
            error_msg = "Missing telescope_target_position_mm";
            setOutput("error_code", error_code);
            setOutput("error_msg", error_msg);
            std::cout << "[Action] MoveTelescope failed: " << error_msg << std::endl;
            return NodeStatus::FAILURE;
        }
        getInput("control_mode", mode_);
        start_time_ = std::chrono::steady_clock::now();
        std::cout << "[Action] MoveTelescope started to " << telescope_target_pos_ << " mm (" << mode_ << ")" << std::endl;
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time_);
        if (elapsed >= std::chrono::seconds(3)) {
            setOutput("current_telescope_position_mm", telescope_target_pos_);
            setOutput("error_code", 0);
            setOutput("error_msg", "Success");
            current_positions["current_telescope_pos"] = std::to_string(telescope_target_pos_);
            std::cout << "[Action] MoveTelescope reached target: " << telescope_target_pos_ << " mm (" << mode_ << ")" << std::endl;
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::RUNNING;
    }

    void onHalted() override {
        std::cout << "[Action] MoveTelescope halted" << std::endl;
    }

private:
    int telescope_target_pos_;
    std::string mode_ = "position";
    std::chrono::steady_clock::time_point start_time_;
};

class MoveTurntableNode : public StatefulActionNode {
public:
    MoveTurntableNode(const std::string& name, const NodeConfig& config)
        : StatefulActionNode(name, config), start_time_(std::chrono::steady_clock::now()) {}

    static PortsList providedPorts() {
        return {
            InputPort<std::string>("turntable_target_position"),
            OutputPort<std::string>("current_turntable_position"),
            OutputPort<int>("error_code"),
            OutputPort<std::string>("error_msg")
        };
    }

    NodeStatus onStart() override {
        int error_code = 0;
        std::string error_msg = "Success";
        if (!getInput("turntable_target_position", turntable_target_pos_)) {
            error_code = 1;
            error_msg = "Missing turntable_target_position";
            setOutput("error_code", error_code);
            setOutput("error_msg", error_msg);
            std::cout << "[Action] MoveTurntable failed: " << error_msg << std::endl;
            return NodeStatus::FAILURE;
        }
        start_time_ = std::chrono::steady_clock::now();
        std::cout << "[Action] MoveTurntable started to " << turntable_target_pos_ << std::endl;
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time_);
        if (elapsed >= std::chrono::seconds(3)) {
            setOutput("current_turntable_position", turntable_target_pos_);
            setOutput("error_code", 0);
            setOutput("error_msg", "Success");
            current_positions["current_turntable_pos"] = turntable_target_pos_;
            std::cout << "[Action] MoveTurntable reached target: " << turntable_target_pos_ << std::endl;
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::RUNNING;
    }

    void onHalted() override {
        std::cout << "[Action] MoveTurntable halted" << std::endl;
    }

private:
    std::string turntable_target_pos_;
    std::chrono::steady_clock::time_point start_time_;
};

class MoveForksNode : public StatefulActionNode {
public:
    MoveForksNode(const std::string& name, const NodeConfig& config)
        : StatefulActionNode(name, config), start_time_(std::chrono::steady_clock::now()) {}

    static PortsList providedPorts() {
        return {
            InputPort<double>("fork_position_deg"),
            OutputPort<double>("current_fork_position_deg"),
            OutputPort<int>("error_code"),
            OutputPort<std::string>("error_msg")
        };
    }

    NodeStatus onStart() override {
        int error_code = 0;
        std::string error_msg = "Success";
        if (!getInput("fork_position_deg", fork_target_pos_)) {
            error_code = 1;
            error_msg = "Missing fork_position_deg";
            setOutput("error_code", error_code);
            setOutput("error_msg", error_msg);
            std::cout << "[Action] MoveForks failed: " << error_msg << std::endl;
            return NodeStatus::FAILURE;
        }
        start_time_ = std::chrono::steady_clock::now();
        std::cout << "[Action] MoveForks started to " << fork_target_pos_ << " deg" << std::endl;
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time_);
        if (elapsed >= std::chrono::seconds(3)) {
            setOutput("current_fork_position_deg", fork_target_pos_);
            setOutput("error_code", 0);
            setOutput("error_msg", "Success");
            current_positions["current_fork_pos"] = std::to_string(fork_target_pos_);
            std::cout << "[Action] MoveForks reached target: " << fork_target_pos_ << " deg" << std::endl;
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::RUNNING;
    }

    void onHalted() override {
        std::cout << "[Action] MoveForks halted" << std::endl;
    }

private:
    double fork_target_pos_ = 0.0;
    std::chrono::steady_clock::time_point start_time_;
};

int main() {
    LiftControlConfig config;
    try {
        config = loadYaml("./../lift_positions.yaml");
    } catch (const std::exception& e) {
        std::cerr << "Failed to load YAML: " << e.what() << std::endl;
        return 1;
    }

    BehaviorTreeFactory factory;

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

    for (const auto& s : sensors) {
        conditions[s] = NodeStatus::FAILURE;
    }

    factory.registerSimpleCondition("BinClearSensors", std::bind(CheckBinClearSensors));
    factory.registerSimpleCondition("BinPresenceSensors", std::bind(CheckBinPresenceSensors));
    factory.registerSimpleCondition("TelescopeHomeSensors", std::bind(CheckTelescopeHomeSensors));
    factory.registerSimpleCondition("RackSensors", std::bind(CheckRackSensors));
    factory.registerSimpleCondition("ToteOutOfReach", std::bind(CheckToteOutOfReach, std::placeholders::_1),
                                   {OutputPort<bool>("tote_out_of_reach")});
    factory.registerSimpleCondition("TurntablePosition", std::bind(CheckTurntablePosition, std::placeholders::_1),
                                   {InputPort<std::string>("turntable_position")});
    factory.registerSimpleCondition("LiftPosition", std::bind(CheckLiftPosition, std::placeholders::_1),
                                   {InputPort<double>("lift_position_m")});
    factory.registerSimpleCondition("ForkPosition", std::bind(CheckForkPosition, std::placeholders::_1),
                                   {InputPort<double>("fork_position_deg"), OutputPort<double>("fork_position_deg")});
    factory.registerSimpleCondition("TelescopePosition", std::bind(CheckTelescopePosition, std::placeholders::_1),
                                   {InputPort<int>("telescope_position_mm")});

    factory.registerNodeType<MoveLiftNode>("MoveLift");
    factory.registerNodeType<MoveTelescopeNode>("MoveTelescope");
    factory.registerNodeType<MoveTurntableNode>("MoveTurntable");
    factory.registerNodeType<MoveForksNode>("MoveForks");

    auto blackboard = Blackboard::create();

    blackboard->set("lift_level", 1);
    blackboard->set("depth", 1);
    blackboard->set("turntable_dir", std::string("Middle"));
    blackboard->set("telescope_mode", std::string("position"));
    blackboard->set("tote_out_of_reach", false);
    blackboard->set("picked", false);
    double lift_pos_m;
    std::string turntable_pos;
    int telescope_pos_mm;
    double fork_pos_deg;
    if (config.getParams(1, "Middle", 1, lift_pos_m, turntable_pos, telescope_pos_mm, fork_pos_deg)) {
        blackboard->set("target_lift_pos", lift_pos_m);
        blackboard->set("target_turntable_pos", turntable_pos);
        blackboard->set("target_telescope_pos", telescope_pos_mm);
        blackboard->set("target_fork_pos", fork_pos_deg);
    } else {
        std::cerr << "Failed to initialize blackboard from YAML" << std::endl;
        return 1;
    }

    Tree tree = factory.createTreeFromFile("./../veloce_lift_new.xml", blackboard);
    Groot2Publisher publisher(tree);

    std::vector<BBValue> editable = {
        {"lift_level", "int", "1"},
        {"depth", "int", "1"},
        {"turntable_dir", "string", "Middle"},
        {"tote_out_of_reach", "bool", "false"},
        {"picked", "bool", "false"},
        {"target_lift_pos", "double", std::to_string(lift_pos_m)},
        {"target_telescope_pos", "int", std::to_string(telescope_pos_mm)},
        {"target_fork_pos", "double", std::to_string(fork_pos_deg)},
        {"telescope_mode", "string", "position"}
    };

    initscr();
    // start_color();
    // use_default_colors();
    noecho();
    cbreak();
    curs_set(0);
    keypad(stdscr, TRUE);
    timeout(100);

    int selected = 0;
    bool show_tick_message = false;

    while (true) {
        if (!show_tick_message) {
            clear();
        }

        mvprintw(0, 2, "BehaviorTree Blackboard Editor - ↑↓ to navigate, Enter to toggle/type, F5 to tick, q to quit");

        int row = 2;
        for (size_t i = 0; i < sensors.size(); ++i) {
            bool active = (selected == (int)i);
            attron(active ? A_REVERSE : A_NORMAL);
            auto status = conditions[sensors[i]] == NodeStatus::SUCCESS ? "[o]" : "[ ]";
            mvprintw(row++, 2, "Sensor %-20s %s", sensors[i].c_str(), status);
            attroff(active ? A_REVERSE : A_NORMAL);
        }

        for (size_t i = 0; i < editable.size(); ++i) {
            int j = i + sensors.size();
            bool active = (selected == j);
            attron(active ? A_REVERSE : A_NORMAL);
            mvprintw(row++, 0, "BB %-20s = %s", editable[i].key.c_str(), editable[i].value.c_str());
            attroff(active ? A_REVERSE : A_NORMAL);
        }

        if (show_tick_message) {
            mvprintw(row + 1, 0, "Ticked MainTree!");
            refresh();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            show_tick_message = false;
        }

        refresh();

        int ch = getch();
        if (ch == ERR) continue;

        if (ch == 'q') break;
        else if (ch == KEY_DOWN && selected < (int)(sensors.size() + editable.size()) - 1) selected++;
        else if (ch == KEY_UP && selected > 0) selected--;
        else if (ch == 10 || ch == KEY_ENTER) {
            if (selected < (int)sensors.size()) {
                auto& s = sensors[selected];
                conditions[s] = (conditions[s] == NodeStatus::SUCCESS) ? NodeStatus::FAILURE : NodeStatus::SUCCESS;
            } else {
                echo();
                curs_set(1);
                int i = selected - sensors.size();
                mvprintw(row + 1, 0, "Enter new value for %s: ", editable[i].key.c_str());
                char input[64] = {0};
                getnstr(input, sizeof(input) - 1);
                std::string input_str = input;
                if (input_str.empty()) {
                    mvprintw(row + 2, 1, "Error: Input cannot be empty");
                    refresh();
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    noecho();
                    curs_set(0);
                    continue;
                }
                if (editable[i].type == "double") {
                    try {
                        std::stod(input_str);
                        editable[i].value = input_str;
                    } catch (const std::exception& e) {
                        mvprintw(row + 2, 1, "Invalid double value: %s", input_str.c_str());
                        refresh();
                        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    }
                } else if (editable[i].type == "int") {
                    try {
                        int val = std::stoi(input_str);
                        if (editable[i].key == "lift_level" && (val < 1 || val > 8)) {
                            mvprintw(row + 2, 1, "Invalid lift_level: %s (use 1-8)", input_str.c_str());
                            refresh();
                            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                        } else if (editable[i].key == "depth" && (val < 1 || val > 2)) {
                            mvprintw(row + 2, 1, "Invalid depth: %s (use 1-2)", input_str.c_str());
                            refresh();
                            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                        } else {
                            editable[i].value = input_str;
                        }
                    } catch (const std::exception& e) {
                        mvprintw(row + 2, 1, "Invalid int value: %s", input_str.c_str());
                        refresh();
                        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    }
                } else if (editable[i].key == "turntable_dir") {
                    if (input_str != "Left" && input_str != "Middle" && input_str != "Right") {
                        mvprintw(row + 2, 1, "Invalid turntable_dir: %s (use 'Left', 'Middle', 'Right')", input_str.c_str());
                        refresh();
                        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    } else {
                        editable[i].value = input_str;
                    }
                } else if (editable[i].type == "bool") {
                    if (input_str != "true" && input_str != "false" && input_str != "1" && input_str != "0") {
                        mvprintw(row + 2, 1, "Invalid bool value: %s (use 'true', 'false', '1', '0')", input_str.c_str());
                        refresh();
                        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    } else {
                        editable[i].value = input_str;
                    }
                } else {
                    editable[i].value = input_str;
                }
                noecho();
                curs_set(0);
            }
        } else if (ch == KEY_F(5)) {
            endwin();
            int lift_level;
            int depth;
            std::string turntable_dir;
            if (blackboard->get("lift_level", lift_level) && blackboard->get("depth", depth) &&
                blackboard->get("turntable_dir", turntable_dir)) {
                double lift_pos_m;
                std::string turntable_pos;
                int telescope_pos_mm;
                double fork_pos_deg;
                if (config.getParams(lift_level, turntable_dir, depth, lift_pos_m, turntable_pos, telescope_pos_mm, fork_pos_deg)) {
                    blackboard->set("target_lift_pos", lift_pos_m);
                    blackboard->set("target_turntable_pos", turntable_pos);
                    blackboard->set("target_telescope_pos", telescope_pos_mm);
                    blackboard->set("target_fork_pos", fork_pos_deg);
                    for (auto& val : editable) {
                        if (val.key == "target_lift_pos") val.value = std::to_string(lift_pos_m);
                        else if (val.key == "target_telescope_pos") val.value = std::to_string(telescope_pos_mm);
                        else if (val.key == "target_fork_pos") val.value = std::to_string(fork_pos_deg);
                    }
                    std::cout << "[Debug] Set from YAML: lift_level=" << lift_level << ", turntable_dir=" << turntable_dir
                              << ", depth=" << depth << ", lift_pos_m=" << lift_pos_m
                              << ", turntable_pos=" << turntable_pos << ", telescope_pos_mm=" << telescope_pos_mm
                              << ", fork_pos_deg=" << fork_pos_deg << std::endl;
                } else {
                    std::cout << "[Debug] Invalid YAML lookup: lift_level=" << lift_level
                              << ", turntable_dir=" << turntable_dir << ", depth=" << depth << std::endl;
                }
            }

            for (const auto& val : editable) {
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

            NodeStatus status = tree.tickWhileRunning();
            std::cout << "[Debug] Tree status: " << status << std::endl;

            std::this_thread::sleep_for(std::chrono::milliseconds(2000));

            initscr();
            // start_color();
            // use_default_colors();
            noecho();
            cbreak();
            curs_set(0);
            keypad(stdscr, TRUE);
            timeout(100);

            show_tick_message = true;
        }
    }

    endwin();
    return 0;
}