#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <map>
#include <functional>
#include <chrono>
#include <sstream>

using namespace BT;

class ConsoleCapture : public std::stringbuf {
public:
    std::string getOutput() const { return str(); }
    void clear() { str(""); }
protected:
    int sync() override { return std::stringbuf::sync(); }
};

struct BBValue {
    std::string key;
    std::string type;
    std::string value;
};

class BTLogic {
public:
    BTLogic() {
        console_capture_ = std::make_unique<ConsoleCapture>();
        old_cout_ = std::cout.rdbuf(console_capture_.get());

        for (const auto& s : sensors_) {
            conditions_[s] = NodeStatus::FAILURE;
        }

        blackboard_ = Blackboard::create();
        blackboard_->set("bt_logic", this);
        blackboard_->set("lift_level", 1);
        blackboard_->set("turntable_dir", std::string("Middle"));
        blackboard_->set("deep", std::string("single"));
        blackboard_->set("telescope_mode", std::string("position"));
        blackboard_->set("tote_out_of_reach", false);
        blackboard_->set("picked", false);
        blackboard_->set("target_lift_pos", 0.0);
        blackboard_->set("target_telescope_pos", 0);
        blackboard_->set("target_fork_pos", 0.0);
        blackboard_->set("current_lift_position_m", 0.0);
        blackboard_->set("current_telescope_position_mm", 0);
        blackboard_->set("current_fork_position_deg", 0.0);
        blackboard_->set("current_turntable_position", std::string("Middle"));
        blackboard_->set("lift_error_code", 0);
        blackboard_->set("telescope_error_code", 0);
        blackboard_->set("turntable_error_code", 0);
        blackboard_->set("forks_error_code", 0);
        blackboard_->set("lift_error_msg", std::string(""));
        blackboard_->set("telescope_error_msg", std::string(""));
        blackboard_->set("turntable_error_msg", std::string(""));
        blackboard_->set("forks_error_msg", std::string(""));

        registerNodes();
        tree_ = factory_.createTreeFromFile("./../veloce_lift_new.xml", blackboard_);
        publisher_ = std::make_unique<Groot2Publisher>(tree_);
    }

    ~BTLogic() {
        std::cout.rdbuf(old_cout_);
    }

    std::map<std::string, bool> getSensors() const {
        std::map<std::string, bool> result;
        for (const auto& s : sensors_) {
            result[s] = (conditions_.at(s) == NodeStatus::SUCCESS);
        }
        return result;
    }

    std::map<std::string, std::string> getValues() const {
        std::map<std::string, std::string> result;
        for (const auto& v : editable_) {
            result[v.key] = v.value;
        }
        return result;
    }

    bool toggleSensor(const std::string& sensor) {
        if (std::find(sensors_.begin(), sensors_.end(), sensor) == sensors_.end()) {
            return false;
        }
        conditions_[sensor] = (conditions_[sensor] == NodeStatus::SUCCESS) ? NodeStatus::FAILURE : NodeStatus::SUCCESS;
        return true;
    }

    std::pair<bool, std::string> updateValue(const std::string& key, const std::string& value) {
        auto it = std::find_if(editable_.begin(), editable_.end(), [&](const BBValue& v) { return v.key == key; });
        if (it == editable_.end()) {
            return {false, "Invalid key"};
        }
        if (it->type == "int") {
            try {
                int val = std::stoi(value);
                if (key == "lift_level" && (val < 1 || val > 8)) {
                    return {false, "Invalid lift_level: use 1-8"};
                }
                it->value = value;
            } catch (const std::exception&) {
                return {false, "Invalid int value"};
            }
        } else if (it->type == "string" && key == "deep") {
            if (value != "single" && value != "double") {
                return {false, "Invalid deep value: use 'single', 'double'"};
            }
            it->value = value;
        } else if (it->type == "string" && key == "turntable_dir") {
            if (value != "Left" && value != "Middle" && value != "Right") {
                return {false, "Invalid turntable_dir: use 'Left', 'Middle', 'Right'"};
            }
            it->value = value;
        } else if (it->type == "bool") {
            if (value != "true" && value != "false" && value != "1" && value != "0") {
                return {false, "Invalid bool value: use 'true', 'false'"};
            }
            it->value = value;
        } else {
            it->value = value;
        }
        return {true, ""};
    }

    std::string tickTree() {
        console_capture_->clear();

        std::string deep_value;
        if (blackboard_->get("deep", deep_value)) {
            std::cout << "[Debug] Blackboard deep = " << deep_value << " (type: string)" << std::endl;
        }
        std::string turntable_dir;
        if (blackboard_->get("turntable_dir", turntable_dir)) {
            std::cout << "[Debug] Blackboard turntable_dir = " << turntable_dir << " (type: string)" << std::endl;
        }
        int lift_level;
        if (blackboard_->get("lift_level", lift_level)) {
            std::cout << "[Debug] Blackboard lift_level = " << lift_level << " (type: int)" << std::endl;
        }

        double lift_pos_m;
        std::string turntable_pos;
        int telescope_pos_mm;
        double fork_pos_deg;
        if (load_yaml_params("./../lift_position.yaml", lift_level, turntable_dir, deep_value,
                             lift_pos_m, turntable_pos, telescope_pos_mm, fork_pos_deg)) {
            blackboard_->set("target_lift_pos", lift_pos_m);
            blackboard_->set("turntable_dir", turntable_pos);
            blackboard_->set("target_telescope_pos", telescope_pos_mm);
            blackboard_->set("target_fork_pos", fork_pos_deg);
            std::cout << "[Debug] Loaded YAML: lift_pos_m=" << lift_pos_m
                      << ", turntable_pos=" << turntable_pos
                      << ", telescope_pos_mm=" << telescope_pos_mm
                      << ", fork_pos_deg=" << fork_pos_deg << std::endl;
        } else {
            std::cout << "[Error] Failed to load YAML parameters" << std::endl;
        }

        for (const auto& val : editable_) {
            try {
                if (val.type == "string")
                    blackboard_->set(val.key, val.value);
                else if (val.type == "bool")
                    blackboard_->set(val.key, (val.value == "true" || val.value == "1"));
                else if (val.type == "int")
                    blackboard_->set(val.key, std::stoi(val.value));
            } catch (const std::exception& e) {
                std::cout << "Error setting blackboard key " << val.key << ": " << e.what() << std::endl;
            }
        }

        NodeStatus status = tree_.tickWhileRunning();
        std::cout << "[Debug] Tree status: " << status << std::endl;

        return console_capture_->getOutput();
    }

private:
    BehaviorTreeFactory factory_;
    Tree tree_;
    std::unique_ptr<Groot2Publisher> publisher_;
    Blackboard::Ptr blackboard_;
    std::unique_ptr<ConsoleCapture> console_capture_;
    std::streambuf* old_cout_;
    std::map<std::string, NodeStatus> conditions_;
    std::map<std::string, std::string> current_positions_ = {
        {"current_lift_pos", "0.0"},
        {"current_telescope_pos", "0"},
        {"current_fork_pos", "0.0"},
        {"current_turntable_pos", "Middle"}
    };
    std::vector<std::string> sensors_ = {
        "BinClearSensors",
        "BinPresenceSensors",
        "TelescopeHomeSensors",
        "RackSensors",
        "CheckToteOutOfReach",
        "CheckTurntablePosition",
        "CheckLiftPosition",
        "CheckForkPosition",
        "CheckTelescopePosition"
    };
    std::vector<BBValue> editable_ = {
        {"lift_level", "int", "1"},
        {"turntable_dir", "string", "Middle"},
        {"deep", "string", "single"},
        {"tote_out_of_reach", "bool", "false"},
        {"picked", "bool", "false"},
        {"telescope_mode", "string", "position"}
    };

    

    void registerNodes() {

        for (const auto& s : sensors_) {
            conditions_[s] = NodeStatus::FAILURE;
        }


        factory_.registerSimpleCondition("BinClearSensors", std::bind(&BTLogic::CheckBinClearSensors, this));
        factory_.registerSimpleCondition("BinPresenceSensors", std::bind(&BTLogic::CheckBinPresenceSensors, this));
        factory_.registerSimpleCondition("TelescopeHomeSensors", std::bind(&BTLogic::CheckTelescopeHomeSensors, this));
        factory_.registerSimpleCondition("RackSensors", std::bind(&BTLogic::CheckRackSensors, this));
        factory_.registerSimpleCondition("ToteOutOfReach", [this](TreeNode& node) { return this->CheckToteOutOfReach(node); },
                                        {OutputPort<bool>("tote_out_of_reach")});
        factory_.registerSimpleCondition("TurntablePosition", [this](TreeNode& node) { return this->CheckTurntablePosition(node); },
                                        {InputPort<std::string>("turntable_position")});
        factory_.registerSimpleCondition("LiftPosition", [this](TreeNode& node) { return this->CheckLiftPosition(node); },
                                        {InputPort<double>("lift_position_m")});
        factory_.registerSimpleCondition("ForkPosition", [this](TreeNode& node) { return this->CheckForkPosition(node); },
                                        {InputPort<double>("fork_position_deg"), OutputPort<double>("fork_position_deg")});
        factory_.registerSimpleCondition("TelescopePosition", [this](TreeNode& node) { return this->CheckTelescopePosition(node); },
                                        {InputPort<int>("telescope_position_mm")});

        factory_.registerNodeType<MoveLiftNode>("MoveLift");
        factory_.registerNodeType<MoveTelescopeNode>("MoveTelescope");
        factory_.registerNodeType<MoveTurntableNode>("MoveTurntable");
        factory_.registerNodeType<MoveForksNode>("MoveForks");

  }

    bool load_yaml_params(const std::string& filename, int lift_level, const std::string& turntable_dir, const std::string& depth,
                          double& lift_pos_m, std::string& turntable_pos, int& telescope_pos_mm, double& fork_pos_deg) {
        try {
            YAML::Node config = YAML::LoadFile(filename);
            YAML::Node operation_settings = config["lift_control"]["operation_settings"];
            std::string depth_key = (depth == "single") ? "1" : "2";

            for (const auto& level : operation_settings) {
                if (level["lift_level"].as<int>() == lift_level) {
                    for (const auto& turntable : level["turntable"]) {
                        if (turntable["direction"].as<std::string>() == turntable_dir) {
                            lift_pos_m = turntable["lift_position_mm"].as<double>() / 1000.0;
                            double angle_deg = turntable["angle_deg"].as<double>();
                            turntable_pos = (angle_deg == 0) ? "Middle" : (angle_deg < 0 ? "Left" : "Right");
                            for (const auto& telescope : turntable["telescope"]) {
                                if (telescope["depth"].as<std::string>() == depth_key) {
                                    telescope_pos_mm = telescope["length_mm"].as<int>();
                                    fork_pos_deg = telescope["fork_angle_deg"].as<double>();
                                    return true;
                                }
                            }
                        }
                    }
                }
            }
            std::cout << "[Error] No matching YAML entry for lift_level=" << lift_level << ", turntable_dir=" << turntable_dir << ", depth=" << depth << std::endl;
            return false;
        } catch (const YAML::Exception& e) {
            std::cout << "[Error] Failed to load YAML: " << e.what() << std::endl;
            return false;
        }
    }

    NodeStatus CheckBinClearSensors() {
        if (conditions_.at("BinClearSensors") == NodeStatus::FAILURE) {
            std::cout << "[Condition] BinClearSensors: Failure" << std::endl;
        }
        else if (conditions_.at("BinClearSensors") == NodeStatus::SUCCESS) {
            std::cout << "[Condition] BinClearSensors: Success" << std::endl;
        }    
        return conditions_.at("BinClearSensors");
    }

    NodeStatus CheckBinPresenceSensors() {
        if (conditions_.at("BinPresenceSensors") == NodeStatus::FAILURE) {
            std::cout << "[Condition] BinPresenceSensors: Failure" << std::endl;
        }
        else if (conditions_.at("BinPresenceSensors") == NodeStatus::SUCCESS) {
            std::cout << "[Condition] BinPresenceSensors: Success" << std::endl;
        }
        return conditions_.at("BinPresenceSensors");
    }

    NodeStatus CheckTelescopeHomeSensors() {
        if (conditions_.at("TelescopeHomeSensors") == NodeStatus::FAILURE) {
            std::cout << "[Condition] TelescopeHomeSensors: Failure" << std::endl;
        }
        else if (conditions_.at("TelescopeHomeSensors") == NodeStatus::SUCCESS) {
            std::cout << "[Condition] TelescopeHomeSensors: Success" << std::endl;
        }
        return conditions_.at("TelescopeHomeSensors");
    }

    NodeStatus CheckRackSensors() {
        if (conditions_.at("RackSensors") == NodeStatus::FAILURE) {
            std::cout << "[Condition] RackSensors: Failure" << std::endl;
        }
        else if (conditions_.at("RackSensors") == NodeStatus::SUCCESS) {
            std::cout << "[Condition] RackSensors: Success" << std::endl;
        }
        return conditions_.at("RackSensors");
    }

    NodeStatus CheckToteOutOfReach(TreeNode& node) {
        bool tote_out = false;
        node.setOutput("tote_out_of_reach", tote_out);
        std::cout << "[Condition] ToteOutOfReach setting tote_out_of_reach: " << tote_out << std::endl;
        return conditions_.at("ToteOutOfReach");
    }

    NodeStatus CheckTurntablePosition(TreeNode& node) {
        std::string target_pos;
        if (!node.getInput("turntable_position", target_pos)) {
            std::cout << "[Condition] TurntablePosition: Missing turntable_position" << std::endl;
            return NodeStatus::FAILURE;
        }
        auto current_pos = current_positions_["current_turntable_pos"];
        bool is_equal = (current_pos == target_pos);
        std::cout << "[Condition] TurntablePosition checking: current=" << current_pos
                << ", target=" << target_pos << ", equal=" << is_equal << std::endl;
        conditions_["TurntablePosition"] = is_equal ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
        return conditions_["TurntablePosition"];
    }

    NodeStatus CheckLiftPosition(TreeNode& node) {
        double target_pos;
        if (!node.getInput("lift_position_m", target_pos)) {
            std::cout << "[Condition] LiftPosition: Missing lift_position_m" << std::endl;
            return NodeStatus::FAILURE;
        }
        double current_pos;
        try {
            current_pos = std::stod(current_positions_["current_lift_pos"]);
        } catch (const std::exception& e) {
            std::cout << "[Condition] LiftPosition: Invalid current_lift_pos: " << current_positions_["current_lift_pos"] << std::endl;
            return NodeStatus::FAILURE;
        }
        bool is_equal = std::abs(current_pos - target_pos) < 0.0001;
        std::cout << "[Condition] LiftPosition checking: current=" << current_pos
                << ", target=" << target_pos << ", equal=" << is_equal << std::endl;
        conditions_["LiftPosition"] = is_equal ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
        return conditions_["LiftPosition"];
    }

    NodeStatus CheckForkPosition(TreeNode& node) {
        double target_pos;
        if (!node.getInput("fork_position_deg", target_pos)) {
            std::cout << "[Condition] ForkPosition: Missing fork_position_deg" << std::endl;
            return NodeStatus::FAILURE;
        }
        double current_pos;
        try {
            current_pos = std::stod(current_positions_["current_fork_pos"]);
        } catch (const std::exception& e) {
            std::cout << "[Condition] ForkPosition: Invalid current_fork_pos: " << current_positions_["current_fork_pos"] << std::endl;
            return NodeStatus::FAILURE;
        }
        bool is_equal = std::abs(current_pos - target_pos) < 0.0001;
        node.setOutput("fork_position_deg", current_pos);
        std::cout << "[Condition] ForkPosition checking: current=" << current_pos
                << ", target=" << target_pos << ", equal=" << is_equal << std::endl;
        conditions_["ForkPosition"] = is_equal ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
        return conditions_["ForkPosition"];
    }

    NodeStatus CheckTelescopePosition(TreeNode& node) {
        int target_pos;
        if (!node.getInput("telescope_position_mm", target_pos)) {
            std::cout << "[Condition] TelescopePosition: Missing telescope_position_mm" << std::endl;
            return NodeStatus::FAILURE;
        }
        int current_pos;
        try {
            current_pos = std::stoi(current_positions_["current_telescope_pos"]);
        } catch (const std::exception& e) {
            std::cout << "[Condition] TelescopePosition: Invalid current_telescope_pos: " << current_positions_["current_telescope_pos"] << std::endl;
            return NodeStatus::FAILURE;
        }
        bool is_equal = (current_pos == target_pos);
        std::cout << "[Condition] TelescopePosition checking: current=" << current_pos
                << ", target=" << target_pos << ", equal=" << is_equal << std::endl;
        conditions_["TelescopePosition"] = is_equal ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
        return conditions_["TelescopePosition"];
    }

class MoveLiftNode : public StatefulActionNode {
    public:
        MoveLiftNode(const std::string& name, const NodeConfig& config)
            : StatefulActionNode(name, config), start_time_(std::chrono::steady_clock::now()) {
            // Retrieve BTLogic* from blackboard
            if (!config.blackboard->get("bt_logic", bt_logic_)) {
                throw std::runtime_error("MoveLiftNode: Failed to retrieve bt_logic from blackboard");
            }
        }

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
                bt_logic_->current_positions_["current_lift_pos"] = std::to_string(lift_target_pos_);
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
        BTLogic* bt_logic_;
        friend class BTLogic;
    };

class MoveTelescopeNode : public StatefulActionNode {
    public:
        MoveTelescopeNode(const std::string& name, const NodeConfig& config)
            : StatefulActionNode(name, config), start_time_(std::chrono::steady_clock::now()) {
            if (!config.blackboard->get("bt_logic", bt_logic_)) {
                throw std::runtime_error("MoveTelescopeNode: Failed to retrieve bt_logic from blackboard");
            }
        }

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
                bt_logic_->current_positions_["current_telescope_pos"] = std::to_string(telescope_target_pos_);
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
        BTLogic* bt_logic_;
        friend class BTLogic;
    };

class MoveTurntableNode : public StatefulActionNode {
    public:
        MoveTurntableNode(const std::string& name, const NodeConfig& config)
            : StatefulActionNode(name, config), start_time_(std::chrono::steady_clock::now()) {
            if (!config.blackboard->get("bt_logic", bt_logic_)) {
                throw std::runtime_error("MoveTurntableNode: Failed to retrieve bt_logic from blackboard");
            }
        }

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
                bt_logic_->current_positions_["current_turntable_pos"] = turntable_target_pos_;
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
        BTLogic* bt_logic_;
        friend class BTLogic;
    };

class MoveForksNode : public StatefulActionNode {
    public:
        MoveForksNode(const std::string& name, const NodeConfig& config)
            : StatefulActionNode(name, config), start_time_(std::chrono::steady_clock::now()) {
            if (!config.blackboard->get("bt_logic", bt_logic_)) {
                throw std::runtime_error("MoveForksNode: Failed to retrieve bt_logic from blackboard");
            }
        }

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
                bt_logic_->current_positions_["current_fork_pos"] = std::to_string(fork_target_pos_);
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
        BTLogic* bt_logic_;
        friend class BTLogic;
    };

    friend class MoveLiftNode;
    friend class MoveTelescopeNode;
    friend class MoveTurntableNode;
    friend class MoveForksNode;
};
