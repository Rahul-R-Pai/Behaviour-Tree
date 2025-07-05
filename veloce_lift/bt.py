import yaml
import behaviortree_cpp as bt
from io import StringIO
import sys

class ConsoleCapture:
    def __init__(self):
        self.buffer = StringIO()

    def write(self, text):
        self.buffer.write(text)

    def flush(self):
        pass

    def get_output(self):
        return self.buffer.getvalue()

    def clear(self):
        self.buffer.truncate(0)
        self.buffer.seek(0)

class BTLogic:
    def __init__(self):
        self.console_capture = ConsoleCapture()
        self.old_stdout = sys.stdout
        sys.stdout = self.console_capture

        self.sensors = [
            "BinClearSensors", "BinPresenceSensors", "TelescopeHomeSensors", "RackSensors",
            "CheckToteOutOfReach", "CheckTurntablePosition", "CheckLiftPosition",
            "CheckForkPosition", "CheckTelescopePosition"
        ]
        self.conditions = {s: bt.NodeStatus.FAILURE for s in self.sensors}
        self.editable = [
            {"key": "lift_level", "type": "int", "value": "1"},
            {"key": "turntable_dir", "type": "string", "value": "Middle"},
            {"key": "deep", "type": "string", "value": "single"},
            {"key": "tote_out_of_reach", "type": "bool", "value": "false"},
            {"key": "picked", "type": "bool", "value": "false"},
            {"key": "telescope_mode", "type": "string", "value": "position"}
        ]
        self.current_positions = {
            "current_lift_pos": "0.0",
            "current_telescope_pos": "0",
            "current_fork_pos": "0.0",
            "current_turntable_pos": "Middle"
        }

        self.factory = bt.BehaviorTreeFactory()
        self.blackboard = bt.Blackboard.create()
        self.blackboard.set("bt_logic", self)
        self.blackboard.set("lift_level", 1)
        self.blackboard.set("turntable_dir", "Middle")
        self.blackboard.set("deep", "single")
        self.blackboard.set("telescope_mode", "position")
        self.blackboard.set("tote_out_of_reach", False)
        self.blackboard.set("picked", False)
        self.blackboard.set("target_lift_pos", 0.0)
        self.blackboard.set("target_telescope_pos", 0)
        self.blackboard.set("target_fork_pos", 0.0)
        self.blackboard.set("current_lift_position_m", 0.0)
        self.blackboard.set("current_telescope_position_mm", 0)
        self.blackboard.set("current_fork_position_deg", 0.0)
        self.blackboard.set("current_turntable_position", "Middle")
        self.blackboard.set("lift_error_code", 0)
        self.blackboard.set("telescope_error_code", 0)
        self.blackboard.set("turntable_error_code", 0)
        self.blackboard.set("forks_error_code", 0)
        self.blackboard.set("lift_error_msg", "")
        self.blackboard.set("telescope_error_msg", "")
        self.blackboard.set("turntable_error_msg", "")
        self.blackboard.set("forks_error_msg", "")

        self.register_nodes()
        self.tree = self.factory.create_tree_from_file("/workspace/veloce_lift_new.xml", self.blackboard)

    def __del__(self):
        sys.stdout = self.old_stdout

    def get_sensors(self):
        return {s: self.conditions[s] == bt.NodeStatus.SUCCESS for s in self.sensors}

    def get_values(self):
        return {v["key"]: v["value"] for v in self.editable}

    def toggle_sensor(self, sensor):
        if sensor not in self.sensors:
            return False
        self.conditions[sensor] = bt.NodeStatus.FAILURE if self.conditions[sensor] == bt.NodeStatus.SUCCESS else bt.NodeStatus.SUCCESS
        return True

    def update_value(self, key, value):
        for v in self.editable:
            if v["key"] == key:
                if v["type"] == "int":
                    try:
                        val = int(value)
                        if key == "lift_level" and (val < 1 or val > 8):
                            return False, "Invalid lift_level: use 1-8"
                        v["value"] = value
                    except ValueError:
                        return False, "Invalid int value"
                elif v["type"] == "string" and key == "deep":
                    if value not in ["single", "double"]:
                        return False, "Invalid deep value: use 'single', 'double'"
                    v["value"] = value
                elif v["type"] == "string" and key == "turntable_dir":
                    if value not in ["Left", "Middle", "Right"]:
                        return False, "Invalid turntable_dir: use 'Left', 'Middle', 'Right'"
                    v["value"] = value
                elif v["type"] == "bool":
                    if value not in ["true", "false", "1", "0"]:
                        return False, "Invalid bool value: use 'true', 'false'"
                    v["value"] = value
                else:
                    v["value"] = value
                return True, ""
        return False, "Invalid key"

    def tick_tree(self):
        self.console_capture.clear()
        deep_value = self.blackboard.get("deep")
        turntable_dir = self.blackboard.get("turntable_dir")
        lift_level = self.blackboard.get("lift_level")
        print(f"[Debug] Blackboard deep = {deep_value} (type: string)")
        print(f"[Debug] Blackboard turntable_dir = {turntable_dir} (type: string)")
        print(f"[Debug] Blackboard lift_level = {lift_level} (type: int)")

        lift_pos_m, turntable_pos, telescope_pos_mm, fork_pos_deg = self.load_yaml_params(
            "/workspace/lift_control.yaml", lift_level, turntable_dir, deep_value)
        if lift_pos_m is not None:
            self.blackboard.set("target_lift_pos", lift_pos_m)
            self.blackboard.set("turntable_dir", turntable_pos)
            self.blackboard.set("target_telescope_pos", telescope_pos_mm)
            self.blackboard.set("target_fork_pos", fork_pos_deg)
            print(f"[Debug] Loaded YAML: lift_pos_m={lift_pos_m}, turntable_pos={turntable_pos}, "
                  f"telescope_pos_mm={telescope_pos_mm}, fork_pos_deg={fork_pos_deg}")
        else:
            print("[Error] Failed to load YAML parameters")

        for v in self.editable:
            try:
                if v["type"] == "string":
                    self.blackboard.set(v["key"], v["value"])
                elif v["type"] == "bool":
                    self.blackboard.set(v["key"], v["value"] in ["true", "1"])
                elif v["type"] == "int":
                    self.blackboard.set(v["key"], int(v["value"]))
            except Exception as e:
                print(f"Error setting blackboard key {v['key']}: {e}")

        status = self.tree.tick_while_running()
        print(f"[Debug] Tree status: {status}")
        return self.console_capture.get_output()

    def load_yaml_params(self, filename, lift_level, turntable_dir, depth):
        try:
            with open(filename, 'r') as f:
                config = yaml.safe_load(f)
            operation_settings = config["lift_control"]["operation_settings"]
            depth_key = "1" if depth == "single" else "2"
            for level in operation_settings:
                if level["lift_level"] == lift_level:
                    for turntable in level["turntable"]:
                        if turntable["direction"] == turntable_dir:
                            lift_pos_m = turntable["lift_position_mm"] / 1000.0
                            angle_deg = turntable["angle_deg"]
                            turntable_pos = "Middle" if angle_deg == 0 else ("Left" if angle_deg < 0 else "Right")
                            for telescope in turntable["telescope"]:
                                if telescope["depth"] == depth_key:
                                    return lift_pos_m, turntable_pos, telescope["length_mm"], telescope["fork_angle_deg"]
            print(f"[Error] No matching YAML entry for lift_level={lift_level}, turntable_dir={turntable_dir}, depth={depth}")
            return None, None, None, None
        except Exception as e:
            print(f"[Error] Failed to load YAML: {e}")
            return None, None, None, None

    def register_nodes(self):
        for sensor in self.sensors:
            self.conditions[sensor] = bt.NodeStatus.FAILURE

        self.factory.register_simple_condition("BinClearSensors", lambda node: self.check_bin_clear_sensors())
        self.factory.register_simple_condition("BinPresenceSensors", lambda node: self.check_bin_presence_sensors())
        self.factory.register_simple_condition("TelescopeHomeSensors", lambda node: self.check_telescope_home_sensors())
        self.factory.register_simple_condition("RackSensors", lambda node: self.check_rack_sensors())
        self.factory.register_simple_condition("ToteOutOfReach", self.check_tote_out_of_reach, [bt.OutputPort("tote_out_of_reach", bool)])
        self.factory.register_simple_condition("TurntablePosition", self.check_turntable_position, [bt.InputPort("turntable_position", str)])
        self.factory.register_simple_condition("LiftPosition", self.check_lift_position, [bt.InputPort("lift_position_m", float)])
        self.factory.register_simple_condition("ForkPosition", self.check_fork_position, [bt.InputPort("fork_position_deg", float), bt.OutputPort("fork_position_deg", float)])
        self.factory.register_simple_condition("TelescopePosition", self.check_telescope_position, [bt.InputPort("telescope_position_mm", int)])

        self.factory.register_node_type("MoveLift", MoveLiftNode)
        self.factory.register_node_type("MoveTelescope", MoveTelescopeNode)
        self.factory.register_node_type("MoveTurntable", MoveTurntableNode)
        self.factory.register_node_type("MoveForks", MoveForksNode)

    def check_bin_clear_sensors(self):
        status = self.conditions["BinClearSensors"]
        print(f"[Condition] BinClearSensors: {status}")
        return status

    def check_bin_presence_sensors(self):
        status = self.conditions["BinPresenceSensors"]
        print(f"[Condition] BinPresenceSensors: {status}")
        return status

    def check_telescope_home_sensors(self):
        status = self.conditions["TelescopeHomeSensors"]
        print(f"[Condition] TelescopeHomeSensors: {status}")
        return status

    def check_rack_sensors(self):
        status = self.conditions["RackSensors"]
        print(f"[Condition] RackSensors: {status}")
        return status

    def check_tote_out_of_reach(self, node):
        tote_out = False
        node.set_output("tote_out_of_reach", tote_out)
        status = self.conditions["CheckToteOutOfReach"]
        print(f"[Condition] ToteOutOfReach setting tote_out_of_reach: {tote_out}")
        return status

    def check_turntable_position(self, node):
        target_pos = node.get_input("turntable_position")
        if not target_pos:
            print("[Condition] TurntablePosition: Missing turntable_position")
            return bt.NodeStatus.FAILURE
        current_pos = self.current_positions["current_turntable_pos"]
        is_equal = current_pos == target_pos
        print(f"[Condition] TurntablePosition checking: current={current_pos}, target={target_pos}, equal={is_equal}")
        self.conditions["CheckTurntablePosition"] = bt.NodeStatus.SUCCESS if is_equal else bt.NodeStatus.FAILURE
        return self.conditions["CheckTurntablePosition"]

    def check_lift_position(self, node):
        target_pos = node.get_input("lift_position_m")
        if not target_pos:
            print("[Condition] LiftPosition: Missing lift_position_m")
            return bt.NodeStatus.FAILURE
        try:
            current_pos = float(self.current_positions["current_lift_pos"])
        except ValueError:
            print(f"[Condition] LiftPosition: Invalid current_lift_pos: {self.current_positions['current_lift_pos']}")
            return bt.NodeStatus.FAILURE
        is_equal = abs(current_pos - target_pos) < 0.0001
        print(f"[Condition] LiftPosition checking: current={current_pos}, target={target_pos}, equal={is_equal}")
        self.conditions["CheckLiftPosition"] = bt.NodeStatus.SUCCESS if is_equal else bt.NodeStatus.FAILURE
        return self.conditions["CheckLiftPosition"]

    def check_fork_position(self, node):
        target_pos = node.get_input("fork_position_deg")
        if not target_pos:
            print("[Condition] ForkPosition: Missing fork_position_deg")
            return bt.NodeStatus.FAILURE
        try:
            current_pos = float(self.current_positions["current_fork_pos"])
        except ValueError:
            print(f"[Condition] ForkPosition: Invalid current_fork_pos: {self.current_positions['current_fork_pos']}")
            return bt.NodeStatus.FAILURE
        is_equal = abs(current_pos - target_pos) < 0.0001
        node.set_output("fork_position_deg", current_pos)
        print(f"[Condition] ForkPosition checking: current={current_pos}, target={target_pos}, equal={is_equal}")
        self.conditions["CheckForkPosition"] = bt.NodeStatus.SUCCESS if is_equal else bt.NodeStatus.FAILURE
        return self.conditions["CheckForkPosition"]

    def check_telescope_position(self, node):
        target_pos = node.get_input("telescope_position_mm")
        if not target_pos:
            print("[Condition] TelescopePosition: Missing telescope_position_mm")
            return bt.NodeStatus.FAILURE
        try:
            current_pos = int(self.current_positions["current_telescope_pos"])
        except ValueError:
            print(f"[Condition] TelescopePosition: Invalid current_telescope_pos: {self.current_positions['current_telescope_pos']}")
            return bt.NodeStatus.FAILURE
        is_equal = current_pos == target_pos
        print(f"[Condition] TelescopePosition checking: current={current_pos}, target={target_pos}, equal={is_equal}")
        self.conditions["CheckTelescopePosition"] = bt.NodeStatus.SUCCESS if is_equal else bt.NodeStatus.FAILURE
        return self.conditions["CheckTelescopePosition"]

class MoveLiftNode(bt.StatefulActionNode):
    def __init__(self, name, config):
        super().__init__(name, config)
        self.start_time = None
        self.bt_logic = config.blackboard.get("bt_logic")
        if not self.bt_logic:
            raise RuntimeError("MoveLiftNode: Failed to retrieve bt_logic from blackboard")

    @staticmethod
    def provided_ports():
        return [
            bt.InputPort("lift_target_position_m", float),
            bt.OutputPort("current_lift_position_m", float),
            bt.OutputPort("error_code", int),
            bt.OutputPort("error_msg", str)
        ]

    def on_start(self):
        error_code = 0
        error_msg = "Success"
        target_pos = self.get_input("lift_target_position_m")
        if not target_pos:
            error_code = 1
            error_msg = "Missing lift_target_position_m"
            self.set_output("error_code", error_code)
            self.set_output("error_msg", error_msg)
            print(f"[Action] MoveLift failed: {error_msg}")
            return bt.NodeStatus.FAILURE
        self.lift_target_pos = target_pos
        self.start_time = bt.get_time()
        print(f"[Action] MoveLift started to {self.lift_target_pos} m")
        return bt.NodeStatus.RUNNING

    def on_running(self):
        elapsed = (bt.get_time() - self.start_time).total_seconds()
        if elapsed >= 3:
            self.set_output("current_lift_position_m", self.lift_target_pos)
            self.set_output("error_code", 0)
            self.set_output("error_msg", "Success")
            self.bt_logic.current_positions["current_lift_pos"] = str(self.lift_target_pos)
            print(f"[Action] MoveLift reached target: {self.lift_target_pos} m")
            return bt.NodeStatus.SUCCESS
        return bt.NodeStatus.RUNNING

    def on_halted(self):
        print("[Action] MoveLift halted")

class MoveTelescopeNode(bt.StatefulActionNode):
    def __init__(self, name, config):
        super().__init__(name, config)
        self.start_time = None
        self.bt_logic = config.blackboard.get("bt_logic")
        if not self.bt_logic:
            raise RuntimeError("MoveTelescopeNode: Failed to retrieve bt_logic from blackboard")

    @staticmethod
    def provided_ports():
        return [
            bt.InputPort("telescope_target_position_mm", int),
            bt.InputPort("control_mode", str),
            bt.OutputPort("current_telescope_position_mm", int),
            bt.OutputPort("error_code", int),
            bt.OutputPort("error_msg", str)
        ]

    def on_start(self):
        error_code = 0
        error_msg = "Success"
        target_pos = self.get_input("telescope_target_position_mm")
        if not target_pos:
            error_code = 1
            error_msg = "Missing telescope_target_position_mm"
            self.set_output("error_code", error_code)
            self.set_output("error_msg", error_msg)
            print(f"[Action] MoveTelescope failed: {error_msg}")
            return bt.NodeStatus.FAILURE
        self.telescope_target_pos = target_pos
        self.mode = self.get_input("control_mode") or "position"
        self.start_time = bt.get_time()
        print(f"[Action] MoveTelescope started to {self.telescope_target_pos} mm ({self.mode})")
        return bt.NodeStatus.RUNNING

    def on_running(self):
        elapsed = (bt.get_time() - self.start_time).total_seconds()
        if elapsed >= 3:
            self.set_output("current_telescope_position_mm", self.telescope_target_pos)
            self.set_output("error_code", 0)
            self.set_output("error_msg", "Success")
            self.bt_logic.current_positions["current_telescope_pos"] = str(self.telescope_target_pos)
            print(f"[Action] MoveTelescope reached target: {self.telescope_target_pos} mm ({self.mode})")
            return bt.NodeStatus.SUCCESS
        return bt.NodeStatus.RUNNING

    def on_halted(self):
        print("[Action] MoveTelescope halted")

class MoveTurntableNode(bt.StatefulActionNode):
    def __init__(self, name, config):
        super().__init__(name, config)
        self.start_time = None
        self.bt_logic = config.blackboard.get("bt_logic")
        if not self.bt_logic:
            raise RuntimeError("MoveTurntableNode: Failed to retrieve bt_logic from blackboard")

    @staticmethod
    def provided_ports():
        return [
            bt.InputPort("turntable_target_position", str),
            bt.OutputPort("current_turntable_position", str),
            bt.OutputPort("error_code", int),
            bt.OutputPort("error_msg", str)
        ]

    def on_start(self):
        error_code = 0
        error_msg = "Success"
        target_pos = self.get_input("turntable_target_position")
        if not target_pos:
            error_code = 1
            error_msg = "Missing turntable_target_position"
            self.set_output("error_code", error_code)
            self.set_output("error_msg", error_msg)
            print(f"[Action] MoveTurntable failed: {error_msg}")
            return bt.NodeStatus.FAILURE
        self.turntable_target_pos = target_pos
        self.start_time = bt.get_time()
        print(f"[Action] MoveTurntable started to {self.turntable_target_pos}")
        return bt.NodeStatus.RUNNING

    def on_running(self):
        elapsed = (bt.get_time() - self.start_time).total_seconds()
        if elapsed >= 3:
            self.set_output("current_turntable_position", self.turntable_target_pos)
            self.set_output("error_code", 0)
            self.set_output("error_msg", "Success")
            self.bt_logic.current_positions["current_turntable_pos"] = self.turntable_target_pos
            print(f"[Action] MoveTurntable reached target: {self.turntable_target_pos}")
            return bt.NodeStatus.SUCCESS
        return bt.NodeStatus.RUNNING

    def on_halted(self):
        print("[Action] MoveTurntable halted")

class MoveForksNode(bt.StatefulActionNode):
    def __init__(self, name, config):
        super().__init__(name, config)
        self.start_time = None
        self.bt_logic = config.blackboard.get("bt_logic")
        if not self.bt_logic:
            raise RuntimeError("MoveForksNode: Failed to retrieve bt_logic from blackboard")

    @staticmethod
    def provided_ports():
        return [
            bt.InputPort("fork_position_deg", float),
            bt.OutputPort("current_fork_position_deg", float),
            bt.OutputPort("error_code", int),
            bt.OutputPort("error_msg", str)
        ]

    def on_start(self):
        error_code = 0
        error_msg = "Success"
        target_pos = self.get_input("fork_position_deg")
        if not target_pos:
            error_code = 1
            error_msg = "Missing fork_position_deg"
            self.set_output("error_code", error_code)
            self.set_output("error_msg", error_msg)
            print(f"[Action] MoveForks failed: {error_msg}")
            return bt.NodeStatus.FAILURE
        self.fork_target_pos = target_pos
        self.start_time = bt.get_time()
        print(f"[Action] MoveForks started to {self.fork_target_pos} deg")
        return bt.NodeStatus.RUNNING

    def on_running(self):
        elapsed = (bt.get_time() - self.start_time).total_seconds()
        if elapsed >= 3:
            self.set_output("current_fork_position_deg", self.fork_target_pos)
            self.set_output("error_code", 0)
            self.set_output("error_msg", "Success")
            self.bt_logic.current_positions["current_fork_pos"] = str(self.fork_target_pos)
            print(f"[Action] MoveForks reached target: {self.fork_target_pos} deg")
            return bt.NodeStatus.SUCCESS
        return bt.NodeStatus.RUNNING

    def on_halted(self):
        print("[Action] MoveForks halted")