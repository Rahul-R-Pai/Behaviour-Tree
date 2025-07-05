#include <crow.h>
#include <fstream>
#include <sstream>
#include "bt.cpp"

BTLogic* bt_logic = nullptr;

int main() {
    if (!bt_logic) {
        bt_logic = new BTLogic();
    }

    crow::SimpleApp app;

    CROW_ROUTE(app, "/")([]() {
        std::ifstream file("/workspace/bt_demo/veloce_lift/index.html");
        if (!file.is_open()) {
            return crow::response(500, "Failed to load index.html");
        }
        std::stringstream buffer;
        buffer << file.rdbuf();
        file.close();
        return crow::response(buffer.str());
    });

    CROW_ROUTE(app, "/api/status").methods(crow::HTTPMethod::GET)([]() {
        crow::json::wvalue response;
        auto sensors = bt_logic->getSensors();
        crow::json::wvalue::object sensors_json;
        for (const auto& [key, value] : sensors) {
            sensors_json[key] = value;
        }
        response["sensors"] = std::move(sensors_json);

        auto values = bt_logic->getValues();
        crow::json::wvalue::object values_json;
        for (const auto& [key, value] : values) {
            values_json[key] = value;
        }
        response["values"] = std::move(values_json);

        return crow::response(response);
    });

    CROW_ROUTE(app, "/api/toggle_sensor").methods(crow::HTTPMethod::POST)([](const crow::request& req) {
        crow::json::wvalue response;
        auto body = crow::json::load(req.body);
        if (!body || !body.has("sensor")) {
            return crow::response(400, "Missing sensor key");
        }
        std::string sensor = body["sensor"].s();
        if (!bt_logic->toggleSensor(sensor)) {
            return crow::response(400, "Invalid sensor");
        }
        auto sensors = bt_logic->getSensors();
        crow::json::wvalue::object sensors_json;
        for (const auto& [key, value] : sensors) {
            sensors_json[key] = value;
        }
        response["sensors"] = std::move(sensors_json);
        return crow::response(response);
    });

    CROW_ROUTE(app, "/api/update_value").methods(crow::HTTPMethod::POST)([](const crow::request& req) {
        crow::json::wvalue response;
        auto body = crow::json::load(req.body);
        if (!body || !body.has("key") || !body.has("value")) {
            return crow::response(400, "Missing key or value");
        }
        std::string key = body["key"].s();
        std::string value = body["value"].s();
        auto [success, error] = bt_logic->updateValue(key, value);
        if (!success) {
            return crow::response(400, error);
        }
        auto values = bt_logic->getValues();
        crow::json::wvalue::object values_json;
        for (const auto& [key, value] : values) {
            values_json[key] = value;
        }
        response["values"] = std::move(values_json);
        return crow::response(response);
    });

    CROW_ROUTE(app, "/api/tick").methods(crow::HTTPMethod::POST)([]() {
        crow::json::wvalue response;
        std::string output = bt_logic->tickTree();
        response["output"] = output;
        auto sensors = bt_logic->getSensors();
        crow::json::wvalue::object sensors_json;
        for (const auto& [key, value] : sensors) {
            sensors_json[key] = value;
        }
        response["sensors"] = std::move(sensors_json);
        auto values = bt_logic->getValues();
        crow::json::wvalue::object values_json;
        for (const auto& [key, value] : values) {
            values_json[key] = value;
        }
        response["values"] = std::move(values_json);
        return crow::response(response);
    });

    app.port(8081).multithreaded().run();
    delete bt_logic;
    bt_logic = nullptr;
    return 0;
}