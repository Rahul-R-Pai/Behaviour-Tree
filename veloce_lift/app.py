from flask import Flask, render_template, jsonify, request
from bt import BTLogic

app = Flask(__name__)
bt_logic = BTLogic()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/status', methods=['GET'])
def status():
    return jsonify({
        'sensors': bt_logic.get_sensors(),
        'values': bt_logic.get_values()
    })

@app.route('/api/toggle_sensor', methods=['POST'])
def toggle_sensor():
    data = request.get_json()
    if not data or 'sensor' not in data:
        return jsonify({'error': 'Missing sensor key'}), 400
    sensor = data['sensor']
    if not bt_logic.toggle_sensor(sensor):
        return jsonify({'error': 'Invalid sensor'}), 400
    return jsonify({'sensors': bt_logic.get_sensors()})

@app.route('/api/update_value', methods=['POST'])
def update_value():
    data = request.get_json()
    if not data or 'key' not in data or 'value' not in data:
        return jsonify({'error': 'Missing key or value'}), 400
    key = data['key']
    value = data['value']
    success, error = bt_logic.update_value(key, value)
    if not success:
        return jsonify({'error': error}), 400
    return jsonify({'values': bt_logic.get_values()})

@app.route('/api/tick', methods=['POST'])
def tick():
    output = bt_logic.tick_tree()
    return jsonify({
        'output': output,
        'sensors': bt_logic.get_sensors(),
        'values': bt_logic.get_values()
    })

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)