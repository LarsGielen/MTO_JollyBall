from flask import Flask, jsonify, request
import serial.tools.list_ports
from flask import Flask, Blueprint, jsonify, render_template, request
from arduino import open_serial_connection, close_serial_connection, read_data, send_data, get_open_connection, clear_serial_data
import time

app = Flask(__name__)

# Define a Blueprint for the views
main_view = Blueprint('main_view', __name__)
app.register_blueprint(main_view)

@app.route('/')
def control_panel():
    return render_template('index.html')

@app.route('/update', methods=['POST'])
def handle_live_update():
    data = request.json
    connection = get_open_connection()

    if connection and connection.is_open:
        if isinstance(data, list):
            for item in data:
                send_data(connection, str({
                    "key": list(item.keys())[0],
                    "value": list(item.values())[0]
                }))
                time.sleep(0.01)
        else:
            send_data(connection, str({
                "key": list(data.keys())[0],
                "value": list(data.values())[0]
            }))

    return jsonify({"status": "success", "updated_data": data})

@app.route('/scan-com-ports', methods=['GET'])
def scan_com_ports():
    ports = [port.device for port in serial.tools.list_ports.comports()]
    return jsonify(ports)

serial_connection = None

@app.route('/connect', methods=['POST'])
def connect_to_arduino():
    global serial_connection
    data = request.json
    com_port = data.get('com_port')
    if not com_port:
        return jsonify({'message': 'COM port is required.'}), 400

    # Close any existing connection
    if serial_connection:
        close_serial_connection(serial_connection)

    # Try to open a new connection
    serial_connection = open_serial_connection(com_port, 9600)
    if serial_connection:
        return jsonify({'message': f'Connected to {com_port}.'})
    else:
        return jsonify({'message': f'Failed to connect to {com_port}.'}), 500

@app.route('/serial-data', methods=['GET'])
def get_serial_data():
    """
    Endpoint to fetch the latest serial data.
    """
    data, serial_data = read_data(get_open_connection())
    return jsonify({'data': serial_data})

@app.route('/clear-serial-data', methods=['POST'])
def clear_arduino_serial_data():
    """
    Endpoint to clear the serial data.
    """
    clear_serial_data()
    return jsonify({"status": "success"})

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)