from flask import Flask, jsonify, request
import serial.tools.list_ports
from flask import Flask
from views import main_view
from arduino import open_serial_connection, close_serial_connection

app = Flask(__name__)

# Register Blueprints
app.register_blueprint(main_view)

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

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)