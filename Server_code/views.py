from flask import Blueprint, jsonify, render_template, request
from arduino import send_data, get_open_connection

# Define a Blueprint for the views
main_view = Blueprint('main_view', __name__)

@main_view.route('/')
def control_panel():
    return render_template('index.html')

@main_view.route('/update', methods=['POST'])
def handle_live_update():
    data = request.json
    print("Live update received:", data)

    connection = get_open_connection()

    if connection and connection.is_open:
        send_data(connection, str({
            "key": list(data.keys())[0],
            "value": list(data.values())[0]
        }))

    return jsonify({"status": "success", "updated_data": data})