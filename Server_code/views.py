from flask import Blueprint, jsonify, render_template, request

# Define a Blueprint for the views
main_view = Blueprint('main_view', __name__)

@main_view.route('/')
def control_panel():
    return render_template('index.html')

@main_view.route('/update', methods=['POST'])
def handle_live_update():
    data = request.json
    print("Live update received:", data)
    return jsonify({"status": "success", "updated_data": data})