from flask import Flask
from views import main_view

app = Flask(__name__)

# Register Blueprints
app.register_blueprint(main_view)

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)