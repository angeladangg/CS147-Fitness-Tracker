from flask import Flask, request, jsonify, render_template

app = Flask(__name__)

# In-memory storage for data
data = {"steps": 0, "intensity": 0.0}

@app.route("/", methods=["GET"])
def home():
    return render_template("index.html")  # Serve HTML

@app.route("/get-data", methods=["GET"])
def get_data():
    return jsonify(data)

@app.route("/", methods=["POST"])
def update():
    try:
        # Parse JSON data from ESP32
        content = request.get_json()
        data["steps"] = content.get("steps", data["steps"])
        data["intensity"] = content.get("intensity", data["intensity"])
        data["intensity"] = content.get("intensity", data["intensity"])

        print(f"Received: Steps = {data['steps']}, Intensity = {data['intensity']}")
        return jsonify({"status": "success"}), 200
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 400

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
