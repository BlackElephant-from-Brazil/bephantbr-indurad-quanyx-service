from flask import Flask, jsonify

app = Flask(__name__)

@app.route("/")
def hello_world():
    return jsonify({"resp":"Hello, World!"})

app.run(host='0.0.0.0')
