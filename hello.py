from flask import Flask

app = Flask(__name__)
#app.run(host='0.0.0.0', port=8080)

@app.route("/")
def index():
    return "<h1>Hello!</h1>"

def create_app():
   return app