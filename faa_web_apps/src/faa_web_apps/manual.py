#!/usr/bin/env python
import flask


app = flask.Flask(__name__)

@app.route("/")
def index():
    return flask.render_template('manual.html')

def webserver():
    server = 'local'
    if server == 'local':
        print(' * using debug server - localhost only')
        app.run(debug=True)
    else:
        print(' * using builtin server - remote access possible')
        app.run(host='0.0.0.0')


if __name__ == "__main__":
    webserver()
