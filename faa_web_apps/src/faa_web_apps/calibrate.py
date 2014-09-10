#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('faa_web_apps')
import rospy
import flask,flask_sijax
import argparse
import os
import signal
import json

network_port = 5000

# Setup application
app = flask.Flask(__name__)
# Setup application w/ sijax
path = os.path.join('.', os.path.dirname(__file__), 'static/js/sijax/')
app.config["SIJAX_STATIC_PATH"] = path
app.config["SIJAX_JSON_URI"] = '/static/js/sijax/json2.js'
flask_sijax.Sijax(app)

# class SijaxHandler(object):
#     """A container class for all Sijax handlers.

#     Grouping all Sijax handler functions in a class
#     (or a Python module) allows them all to be registered with
#     a single line of code.
#     """

    # @staticmethod
    # def updateLed(obj_response,tunnel,led,duty_cycle):
    #     print('updateLed: tunnel={0},led={1},duty_cycle={2}'.format(tunnel,led,duty_cycle))
    #     if not test:
    #         actuation.controller.updateLed(tunnel,led,duty_cycle)



@flask_sijax.route(app, "/")
def index():
    if flask.g.sijax.is_sijax_request:
        # The request looks like a valid Sijax request
        # Let's register the handlers and tell Sijax to process it
        # flask.g.sijax.register_object(SijaxHandler)
        return flask.g.sijax.process_request()

    return flask.render_template('calibrate.html')


def webserver():
    server = 'local'
    if server == 'local':
        print(' * using debug server - localhost only')
        app.run(debug=True)
    else:
        print(' * using builtin server - remote access possible')
        app.run(host='0.0.0.0')


# -----------------------------------------------------------------------------
if __name__ == '__main__':
    webserver()
