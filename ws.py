#!/usr/bin/env python2
from geventwebsocket import WebSocketServer, WebSocketApplication, Resource
import itseml
import json

class ItsApplication(WebSocketApplication):
    def on_message(self, message):
	itseml.process_message(json.loads(message))

WebSocketServer(
    ('', itseml.BASE_PORT),
    Resource({'/': ItsApplication})
).serve_forever()
