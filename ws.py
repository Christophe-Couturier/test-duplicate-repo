#!/usr/bin/env python2
from geventwebsocket import WebSocketServer, WebSocketApplication, Resource
import itseml
import json

class ItsApplication(WebSocketApplication):
    def on_message(self, message):
	try:
		response = itseml.process_message(json.loads(message))
	except ValueError, e:
		response = "400 Invalid JSON"
	self.ws.send(response)

WebSocketServer(
    ('', itseml.BASE_PORT),
    Resource({'/': ItsApplication})
).serve_forever()
