from websocket import create_connection
import json
from threading import Thread
import time

## Maybe change the name
class Choir():
	def __init__(self, ip="127.0.0.1", port=8080, app_id=0):
		"""
		@param ip : middleware ip to connect to.
		@type ip : string

		@param app_id : application id.
		@type app_id : int.
		"""
		self.ip = ip
		self.port = port
		self.app_id = app_id
		self.subscriber_dict = {}
		self.registrer_dict = {}

		## pubsub API
	def subscribe(self, msg_id, received_message_callback, format=""):
		"""
		Subscribe to a specific message id.

		@param msg_id : message id to subscribe to.
		@type msg_id : int.

		@param received_message_callback : callback method called when a message has been received.
		@type received_message_callback : method(string).

		@param format : Format use to subscribe messages ('s0' = ANY, 's1' = GENERIC, 's2' = ASN1_UPER, 's7' = JSON, 'b' = BINARY).
		@type format : string.

		@return : a numerical identifier of the registered subscription.
		@rtype : int
		"""
		if format != "":
			format = "/{}".format(format)
		subscribe_thread = SubscribeThread(msg_id, received_message_callback, self.ip, self.port, self.app_id, format)
		subscribe_thread.start()
		self.subscriber_dict[len(self.subscriber_dict)+1] = subscribe_thread
		return len(self.subscriber_dict)

	def unsubscribe(self, subscription_id):
		"""
		Unsubscribe from a previously subscription.

		@param subscription_id : the numerical identifier of the registered subscription.
		@type subscription_id : int.
		"""
		self.subscriber_dict[subscription_id].stop()

	def publish(self, msg_id, message, format=""):
		"""
		Publish on a specific message id.

		@param msg_id : message id to publish on.
		@type msg_id : int.

		@param message : message to publish.
		@type message : string.

		@param format : Format use to publish messages ('s0' = ANY, 's1' = GENERIC, 's2' = ASN1_UPER, 's7' = JSON, 'b' = BINARY).
		@type format : string.
		"""
		if format != "":
			format = "/{}".format(format)
		socket = create_connection("ws://{}:{}/pubsub/pub/{}/{}{}".format(self.ip, self.port, msg_id, self.app_id, format))
		socket.send(message)

	def request(self, msg_id, message, format=""):
		"""
		Publish a message for reliable delivery to all subscribers, then wait for their response.

		@param msg_id : message id to publish on.
		@type msg_id : int.

		@param message : message to publish.
		@type message : string.

		@param format : Format use to publish messages ('s0' = ANY, 's1' = GENERIC, 's2' = ASN1_UPER, 's7' = JSON, 'b' = BINARY).
		@type format : string.
		"""
		if format != "":
			format = "/{}".format(format)
		socket = create_connection("ws://{}:{}/pubsub/req/{}/{}{}".format(self.ip, self.port, msg_id, self.app_id, format))
		socket.send(message)

	## cache API
	def cache_list(self, cache_id):
		"""
		List all object id in a specific cache.

		@param cache_id : id of the target cache.
		@type cache_id : int.

		@return : list of object id.
		@rtype : [int].
		"""
		socket = create_connection("ws://{}:{}/cache/list/{}/{}".format(self.ip, self.port, cache_id, self.app_id))
		return json.loads(socket.recv())

	def cache_get(self, cache_id, object_id):
		"""
		Get specified cache object.

		@param cache_id : id of the target cache.
		@type cache_id : int.

		@param object_id : id of the target object.
		@type object_id : int.

		@return : data of the object. Nothing if the cache does not exist.
		@rtype : {string:value}.
		"""
		socket = create_connection("ws://{}:{}/cache/get/{}/{}/{}".format(self.ip, self.port, cache_id, object_id, self.app_id))
		return json.loads(socket.recv())

	def cache_set(self, cache_id, object_id, entries_mapping):
		"""
		Set specified entries in cache object.

		@param cache_id : id of the target cache.
		@type cache_id : int.

		@param object_id : id of the target object.
		@type object_id : int.

		@param entries_mapping : mapping key->value to set in the object.
		@type entries_mapping : {string:data}.
		"""
		socket = create_connection("ws://{}:{}/cache/set/{}/{}/{}".format(self.ip, self.port, cache_id, object_id, self.app_id))
		json_data = json.dumps(entries_mapping)
		socket.send(json_data)

	def cache_register(self, cache_id, object_id, cache_update_callback):
		"""
		Register on a cache to be notified on update.

		@param cache_id : id of the target cache.
		@type cache_id : int.

		@param object_id : id of the target object.
		@type object_id : int.

		@param cache_update_callback : callback method called when a update occurs (List of updated entries, Data of the object).
		@type cache_update_callback : method([string], {string:value}).

		@return : a numerical identifier of the registered registration.
		@rtype : int
		"""
		register_thread = RegisterThread(cache_id, object_id, cache_update_callback, self.ip, self.port, self.app_id)
		register_thread.start()
		self.registrer_dict[len(self.registrer_dict)+1] = register_thread
		return len(self.registrer_dict)

	def cache_unregister(self, registration_id):
		"""
		Unregister a previously register..

		@param registration_id : the numerical identifier of the registered registration.
		@type registration_id : int.
		"""
		self.registrer_dict[registration_id].stop()

	def cache_remove(self, cache_id, object_id):
		"""
		Remove a cache object.

		@param cache_id : id of the target cache.
		@type cache_id : int.

		@param object_id : id of the target object.
		@type object_id : int.
		"""
		socket = create_connection("ws://{}:{}/cache/remove/{}/{}/{}".format(self.ip, self.port, cache_id, object_id, self.app_id))

	## config API
	def config_list_services(self):
		"""
		Return a list of all services.

		@return : list of all services, their instance and status.
		@rtype : [{"instanceId":int, 'started': boolean, 'serviceName': string}]
		"""
		socket = create_connection("ws://{}:{}/config".format(self.ip, self.port))
		message = {"command":"list-services"}
		socket.send(json.dumps(message))
		return json.loads(socket.recv())["services"]

	def config_is_started(self, service_name):
		"""
		Return True if service in started.
		Raise a ChoirConfigError if the service does not exist.

		@param service_name : name of the target service.
		@type service_name : string.

		@return : True if started, False else.
		@rtype : boolean
		"""
		socket = create_connection("ws://{}:{}/config".format(self.ip, self.port))
		message = {"command":"config-status","service-name":service_name}
		socket.send(json.dumps(message))
		data_mapping = json.loads(socket.recv())
		if 'status' in data_mapping:
			if (data_mapping['status'] == "started"):
				return True
			else:
				return False
		elif "error" in data_mapping:
			raise ChoirConfigError(data_mapping["error"])
		else:
			raise ChoirConfigError("No response from Middleware.")

	def config_status(self, service_name):
		"""
		Return True if service in started.
		Raise a ChoirConfigError if the service does not exist.

		@param service_name : name of the target service.
		@type service_name : string.

		@return : True if started, False else.
		@rtype : boolean
		"""
		socket = create_connection("ws://{}:{}/config".format(self.ip, self.port))
		message = {"command":"config-status","service-name":service_name}
		socket.send(json.dumps(message))
		data_mapping = json.loads(socket.recv())
		if 'status' in data_mapping:
			return data_mapping['status']
		elif "error" in data_mapping:
			raise ChoirConfigError(data_mapping["error"])
		else:
			raise ChoirConfigError("No response from Middleware.")

	def config_stop(self, service_name):
		"""
		Stop a service.
		Raise a ChoirConfigError if the service does not exist or if the stop failed.

		@param service_name : name of the target service.
		@type service_name : string.
		"""
		socket = create_connection("ws://{}:{}/config".format(self.ip, self.port))
		message = {"command":"config-stop","service-name":service_name}
		socket.send(json.dumps(message))
		data_mapping = json.loads(socket.recv())
		if 'started' in data_mapping and data_mapping['started']:
			raise ChoirConfigError("Failed to stop service {}".format(service_name))
		elif "error" in data_mapping:
			raise ChoirConfigError(data_mapping["error"])

	def config_start(self, service_name):
		"""
		Start a service.
		Raise a ChoirConfigError if the service does not exist.

		@param service_name : name of the target service.
		@type service_name : string.
		"""
		socket = create_connection("ws://{}:{}/config".format(self.ip, self.port))
		message = {"command":"config-start","service-name":service_name}
		socket.send(json.dumps(message))
		data_mapping = json.loads(socket.recv())
		if 'status' in data_mapping and data_mapping['status'] != "stopped":
			raise ChoirConfigError("Failed to start service {}".format(service_name))
		elif "error" in data_mapping:
			raise ChoirConfigError(data_mapping["error"])

	def config_restart(self, service_name):
		"""
		Restart a service.
		Raise a ChoirConfigError if the service does not exist.

		@param service_name : name of the target service.
		@type service_name : string.
		"""
		socket = create_connection("ws://{}:{}/config".format(self.ip, self.port))
		message = {"command":"config-restart","service-name":service_name}
		socket.send(json.dumps(message))
		data_mapping = json.loads(socket.recv())
		if 'status' in data_mapping and data_mapping['status'] != "started":
			raise ChoirConfigError("Failed to start service {}".format(service_name))
		elif "error" in data_mapping:
			raise ChoirConfigError(data_mapping["error"])

	def config_mib_get_all(self, service_name, offline = False):
		"""
		Get the full mib of a service.
		Raise a ChoirConfigError if the service does not exist or if no mib available.

		@param service_name : name of the target service.
		@type service_name : string.

		@param offline : offline mode.
		@type offline : boolean.

		@return : The full mib.
		@rtype : {String:value}
		"""
		socket = create_connection("ws://{}:{}/config".format(self.ip, self.port))
		message = {"command":"mib-getall","service-name":service_name ,"offline" : str(offline).lower()}
		socket.send(json.dumps(message))
		data_mapping = json.loads(socket.recv())
		if "error" in data_mapping:
			raise ChoirConfigError(data_mapping["error"])
		elif "mib" not in data_mapping:
			raise ChoirConfigError("No mib available for service {}".format(service_name))
		return (data_mapping["mib"])

	def config_mib_get(self, service_name, keys, offline = False):
		"""
		Get specified mib entries. All keys not present in mib are ignored.
		Raise a ChoirConfigError if the service does not exist or if no mib available.

		@param service_name : name of the target service.
		@type service_name : string.

		@param keys : list of entries to get.
		@type keys : [string]

		@param offline : offline mode.
		@type offline : boolean.

		@return : The full mib.
		@rtype : {String:value}
		"""
		socket = create_connection("ws://{}:{}/config".format(self.ip, self.port))
		message = {"command":"mib-get", "service-name":service_name , "offline" : str(offline).lower(), "keys": keys}
		socket.send(json.dumps(message))
		data_mapping = json.loads(socket.recv())
		if "error" in data_mapping:
			raise ChoirConfigError(data_mapping["error"])
		elif "mib" not in data_mapping:
			raise ChoirConfigError("No mib available for service {}".format(service_name))
		return (data_mapping["mib"])

	def config_mib_set(self, service_name, values_to_set, offline = False):
		"""
		Set specified mib entries.
		Raise a ChoirConfigError if the service does not exist.

		@param service_name : name of the target service.
		@type service_name : string.

		@param values_to_set : Mapping keys->values to set in the mib.
		@type values_to_set : {string:value}

		@param offline : offline mode.
		@type offline : boolean.
		"""
		socket = create_connection("ws://{}:{}/config".format(self.ip, self.port))
		message = {"command":"mib-set", "service-name":service_name , "offline" : str(offline).lower(), "mib": values_to_set}
		socket.send(json.dumps(message))
		data_mapping = json.loads(socket.recv())
		if "error" in data_mapping:
			raise ChoirConfigError(data_mapping["error"])

	def config_mib_reset(self, service_name, offline = False):
		"""
		Reset the full mib of a service.
		Raise a ChoirConfigError if the service does not exist.

		@param service_name : name of the target service.
		@type service_name : string.

		@param offline : offline mode.
		@type offline : boolean.
		"""
		socket = create_connection("ws://{}:{}/config".format(self.ip, self.port))
		message = {"command":"mib-reset", "service-name":service_name , "offline" : str(offline).lower()}
		socket.send(json.dumps(message))
		data_mapping = json.loads(socket.recv())
		if "error" in data_mapping:
			raise ChoirConfigError(data_mapping["error"])

	def mwversion(self):
		"""
		Return the middleware version

		@return : The version.
		@rtype : String
		"""
		socket = create_connection("ws://{}:{}/version".format(self.ip, self.port))
		return socket.recv()


class StoppableThread(Thread):
	def __init__(self):
		Thread.__init__(self)
		self.must_stop = False

	def stop(self):
		self.must_stop = True

class SubscribeThread(StoppableThread):
	def __init__(self, msg_id, received_message_callback, ip, port, app_id, format):
		StoppableThread.__init__(self)
		self.msg_id = msg_id
		self.received_message_callback = received_message_callback
		self.ip = ip
		self.port = port
		self.app_id = app_id
		self.format = format
		self.url = "ws://{}:{}/pubsub/sub/{}/{}{}".format(self.ip, self.port, self.msg_id, self.app_id, self.format)


	def run(self):
		# Create connection to the middleware.
		socket = create_connection(self.url, timeout=1)
		# Loop on receive.
		while True:
			try:
				received_message = socket.recv()
				if received_message != "__pong__":
					self.received_message_callback(received_message)
			except Exception as ex:
				socket.send("__ping__")
			# If an unsubscribe has been asked.
			if self.must_stop:
				break
			# If connection is lost, try to re-connect.
			if not socket.connected:
				try:
					socket = create_connection(self.url, timeout=1)
				# If no connection, wait.
				except:
					time.sleep(1)


class RegisterThread(StoppableThread):
	def __init__(self, cache_id, object_id, cache_update_callback, ip, port, app_id):
		StoppableThread.__init__(self)
		self.cache_id = cache_id
		self.object_id = object_id
		self.cache_update_callback = cache_update_callback
		self.ip = ip
		self.port = port
		self.app_id = app_id
		self.url = "ws://{}:{}/cache/register/{}/{}/{}".format(self.ip, self.port, self.cache_id, self.object_id, self.app_id)

	def getCacheDiff(self, current, new):
		diffKeys = []
		currentKeysSet = set(current.keys())
		newKeysSet = set(new.keys())

		diffKeys.extend(currentKeysSet.symmetric_difference(newKeysSet))
		for key in currentKeysSet.intersection(newKeysSet):
			if current[key] != new[key]:
				diffKeys.append(key)
		return diffKeys

	def run(self):
		socket = create_connection(self.url, timeout=1)
		current_cache = None
		diff_keys = []
		while True:
			try:
				received_message = socket.recv()
				if received_message != "__pong__":
					new_cache = json.loads(received_message)
					del new_cache["updateType"]
					if not current_cache:
						current_cache = new_cache
					else:
						diff_keys = self.getCacheDiff(current_cache, new_cache)
						current_cache = new_cache
					self.cache_update_callback(diff_keys, current_cache)
			except Exception as ex:
				socket.send("__ping__")
			# If an unregister has been asked.
			if self.must_stop:
				break
			# If connection is lost, try to re-connect.
			if not socket.connected:
				try:
					socket = create_connection(self.url, timeout=1)
				# If no connection, wait.
				except:
					time.sleep(1)

class ChoirConfigError(Exception):
	""" Raised when an error is returned from a choir config access"""
	def __init__(self, message):
		super().__init__(message)
