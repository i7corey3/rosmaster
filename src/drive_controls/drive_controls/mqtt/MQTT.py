import paho.mqtt.client as mqtt
import threading
import os


class MQTT():
    def __init__(self, MQTT_server, port=1883):
        self.username = 'advent'
        self.password = 'Advent400!@#'
        self.port = port
        self.MQTT_server = MQTT_server
        self.MQTT_Message = {}
        self.MQTT_Clients = {}
        self.stay_disconnected = False
        self.disconnect = False

    def createSender(self, name, sub):  ## Send

        def clientConnection(client, userdata, flags, rc):
            if rc == 0:
                self.MQTT_Clients[name].subscribe(sub)
            else:
                print("Failed to Connect")

        def on_disconnect(client, userdata, rc):
            self.MQTT_Clients[name].loop_stop()
            print(f"{name} Disconnect")
            self.disconnect = True

        self.MQTT_Clients[name] = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, name)
        self.MQTT_Clients[name].tls_set(ca_certs=f'{os.path.dirname(__file__)}/emqxsl-ca.crt')
        self.MQTT_Clients[name].username_pw_set(self.username, self.password)
        self.MQTT_Clients[name].on_connect = clientConnection
        self.MQTT_Clients[name].on_disconnect = on_disconnect
        self.MQTT_Clients[name].connect(self.MQTT_server, port=self.port)
        self.MQTT_Clients[name].loop_start()

    def startClient(self, name, sub):  ## Listen

        self.MQTT_Message[sub] = []

        def connectionStatus(client, userdata, flags, rc):
            if rc == 0:
                self.MQTT_Clients[name].subscribe(sub)
            else:
                print("Failed to Connect")

        def messageDecoder(client, userdata, msg):
            message = msg.payload
            # print(message)
            self.MQTT_Message[sub] = message

        def on_disconnect(client, userdata, rc):
            self.MQTT_Clients[name].loop_stop()
            print(f"{name} Disconnect")
            self.disconnect = True

        while True:
            try:
                if not self.stay_disconnected:
                    self.MQTT_Clients[name] = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, name)
                    self.MQTT_Clients[name].tls_set(ca_certs=f'{os.path.dirname(__file__)}/emqxsl-ca.crt')
                    self.MQTT_Clients[name].username_pw_set(self.username, self.password)
                    self.MQTT_Clients[name].on_connect = connectionStatus
                    self.MQTT_Clients[name].on_message = messageDecoder
                    self.MQTT_Clients[name].on_disconnect = on_disconnect
                    self.MQTT_Clients[name].connect(self.MQTT_server, port=self.port)
                    self.MQTT_Clients[name].loop_forever()

            except TypeError:
                pass

    def send(self, name, sub, message, qos=0):
        self.MQTT_Clients[name].publish(sub, message, qos=qos)

    def createListener(self, name, sub):
        threading.Thread(target=self.startClient, args=(name, sub,), daemon=True).start()


class OldMQTT():
    def __init__(self, MQTT_server, port=1883, username=None, password=None):
        
        self.port = port
        self.password=password
        self.username = username
        self.MQTT_server = MQTT_server
        self.MQTT_Message = {}
        self.MQTT_Clients = {}
        self.stay_disconnected = False
        self.disconnect = False

    def createSender(self, name, sub):  ## Send

        def clientConnection(client, userdata, flags, rc):
            if rc == 0:
                self.MQTT_Clients[name].subscribe(sub)
            else:
                print("Failed to Connect")

        def on_disconnect(client, userdata, rc):
            self.MQTT_Clients[name].loop_stop()
            print(f"{name} Disconnect")
            self.disconnect = True

        self.MQTT_Clients[name] = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, name)
        if self.username is not None and self.password is not None:
            self.MQTT_Clients[name].username_pw_set(self.username, self.password)
        self.MQTT_Clients[name].on_connect = clientConnection
        self.MQTT_Clients[name].on_disconnect = on_disconnect
        self.MQTT_Clients[name].connect(self.MQTT_server, port=self.port)
        self.MQTT_Clients[name].loop_start()

    def startClient(self, name, sub):  ## Listen

        self.MQTT_Message[sub] = []

        def connectionStatus(client, userdata, flags, rc):
            if rc == 0:
                self.MQTT_Clients[name].subscribe(sub)
            else:
                print("Failed to Connect")

        def messageDecoder(client, userdata, msg):
            message = msg.payload
            # print(message)
            self.MQTT_Message[sub] = message.decode()

        def on_disconnect(client, userdata, rc):
            self.MQTT_Clients[name].loop_stop()
            print(f"{name} Disconnect")
            self.disconnect = True

        while True:
            try:
                if not self.stay_disconnected:
                    self.MQTT_Clients[name] = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, name)
                    if self.username is not None and self.password is not None:
                        self.MQTT_Clients[name].username_pw_set(self.username, self.password)
                    self.MQTT_Clients[name].on_connect = connectionStatus
                    self.MQTT_Clients[name].on_message = messageDecoder
                    self.MQTT_Clients[name].on_disconnect = on_disconnect
                    self.MQTT_Clients[name].connect(self.MQTT_server, port=self.port)
                    self.MQTT_Clients[name].loop_forever()

            except TypeError:
                pass

    def send(self, name, sub, message, qos=0):
        self.MQTT_Clients[name].publish(sub, message, qos=qos)

    def createListener(self, name, sub):
        threading.Thread(target=self.startClient, args=(name, sub,), daemon=True).start()

    