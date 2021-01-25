import paho.mqtt.client as mqtt
from drivers.sds011 import SDS011_MODEL, MOBILE_GPS
from stations import *
import nacl.signing
import threading
import time
import rospy
import json
import copy


thlock = threading.RLock()
sessions = dict()


def _generate_pubkey() -> str:
    signing_key = nacl.signing.SigningKey.generate()
    verify_key = signing_key.verify_key
    verify_key_hex = bytes(verify_key).hex()
    return str(verify_key_hex)

class MQTTHandler(mqtt.Client):
    def __init__(self, host: str, port: int):
        threading.Thread.__init__(self)
        mqtt.Client.__init__(self)
        self.host = host
        self.port = port

    def on_connect(self, client, obj, flags, rc):
        rospy.loginfo(f'Connected to mqtt with result code {str(rc)} ')

    def _parser(self, data: dict) -> Measurement: 
        global sessions
        global thlock
        try:
            if 'esp8266id' in data.keys():
                    self.client_id = int(data["esp8266id"])
                    temperature = None
                    pressure = None
                    humidity = None
                    for d in data["sensordatavalues"]:
                        if d["value_type"] == "SDS_P1":
                            pm10 = float(d["value"])
                        if d["value_type"] == "SDS_P2":
                            pm25 = float(d["value"])
                        if d["value_type"] == "GPS_lat":
                            geo_lat = d["value"]
                        if d["value_type"] == "GPS_lon":
                            geo_lon = d["value"]

                        if d["value_type"] == "BME280_temperature" or d["value_type"] == "HTU21D_temperature":
                            temperature = float(d["value"])
                        if d["value_type"] == "BME280_pressure":
                            pressure = float(d["value"])
                        if d["value_type"] == "BME280_humidity" or d["value_type"] == "HTU21D_humidity":
                            humidity = float(d["value"])

                    meas = {}
                    meas.update({'pm10': pm10, 'pm25': pm25, 'temperature': temperature, 'pressure': pressure, 'humidity': humidity})

            timestamp = int(time.time())
            meas.update({'timestamp': timestamp})
            with thlock:
                if self.client_id not in sessions:
                    public = _generate_pubkey()
                else:
                    public = sessions[self.client_id].public
            
            measurement = Measurement(public,
                                     SDS011_MODEL,
                                     geo_lat,
                                     geo_lon,
                                     meas)

            rospy.loginfo(f'measurment: {measurement}')
        except Exception as e:
            rospy.logerr(e)
            return
        return measurement

    def on_message(self, client, userdata, msg: dict):
        global thlock
        global sessions
       
        data = json.loads(msg.payload.decode())

        print(f'msg.payload {msg.payload}')
        print(data)
        parse_data = self._parser(data)
        print(f'parse {parse_data}')
        with thlock:
            if parse_data:
                sessions[self.client_id] = parse_data

    def on_subscribe(self, client, userdata, mid, granted_qos):
        rospy.loginfo(f'Subscribed {str(mid)}, client {client}')

    def run(self):
        self.connect(self.host, self.port, 60)
        #self.subscribe("$SYS/#")
        self.subscribe("sensors", 0)
        rc = 0
        while rc == 0:
            rc = self.loop()

        return rc
    


class MQTTStation(IStation):
    def __init__(self, config: dict):
        super().__init__(config)
        self.version = f"airalab-http-{STATION_VERSION}"
        self.DEAD_SENSOR_TIME = 60*60 # 1 hour
        host = config["mqttstation"]["host"]
        port = int(config["mqttstation"]["port"])
        #MQTTHandler(host, port).run()
        client = MQTTHandler(host, port)
        rc = client.run()
    
    
    def get_data(self) -> StationData:
        global sessions
        result = []
        for k, v in self._drop_dead_sensors().items():
            result.append(StationData(
                self.version,
                self.mac_address,
                time.time() - self.start_time,
                v
            ))
        rospy.loginfo(f'result {result}')
        return result


    def _drop_dead_sensors(self) -> dict:
        global thlock
        global sessions
        stripped = dict()
        current_time = int(time.time())
        with thlock:
            sessions_copy = copy.deepcopy(sessions)
            for k, v in sessions_copy.items():
                if (current_time - v.measurement["timestamp"]) < self.DEAD_SENSOR_TIME:
                    stripped[k] = v
                else:
                    del sessions[k]

        return stripped
