# -*- coding: utf-8 -*-
import asyncio
from asyncio import StreamReader, StreamWriter
import threading
import time
import rospy
from flask import Flask
from flask import request
from flask import jsonify
import time 
from drivers.sds011 import SDS011_MODEL

from stations import IStation, StationData, Measurement, STATION_VERSION
from collections import deque


app = Flask(__name__)


class HTTPServer(threading.Thread):
    def __init__(self, q: deque):
        threading.Thread.__init__(self)
        self.q = q

    @app.route('/', methods = ['GET', 'POST'])

    def getting_data(self, q: deque):
        print('data is coming!')
        
        if(request.method == 'POST'):
            data = request.get_json()
            meas = self.parser(data)
            self.q.append(meas)
            return jsonify(data)

    def parser(self, data):
        for dict in python_data['sensordatavalues']:
            if dict['value_type'] == 'SDS_P1':
                pm10 = dict['value']
            if dict['value_type'] == 'SDS_P2':
                pm25 = dict['value']
        public = '655a40d4b951b4fbc0c9a7658e66377f1a5ff92111f10145256e0026ab07a669'
        geo_lon = 30.3350986
        geo_lat = 59.9342802
        timestamp = int(time.time())
        measurement = Measurement(public,
                                  SDS011_MODEL,
                                  pm25,
                                  pm10,
                                  geo_lat,
                                  geo_lon,
                                  timestamp)
        return measurement
    
    def run(self):
        app.debug = False # otherwise problem with pyhton3.8
        app.run(host='0.0.0.0')


class HTTPStation(IStation):

    def __init__(self, config: dict):
        super().__init__(config)
        self.q = deque(maxlen=1)
        HTTPServer(self.q).start()
        self.version = f"airalab-com-{STATION_VERSION}"

    def get_data(self):
        if self.q:
            value = self.q[0]
        else:
            value = Measurement()
        return [StationData(
            self.version,
            self.mac_address,
            time.time() - self.start_time,
            value
        )]






