# -*- coding: utf-8 -*-
import asyncio
from asyncio import StreamReader, StreamWriter
import threading
import time
import rospy
import time 
from drivers.sds011 import SDS011_MODEL

from stations import IStation, StationData, Measurement, STATION_VERSION
from collections import deque
from http.server import BaseHTTPRequestHandler, HTTPServer

class RequestHandler(BaseHTTPRequestHandler):

    def _set_headers(self):
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.end_headers()
        
    def do_HEAD(self):
        self._set_headers()
    
    def do_GET(self):
        self._set_headers()
        self.wfile.write(json.dumps({'hello': 'world', 'received': 'ok'}))

    def do_POST(self, q: deque):
        
        rospy.loginfo("data is coming!")
        ctype, pdict = cgi.parse_header(self.headers.getheader('content-type'))
        length = int(self.headers.get('content-length'))
        self.data = json.loads(self.rfile.read(length))
        #meas = self.parser(data)
        #self.q.append(meas)
        self._set_headers()
        self.wfile.write(json.dumps(message))
        return data



class HTTPserver(threading.Thread):
    def __init__(self, q: deque):
        threading.Thread.__init__(self)
        #BaseHTTPRequestHandler.__init__(self)
        self.q = q
        #self.data = data

    def parser(self, data):
        for dict in data['sensordatavalues']:
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
        rospy.loginfo('run func')
        self.server_address = ('', 8001)
        self.httpd = HTTPServer(self.server_address, RequestHandler)
        rospy.loginfo('Starting httpd')
        self.httpd.serve_forever()


class HTTPStation(IStation):

    def __init__(self, config: dict):
        super().__init__(config)
        self.q = deque(maxlen=1)
        HTTPserver(self.q).start()
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






