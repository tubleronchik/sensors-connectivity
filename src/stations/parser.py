# -*- coding: utf-8 -*-
import json
import time
#from drivers.sds011 import SDS011_MODEL

def get_data():
    with open("new.json") as f:
        python_data = json.load(f)
    if 'sensordatavalues' in python_data:
        for dict in python_data['sensordatavalues']:
            if dict['value_type'] == 'SDS_P1':
                pm10 = dict['value']
            if dict['value_type'] == 'SDS_P2':
                pm25 = dict['value']
        public = '655a40d4b951b4fbc0c9a7658e66377f1a5ff92111f10145256e0026ab07a669'
        geo_lon = 30.3350986
        geo_lat = 59.9342802
        timestamp = 12
        SDS011_MODEL = 11
        measurement = {
            "public": public,
            "model": SDS011_MODEL,
            "pm25": pm25,
            "pm10": pm10,
            "geo_lat": geo_lat,
            "geo_lon": geo_lon,
            "timestamp": timestamp
        }
    else:
        return measurement
    return measurement


    

print(get_data())




