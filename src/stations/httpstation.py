# -*- coding: utf-8 -*-
from flask import Flask
from flask import request
from flask import jsonify
import time 
#from drivers.sds011 import SDS011_MODEL

def get_data(python_data):
    if 'sensordatavalues' in python_data:
        for dict in python_data['sensordatavalues']:
            if dict['value_type'] == 'SDS_P1':
                pm10 = dict['value']
            if dict['value_type'] == 'SDS_P2':
                pm25 = dict['value']
        public = '655a40d4b951b4fbc0c9a7658e66377f1a5ff92111f10145256e0026ab07a669'
        geo_lon = 30.3350986
        geo_lat = 59.9342802
        timestamp = int(time.time())
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

app = Flask(__name__)
@app.route('/', methods = ['GET', 'POST'])
def getting_data():
    print('data is coming!')
    if(request.method == 'POST'):
        data = request.get_json() #for content with "application/json" headers
        get_data(data)
        print(get_data(data))
        return jsonify(data)

    else:
        return 'no data'



if __name__ == '__main__':
    app.debug = True
    app.run(host='0.0.0.0')




