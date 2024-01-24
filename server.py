from flask import Flask, jsonify, request
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS
import paho.mqtt.client as mqtt
import json


app = Flask(__name__)


# InfluxDB Configuration
token = "siBpzLCX7_nCo7446-GCfViJhp6noh2rAw4FnpoZUTkD3WOvyf41bhS_TRhVN1PqpO-Noz1hZ1J0c7ls-RvvDA=="
org = "FTN"
url = "http://localhost:8086"
bucket = "IOT"
influxdb_client = InfluxDBClient(url=url, token=token, org=org)

def save_to_db(data):
    print(data)
    write_api = influxdb_client.write_api(write_options=SYNCHRONOUS)
    point = (
        Point(data["title"])
        .field("pin", data["pin"])
    )
    write_api.write(bucket=bucket, org=org, record=point)



# MQTT Configuration
mqtt_client = mqtt.Client()
mqtt_client.username_pw_set("admin", "12345678")

mqtt_client.on_connect = lambda client, userdata, flags, rc: client.subscribe("tries")
mqtt_client.on_message = lambda client, userdata, msg: save_to_db(json.loads(msg.payload.decode('utf-8')))

mqtt_client.connect("192.168.43.75", 1883, 60)
mqtt_client.loop_start()

if __name__ == '__main__':
    app.run()