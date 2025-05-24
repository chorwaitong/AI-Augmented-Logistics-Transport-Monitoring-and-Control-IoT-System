# app.py
from flask import Flask, request, render_template, jsonify
from flask_socketio import SocketIO
from langchain_core.output_parsers import JsonOutputParser
from langchain_google_genai import ChatGoogleGenerativeAI
import paho.mqtt.client as mqtt
import serial
import serial.tools.list_ports
import threading
import time

app = Flask(__name__)
socketio = SocketIO(app)
mqtt_topic_servo = "r4_cwt/servo"

llm = ChatGoogleGenerativeAI(
        model="gemini-2.0-flash",
        max_tokens=None,
        timeout=None,
        max_retries=2,               
    )
chainJSON = llm | JsonOutputParser()  
print ('LLM initialized')

# Configure the serial port
# SERIAL_PORT = 'COM3'
# BAUD_RATE = 115200
def on_connect(client, userdata, flags, reason_code, properties):
    if client.is_connected():
        print("Connection succesfull: reason code {}, fkags {}\n".format(reason_code, flags))
        client.subscribe("r4_cwt/temperature")
        client.subscribe("r4_cwt/humidity")
        client.subscribe("r4_cwt/dist")
        client.subscribe("r4_cwt/nfc")
        client.subscribe("r4_cwt/servo")
    else:
        print("Did not connect: reason code {}, flags {}\n".format(reason_code, flags))
        
def on_message(client, userdata, msg):
    payload = msg.payload
    print("Received {} - {}".format(msg.topic, payload))
    try:
        if msg.topic == "r4_cwt/temperature":
            socketio.emit("update_sensor", {"sensor": "temperature", "value": float(payload)})
        elif msg.topic == "r4_cwt/humidity":
            socketio.emit("update_sensor", {"sensor": "humidity", "value": float(payload)})
        elif msg.topic == "r4_cwt/dist":
            socketio.emit("update_sensor", {"sensor": "distance", "value": float(payload)})
        elif msg.topic == "r4_cwt/nfc":
            nfc_uid = ' '.join(f'{x:02X}' for x in payload)
            socketio.emit("update_nfc", nfc_uid)
    except Exception as e:
        print ('Exception occured when receiving message: {}'.format(e))
    

mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message

@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('request_update')
def getAIStatusUpdate(data):
    promptAI = f"**Instruction** Consider the following condition in a transportation vehicle, \n\n **Conditions** \n {data['data']}"
    promptAI += '''\n\n You should provide a short status update to the driver, so that the driver understands the condition of the goods
                being currently transported. Your response should be strictly in JSON format. A sample of the response is {"response": your_response}.'''
    print(f"Received request to obtain status update considering: {data['data']}")
    try:
        ai_msg = chainJSON.invoke(promptAI)
        if 'response' in ai_msg:
            ai_msg['error'] = 0
            socketio.emit('received_update', ai_msg)
        else:
            ai_msg['error'] = 1
            ai_msg['error_msg'] = f"Invalid json format, got {ai_msg}"
            socketio.emit('received_update', ai_msg)
    except Exception as e:
        ai_msg['error'] = 2
        ai_msg['error_msg'] = f"Exception occured, {e}"
        socketio.emit('received_update', ai_msg)
   

@socketio.on('transport_goods')
def getAIRecommendations(data):
    promptAIenv = f"**Instruction** Considering a goods transportation for goods of type {data['goods_type']}, "
    promptAIenv += '''provide the recommended temperature (in celcius) and the humidity, 
            strictly in JSON format. A sample of the response is {"temp": 21, "hum":60}.'''
    print(f"Received request to obtain recommended environment data for: {data}")
    try:
        ai_msg = chainJSON.invoke(promptAIenv)
        if 'temp' in ai_msg and 'hum' in ai_msg:
            ai_msg['error'] = 0
            socketio.emit('ai_recommendation', ai_msg)
        else:
            ai_msg['error'] = 1
            ai_msg['error_msg'] = f"Invalid json format, got {ai_msg}"
            socketio.emit('ai_recommendation', ai_msg)
    except Exception as e:
        ai_msg['error'] = 2
        ai_msg['error_msg'] = f"Exception occured, {e}"
        socketio.emit('ai_recommendation', ai_msg)
   
@socketio.on('servoControl')
def handle_servo_control(data):
    """
    This function is triggered when the frontend sends a signal to toggle the servo.
    """
    print(f"Servo control command received: {data}")
    if 'currentKnobAngle' in data:
        mqtt_client.publish(mqtt_topic_servo, data['currentKnobAngle'])

if __name__ == '__main__':
    print("Starting MQTT")
    mqtt_client.connect("broker.emqx.io",1883)
    mqtt_client.loop_start()  
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)
