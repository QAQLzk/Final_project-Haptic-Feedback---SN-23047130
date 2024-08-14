#import os
#import time
#import threading
#import serial
#from keras.models import load_model
#import joblib
## Load model
##file_path = os.path.join(os.path.dirname(__file__), '../../data collection_2 (Z)/Test/Z_best_model.h5')
#file_path = os.path.join(os.path.dirname(__file__), '../../data collection_2 (Z)/Test/linear_regression_model.pkl')
#if os.path.exists(file_path):
#    #model = load_model(file_path)
#    model = joblib.load(file_path)
#else:
#    raise FileNotFoundError("Model file not found.")


#latest_serial_data = None
#data_lock = threading.Lock()
#Z_force = 0

## Serial setup
#serial_port = 'COM3'
#baud_rate = 115200
#ser = serial.Serial(serial_port, baud_rate, timeout=0.1)

#def read_serial_data():
#    global latest_serial_data
#    while True:
#        try:
#            line = ser.readline().decode('utf-8').strip()
#            if line and line.startswith('X:'):
#                line = line.replace(' uT', '').replace('\\', '').strip()
#                parts = line.split()
#                if len(parts) >= 6:
#                    x = float(parts[1])
#                    y = float(parts[3])
#                    z = float(parts[5])
#                    with data_lock:
#                        latest_serial_data = [x, y, z]  
#        except Exception as e:
#            print(f"ERROR: {e}")
#        time.sleep(0.001)  

#def prediction_thread():
#    global Z_force
#    while True:
#        start_time = time.perf_counter()
#        with data_lock:
#            if latest_serial_data:
#                z = latest_serial_data[2]
#                input_data = [[z]]
#                Z_predicted_force = model.predict(input_data)
#                #Z_force = Z_predicted_force[0][0] * 0.0098 * 0.5
#                Z_force = Z_predicted_force[0] * 0.0098 * 0.5
#                print(f"Z_force: {Z_force}")
        
#        # Calculate the prediciton time
#        pred_end_time = time.perf_counter()
#        prediction_time = pred_end_time - start_time
#        print(f"Time from read to prediction: {prediction_time:.6f} seconds")
        
#        time.sleep(0.001) 


#serial_thread = threading.Thread(target=read_serial_data, daemon=True)
#serial_thread.start()

#predict_thread = threading.Thread(target=prediction_thread, daemon=True)
#predict_thread.start()

## 主循环，保持程序运行
#try:
#    while True:
#        time.sleep(1)  
#except KeyboardInterrupt:
#    print("Program interrupted and exiting.")












###################################33

import os
import time
import threading
from keras.models import load_model
import joblib
import random
import cProfile
import numpy as np
import tensorflow as tf
import tensorflow.lite as tflite

# Ensure GPU is used if available
physical_devices = tf.config.list_physical_devices('GPU')
if physical_devices:
    try:
        tf.config.experimental.set_memory_growth(physical_devices[0], True)
    except:
        pass

# Load model
file_path = os.path.join(os.path.dirname(__file__), '../../data collection_2 (Z)/Test/Z_best_model.h5')

if os.path.exists(file_path):
    model = load_model(file_path)
else:
    raise FileNotFoundError("Model file not found.")

latest_serial_data = None
data_lock = threading.Lock()
Z_force = 0

# Convert model to TensorFlow Lite
converter = tflite.TFLiteConverter.from_keras_model(model)
tflite_model = converter.convert()

# Save the model
tflite_model_path = 'model.tflite'
with open(tflite_model_path, 'wb') as f:
    f.write(tflite_model)

# Load TFLite model and allocate tensors
interpreter = tflite.Interpreter(model_path=tflite_model_path)
interpreter.allocate_tensors()

# Get input and output tensors
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

def read_serial_data():
    global latest_serial_data
    while True:
        try:
            x = random.uniform(1, 100)
            y = random.uniform(1, 100)
            z = random.uniform(1, 100)
            with data_lock:
                latest_serial_data = [x, y, z]
        except Exception as e:
            print(f"ERROR: {e}")
        time.sleep(0.001)

def prediction_thread():
    global Z_force
    while True:
        start_time = time.perf_counter()
        with data_lock:
            if latest_serial_data:
                z = latest_serial_data[2]
                input_data = np.array([[z]], dtype=np.float32)
                interpreter.set_tensor(input_details[0]['index'], input_data)
                interpreter.invoke()
                Z_predicted_force = interpreter.get_tensor(output_details[0]['index'])
                Z_force = Z_predicted_force[0][0] * 0.0098 * 0.5

                print(f"Z_force: {Z_force}")
        
        pred_end_time = time.perf_counter()
        prediction_time = pred_end_time - start_time
        print(f"Time from read to prediction: {prediction_time:.6f} seconds")
        
        time.sleep(0.001)

def main():
    serial_thread = threading.Thread(target=read_serial_data, daemon=True)
    serial_thread.start()

    predict_thread = threading.Thread(target=prediction_thread, daemon=True)
    predict_thread.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Program interrupted and exiting.")

if __name__ == "__main__":
    profiler = cProfile.Profile()
    profiler.enable()
    
    main()
    
    profiler.disable()
    profiler.print_stats(sort='time')

