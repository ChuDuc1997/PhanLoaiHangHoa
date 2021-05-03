import io
import time
import cv2
import serial
import sqlite3
import argparse
import numpy as np
from picamera import PiCamera
from picamera.array import PiRGBArray
from tflite_runtime.interpreter import Interpreter
from flask import Flask, render_template, Response, request

def shutdown_server():
    func = request.environ.get('werkzeug.server.shutdown')
    if func is None:
        raise RuntimeError('Not running with the Werkzeug Server')
    func()
    
def log_values(product):
    db = sqlite3.connect("/home/pi/PhanLoaiHangHoa/Tensorflow/RaspberryPi/mydb.db")
    cur = db.cursor()
    cur.execute("""INSERT INTO products(product, datetime) VALUES((?), DATETIME(CURRENT_TIMESTAMP, 'localtime'))""", (product,))
    db.commit()
    db.close()
    
def get_data():
    db = sqlite3.connect("/home/pi/PhanLoaiHangHoa/Tensorflow/RaspberryPi/mydb.db")
    cur = db.cursor()
    cur.execute("SELECT * FROM products")
    data = cur.fetchall()
    db.close()
    return data

def load_labels(path):
  with open(path, 'r') as f:
    return {i: line.strip() for i, line in enumerate(f.readlines())}


def set_input_tensor(interpreter, image):
  tensor_index = interpreter.get_input_details()[0]['index']
  input_tensor = interpreter.tensor(tensor_index)()[0]
  input_tensor[:, :] = image


def classify_image(interpreter, image, top_k=1):
  """Returns a sorted array of classification results."""
  set_input_tensor(interpreter, image)
  interpreter.invoke()
  output_details = interpreter.get_output_details()[0]
  output = np.squeeze(interpreter.get_tensor(output_details['index']))

  if output_details['dtype'] == np.uint8:
    scale, zero_point = output_details['quantization']
    output = scale * (output - zero_point)

  ordered = np.argpartition(-output, top_k)
  return [(i, output[i]) for i in ordered[:top_k]]

parser = argparse.ArgumentParser(
      formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument(
      '--model', help='File path of .tflite file.', required=True)
parser.add_argument(
      '--labels', help='File path of labels file.', required=True)
args = parser.parse_args()

labels = load_labels(args.labels)

interpreter = Interpreter(args.model)
interpreter.allocate_tensors()
_, height, width, _ = interpreter.get_input_details()[0]['shape']

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
ser.flush()

def gen():
  camera = PiCamera(resolution=(640, 480), framerate=30)
  stream = PiRGBArray(camera, size=(640, 480))
  global flag
  flag = False
  time.sleep(0.1)

  for frame in camera.capture_continuous(stream, format="bgr", use_video_port=True):
    if ser.in_waiting > 0:
        read_serial=ser.readline().decode('utf-8').rstrip()
        if(read_serial == "BOX:"):
            #print("BOX: ", end='')
            flag = True
    
    image = frame.array
    if(flag): 
        image_resize = cv2.resize(image, (width, height))
        results = classify_image(interpreter, image_resize)
        label_id, prob = results[0]
        if(label_id != 3) and (prob >= 0.7):
            text = '%s: %.0f' % (labels[label_id], prob*100) + '%'
            cv2.putText(image, text, (230,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2, cv2.LINE_AA)
            log_values(labels[label_id])
            ser.write(str(label_id + 1).encode('utf-8'))
            #print(labels[label_id])
            flag = False
    
    jpeg = (cv2.imencode('.jpg', image))[1].tobytes()
    stream.truncate(0)

    if (cv2.waitKey(1) & 0xFF) == ord("q"):
      break

    yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + jpeg + b'\r\n\r\n')

app = Flask(__name__)

@app.route("/")
def index():
    return render_template('index.html')

@app.route("/VIDEO")
def video():
    return render_template('video.html')

@app.route("/RUN")
def run():
    ser.write(b"RUN\n")
    gen()
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(gen(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/OFF")
def off():
    ser.write(b"STOP\n")
    shutdown_server()
    return render_template('index.html')

@app.route("/DATABASE")
def database():
    data = get_data()
    return render_template('database.html', data=data)

if __name__=="__main__":
    app.run(host='0.0.0.0', debug=True, threaded=True)
