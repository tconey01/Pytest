# Flask web app with PyBullet simulation and real-time video streaming
from flask import Flask, render_template
from flask_socketio import SocketIO
import pybullet as p
import pybullet_data
import cv2
import numpy as np
import time
from threading import Thread

# Initialize Flask and WebSocket communication
app = Flask(__name__)
socketio = SocketIO(app, async_mode='threading', cors_allowed_origins='*')

# Set up PyBullet simulation environment
physicsClient = p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.setTimeStep(1 / 240)

# Load plane and URDF models (ball-beam and ball)
planeId = p.loadURDF("plane.urdf")
ballbeam = p.loadURDF("resources/ballbeam/ballbeam.urdf", [0, 0, 0.5], useFixedBase=True)
ball = p.loadURDF("resources/ball/ball.urdf", [0, 0, 1])

# Configure camera settings
width, height = 640, 360
view_matrix = p.computeViewMatrix([0, 1.2, 0.6], [0, 0, 0.5], [0, 0, 1])
proj_matrix = p.computeProjectionMatrixFOV(60, width / height, 0.1, 100)

# Initialize beam control parameters
beam_angle = 0
angle_step = 0.02

def run_simulation():
    """Continuously update the beam angle in the PyBullet simulation."""
    global beam_angle
    last_time = time.time()
    while True:
        elapsed = time.time() - last_time
        last_time = time.time()

        # Adjust beam angle and step the simulation
        p.setJointMotorControl2(ballbeam, 1, p.POSITION_CONTROL, targetPosition=beam_angle, force=500)
        p.stepSimulation()

        # Limit CPU usage by syncing with the timestep
        if elapsed < 1 / 240:
            time.sleep(1 / 240 - elapsed)

def stream_camera():
    """Stream camera frames to the client via WebSocket."""
    while True:
        # Capture PyBullet camera image
        img = p.getCameraImage(width, height, viewMatrix=view_matrix, projectionMatrix=proj_matrix)[2]
        frame = np.array(img, dtype=np.uint8).reshape((height, width, 4))
        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)

        # Encode and emit the frame
        _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        socketio.emit('new_frame', {'image': buffer.tobytes()})
        time.sleep(1 / 30)  # 30 FPS limit for efficiency

@app.route('/')
def index():
    """Render the main HTML page."""
    return render_template('index.html')

@socketio.on('keypress')
def handle_keypress(data):
    """Update beam angle based on user input."""
    global beam_angle
    key = data['key']
    if key == 'ArrowLeft':
        beam_angle = max(beam_angle - angle_step, -0.5)
    elif key == 'ArrowRight':
        beam_angle = min(beam_angle + angle_step, 0.5)

if __name__ == '__main__':
    # Start simulation and camera threads
    Thread(target=run_simulation, daemon=True).start()
    Thread(target=stream_camera, daemon=True).start()

    # Launch the Flask app with WebSocket support
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)
