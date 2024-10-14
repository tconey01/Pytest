# PyBullet Beam Control - README

This project folder is a web-based interface that connects a **PyBullet physics simulation** to a client through **Flask** and **Socket.IO**. 
Users can control the angle of a beam in the simulation by pressing the left and right arrow keys, with real-time video feedback streamed to the browser. 
The app runs the simulation and camera streaming on parallel threads, ensuring smooth interaction and frame updates at 30 FPS.

Requirements
Install the following dependencies:

```bash
pip install flask flask-socketio pybullet opencv-python-headless numpy
```
