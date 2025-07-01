# Raspberry Pi 3D Scanner

This project provides Python scripts to build a basic 3D scanner using a
Raspberry Pi 5.  The scanner rotates an object on a turntable using a NEMA17
stepper motor and collects distance data from an STL27L lidar module.  The
captured points are written to a PLY file that can be viewed in tools such as
MeshLab.

Features
--------
- Controls a stepper motor to rotate the object platform
- Reads distance measurements from the lidar
- Produces a 3D point cloud in PLY format

Requirements
------------
- Raspberry Pi OS 64‑bit (tested with Python 3.9 and later)
- `pyserial` and `RPi.GPIO` (install via ``pip install -r requirements.txt``)
- STL27L lidar module with USB interface
- NEMA17 42‑23 stepper with A4988 driver

Usage
-----
1. Install the dependencies::

       pip install -r requirements.txt

2. Wire the hardware:
   - Connect the lidar's USB cable to the Pi so it appears as ``/dev/ttyUSB0``.
   - Hook up the A4988 driver to the GPIO pins configured in
     ``scanner/stepper.py`` (defaults are ``STEP=21``, ``DIR=20`` and
     ``ENABLE=16``).  Provide 12 V to the motor power pins of the driver.

3. Perform a scan and write a point cloud::

       python3 scripts/run_scan.py output.ply

   The script rotates the stepper once, collecting a single distance reading per
   step.  When finished, ``output.ply`` contains the 3D points which can be
   loaded in tools such as MeshLab.

4. (Optional) Run the included unit tests and linter::

       flake8
       pytest -q

Disclaimer
----------
This code provides a minimal example intended for educational use.  Real-world
scanning generally requires calibration and noise filtering which are outside
the scope of this project.
