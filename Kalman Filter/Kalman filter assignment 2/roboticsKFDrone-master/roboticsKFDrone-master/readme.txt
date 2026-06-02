solar.bin and solar.mat is a bin file from Ardupilot of a flight done at Stellenbosch High School where a drone washed the PV panels.

The flight controller is a Cube Orange.

--- Files ---

bin2mat.py
  Python script that reads solar.bin (ArduPilot MAVLink log) and converts all
  message types into a MATLAB-compatible .mat file (solar.mat, HDF5 v7.3).
  Each message type becomes a struct of arrays (e.g. data.IMU.AccZ).
  Requires: pymavlink, hdf5storage, numpy.

kalmanAltitude.m
  MATLAB script that runs a discrete Kalman filter to estimate drone altitude.
  Uses IMU vertical acceleration (body-frame AccZ, corrected for roll/pitch) as
  the predict step control input, and barometer altitude as the measurement update.
  State vector: [altitude; vertical_velocity]. Loads data from solar.mat.
  Plots Kalman estimate vs barometer, GPS, and ArduPilot EKF altitude.

plotAltitude.m
  MATLAB script that reads solar.bin directly via pymavlink (Python-MATLAB bridge)
  and plots raw barometer altitude, GPS altitude, and ArduPilot EKF estimated
  altitude side by side. Useful for quick visual inspection without converting
  the log first.

list_ardupilot_message_types.m
  MATLAB script that scans solar.bin via pymavlink and prints all unique MAVLink
  message types found in the log. Also saves the list to ardupilot_message_types.txt.

ardupilot_message_types.txt
  Output of list_ardupilot_message_types.m — a plain-text list of all 72 MAVLink
  message types present in solar.bin (e.g. IMU, BARO, GPS, ATT, XKF1, ESC, ...).

--- Data files (not in repo — too large) ---

solar.bin  (143 MB)  Raw ArduPilot flight log from the Cube Orange.
solar.mat  ( 72 MB)  MATLAB struct generated from solar.bin by bin2mat.py.
