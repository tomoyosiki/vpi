import sys
sys.path.insert(0, '/mnt/vpi_ws/src/vpi/src')

import vpi
from datetime import datetime

vehicle_id = 0

PID_speed1 = [0.15, 0, 0]
PID_steering1 = [0.2, 0.0, 0.0]

PID_speed2 = [0.2, 0.001, 0.01]
PID_steering2 = [0.1,0.0, 0.0]

smoothSpeed = 0.25

parking_pose = vpi.GetNearestParkingSpot(vehicle_id)
start_parking_pose = vpi.calculateDesiredX(vehicle_id, parking_pose)

print(parking_pose)
print(start_parking_pose)

vpi.MoveVehicleToPoseWithFixedSpeed(vehicle_id, smoothSpeed, start_parking_pose, PID_speed1, PID_steering1)
vpi.sleep_ms(500)
vpi.MoveVehicleToPose2(vehicle_id, start_parking_pose, PID_speed2, PID_steering2)
vpi.sleep_ms(500)
vpi.MoveVehicleInBackwardSShape(vehicle_id, parking_pose)
vpi.sleep_ms(500)
vpi.MoveVehicleStraight(vehicle_id, parking_pose[0])