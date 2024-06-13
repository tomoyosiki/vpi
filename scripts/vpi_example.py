import sys
sys.path.insert(0, '/mnt/vpi_ws/src/vpi/src')

import vpi
from datetime import datetime

# vpi.MoveVehicleToPoseWithFixedSpeed(0, 0.25, [3.05, 0.05, -0.12, -0.005, 0.007, -0.026, 0.996], [0.1, 0, 0], [0.2, 0.0, 0.0])
# vpi.sleep_ms(500)
# vpi.MoveVehicleToPose2(0, [3.05, 0.05, -0.12, -0.005, 0.007, -0.026, 0.996], [0.2, 0.001, 0.01], [0.1, 0.0, 0.0])

# vpi.MoveVehicleToPoseWithFixedSpeed(0, 0.25, [10.26134, -0.9596, 0, -0.002, -0.009, -0.0203, 0.9997], [0.15, 0, 0], [0.2, 0.0, 0.0])
# vpi.sleep_ms(500)
# vpi.MoveVehicleToPose2(0, [10.26134, -0.9596, 0, -0.002, -0.009, -0.0203, 0.9997], [0.2, 0.001, 0.01], [0.1, 0.0, 0.0])
# vpi.sleep_ms(500)
# vpi.MoveVehicleInBackwardSShape(0, (7.721, 0.507))
# vpi.MoveVehicleStraight(0, 7.721)

vpi.MoveVehicleToPose3(0, [9.620341184434354, -1.08266, 0, -0.002, -0.009, -0.0203, 0.9997], [0.2, 0.001, 0.01], [0.1, 0.0, 0.0])




# vpi.MoveVehicleToPoseWithFixedSpeed(0, 0.35, [9.620341184434354, -1.08266, 0, -0.002, -0.009, -0.0203, 0.9997], [0.15, 0, 0], [0.2, 0.0, 0.0])
# vpi.sleep_ms(500)
# vpi.MoveVehicleToPose2(0, [9.620341184434354, -1.08266, 0, -0.002, -0.009, -0.0203, 0.9997], [0.2, 0.001, 0.01], [0.1, 0.0, 0.0])
# vpi.sleep_ms(500)
# vpi.MoveVehicleInBackwardSShape(0, (7.08, 0.384))
# vpi.MoveVehicleStraight(0, 7.08)




# in the outside
# vpi.MoveVehicleToPose(0, [10.26134, -0.9596, 0, -0.002, -0.009, -0.0203, 0.9997], [0.2, 0.001, 0.01], [0.1, 0.0, 0.0])
# vpi.MoveVehicleInBackwardSShape(0, (7.721, 0.507))




# vpi.MoveVehicleStraight(0, 7.703)
# vpi.MoveVehicleStraight(0, -0.73)
#vpi.MoveVehicleToPose(0, [-0.73, -6.0, 0.09, 0.04, 0.12, 0.998, 0.0335], [0.2, 0.001, 0.01], [0.1, 0.0, 0.0])
#vpi.MoveVehicleInBackwardSShape(0, (1.45, -7.12))
#vpi.MoveVehicleStraight(0, 1.45)

# vpi.MoveVehicleToPose(0, [1.0, -6.0, 0.09, 0.04, 0.12, 0.998, 0.0335], [0.2, 0, 0.01], [0.1, 0.0, 0.01])

# vpi.MoveVehicleToPose(0, [-0.73, -6.0, 0.09, 0.04, 0.12, 0.998, 0.0335], [0.2, 0.001, 0.01], [0.1, 0.0, 0.0])
# vpi.MoveVehicleInBackwardSShape(0, (1.45, -7.12))
# vpi.MoveVehicleToPose(0, [1.45, -7.12, 0.09, 0.04, 0.12, 0.998, 0.0335], [0.2, 0.001, 0.01], [0.1, 0.0, 0.0])


# vpi.MoveVehicleForward(0, 3.0)
# vpi.MoveVehicleBackward(0, 2.0)

# vpi.sleep_ms(100)

# for i in range(20):
#     vpi.sleep_ms(20)
#     current_timestamp = datetime.timestamp(datetime.now())
#     ndt_pose = vpi.getSensorData(0, 'ndt_pose', None, None)
#     if ndt_pose != None:
#         print(current_timestamp)
    # print(f"{i}, {vpi.getSensorData(0, 'ndt_pose', None, None)}")

# vpi.MoveVehicleStraight(0, -0.44)
# vpi.MoveVehicleInBackwardSShape(0, (1.522, -7.163))
