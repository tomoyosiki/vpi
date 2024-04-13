import sys
sys.path.insert(0, '/mnt/vpi_ws/src/vpi/src')

import vpi
from datetime import datetime

vpi.MoveVehicleStraight(0, -0.73)
vpi.MoveVehicleInBackwardSShape(0, (1.45, -7.12))
vpi.MoveVehicleStraight(0, 1.45)

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