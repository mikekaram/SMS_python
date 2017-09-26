import SMSLibrary as sms
import numpy as np
import time as t
import ikpy.geometry_utils as gu


def homing():
    # offset = np.zeros((6, 1))
    sms.init(0)
    ticks_to_go_positive = np.zeros((6, 1))
    sms.broadcastStart()
    t.sleep(0.02)
    # absolute_positions_homing = 16161
    absolute_positions_ticks_positive = np.zeros((6, 1))
    absolute_positions_entry_positions = np.zeros((6, 1))
    for i, mId in enumerate(motorIds):
        print(i, mId)
        absolute_positions_entry_positions[i] = sms.getAbsolutePosition(mId)[1]
        t.sleep(0.01)
        ticks_to_go_positive[i] = gu.angle_to_ticks(gu.angle_difference(gu.ticks_to_angle(absolute_positions_homing[i]), gu.ticks_to_angle(absolute_positions_entry_positions[i])))
        absolute_positions_ticks_positive = ticks_to_go_positive
        print(absolute_positions_ticks_positive)
        sms.setProfiledAbsolutePositionSetpoint(mId, int(absolute_positions_ticks_positive[i]))
        t.sleep(0.04)
    sms.broadcastDoMove()
    t.sleep(10)
    sms.broadcastStop()
    sms.shut_down(0)


motorIds = [4, 5, 6]
global absolute_position_homing
absolute_positions_homing = np.array([11116.0, 7582.0, 12024.0])
homing()
