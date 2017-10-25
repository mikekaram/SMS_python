import SMSLibrary as sms
import time as t
sms.init(1)


motorId = [4, 5, 6, 7, 8, 9]
t.sleep(0.02)
sms.broadcastStop()
t.sleep(0.02)
# sms.broadcastStart()

# # sms.start(motorId)
# for i in motorId:
#     sms.resetErrors(i)

#     # print(sms.getPosition(i))
#     t.sleep(0.02)
# for i in motorId:
#     # sms.setProfiledVelocitySetpoint(i, 128)
#     print(sms.getAbsolutePosition(i)[1])
#     t.sleep(1)
# # sms.broadcastDoMove()
# t.sleep(1)
# print(sms.getPIDgainP(motorId))
# print(sms.getPIDgainI(motorId))
# print(sms.getPIDgainD(motorId))

# sms.profiledMoveToRelativePosition(motorId, 10000)
# t.sleep(5)
# print(float(sms.getPosition(motorId)[1]))

# sms.moveToRelativePosition(motorId, -16384)
# t.sleep(5)
# sms.moveWithVelocity(motorId, 10000)
# t.sleep(5)
# sms.profiledMoveWithVelocity(motorId, -10000)
# t.sleep(5)
# sms.moveToAbsolutePosition(motorId, 16384)
# sms.stop(5)
sms.broadcastStop()
sms.shut_down(0)
