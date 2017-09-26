import SMSLibrary as sms
import time as t
sms.init(0)

# sms.stop(5)
motorId = 5
t.sleep(0.02)
sms.start(motorId)
print(float(sms.getPosition(motorId)[1]))

print(sms.getPIDgainP(motorId))
print(sms.getPIDgainI(motorId))
print(sms.getPIDgainD(motorId))

sms.profiledMoveToRelativePosition(motorId, 10000)
t.sleep(5)
print(float(sms.getPosition(motorId)[1]))

sms.moveToRelativePosition(motorId, -16384)
t.sleep(5)
sms.moveWithVelocity(motorId, 10000)
t.sleep(5)
sms.profiledMoveWithVelocity(motorId, -10000)
t.sleep(5)
sms.moveToAbsolutePosition(motorId, 16384)
sms.stop(5)

sms.shut_down(0)
