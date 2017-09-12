import SMSLibrary as sms
import time as t
sms.init(0)

# sms.stop(5)

t.sleep(0.02)
sms.start(5)
print(float(sms.getPosition(5)[1]))

print(sms.getPIDgainP(5))
print(sms.getPIDgainI(5))
print(sms.getPIDgainD(5))

sms.profiledMoveToRelativePosition(5, 10000)
t.sleep(5)
print(float(sms.getPosition(5)[1]))

sms.moveToRelativePosition(5,-16384)
t.sleep(5)
sms.moveWithVelocity(5,10000)
t.sleep(5)
sms.profiledMoveWithVelocity(5,-10000)
t.sleep(5)
sms.moveToAbsolutePosition(5,16384)
sms.stop(5)

sms.shut_down(0)


