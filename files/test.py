import smbus, time, math

def delta_time(lasttime):
    return time.time() - lasttime

def read_reg(reg):
    return bus.read_byte_data(0x58, reg)

def mode(mode=None):
    if mode is not None:
        bus.write_byte_data(0x58, 15, mode)
    return read_reg(15)

def enc_count(enc=0):
    return read_reg(2) << 24 | read_reg(3) << 16 | read_reg(4) << 8 | read_reg(5)

def voltage():
    return float(read_reg(10)) / 10

def current():
    return float(read_reg(11)) / 10, float(read_reg(12)) / 10

def desired_speed(speed=None):
    if speed is not None:
        bus.write_byte_data(0x58, 0, speed)
    return read_reg(0)

def delta_count(last_count):
    return enc_count() - last_count

def print_regs():
    regs = []
    for reg in range(0,17):
        regs.append(bus.read_byte_data(0x58, reg))
    print(regs, '\n')

def stop():
    if mode() == 0 or mode() == 2:
        bus.write_byte_data(0x58, 0, 128)
        bus.write_byte_data(0x58, 1, 128)
    elif mode() == 1 or mode() == 3:
        bus.write_byte_data(0x58, 0, 0)
        bus.write_byte_data(0x58, 1, 0)

def enc_speed(last_count, last_time):
    """ Returns the measured speed in m/s """
    radius = .05  # m
    theta = delta_count(last_count) / resolution
    dist = theta * radius
    return theta / delta_time(last_time)
    

bus = smbus.SMBus(1)

speed = 128
count = 0
resolution = 360 / (2* math.pi)  # counts / rad
max_speed = (170 / 60) * 2 * math.pi  # rads/sec

print("max speed: {:4.2} rads/s".format(max_speed))

stop()
mode(0)
currenttime = time.time()

stoptime = time.time() + 20

while time.time() < stoptime:
    speed = int(10 * math.sin(time.time()) + 128 + 50)
    time.sleep(.1)
    desired_speed(speed)
    time.sleep(.05)
    lastcount = count
    count = enc_count()
    lasttime = currenttime
    currenttime = time.time()
    rads_speed = ((desired_speed()-128) / 128 ) * max_speed
    print("{} {:<5.3} {:<5.3} {:<5.3} {}".format(desired_speed(), desired_speed()-128, (desired_speed()-128) / 128, rads_speed, enc_speed(lastcount, lasttime),delta_time(lasttime), delta_count(lastcount)))

print("done with one wheel", '\n')
time.sleep(0.05)
stop()

stoptime = time.time() + 10
mode(2)

while time.time() < stoptime:
    turn = int(20 * math.cos(time.time()) + 128)
    time.sleep(.05)
    bus.write_byte_data(0x58, 1, turn)
    time.sleep(.05)
    lastcount = count
    count = enc_count()
    print(speed, voltage(), current(), count, delta_count(lastcount))

time.sleep(0.05)
stop()
