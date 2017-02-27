try:
    import smbus
except ImportError:
    import smbus2 as smbus
import datetime


class I2CDevice:

    def __init__(self, i2c_port=1, address=None, debug=False):
        self.i2c_port = i2c_port
        self.address = address
        self.bus = None
        self.debug = debug
        self._connect()

    def _connect(self):
        if self.debug:
            print('Opening I2C bus...')

        try:
            try:
                self.bus = smbus.SMBus(self.i2c_port)
            except FileNotFoundError:  # Exception for Raspi version. port 0 vs port 1
                self.bus = smbus.SMBus(1 - self.i2c_port)

            if self.debug:
                print('Bus open.')

        except PermissionError:
            print('Permissions insufficient to open bus. Try running as sudo or making the device accessible to the '
                  'current user.')
            return

    def _write_reg(self, register, value):
        self.bus.write_byte_data(self.address, register, value)
        if self.debug:
            print("Wrote {} to register {} at address {}".format(value, register, self.address))

    def _read_reg(self, register):
        value = self.bus.read_byte_data(self.address, register)
        if self.debug:
            print("Read {} from register {} at address {}".format(value, register, self.address))
        return value


REGISTER = {'Speed1': 0,
            'Speed2': 1,
            'Enc1a': 2,
            'Enc1b': 3,
            'Enc1c': 4,
            'Enc1d': 5,
            'Enc2a': 6,
            'Enc2b': 7,
            'Enc2c': 8,
            'Enc2d': 9,
            'Voltage': 10,
            'Current1': 11,
            'Current2': 12,
            'Version': 13,
            'Accel': 14,
            'Mode': 15,
            'Command': 16}


class MD25(I2CDevice):
    """
    Class for the Devantech MD25 Dual H-bridge motor controller operating on i2c mode with the Raspberry Pi 2/3.
    Default i2c address is 0xB0
    Default Mode is 0

    Modes:
    0 - direct speed assignment to each motor by it's respective register (0 rev, 128 stop, 255 forward)
    1 - direct speed assignment to each motor by it's respective register (-128 rev, 0 stop, 127 forward)
    2 - Speed1 reg is speed for both motors, Speed2 becomes speed difference (0 rev, 128 stop, 255 forward)
    3 - Speed1 reg is speed for both motors, Speed2 becomes speed difference (-128 rev, 0 stop, 127 forward)
    """

    def __init__(self, i2c_port=1, address=0xB0, mode=0):
        super().__init__(i2c_port=i2c_port, address=address)
        self.version = self._read_reg(REGISTER['Version'])
        self.set_mode(mode)
        self.micros = float(datetime.datetime.now().microsecond) / 1000000
        self.last_micros = self.time
        # self.command = self._read_reg(REGISTER['Command'])

    def time(self):
        self.last_micros = self.micros
        self.micros = float(datetime.datetime.now().microsecond) / 1000000
        return self.micros - self.last_micros

    def set_mode(self, mode):
        self._write_reg(register=REGISTER['Mode'], value=mode)

    def set_speed(self, speed, motor=0):
        self._write_reg(register=motor, value=speed)

    def turn(self, amount):
        self._write_reg(register=REGISTER['Speed2'], value=amount)

    def set_accel(self, accel):
        self._write_reg(register=REGISTER['Accel'], value=accel)

    def get_voltage(self):
        return float(self._read_reg(REGISTER['Voltage'])) / 10

    def get_current(self):
        return float(self._read_reg(REGISTER['Current1'])) / 10, float(self._read_reg(REGISTER['Current2'])) / 10

    def clear_encoders(self):
        self._write_reg(REGISTER['Command'], 0x20)
        # self.command = self._read_reg(REGISTER['Command'])

    def read_encoders(self):
        encoder_value1 = 0
        encoder_value2 = 0

        encoder_list1 = self.bus.read_i2c_block_data(self.address, REGISTER['Enc1a'], 4)  # read all 4 registers
        for byte, shift in zip(encoder_list1, range(start=24, stop=0, step=8)):
            encoder_value1 += byte * shift  # Bytes form a signed 32-bit int, this step sums the bytes in the list

        encoder_list2 = self.bus.read_i2c_block_data(self.address, REGISTER['Enc2a'], 4)
        for byte, shift in zip(encoder_list2, range(start=24, stop=0, step=8)):
            encoder_value2 += byte * shift

        return encoder_value1, encoder_value2

    def get_speed(self):
        return (self.time() * speed for speed in self.read_encoders())


if __name__ == '__main__':
    # TODO
    MD25 = MD25(i2c_port=1, address=0xB0)
    MD25.set_speed(motor=0, speed=128)
    MD25.turn(amount=0)
    MD25.set_accel(accel=5)
    MD25.get_speed()
    MD25.get_current()
    MD25.get_voltage()
    MD25.version()