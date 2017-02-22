import I2C


class MD25(I2C.I2CDevice):

    def __init__(self, i2c_port=1, address=0xB0):
        super(MD25, self).__init__(i2c_port=i2c_port, address=address)
        # self.num_motors = num_motors
        # assert mode == 'DIRECT' or 'TURN'
        # self.mode = mode
        self.registers = {'Speed1': 0,
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

    def set_speed(self, speed, motor=0):
        self._write_reg(register=motor, value=speed)

    def turn(self, amount):
        self._write_reg(register=self.registers['Speed2'], value=amount)

    def set_accel(self, accel):
        self._write_reg(register=self.registers['Accel'], value=accel)

    def read_voltage(self):
        return float(self._read_reg(self.registers['Voltage'])) / 10


if __name__ == '__main__':
    # TODO
    MD25 = MD25(i2c_port=1, address=0xB0)
    MD25.set_speed(motor=0, speed=128)
    MD25.turn(amount=0)
    MD25.set_accel(accel=5)
    MD25.read_encoder(motor)
    MD25.read_current(motor)
    MD25.read_voltage()
    MD25.version
    MD25.set_command_reg(values)