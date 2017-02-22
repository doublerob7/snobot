import smbus


class I2CDevice:

    def __init__(self, i2c_port=1, address=0xB0):
        self.i2c_port = i2c_port
        self.address = address
        self.bus = None
        self._connect()

    def _connect(self):
        print('Opening I2C bus...')

        try:
            try:
                self.bus = smbus.SMBus(self.i2c_port)
            except FileNotFoundError:  # Exception for Raspi version. port 0 vs port 1
                self.bus = smbus.SMBus(1 - self.i2c_port)

            print('Bus open.')

        except PermissionError:
            print('Permissions insufficient to open bus. Try running as sudo or making the device accessible to the '
                  'current user.')
            return

    def _write_reg(self, register, value):
        self.bus.write_byte_data(self.address, register, value)

    def _read_reg(self, register):
        return self.bus.read_byte_data(self.address, register)



if __name__ == '__main__':
    # TODO
    pass