import serial
import gpiozero

HEAD_FIRST = 0x42
HEAD_SECOND = 0x4d
DATA_LENGTH = 32
BODY_LENGTH = DATA_LENGTH - 1 - 1
P_CF_PM10 = 2
P_CF_PM25 = 4
P_CF_PM100 = 6
P_C_PM10 = 8
P_C_PM25 = 10
P_C_PM100 = 12
P_C_03 = 14
P_C_05 = 16
P_C_10 = 18
P_C_25 = 20
P_C_50 = 22
P_C_100 = 24
DATA_DESC = [
    (P_CF_PM10, 'CF=1, PM1.0', 'μg/m3'),
    (P_CF_PM25, 'CF=1, PM2.5', 'μg/m3'),
    (P_CF_PM100, 'CF=1, PM10', 'μg/m3'),
    (P_C_PM10, 'PM1.0', 'μg/m3'),
    (P_C_PM25, 'PM2.5', 'μg/m3'),
    (P_C_PM100, 'PM10', 'μg/m3'),
    (P_C_03, '0.1L, d>0.3μm', ''),
    (P_C_05, '0.1L, d>0.5μm', ''),
    (P_C_10, '0.1L, d>1μm', ''),
    (P_C_25, '0.1L, d>2.5μm', ''),
    (P_C_50, '0.1L, d>5.0μm', ''),
    (P_C_100, '0.1L, d>10μm', ''),
]


class PMS7003:
    def __init__(self, serial_device='/dev/ttyAMA0', gpio_set_no=17, gpio_reset_no=18):
        self.gpio_set = gpiozero.LED(gpio_set_no)
        self.gpio_reset = gpiozero.LED(gpio_reset_no)
        self.ser = None
        self.serial_device = serial_device
        self._setup_serial()

    def reset_device(self):
        self.gpio_reset.off()

    def _setup_serial(self):
        self.ser = serial.Serial(port=self.serial_device, baudrate=9600, bytesize=serial.EIGHTBITS,
                                 parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)

    def set_device_working_state(self):
        self.gpio_set.on()

    def set_device_sleep_state(self):
        self.gpio_set.off()

    def get_frame(self):
        self.set_device_working_state()
        while True:
            b = self.ser.read()
            if b != chr(HEAD_FIRST):
                continue
            b = self.ser.read()
            if b != chr(HEAD_SECOND):
                continue
            body = self.ser.read(BODY_LENGTH)
            if len(body) != BODY_LENGTH:
                continue
            return body
        self.set_device_sleep_state()

    def is_valid_frame(self, _frame):
        checksum = ord(_frame[-2]) << 8 | ord(_frame[-1])
        calculated_checksum = HEAD_FIRST + HEAD_SECOND
        for field in _frame[:-2]:
            calculated_checksum += ord(field)
        return checksum == calculated_checksum

    def decode_frame(self, _frame):
        data = {}
        for item in DATA_DESC:
            start, desc, unit = item
            value = int(ord(_frame[start]) << 8 | ord(_frame[start + 1]))
            data[str(start)] = (desc, value, unit)
        return data

    def get_version_and_error_code(self, _frame):
        return _frame[-4], _frame[-3]

    def read(self):
        try:
            frame = self.get_frame()
        except Exception as e:
            print('get frame got exception: {}'.format(e.message))
        else:
            if not self.is_valid_frame(frame):
                print('frame checksum mismatch')
                return
            data = {'data': self.decode_frame(frame)}
            version, error_code = self.get_version_and_error_code(frame)
            data['version'] = version
            data['errcode'] = error_code
            return data


if __name__ == '__main__':
    pms7003 = PMS7003()
    print(pms7003.read())
