import pygame
from Adafruit_PCA9685 import PCA9685
from time import sleep, time
from math import sqrt
from smbus import SMBus
from Adafruit_BNO055 import BNO055
from pid_controller.pid import PID

LD = 11
RD = 3
LF = 7
RF = 15
calibration  = [223, 255, 249, 255, 219, 255, 66, 253, 142, 255, 49, 251, 254, 255, 253, 255, 0, 0, 232, 3, 151, 2]
    
class PWM(object):

    __ARM = 290
    __pwm = PCA9685()

    def __init__(self):
        self.__pwm.set_pwm_freq(48)

    def arm(self):
        self.__pwm.set_pwm(LF, 0, self.__ARM)
        self.__pwm.set_pwm(RF, 0, self.__ARM)
        self.__pwm.set_pwm(LD, 0, self.__ARM)
        self.__pwm.set_pwm(RD, 0, self.__ARM)
    
    def set_speed(self, code, speed):
        speed += self.__ARM
        if speed > 350:
            speed = 350
        elif speed < 150:
            speed = 150
        self.__pwm.set_pwm(code, 0, speed)
        sleep(1.0/48)

class PressureSensor(object):

    __bus = SMBus(1)
    __delay = 0.01
    __ADDR = 0x76

    def __init__(self):
        self.__bus.write_byte(self.__ADDR, 0X1E)
        sleep(self.__delay)
    
    def calibrate(self):
        # Read pressure sensitivity
        data = self.__bus.read_i2c_block_data(self.__ADDR, 0xA2, 2)
        self.__C1 = data[0] * 256 + data[1]

        # Read pressure offset
        data = self.__bus.read_i2c_block_data(self.__ADDR, 0xA4, 2)
        self.__C2 = data[0] * 256 + data[1]

        # Read temperature coefficient of pressure sensitivity
        data = self.__bus.read_i2c_block_data(self.__ADDR, 0xA6, 2)
        self.__C3 = data[0] * 256 + data[1]

        # Read temperature coefficient of pressure offset
        data = self.__bus.read_i2c_block_data(self.__ADDR, 0xA8, 2)
        self.__C4 = data[0] * 256 + data[1]

        # Read reference temperature
        data = self.__bus.read_i2c_block_data(self.__ADDR, 0xAA, 2)
        self.__C5 = data[0] * 256 + data[1]

        # Read temperature coefficient of the temperature
        data = self.__bus.read_i2c_block_data(self.__ADDR, 0xAC, 2)
        self.__C6 = data[0] * 256 + data[1]
    
    def __get_digital_pressure(self):
        self.__bus.write_byte(self.__ADDR, 0x40)
        sleep(self.__delay)
        value = self.__bus.read_i2c_block_data(self.__ADDR, 0x00, 3)
        return value[0] * 65536 + value[1] * 256 + value[2]


    def __get_digital_temperature(self):
        self.__bus.write_byte(self.__ADDR, 0x50)
        sleep(self.__delay)
        value = self.__bus.read_i2c_block_data(self.__ADDR, 0x00, 3)
        return value[0] * 65536 + value[1] * 256 + value[2]

    def get_pressure(self):
        D1 = self.__get_digital_pressure()
        D2 = self.__get_digital_temperature()
        dT = D2 - self.__C5 * 256
        temp = 2000 + dT * self.__C6 / 8388608
        OFF = self.__C2 * 65536 + (self.__C4 * dT) / 128
        sens = self.__C1 * 32768 + (self.__C3 * dT ) / 256
        T2 = 0
        off2 = 0
        sens2 = 0

        if temp > 2000 :
            T2 = 7 * (dT * dT)/ 137438953472
            off2 = ((temp - 2000) * (temp - 2000)) / 16
            sens2= 0
        elif temp < 2000 :
                T2 = 3 * (dT * dT) / 8589934592
                off2 = 3 * ((temp - 2000) * (temp - 2000)) / 8
                sens2 = 5 * ((temp - 2000) * (temp - 2000)) / 8
                if temp < -1500:
                        off2 = off2 + 7 * ((temp + 1500) * (temp + 1500))
                        sens2 = sens2 + 4 * ((temp + 1500) * (temp + 1500))

        temp = temp - T2
        OFF = OFF - off2
        sens = sens - sens2
        return ((((D1 * sens) / 2097152) - OFF) / 32768.0) / 10.0
    
    def get_depth(self):
        rho = 997.0
        g = 9.81
        abs_pressure = 970
        return (self.get_pressure() - abs_pressure)*100/(rho*g)


class IMU(object):

    def __init__(self):
        self.__bno = BNO055.BNO055(serial_port='/dev/ttyAMA0', rst=18)
        if not self.__bno.begin():
            raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')
        self.__bno.set_calibration(calibration)

    def get_required(self, ref_heading):
        roll_bias = 7.5
        heading, roll, _ = self.__bno.read_euler()
        _, accy, _ = self.__bno.read_linear_acceleration()
        if heading > (180 + ref_heading):
            heading = heading - 360
        
        roll -= roll_bias
        if roll > 180:
            roll -= 360
        return heading, roll, accy

if __name__ == '__main__':
    # initialize joystick
    #j = pygame.joystick.Joystick(0)
    # j.init()

    # initialize PWM board
    pwm = PWM()
    
    # arm the motors
    pwm.arm()

    # initialize Pressure Sensor
    pressure = PressureSensor()

    # calibrate Pressure Sensor
    pressure.calibrate()

    imu = IMU()
    sleep(1)

    depthPID = PID(p=145, i=0.1, d=1)
    depthPID.target = 0.9

    headingPID = PID(p=0.75, i=0.01, d=0)
    init_heading, _, _ = imu.get_required(0)
    headingPID.target = init_heading - 15
    count = 0
    start = time()
    while True:
        try:
            depth = pressure.get_depth()
            heading, roll, accy = imu.get_required(ref_heading=headingPID.target)
            depthCorrection = -int(depthPID(feedback=depth))
            headingCorrection = -int(headingPID(feedback=heading))
            if depth > 0.1:
                pwm.set_speed(LD, depthCorrection)
                pwm.set_speed(RD, depthCorrection)
                pwm.set_speed(LF, headingCorrection + 30)
                pwm.set_speed(RF, headingCorrection - 30)
            end = time() - start
            if end > 8.0 and end <= 24.0:
                if depthPID.target == 0.9:
                    depthPID.target = 0.3
                    print 'Obstacle 1 done'
            elif end > 24.0:
                pwm.arm()
                print 'Obstacle 2 done'
                break
        except KeyboardInterrupt:
            pwm.arm()
            print ("Motor stopped")
            break
        except IOError:
            print ("Oh shit! Remote IO Error")
            sleep(0.2)
            continue







