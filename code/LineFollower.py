from SerialDevice import SerialDevice

class LineFollower:
    STATES = ('FORWARD', 'BACKWARD', 'RELEASE') # 'release' stops the motor (it does not brake)
    SIDES = ('LEFT', 'RIGHT')   # for reading sensors and instructing motors
    # determined by the Adafruit motor shield library
    MIN_SPEED = 0
    MAX_SPEED = 255

    def __init__(self) -> None:
        self.dev = SerialDevice()
        self._left_speed = 0
        self._right_speed = 0
        self._left_state = 'RELEASE'
        self._right_state = 'RELEASE'

    @property
    def left_speed(self) -> int:
        return self._left_speed
    
    @left_speed.setter
    def set_left_speed(self, speed: int) -> None:
        self.set_speed("LEFT", speed)

    @property
    def right_speed(self) -> int:
        return self._right_speed
    
    @right_speed.setter
    def set_right_speed(self, speed: int) -> None:
        self.set_speed("RIGHT", speed)

    @property
    def left_state(self) -> str:
        return self._left_state
    
    @left_state.setter
    def set_left_state(self, state: str) -> None:
        self.set_state("LEFT", state)

    @property
    def right_state(self) -> str:
        return self._right_state
    
    @right_state.setter
    def set_right_state(self, state: str) -> None:
        self.set_state("RIGHT", state)

    def set_speed(self, motor: str, speed: int) -> None:
        '''
        sets the speed of a specified motor.

        Args:
            motor (str): must be "left" or "right", case insensitive.
            speed (int): must be 0-255.

        Raises:
            ValueError: if motor argument isn't "left" or "right".
            ValueError: if speed argument isn't within the range 0-255.
        '''
        motor = motor.upper().strip()

        # raise errors if the args are invalid
        if motor not in self.SIDES:
            raise ValueError('"{}" is not a valid motor'.format(motor))
        if speed not in range(self.MIN_SPEED, self.MAX_SPEED):
            raise ValueError('"{}" is not a valid speed'.format(speed))

        # send the motor instruction
        self.dev.write('setSpeed {} {}'.format(motor, speed))

        # update the object's property
        if motor == 'LEFT':
            self._left_speed = speed
        elif motor == 'RIGHT':
            self._right_speed = speed
    
    def set_state(self, motor: str, state: str) -> None:
        '''
        sets the state of a specified motor.

        Args:
            motor (str): must be "left" or "right", case insensitive.
            state (str): must be "forward", "backward", or "release", case insensitive.
                "release" will stop driving the motor, but not apply any braking.

        Raises:
            ValueError: if motor argument isn't "left" or "right".
            ValueError: if state argument isn't "forward", "backward", or "release".
        '''
        motor = motor.upper().strip()
        state = state.upper().strip()

        # raise errors if the args are invalid
        if motor not in self.SIDES:
            raise ValueError('"{}" is not a valid motor'.format(motor))
        if state not in self.STATES:
            raise ValueError('"{}" is not a valid motor state'.format(state))

        # send the motor instruction
        self.dev.write('setState {} {}'.format(motor, state))

        # update the object's property
        if motor == 'LEFT':
            self._left_state = state
        elif motor == 'RIGHT':
            self._right_state = state
        
    def read_sensor(self, sensor: str) -> int:
        '''
        reads the value from a specified sensor.

        This is blocking, and will continue to loop forever until a sensor reading is
        received over the serial port in the expected format. This isn't ideal, so I
        might implement a timeout later.

        Args:
            sensor (str): must be "left" or "right", case insensitive.

        Raises:
            ValueError: if sensor argument isn't "left" or "right".

        Returns:
            int: the sensor reading, or -1 if non-numeric output was received.
        '''
        sensor = sensor.upper().strip()
        # raise error if the arg is invalid
        if sensor not in self.SIDES:
            raise ValueError('"{}" is not a valid sensor'.format(sensor))

        # send the instruction to get a reading
        self.dev.write('readSensor {}'.format(sensor))

        # read the serial input until we get something that looks like data
        data = ''
        while not data.startswith('{}SENSOR='.format(sensor)):
            data = self.dev.read()
        data = data.split('=')[1].strip()   # get rid of the stuff before and after the number

        if data.isnumeric():    # return the received reading if it's actually a number
            return int(data)
        return -1   # otherwise return -1; something went wrong