#!/usr/bin/env python
import pygame
import time
""" JOYSTICK Mapping """
TURN_AXIS = 3
FORWARD_AXIS = 4
IDLE_MODE = 1
MANUAL_MODE = 2
AUTONOMOUS_MODE = 3
DECREASE_VEL = 4
INCREASE_VEL = 5

""" DRIVE MODES """
IDLE = 0
MANUAL = 1
AUTONOMOUS = 2

class Toggle:
    TIMEOUT = 0.5
    def __init__(self):
	self._state = False
	self._last_update = time.time()
    
    def update(self):
	if time.time() - self._last_update > self.TIMEOUT:
	    self._state = True
	    self._last_update = time.time()

    def get_state(self):
	if self._state:
	    self._state = False
	    return True
	return False

class JoystickHandler:
    def __init__(self):
        pygame.init()
        pygame.joystick.init()
        self.__reset_modes()
	self._idle_mode = True
        self.vel = 0.5
        self.toggle_buttons = {'dec': Toggle(), 'inc': Toggle()}

    def read_joystick(self):
        for event in pygame.event.get():  # User did something
            if event.type == pygame.QUIT:  # If user clicked close
                self.done = True  # Flag that we are done so we exit this loop
        joystick_count = pygame.joystick.get_count()
        for i in range(joystick_count):
            self.joystick = pygame.joystick.Joystick(i)
            self.joystick.init()

        self._joystick_cmd = (self.joystick.get_axis(FORWARD_AXIS), self.joystick.get_axis(TURN_AXIS))
        if self.joystick.get_button(IDLE_MODE) == 1:
            self.__reset_modes()
            self._idle_mode = True
        if self.joystick.get_button(MANUAL_MODE) == 1:
            self.__reset_modes()
            self._joystick_mode = True
        if self.joystick.get_button(AUTONOMOUS_MODE) == 1:
            self.__reset_modes()
            self._autonomous_mode = True
        if self.joystick.get_button(DECREASE_VEL) == 1:
	    self.toggle_buttons['dec'].update()
	    if self.toggle_buttons['dec'].get_state():
                self.dec_vel()
        if self.joystick.get_button(INCREASE_VEL) == 1:
	    self.toggle_buttons['inc'].update()
	    if self.toggle_buttons['inc'].get_state():
                self.inc_vel()

    def __reset_modes(self):
        self._idle_mode = self._joystick_mode = self._autonomous_mode = False

    def get_drive_mode(self):
        """
        0: idle
        1: drive from joystick
        2: autonomy
        :return:
        """
        if self._idle_mode:
            return IDLE
        if self._joystick_mode:
            return MANUAL
        if self._autonomous_mode:
            return AUTONOMOUS
        return None

    def get_joystick_command(self):
        return self._joystick_cmd

    def __str__(self):
        return 'JS | mode: {}   cmd: ({:.2f}, {:.2f})'.format(self.get_drive_mode(), self._joystick_cmd[0], self._joystick_cmd[1])

    def inc_vel(self):
        if self.vel <= 0.99:
            self.vel += 0.1
            print('{}'.format(self.vel))
        else:
            print('Max velocity')

    def dec_vel(self):
        if self.vel > 0.01:
            self.vel -= 0.1
            print('{}'.format(self.vel))
        else:
            print('Min velocity')

    def get_velocity(self):
        return self.vel


if __name__ == '__main__':
    import time
    js = JoystickHandler()
    while True:
        time.sleep(0.5)
        js.read_joystick()
	print js.get_drive_mode()
	print js.get_joystick_command()
    pygame.quit()
