#!/usr/bin/env python
from ros_command_handler import ROSCommandHandler
import pygame

if __name__ == '__main__':
    ch = ROSCommandHandler()
    ch.spin()
    pygame.quit()

