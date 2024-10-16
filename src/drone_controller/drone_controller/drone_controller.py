import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
import pygame
import sys