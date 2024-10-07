import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import Twist
import pygame
from pygame.locals import *

#key mapping for controls 
KEY_MAPPING = {
    K_w: (1,0,0,0),     # Forward
    K_s: (-1,0,0,0),    # Backward
    K_a: (0,1,0,0),     #Left 
    K_d: (0,-1,0,0),    #Right
    K_UP: (0,0,1,0),     #Ascend 
    K_DOWN : (0,0,-1,0), #Descend
    K_q: (0,0,0,1),      #Rotate Left (Yaw)
    K_e: (0,0,0,-1),    #Rotate Right (Yaw)
}

class DroneController(Node):
    def __init__(self):
        super().__init__("drone_controller")
        self.publisher_ = self.create_publisher(Twist,"cmd_vel",10)
        self.velocity = Twist()
        self.linear_speed = 0.5 #m/s
        self.angular_speed = 0.5 #rad/s
        self.get_logger().info("Drone controller node has started")

        #Initialize pygame
        pygame.init()
        self.screen = pygame.display.set_mode((400, 300))
        pygame.display.set_caption("Drone Controller")
    
    def control_loop(self):
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == QUIT:
                    running =False
                    break
                if event.type == KEYDOWN:
                    if event.key in KEY_MAPPING:
                        self.handle_key(KEY_MAPPING[event.key])
                elif event.type == KEYUP:
                    if event.key in KEY_MAPPING:
                        self.stop_movement()

            self.publisher_.publish(self.velocity)
        
        pygame.quit()
    
    def handle_key(self, action):
        # Handle keyboard input to control drone movement
        lin_x,lin_y,lin_z, ang_z = action
        self.velocity.linear.x = lin_x * self.linear_speed
        self.velocity.linear.y = lin_y * self.linear_speed
        self.velocity.linear.z = lin_z * self.linear_speed
        self.velocity.angular.z = ang_z * self.angular_speed
        self.get_logger().info(f"Sending velocity : {self.velocity}")

    def stop_movement(self):
        #stop drone movement when the key is release 
        self.velocity.linear.x = 0.0
        self.velocity.linear.y = 0.0
        self.velocity.linear.z = 0.0
        self.velocity.angular.z = 0.0
        self.get_logger().info("stopping drone")

def main(args=None):
    rclpy.init(args=args)
    dron_controller = DroneController()

    try:
        dron_controller.control_loop()
    except KeyboardInterrupt:
        pass
    finally:
        dron_controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()