import pygame
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class GamepadPublisher(Node):
    def __init__(self):
        super().__init__('cmd_publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    # ===== INIT GAMEPAD =====
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No controller detected")
        exit()

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print(f"Using controller: {joystick.get_name()}")

    # ===== MAIN LOOP =====
    def timer_callback(self):
        pygame.event.pump()

        # AXES (depends on controller)
        steering_axis = joystick.get_axis(0)   # left stick X
        throttle_axis = joystick.get_axis(1)   # left stick Y

        # ===== PROCESS INPUT =====
        # Steering: already -1 → 1
        steering = round(steering_axis, 2)

        # Throttle mapping to your Arduino logic
        if throttle_axis < -0.5:
            throttle = 1      # forward
        elif throttle_axis > 0.5:
            throttle = -1     # reverse
        else:
            throttle = 0      # stop
            
        cmd = Twist()

        cmd.linear.x = float(throttle)
        cmd.angular.z = float(steering)

        self.publisher.publish(cmd)
        
        self.get_logger().info(
            f'Throttle: {throttle}, Steering: {steering}'
        )
    
def main(args=None):
    rclpy.init(args=args)
    
    node = GamepadPublisher
    
    try:
        rclpy.spin(Node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        pygame.quit()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
        