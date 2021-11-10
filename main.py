import rclpy

from src.autonomous_rb5 import AutonomousRB5


def main(args=None):
    rclpy.init(args=args)
    module_init = AutonomousRB5()
    rclpy.spin(module_init)

    module_init.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
