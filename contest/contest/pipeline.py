#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import time
import os
import argparse

curr_path = os.path.dirname(os.path.abspath(__file__))
islab_autopilot_path = os.path.abspath(os.path.join(curr_path, '..', '..', '..', '..', '..', '..', '..', 'islab_autopilot'))

class Pipeline(Node):

    def __init__(self, model, world):
        super().__init__('pipeline')
        self.get_logger().info('Pipeline node started')
        self.commands = [
            # Run the Micro XRCE-DDS Agent
            "MicroXRCEAgent udp4 -p 8888",

            # Run the PX4 SITL simulation
            f"cd {islab_autopilot_path} && make px4_sitl gazebo-classic_{model}__{world}",

            # Run QGroundControl
            # "cd /mnt/WORKPACES/USER/QGroundControl && ./QGroundControl.AppImage"
        ]
        self.run_commands()
    
    def run_commands(self):
        for command in self.commands:
            self.get_logger().info(f'Running command: {command}')
            process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            time.sleep(1)

def main(args=None):
    parser = argparse.ArgumentParser(description='Run PX4 SITL and associated tools.')
    parser.add_argument('--model', type=str, default='islab_contest', help='Model to use in the simulation')
    parser.add_argument('--world', type=str, default='test_1', help='World to use in the simulation')
    parsed_args, _ = parser.parse_known_args()

    rclpy.init(args=args)

    pipeline = Pipeline(parsed_args.model, parsed_args.world)

    try:
        rclpy.spin(pipeline)
    except KeyboardInterrupt:
        pipeline.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        pipeline.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

