#!/usr/bin/env python3
import os
import sys
import shutil
import signal
import threading
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter


class BringUp(Node):
    def __init__(self):
        super().__init__('bringup_node')
        self.get_logger().info('Creating BringUp')

        # --- Defaults ---
        curr_path = os.path.dirname(os.path.abspath(__file__))
        default_autopilot_path = os.path.abspath(
            os.path.join(curr_path, '..', '..', '..', '..', '..', '..', '..', 'islab_autopilot')
        )

        # --- Parameters ---
        self.declare_parameter('world_name', 'contest_1')
        self.declare_parameter('model_name', 'islab_contest')
        self.declare_parameter('autopilot_path', default_autopilot_path)

        self.world_name = self.get_parameter('world_name').get_parameter_value().string_value
        self.model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.autopilot_path = self.get_parameter('autopilot_path').get_parameter_value().string_value

        self.get_logger().info(
            f'Params: world_name={self.world_name}, model_name={self.model_name}, autopilot_path={self.autopilot_path}'
        )

        # Child process handle + reader thread
        self._proc = None
        self._reader_thread = None

        # Run shortly after startup (non-blocking)
        self._ran = False
        self.create_timer(0.1, self._run_once)

    def _run_once(self):
        if self._ran:
            return
        self._ran = True
        self._run_make_async()

    def _run_make_async(self):
        # Validate environment
        if not os.path.isdir(self.autopilot_path):
            self.get_logger().error(f'autopilot_path does not exist: {self.autopilot_path}')
            return
        if shutil.which('make') is None:
            self.get_logger().error('"make" not found in PATH')
            return

        # Example: make px4_sitl gazebo-classic_<model>__<world>
        target = f'gazebo-classic_{self.model_name}__{self.world_name}'
        cmd = ['make', 'px4_sitl', target]

        # Properly set env vars for PRIME offload (no shell tricks)
        env = os.environ.copy()
        env['__NV_PRIME_RENDER_OFFLOAD'] = '1'
        env['__GLX_VENDOR_LIBRARY_NAME'] = 'nvidia'

        self.get_logger().info(f'Running: {" ".join(cmd)} (cwd={self.autopilot_path})')
        self.get_logger().info(f'With env overrides: __NV_PRIME_RENDER_OFFLOAD=1, __GLX_VENDOR_LIBRARY_NAME=nvidia')

        try:
            # Start a new process group so we can kill all children cleanly (POSIX only)
            preexec = os.setsid if os.name == 'posix' else None

            self._proc = subprocess.Popen(
                cmd,
                cwd=self.autopilot_path,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                universal_newlines=True,
                env=env,
                preexec_fn=preexec,
            )
        except Exception as e:
            self.get_logger().error(f'Failed to start make: {e}')
            self._proc = None
            return

        # Stream output to ROS logger
        def _reader():
            try:
                assert self._proc.stdout is not None
                for line in self._proc.stdout:
                    line = line.rstrip('\n')
                    if line:
                        self.get_logger().info(line)
            except Exception as e:
                self.get_logger().warning(f'Log reader stopped: {e}')

        self._reader_thread = threading.Thread(target=_reader, daemon=True)
        self._reader_thread.start()

        # Watch process end in background
        def _waiter():
            rc = self._proc.wait()
            self.get_logger().info(f'make exited with code {rc}')

        threading.Thread(target=_waiter, daemon=True).start()

    def _stop_child(self):
        if self._proc is None:
            return
        try:
            self.get_logger().info('Stopping make process group...')
            if os.name == 'posix':
                # Kill the whole group
                os.killpg(os.getpgid(self._proc.pid), signal.SIGTERM)
            else:
                self._proc.terminate()
        except ProcessLookupError:
            pass
        except Exception as e:
            self.get_logger().warning(f'Failed to stop make: {e}')
        finally:
            self._proc = None

    def destroy_node(self):
        # Ensure child is stopped when node is destroyed
        self._stop_child()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BringUp()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
