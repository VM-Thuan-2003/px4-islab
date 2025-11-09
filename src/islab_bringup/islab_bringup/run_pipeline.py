#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS 2: Run shell aliases `fast_dds` and `qgc`
---------------------------------------------
- Verifies that ~/.bashrc contains alias lines for `fast_dds` and `qgc`.
- Launches both commands via an interactive bash so aliases are available:
    bash -i -c "<alias>"
- Pipes stdout/stderr into ROS 2 logs.
- Gracefully terminates children on node shutdown.

Parameters (declare via ROS params or command line):
  - fast_dds_cmd (string, default: "fast_dds")
  - qgc_cmd      (string, default: "qgc")
  - start_qgc    (bool, default: True)
  - start_fastdds(bool, default: True)
  - delay_qgc_s  (float, default: 2.0)   # start QGC after FastDDS by N seconds
"""

import os
import re
import shlex
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Optional, List

import rclpy
from rclpy.node import Node


def _read_file_text(p: Path) -> str:
    try:
        return p.read_text(encoding="utf-8", errors="ignore")
    except Exception:
        return ""


def _has_alias(bashrc_text: str, alias_name: str) -> bool:
    # Match lines like: alias fast_dds='...'
    pattern = rf"^\s*alias\s+{re.escape(alias_name)}\s*=\s*['\"].+['\"]\s*$"
    return re.search(pattern, bashrc_text, flags=re.MULTILINE) is not None


class ProcPipeThread(threading.Thread):
    """Continuously read a pipe and forward to a logger method."""
    def __init__(self, stream, log_fn):
        super().__init__(daemon=True)
        self._stream = stream
        self._log_fn = log_fn
        self._alive = True

    def run(self):
        try:
            for line in iter(self._stream.readline, b''):
                if not self._alive:
                    break
                text = line.decode(errors="ignore").rstrip()
                if text:
                    self._log_fn(text)
        except Exception as e:
            self._log_fn(f"[pipe] error: {e}")

    def stop(self):
        self._alive = False


class AliasRunner(Node):
    def __init__(self):
        super().__init__("alias_runner")
        self.get_logger().info("AliasRunner starting…")

        # Parameters
        self.declare_parameter("fast_dds_cmd", "fast_dds")
        self.declare_parameter("qgc_cmd", "qgc")
        self.declare_parameter("start_fastdds", True)
        self.declare_parameter("start_qgc", False)
        self.declare_parameter("delay_qgc_s", 2.0)

        self.fast_dds_cmd = self.get_parameter("fast_dds_cmd").get_parameter_value().string_value
        self.qgc_cmd = self.get_parameter("qgc_cmd").get_parameter_value().string_value
        self.start_fastdds = self.get_parameter("start_fastdds").get_parameter_value().bool_value
        self.start_qgc = self.get_parameter("start_qgc").get_parameter_value().bool_value
        self.delay_qgc_s = float(self.get_parameter("delay_qgc_s").get_parameter_value().double_value)

        # Check aliases in ~/.bashrc (informational; we don't edit the file)
        bashrc = Path.home() / ".bashrc"
        text = _read_file_text(bashrc)
        if not text:
            self.get_logger().warn(f"Could not read {bashrc}. Aliases may not load.")

        for alias in ["fast_dds", "qgc"]:
            present = _has_alias(text, alias)
            msg = "FOUND" if present else "NOT FOUND"
            level = self.get_logger().info if present else self.get_logger().warn
            level(f"~/.bashrc alias check: {alias}: {msg}")
            if not present:
                self.get_logger().warn(
                    f"Alias '{alias}' missing in {bashrc}. "
                    f"Ensure a line like: alias {alias}='…' exists."
                )

        # Subprocess handles
        self.procs: List[subprocess.Popen] = []
        self.pipes: List[ProcPipeThread] = []

        # Launch
        if self.start_fastdds:
            self._spawn_alias(self.fast_dds_cmd, name="fast_dds")

        if self.start_qgc:
            # Optional delay to give Fast DDS/agents time to come up
            if self.delay_qgc_s > 0:
                self.get_logger().info(f"Delaying QGC start by {self.delay_qgc_s:.1f}s…")
                time.sleep(self.delay_qgc_s)
            self._spawn_alias(self.qgc_cmd, name="qgc")

        # Periodic timer to check child status
        self.timer = self.create_timer(1.0, self._poll_children)

    def _spawn_alias(self, alias_cmd: str, name: str):
        """
        Launch an alias via interactive bash so ~/.bashrc is sourced.
        We avoid passing user command through shell expansion ourselves.
        """
        self.get_logger().info(f"Starting alias `{name}` with: {alias_cmd}")
        # Use a new process group so we can terminate the whole tree
        preexec = os.setsid if hasattr(os, "setsid") else None
        # bash -i -c "<alias command>"
        # If the alias expands arguments, leave that to bash.
        try:
            p = subprocess.Popen(
                ["bash", "-i", "-c", alias_cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=preexec,
                env=os.environ.copy(),
            )
        except FileNotFoundError:
            self.get_logger().error("`bash` not found in PATH.")
            return
        except Exception as e:
            self.get_logger().error(f"Failed to start `{name}`: {e}")
            return

        self.procs.append(p)

        # Pipe threads
        out_th = ProcPipeThread(p.stdout, lambda s: self.get_logger().info(f"[{name}] {s}"))
        err_th = ProcPipeThread(p.stderr, lambda s: self.get_logger().error(f"[{name}] {s}"))
        out_th.start()
        err_th.start()
        self.pipes.extend([out_th, err_th])

    def _poll_children(self):
        """Check if any child processes exited; log return codes."""
        alive = []
        for p in self.procs:
            ret = p.poll()
            if ret is None:
                alive.append(p)
            else:
                self.get_logger().warn(f"Child exited (pid={p.pid}, code={ret}).")
        self.procs = alive

    def destroy_node(self):
        self.get_logger().info("Shutting down, terminating children…")
        self._terminate_children()
        super().destroy_node()

    def _terminate_children(self):
        # Stop pipe readers
        for th in self.pipes:
            th.stop()
        self.pipes.clear()

        # Send SIGTERM to each process group; fall back to kill.
        for p in self.procs:
            try:
                if p.poll() is None:
                    # Kill process group if possible
                    try:
                        pgid = os.getpgid(p.pid)
                        os.killpg(pgid, signal.SIGTERM)
                    except Exception:
                        p.terminate()
            except Exception:
                pass

        # Wait briefly, then SIGKILL if still alive
        deadline = time.time() + 5.0
        for p in self.procs:
            if p.poll() is None:
                timeout = max(0.0, deadline - time.time())
                try:
                    p.wait(timeout=timeout)
                except Exception:
                    pass
            if p.poll() is None:
                try:
                    pgid = os.getpgid(p.pid)
                    os.killpg(pgid, signal.SIGKILL)
                except Exception:
                    try:
                        p.kill()
                    except Exception:
                        pass
        self.procs.clear()


def main():
    rclpy.init()
    node = AliasRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

