#!/usr/bin/env python3
import time
from typing import Dict, Set, Tuple, List, Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from gazebo_msgs.msg import ContactsState
from std_msgs.msg import Bool


def get_collision_names(st) -> Tuple[str | None, str | None]:
    """
    Return a tuple (c1, c2) for a gazebo_msgs/ContactState entry.
    Some builds expose `collision1_name`/`collision2_name`; others may still have
    `collision1`/`collision2`. We try both defensively.
    """
    c1 = getattr(st, 'collision1_name', None) or getattr(st, 'collision1', None)
    c2 = getattr(st, 'collision2_name', None) or getattr(st, 'collision2', None)
    return c1, c2


class ContactListener(Node):
    """
    Subscribes to multiple Gazebo contact topics and reports start/end events.
    Also republishes a Bool per topic indicating whether any contact is active.
    """

    def __init__(self):
        super().__init__('contact_listener')

        # ---- Parameters (can be overridden in a launch file) ----
        self.declare_parameter(
            'topics',
            [
                '/islab/home/H1/contacts',
                '/islab/home/H2/contacts',
                '/islab/targets/T1/contacts',
                '/islab/targets/T2/contacts',
                '/islab/targets/T3/contacts',
                '/islab/targets/T4/contacts',
                '/islab/targets/T5/contacts',
            ],
        )
        self.declare_parameter('end_timeout_s', 0.20)  # debounce for END events
        self.declare_parameter('log_pairs', True)      # verbose pair logging

        topics: List[str] = self.get_parameter('topics').get_parameter_value().string_array_value
        self.end_timeout = float(self.get_parameter('end_timeout_s').value)
        self.log_pairs = bool(self.get_parameter('log_pairs').value)

        # QoS for sensor streams (best effort, keep last)
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # State per topic
        self.active_pairs: Dict[str, Set[Tuple[str, str]]] = {t: set() for t in topics}
        self.last_seen: Dict[Tuple[str, str], float] = {}  # global last-seen timestamps
        self.topic_contact_state: Dict[str, bool] = {t: False for t in topics}

        # Subscribers and publishers (Bool per topic)
        self.subs = []
        self.pubs: Dict[str, Any] = {}
        for t in topics:
            self.subs.append(self.create_subscription(
                ContactsState, t, lambda msg, topic=t: self._on_contact(msg, topic), qos
            ))
            out = t.replace('/contacts', '/is_contact')  # e.g., /islab/.../is_contact
            self.pubs[t] = self.create_publisher(Bool, out, 10)

        # Timer to sweep for ended contacts (debounce)
        self.timer = self.create_timer(0.05, self._sweep_timeouts)

        self.get_logger().info("ContactListener started.")
        # for t in topics:
        #     self.get_logger().info(f"  Listening: {t}")

    def _on_contact(self, msg: ContactsState, topic: str):
        now = time.time()
        current_pairs: Set[Tuple[str, str]] = set()

        for st in msg.states:
            c1, c2 = get_collision_names(st)
            if not c1 or not c2:
                # Uncomment to debug unexpected message schema:
                # self.get_logger().warn(f"ContactState missing collision names: {st}")
                continue
            pair = tuple(sorted([c1, c2]))
            current_pairs.add(pair)
            self.last_seen[pair] = now

        # Detect STARTs
        new_pairs = current_pairs - self.active_pairs[topic]
        for p in new_pairs:
            if self.log_pairs:
                self.get_logger().info(f"[CONTACT START] {topic} — {p}")
        if new_pairs:
            self.topic_contact_state[topic] = True

        # Update active pairs now; ENDs will be handled by timeout sweep
        self.active_pairs[topic].update(current_pairs)

        # Publish Bool state for this topic
        self._publish_bool(topic, True if self.active_pairs[topic] else False)

    def _sweep_timeouts(self):
        """Remove pairs not seen for `end_timeout` seconds and emit END events."""
        now = time.time()
        for topic, pairs in self.active_pairs.items():
            ended = []
            for p in list(pairs):
                last = self.last_seen.get(p, 0.0)
                if (now - last) > self.end_timeout:
                    ended.append(p)

            for p in ended:
                pairs.remove(p)
                if self.log_pairs:
                    self.get_logger().info(f"[CONTACT END]   {topic} — {p}")

            # Publish Bool reflecting any active contacts for this topic
            state = True if pairs else False
            if state != self.topic_contact_state[topic]:
                self.topic_contact_state[topic] = state
                self._publish_bool(topic, state)

    def _publish_bool(self, topic: str, state: bool):
        msg = Bool(data=state)
        self.pubs[topic].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ContactListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
