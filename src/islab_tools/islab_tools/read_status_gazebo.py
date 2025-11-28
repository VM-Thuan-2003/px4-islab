#!/usr/bin/env python3
import time
from typing import Dict, Set, Tuple, List, Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from gazebo_msgs.msg import ContactsState
from std_msgs.msg import Bool


def get_collision_names(st) -> Tuple[str | None, str | None]:
    """Return collision names in a defensive way."""
    c1 = getattr(st, 'collision1_name', None) or getattr(st, 'collision1', None)
    c2 = getattr(st, 'collision2_name', None) or getattr(st, 'collision2', None)
    return c1, c2


class ContactListener(Node):
    """
    Subscribes to multiple Gazebo contact topics and publishes only filtered contacts:

    RULES:
    - Home pads (H1, H2): require collision name containing "islab::base_link"
    - Targets (T1..T5):   require collision name containing "ball"
    """

    def __init__(self):
        super().__init__('contact_listener')

        # ---- Parameters ----
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
        self.declare_parameter('end_timeout_s', 0.20)
        self.declare_parameter('log_pairs', True)

        topics: List[str] = self.get_parameter('topics').get_parameter_value().string_array_value
        self.end_timeout = float(self.get_parameter('end_timeout_s').value)
        self.log_pairs = bool(self.get_parameter('log_pairs').value)

        # Filtering rules
        self.filter_home = "islab::base_link"
        self.filter_target = "ball"

        # QoS
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # States
        self.active_pairs: Dict[str, Set[Tuple[str, str]]] = {t: set() for t in topics}
        self.last_seen: Dict[Tuple[str, str], float] = {}
        self.topic_contact_state: Dict[str, bool] = {t: False for t in topics}

        # Subscribers and publishers
        self.subs = []
        self.pubs: Dict[str, Any] = {}
        for t in topics:
            self.subs.append(self.create_subscription(
                ContactsState, t,
                lambda msg, topic=t: self._on_contact(msg, topic),
                qos
            ))
            out = t.replace('/contacts', '/is_contact')
            self.pubs[t] = self.create_publisher(Bool, out, 10)

        # Timer
        self.timer = self.create_timer(0.05, self._sweep_timeouts)

        self.get_logger().info("ContactListener with HOME/TARGET filters started.")

    # ------------------------------------------------------------

    def _topic_is_home(self, topic: str) -> bool:
        return "/home/" in topic

    def _topic_is_target(self, topic: str) -> bool:
        return "/targets/" in topic

    # ------------------------------------------------------------

    def _contact_pass_filter(self, topic: str, c1: str, c2: str) -> bool:
        """Apply the correct filter based on topic."""

        c1_l = c1.lower()
        c2_l = c2.lower()

        # HOME = needs "islab::base_link"
        if self._topic_is_home(topic):
            return (self.filter_home.lower() in c1_l) or (self.filter_home.lower() in c2_l)

        # TARGET = needs "ball"
        if self._topic_is_target(topic):
            return (self.filter_target.lower() in c1_l) or (self.filter_target.lower() in c2_l)

        # default — accept everything
        return True

    # ------------------------------------------------------------

    def _on_contact(self, msg: ContactsState, topic: str):
        now = time.time()
        current_pairs: Set[Tuple[str, str]] = set()

        for st in msg.states:
            c1, c2 = get_collision_names(st)
            if not c1 or not c2:
                continue

            # Apply filter rule
            if not self._contact_pass_filter(topic, c1, c2):
                continue

            pair = tuple(sorted([c1, c2]))
            current_pairs.add(pair)
            self.last_seen[pair] = now

        # Detect new contacts
        new_pairs = current_pairs - self.active_pairs[topic]
        for p in new_pairs:
            if self.log_pairs:
                self.get_logger().info(f"[CONTACT START] {topic} — {p}")

        if new_pairs:
            self.topic_contact_state[topic] = True

        # Update list
        self.active_pairs[topic].update(current_pairs)

        # Publish current state
        self._publish_bool(topic, bool(self.active_pairs[topic]))

    # ------------------------------------------------------------

    def _sweep_timeouts(self):
        now = time.time()

        for topic, pairs in self.active_pairs.items():
            ended = []

            for p in list(pairs):
                if (now - self.last_seen.get(p, 0)) > self.end_timeout:
                    ended.append(p)

            for p in ended:
                pairs.remove(p)
                if self.log_pairs:
                    self.get_logger().info(f"[CONTACT END] {topic} — {p}")

            new_state = bool(pairs)
            if new_state != self.topic_contact_state[topic]:
                self.topic_contact_state[topic] = new_state
                self._publish_bool(topic, new_state)

    # ------------------------------------------------------------

    def _publish_bool(self, topic: str, state: bool):
        msg = Bool(data=state)
        self.pubs[topic].publish(msg)


# ------------------------------------------------------------

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
