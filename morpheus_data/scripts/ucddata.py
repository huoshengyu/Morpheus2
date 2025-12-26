#!/usr/bin/env python3
# data.py
# Time-synchronous event writer for rosbag files (plus single-event markers)

import json
from typing import Optional, Dict

import rospy
import rosbag
from rosbag import Compression
from std_msgs.msg import Int32, String
import time


class UCDDATA:
    """
    Minimal rosbag writer with:
      - open/append/close helpers
      - single event markers with optional JSON metadata
      - time-synchronous event stream at fixed Hz using rospy.Timer
    """

    def __init__(
        self,
        f: str,
        compression: Compression = Compression.NONE,
        chunk_threshold: int = 768 * 1024,
        allow_unindexed: bool = False,
        options=None,
        skip_index: bool = False,
    ):
        self.f = f
        self.compression = compression
        self.chunk_threshold = chunk_threshold
        self.allow_unindexed = allow_unindexed
        self.options = options
        self.skip_index = skip_index

        self._bag: Optional[rosbag.Bag] = None
        self._is_open: bool = False

        # Timer-stream state
        self._timer: Optional[rospy.Timer] = None
        self._timer_code: int = 0
        self._event_topic_timer: str = "/trial_event"
        self._meta_timer: Optional[Dict] = None
        self._use_wall_time: bool = False

        # (Optional) message-aligned stream state (off by default)
        self._streaming: bool = False
        self._ref_sub = None
        self._stream_code: int = 0
        self._event_topic_stream: str = "/trial_event"
        self._meta_stream: Optional[Dict] = None
        self._use_header_stamp_stream: bool = True

    # --------------------- bag lifecycle ---------------------

    def read(self):
        self.open(self.f, "r", self.allow_unindexed)

    def write(self):
        self.open(self.f, "w", self.allow_unindexed)

    def append(self):
        self.open(self.f, "a", self.allow_unindexed)

    def open(self, f: str, mode: str, allow_unindexed: bool = False):
        if self._is_open:
            rospy.loginfo(f"Rosbag already open, closing then reopening in mode {mode}.")
            self.close()
        self._bag = rosbag.Bag(
            f, mode, self.compression, self.chunk_threshold, allow_unindexed, self.options, self.skip_index
        )
        self._is_open = True
        rospy.loginfo(f"Opened bag: {f} (mode={mode})")

    def close(self):
        # Stop any active streams/timers first
        self.stop_event_timer()
        self.stop_event_stream()

        if self._bag is not None:
            self._bag.close()
            self._bag = None
            rospy.loginfo("Closed bag.")
        self._is_open = False

    # --------------------- single event ---------------------

    def write_event(
        self,
        code: int,
        meta: Optional[Dict] = None,
        topic: str = "/trial_event",
        stamp: Optional[rospy.Time] = None,
    ):
        """Write a single event marker (Int32) with optional JSON metadata at a specified (or now) stamp."""
        if not self._is_open or self._bag is None:
            rospy.logwarn("write_event called but bag is not open")
            return
        t = stamp if stamp is not None else rospy.Time.now()
        self._bag.write(topic, Int32(data=int(code)), t)
        if meta:
            self._bag.write(topic + "_meta", String(data=json.dumps(meta)), t)

    # --------------------- fixed-rate, time-synced events ---------------------

    def start_event_timer(
        self,
        rate_hz: float,
        code: int = 0,
        event_topic: str = "/trial_event",
        meta: Optional[Dict] = None,
        align_immediately: bool = False,
        use_wall_time: bool = False,   # choose time base
    ):
        """
        Emit events at a fixed, phase-stable rate.
        If use_wall_time=True, stamps with wall clock (ignores /clock).
        Else stamps with ROS time using TimerEvent.current_expected (follows /clock if use_sim_time).
        """
        if not self._is_open or self._bag is None:
            rospy.logwarn("start_event_timer called but bag is not open")
            return

        self.stop_event_timer()  # ensure only one timer is active

        self._timer_code = int(code)
        self._event_topic_timer = event_topic
        self._meta_timer = meta
        self._use_wall_time = use_wall_time

        period = rospy.Duration.from_sec(1.0 / float(rate_hz))

        def _timer_cb(event: rospy.TimerEvent):
            if not self._is_open or self._bag is None:
                return

            if self._use_wall_time:
                stamp = rospy.Time.from_sec(time.time())
            else:
                stamp = event.current_expected
                try:
                    if hasattr(stamp, "to_sec") and stamp.to_sec() == 0.0:
                        stamp = rospy.Time.now()
                except Exception:
                    stamp = rospy.Time.now()

            self._bag.write(self._event_topic_timer, Int32(data=self._timer_code), stamp)
            if self._meta_timer:
                self._bag.write(self._event_topic_timer + "_meta",
                                String(data=json.dumps(self._meta_timer)), stamp)

        # reset=False aligns to the next period boundary unless align_immediately=True
        self._timer = rospy.Timer(period, _timer_cb, oneshot=False, reset=not align_immediately)
        rospy.loginfo(
            f"Started time-synced event timer on {event_topic} at {rate_hz:.3f} Hz "
            f"({'wall' if use_wall_time else 'ros'} time)"
        )

    def stop_event_timer(self):
        if self._timer is not None:
            self._timer.shutdown()
            self._timer = None
            rospy.loginfo("Stopped time-synced event timer.")

    # --------------------- OPTIONAL: message-aligned events ---------------------

    def start_event_stream(
        self,
        ref_topic: str,
        code: int = 0,
        event_topic: str = "/trial_event",
        meta: Optional[Dict] = None,
        use_header_stamp: bool = True,
        queue_size: int = 10,
    ):
        """Write an event on every message of ref_topic, stamped with header.stamp if available."""
        if not self._is_open or self._bag is None:
            rospy.logwarn("start_event_stream called but bag is not open")
            return
        if self._streaming:
            self.stop_event_stream()

        self._streaming = True
        self._stream_code = int(code)
        self._event_topic_stream = event_topic
        self._meta_stream = meta
        self._use_header_stamp_stream = use_header_stamp

        def _ref_cb(msg):
            if not self._streaming or not self._is_open or self._bag is None:
                return
            stamp = None
            if self._use_header_stamp_stream and hasattr(msg, "header") and hasattr(msg.header, "stamp"):
                stamp = msg.header.stamp
                try:
                    if hasattr(stamp, "to_sec") and stamp.to_sec() == 0.0:
                        stamp = None
                except Exception:
                    stamp = None
            if stamp is None:
                stamp = rospy.Time.now()

            self._bag.write(self._event_topic_stream, Int32(data=self._stream_code), stamp)
            if self._meta_stream:
                self._bag.write(self._event_topic_stream + "_meta",
                                String(data=json.dumps(self._meta_stream)), stamp)

        self._ref_sub = rospy.Subscriber(ref_topic, rospy.AnyMsg, _ref_cb, queue_size=queue_size)
        rospy.loginfo(f"Started message-aligned event stream on {event_topic} mirroring {ref_topic}")

    def stop_event_stream(self):
        if self._ref_sub is not None:
            self._ref_sub.unregister()
            self._ref_sub = None
        if self._streaming:
            rospy.loginfo("Stopped message-aligned event stream.")
        self._streaming = False


# --------------------- example usage as a node ---------------------

def _example():
    """
    Minimal example:
      - append to an existing bag (or create if missing)
      - write START marker with metadata
      - start a 50 Hz time-synced heartbeat on /trial_event
      - spin until Ctrl+C (on_shutdown closes and writes FINISH)
    """
    rospy.init_node("ucddata_writer", anonymous=False)

    bag_path = rospy.get_param("~bag_path", default="1_1_001.bag")
    rate_hz = float(rospy.get_param("~rate_hz", default=50.0))
    use_wall = bool(rospy.get_param("~use_wall_time", default=False))

    bagger = UCDDATA(bag_path)
    bagger.append()

    def _on_shutdown():
        try:
            bagger.write_event(2, meta={"result": "shutdown"}, topic="/trial_event")
        except Exception:
            pass
        bagger.close()

    rospy.on_shutdown(_on_shutdown)

    # explicit START (code=1)
    bagger.write_event(1, meta={"subject": 1, "task": "demo", "trial": 1}, topic="/trial_event")

    # time-synced heartbeat at rate_hz (code=0)
    bagger.start_event_timer(rate_hz=rate_hz, code=0, event_topic="/trial_event",
                             meta={"source": f"timer_{rate_hz:.1f}hz"},
                             align_immediately=False, use_wall_time=use_wall)

    rospy.loginfo("Running... Press Ctrl+C to stop.")
    rospy.spin()  # _on_shutdown will fire on Ctrl+C


if __name__ == "__main__":
    _example()
