#!/usr/bin/env python3
import os
import csv
import threading
import time
import traceback
from collections import deque

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from pylsl import StreamInfo, StreamOutlet, resolve_stream, StreamInlet

# ==========================================================
# fallback utils / strings (in case your package ones aren't importable)
# ==========================================================
try:
    from utils import sanitize_filename
    from ui_strings import session_string_dict, session_state_dict
except Exception:
    def sanitize_filename(s):
        return "".join(c for c in s if c.isalnum() or c in ("_", "-")).rstrip()

    session_string_dict = {
        "header_title": "=== UCD session ===",
        "header_id": "ID: ",
        "header_task": "Task: ",
        "header_trial": "Trial: ",
        "intro_description": "Enter ID, task, and trial.",
        "id_description": "Enter subject ID and press ENTER:",
        "task_description": "Enter task name and press ENTER:",
        "trial_description": "Enter trial number and press ENTER:",
        "recording_on_description": "▶ Recording... Press ENTER to stop, ESC to exit.",
        "recording_off_description": "Ready. Press ENTER to start. (ESC to exit)",
    }
    session_state_dict = {
        "intro": ["id"],
        "id": ["task"],
        "task": ["trial"],
    }

# readchar may not exist on the robot, so be defensive
try:
    from readchar import readkey, key
except Exception:
    class DummyKey:
        ENTER = "\n"
        ESC = "\x1b"
        BACKSPACE = "\x08"
        DELETE = "\x7f"
    key = DummyKey()
    def readkey():
        return None


# ==========================================================
# Session: UI + joystick→event LSL + events CSV (one per session)
# ==========================================================
class Session(object):
    def __init__(self, id_="", rate=60):
        id_ = sanitize_filename(id_)
        self._id = id_
        self._task = ""
        self._trial_number = 0
        self._print_count = 0
        self._queue = deque()
        self._rate = rospy.Rate(rate)
        self._is_recording = False
        self._base_dir = rospy.get_param("~base_dir", "/root/catkin_ws/src/morpheus_data/data/")
        os.makedirs(self._base_dir, exist_ok=True)

        # comment handling
        self._awaiting_comment = False
        self._pending_trial_base = ""
        self._comment_buf = ""

        # ROS pubs
        self._csv_basename_pub = rospy.Publisher("csv_basename", String, queue_size=1, latch=True)
        self._csv_trial_comment_pub = rospy.Publisher("csv_trial_comment", String, queue_size=1, latch=True)

        # ===== LSL event outlet (only button presses; single channel) =====
        evt_info = StreamInfo(
            name='Controller_Events',
            type='event',
            channel_count=1,
            nominal_srate=0,
            channel_format='float32',
            source_id='ps3_events_01'
        )
        self._lsl_event_outlet = StreamOutlet(evt_info)
        self._prev_buttons = None  # to detect rising edges

        # define which buttons = which event
        # TODO: set these to your actual indices from your Joy message
        self.SHARE_BTN = 8   # example index
        self.OPEN_BTN  = 1   # example index (square)
        self.CLOSE_BTN = 3   # example index (circle)
        self.HOME_BTN  = 9   # home / PS

        # ===== ONE events CSV per session =====
        self._current_trial = 0
        self._events_file = None
        self._events_writer = None
        self._events_path = None
        self._session_events_opened = False

    # ---------- helpers ----------
    def make_basename(self, id_, task, trial):
        sid = sanitize_filename(str(id_)) or "anon"
        stask = sanitize_filename(str(task)) or "task"
        try:
            tnum = int(trial)
        except Exception:
            tnum = 0
        return f"{sid}_{stask}_{tnum:03d}"

    def enqueue_string(self, queue, key_name, suffix=""):
        try:
            queue.append(session_string_dict[key_name] + str(suffix))
        except Exception:
            queue.append("")
        return queue

    def enqueue_header(self, queue, id_, task, trial):
        queue = self.enqueue_string(queue, "header_title")
        queue = self.enqueue_string(queue, "header_id", id_)
        queue = self.enqueue_string(queue, "header_task", task)
        queue = self.enqueue_string(queue, "header_trial", trial)
        return queue

    def dequeue(self, queue):
        for _ in range(self._print_count):
            print("\x1b[1A\x1b[2K", end="")
        self._print_count = len(queue)
        for _ in range(len(queue)):
            print(queue.popleft())
        return queue

    def get_key(self):
        try:
            return readkey()
        except Exception:
            return None

    def handle_key_events(self, input_buf="", event=None):
        """
        Extend the original handler with 'q' / 'Q' for quit.
        Returns input_buf and event, where event can be:
          - key.ENTER
          - key.ESC
          - 'QUIT'
          - None
        """
        k = self.get_key()
        if k is None:
            return input_buf, event

        # global quit
        if k in ("q", "Q"):
            return input_buf, "QUIT"

        if isinstance(k, str) and k not in (key.ENTER, key.ESC):
            input_buf += k
        elif k in (key.BACKSPACE, key.DELETE):
            if len(input_buf) > 0:
                input_buf = input_buf[:-1]
        elif k == key.ENTER:
            event = key.ENTER
        elif k == key.ESC:
            event = key.ESC
        return input_buf, event

    # ---------- events CSV helpers (one per session) ----------
    def _open_session_events_file(self, id_, task):
        """
        Open a single events CSV for the whole session:
        <ID>_<task>_events.csv
        """
        if self._session_events_opened:
            return

        sid = sanitize_filename(str(id_)) or "anon"
        stask = sanitize_filename(str(task)) or "task"
        base = f"{sid}_{stask}"
        self._events_path = os.path.join(self._base_dir, f"{base}_events.csv")

        new_file = not os.path.exists(self._events_path)
        self._events_file = open(self._events_path, "a", newline="")
        self._events_writer = csv.writer(self._events_file)

        if new_file:
            self._events_writer.writerow(["ros_time", "event", "trial", "button_index"])

        self._session_events_opened = True
        print("[Session] Events file:", self._events_path)

    def _log_event(self, event_name, button_index=None):
        """
        Write one event row: time, event label, current trial, optional button index.
        """
        if not self._session_events_opened or self._events_writer is None:
            return
        t = rospy.get_time()
        self._events_writer.writerow([
            t,
            event_name,
            self._current_trial,
            button_index if button_index is not None else ""
        ])
        # flush so it's safe if things crash
        self._events_file.flush()

    # ---------- ROS Joy → LSL events + CSV ----------
    def joy_cb(self, msg: Joy):
        # if we don't have a previous state yet, assume all buttons were 0
        if self._prev_buttons is None:
            self._prev_buttons = [0] * len(msg.buttons)
            # don't return — we want to detect a press even on the first message

        def pressed(idx):
            return (
                idx < len(msg.buttons)
                and msg.buttons[idx] == 1
                and self._prev_buttons[idx] == 0
            )

        ts = rospy.get_time()

        # SHARE
        if pressed(self.SHARE_BTN):
            self._lsl_event_outlet.push_sample([8.0], ts)
            if self._is_recording:
                self._log_event("share_button", self.SHARE_BTN)

        # SQUARE
        if pressed(self.OPEN_BTN):
            self._lsl_event_outlet.push_sample([1.0], ts)
            if self._is_recording:
                self._log_event("square_button", self.OPEN_BTN)

        # CIRCLE
        if pressed(self.CLOSE_BTN):
            self._lsl_event_outlet.push_sample([3.0], ts)
            if self._is_recording:
                self._log_event("circle_button", self.CLOSE_BTN)

        # HOME (optional)
        if pressed(self.HOME_BTN):
            self._lsl_event_outlet.push_sample([9.0], ts)
            if self._is_recording:
                self._log_event("home_button", self.HOME_BTN)

        # update previous
        self._prev_buttons = list(msg.buttons)

    # ---------- announce-only start/stop ----------
    def _start_recording(self, id_, task, trial):
        # open the single session events file (once)
        self._open_session_events_file(id_, task)

        # store current trial as integer
        try:
            self._current_trial = int(trial)
        except Exception:
            self._current_trial = 0

        base = self.make_basename(id_, task, trial)

        # log start event
        self._log_event("start_recording")

        # publish basename for other recorders
        self._csv_basename_pub.publish(String(data=base))
        self._is_recording = True

    def _stop_recording(self):
        if self._is_recording:
            # log stop event
            self._log_event("stop_recording")

        self._csv_basename_pub.publish(String(data=""))
        self._is_recording = False

    # ---------- UI loop ----------
    def prompt(self, state="", id_="", task="", trial=""):
        if state == "":
            state = "intro"
        input_buf = ""

        while not rospy.is_shutdown():
            self._queue = self.enqueue_header(deque(), id_, task, trial)

            if self._awaiting_comment:
                self._queue.append("Enter trial comment (e.g. 'good' or 'bad'):")
                self._queue.append(self._comment_buf)
                self._queue.append("(Press q to quit without saving a comment.)")
            elif state == "recording":
                keyname = "recording_on_description" if self._is_recording else "recording_off_description"
                self._queue = self.enqueue_string(self._queue, keyname)
                status = "[REC]" if self._is_recording else "[IDLE]"
                next_base = self.make_basename(id_, task, trial)
                self._queue.append(f"Status: {status} | CSV base: {next_base}")
                self._queue.append("Press ENTER to toggle recording, ESC to exit recording, q to quit program.")
            else:
                self._queue = self.enqueue_string(self._queue, state + "_description")
                self._queue.append("(Press q to quit program at any time.)")

            self._queue.append(input_buf)
            self._queue = self.dequeue(self._queue)

            event = None

            if self._awaiting_comment:
                # comment input
                self._comment_buf, event = self.handle_key_events(self._comment_buf, None)

                if event == "QUIT":
                    if self._is_recording:
                        self._stop_recording()
                    rospy.signal_shutdown("User requested quit (q)")
                    break

                if event == key.ENTER:
                    msg = f"{self._pending_trial_base}::{self._comment_buf}"
                    self._csv_trial_comment_pub.publish(String(data=msg))
                    # save to file
                    try:
                        base, comment = msg.split("::", 1)
                        comment_file = os.path.join(self._base_dir, f"{base}_comment.txt")
                        with open(comment_file, "w") as f:
                            f.write(comment.strip() + "\n")
                        print("Saved comment to:", comment_file)
                    except Exception as e:
                        print("Failed to save comment:", e)
                    self._awaiting_comment = False
                    self._pending_trial_base = ""
                    self._comment_buf = ""
                elif event == key.ESC:
                    self._awaiting_comment = False
                    self._pending_trial_base = ""
                    self._comment_buf = ""
            else:
                # normal input
                input_buf, event = self.handle_key_events(input_buf, None)

                if event == "QUIT":
                    if self._is_recording:
                        self._stop_recording()
                    rospy.signal_shutdown("User requested quit (q)")
                    break

                if state == "recording":
                    if event == key.ENTER:
                        if self._is_recording:
                            finished_base = self.make_basename(id_, task, trial)
                            self._stop_recording()
                            self._pending_trial_base = finished_base
                            self._awaiting_comment = True
                            try:
                                trial = int(trial)
                            except Exception:
                                trial = 0
                            trial += 1
                        else:
                            self._start_recording(id_, task, trial)
                        continue
                    if event == key.ESC:
                        self._stop_recording()
                        break
                else:
                    if event == key.ENTER:
                        if state == "id":
                            id_ = sanitize_filename(input_buf)
                        elif state == "task":
                            task = sanitize_filename(input_buf)
                        elif state == "trial":
                            try:
                                trial = int(input_buf or 0)
                            except Exception:
                                trial = 0
                            input_buf = ""
                            self.prompt("recording", id_, task, trial)
                            continue
                        for child_state in session_state_dict.get(state, []):
                            self.prompt(child_state, id_, task, trial)
                        input_buf = ""
                        continue
                    if event == key.ESC:
                        break

            self._rate.sleep()


# ==========================================================
# Multi LSL Recorder: Neon Gaze + Controller_Events → ONE CSV
# ==========================================================
class MultiLSLRecorder(object):
    """
    Modes:
      - combined:   <basename>_lsl.csv  (sparse rows; events one-shot; event rows also include latest Neon)
      - split:      <basename>_gaze.csv, <basename>_controller.csv
      - both:       all three files

    Combined CSV columns:
      ros_time, sys_utc_ns,
      neon_time, neon_ch0..N,
      evt_time,  evt_ch0..M
    """
    def __init__(self, save_dir, write_mode="combined"):
        self.save_dir = save_dir
        os.makedirs(self.save_dir, exist_ok=True)
        self.write_mode = write_mode  # "combined" | "split" | "both"

        self.stream_specs = [
            {"match": "Neon Companion_Neon Gaze", "label": "neon"},
            {"match": "Controller_Events",        "label": "evt"},
        ]
        self.inlets = {}
        self.ch_counts = {}
        self.last_samples = {}
        self.last_ts = {}

        # Writers/paths
        self.current_basename = ""
        self.comb_file = None
        self.comb_writer = None
        self.gaze_file = None
        self.gaze_writer = None
        self.ctrl_file = None
        self.ctrl_writer = None

        self.lock = threading.Lock()

        self._connect_streams_once(print_all=True)
        rospy.Subscriber("csv_basename", String, self.basename_cb)

        self.keep_running = True
        t = threading.Thread(target=self._reconnect_loop_safe, daemon=True)
        t.start()

    # ---------- LSL connect ----------
    def _connect_streams_once(self, print_all=False):
        try:
            streams = resolve_stream()
        except Exception:
            rospy.logerr("[MultiLSL] resolve_stream failed:\n%s", traceback.format_exc())
            return

        if print_all:
            rospy.loginfo("[MultiLSL] Available LSL streams:")
            for s in streams:
                rospy.loginfo("   - %s (%s)", s.name(), s.type())

        for spec in self.stream_specs:
            match = spec["match"]; label = spec["label"]
            if label in self.inlets:
                continue
            chosen = None
            for s in streams:
                if match.lower() == s.name().lower():
                    chosen = s; break
            if chosen is None:
                for s in streams:
                    if match.lower() in s.name().lower():
                        chosen = s; break
            if chosen is None:
                rospy.logwarn("[MultiLSL] stream '%s' not found yet.", match)
                continue
            try:
                inlet = StreamInlet(chosen, max_buflen=360)
                ch = inlet.info().channel_count()
                self.inlets[label] = inlet
                self.ch_counts[label] = ch
                rospy.loginfo("[MultiLSL] CONNECTED to '%s' as '%s' (%d ch)", chosen.name(), label, ch)
            except Exception:
                rospy.logerr("[MultiLSL] failed to connect '%s':\n%s", match, traceback.format_exc())

    def _reconnect_loop_safe(self):
        try:
            while self.keep_running and not rospy.is_shutdown():
                missing = [s for s in self.stream_specs if s["label"] not in self.inlets]
                if missing:
                    self._connect_streams_once(print_all=False)
                time.sleep(2.0)
        except Exception:
            rospy.logerr("[MultiLSL] reconnect crashed:\n%s", traceback.format_exc())

    # ---------- File handling ----------
    def _close_all_nolock(self):
        for fattr, wattr in (("comb_file","comb_writer"),
                             ("gaze_file","gaze_writer"),
                             ("ctrl_file","ctrl_writer")):
            f = getattr(self, fattr)
            if f is not None:
                f.close()
                setattr(self, fattr, None)
                setattr(self, wattr, None)
        self.current_basename = ""
        rospy.loginfo("[MultiLSL] closed files")

    def basename_cb(self, msg: String):
        name = msg.data.strip()
        with self.lock:
            if name == "":
                self._close_all_nolock()
                return

            # rotate files
            self._close_all_nolock()
            self.current_basename = name

            neon_ch = self.ch_counts.get("neon", 0)
            evt_ch  = self.ch_counts.get("evt",  0)

            if self.write_mode in ("combined", "both"):
                comb_path = os.path.join(self.save_dir, f"{name}_lsl.csv")
                self.comb_file = open(comb_path, "w", newline="")
                self.comb_writer = csv.writer(self.comb_file)
                comb_header = (["ros_time","sys_utc_ns","neon_time"] +
                               [f"neon_ch{i}" for i in range(neon_ch)] +
                               ["evt_time"] +
                               [f"evt_ch{i}" for i in range(evt_ch)])
                self.comb_writer.writerow(comb_header)
                rospy.loginfo("[MultiLSL] combined -> %s", comb_path)

            if self.write_mode in ("split", "both"):
                gaze_path = os.path.join(self.save_dir, f"{name}_gaze.csv")
                self.gaze_file = open(gaze_path, "w", newline="")
                self.gaze_writer = csv.writer(self.gaze_file)
                self.gaze_writer.writerow(["ros_time","sys_utc_ns","neon_time"] +
                                          [f"neon_ch{i}" for i in range(neon_ch)])
                rospy.loginfo("[MultiLSL] gaze -> %s", gaze_path)

                ctrl_path = os.path.join(self.save_dir, f"{name}_controller.csv")
                self.ctrl_file = open(ctrl_path, "w", newline="")
                self.ctrl_writer = csv.writer(self.ctrl_file)
                self.ctrl_writer.writerow(["ros_time","sys_utc_ns","evt_time"] +
                                          [f"evt_ch{i}" for i in range(evt_ch)])
                rospy.loginfo("[MultiLSL] controller -> %s", ctrl_path)

    # ---------- Helpers ----------
    @staticmethod
    def _round_neon(vals):
        out = []
        for i, v in enumerate(vals):
            if isinstance(v, (int, float)) and i in (0, 1):
                out.append(round(v, 4))   # neon ch0/ch1 → 4 decimals
            elif isinstance(v, (int, float)):
                out.append(v)
            else:
                out.append(str(v))
        return out

    def _write_gaze_row(self, ts, sample):
        if self.gaze_writer is None: return
        ch = self.ch_counts.get("neon", 0)
        vals = list(sample[:ch]) + [""] * max(0, ch - len(sample))
        row = [rospy.get_time(), time.time_ns(), ts] + self._round_neon(vals)
        self.gaze_writer.writerow(row)

    def _write_ctrl_row(self, ts, sample):
        if self.ctrl_writer is None: return
        ch = self.ch_counts.get("evt", 0)
        vals = list(sample[:ch]) + [""] * max(0, ch - len(sample))
        vals = [v if isinstance(v,(int,float)) else str(v) for v in vals]
        row = [rospy.get_time(), time.time_ns(), ts] + vals
        self.ctrl_writer.writerow(row)

    def _write_combined_row(self, label, ts, sample):
        if self.comb_writer is None: return

        neon_ch = self.ch_counts.get("neon", 0)
        evt_ch  = self.ch_counts.get("evt",  0)

        # By default, put blanks for both blocks
        neon_time, neon_vals = "", [""] * neon_ch
        evt_time,  evt_vals  = "", [""] * evt_ch

        # If Neon tick → fill neon block
        if label == "neon":
            nv = list(sample[:neon_ch]) + [""] * max(0, neon_ch - len(sample))
            neon_time = ts
            neon_vals = self._round_neon(nv)

        # If Event tick → fill evt block AND snapshot latest Neon (if we have it)
        elif label == "evt":
            ev = list(sample[:evt_ch]) + [""] * max(0, evt_ch - len(sample))
            evt_time = ts
            evt_vals = [v if isinstance(v,(int,float)) else str(v) for v in ev]

            # include freshest Neon snapshot if available
            if "neon" in self.last_samples:
                neon_time = self.last_ts.get("neon", "")
                nv = list(self.last_samples["neon"][:neon_ch]) + [""] * max(0, neon_ch - len(self.last_samples["neon"]))
                neon_vals = self._round_neon(nv)

        row = [rospy.get_time(), time.time_ns(), neon_time] + neon_vals + [evt_time] + evt_vals
        self.comb_writer.writerow(row)

    # ---------- Main loop ----------
    def run(self):
        try:
            rate = rospy.Rate(300)
            while not rospy.is_shutdown():
                for label, inlet in list(self.inlets.items()):
                    if inlet is None: continue
                    sample, ts = inlet.pull_sample(timeout=0.0)
                    if sample is None: continue

                    # keep caches always updated
                    self.last_samples[label] = sample
                    self.last_ts[label] = ts

                    # if an event arrived, greedily drain Neon to make snapshot fresher
                    if label == "evt" and "neon" in self.inlets:
                        nin = self.inlets["neon"]
                        while True:
                            ns, nt = nin.pull_sample(timeout=0.0)
                            if ns is None: break
                            self.last_samples["neon"] = ns
                            self.last_ts["neon"] = nt

                    with self.lock:
                        if self.write_mode in ("split", "both"):
                            if label == "neon":
                                self._write_gaze_row(ts, sample)
                            elif label == "evt":
                                self._write_ctrl_row(ts, sample)

                        if self.write_mode in ("combined", "both"):
                            self._write_combined_row(label, ts, sample)

                rate.sleep()
        except Exception:
            rospy.logerr("[MultiLSL] run crashed:\n%s", traceback.format_exc())


# main
# ==========================================================
def main():
    rospy.init_node('UCDsession')

    session = Session()
    rospy.Subscriber("/vx300s/joy", Joy, session.joy_cb)

    recorder = MultiLSLRecorder(save_dir=session._base_dir, write_mode="both")
    rec_thread = threading.Thread(target=recorder.run)
    rec_thread.daemon = True
    rec_thread.start()

    rospy.on_shutdown(lambda: session._csv_basename_pub.publish(String(data="")))

    try:
        session.prompt()
    except KeyboardInterrupt:
        # ensure we stop cleanly on Ctrl+C
        if session._is_recording:
            session._stop_recording()
        rospy.signal_shutdown("KeyboardInterrupt")


if __name__ == "__main__":
    main()
