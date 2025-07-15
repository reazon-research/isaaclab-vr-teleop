import socket
import json
import threading
import numpy as np
from scipy.spatial.transform import Rotation as R


class QuestVRTeleop:
    def __init__(self, ip="0.0.0.0", port=5006):
        self.ip = ip
        self.port = port

        self.buffer = ""
        self.last_pose = None  # (position, rotation matrix)
        self.latest_data = None  # latest parsed message

        self.lock = threading.Lock()
        self.running = True

        # Scaling factors
        self.pos_scale = 15.0
        self.rot_scale = 6.0

        # For smoothing
        self.prev_delta = np.zeros(6)

        # Start listener thread
        self.thread = threading.Thread(target=self._recv_loop)
        self.thread.daemon = True
        self.thread.start()

    def _recv_loop(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((self.ip, self.port))
            s.listen(1)
            print(f"[TCP] Waiting for VR client on {self.ip}:{self.port}...")
            conn, addr = s.accept()
            print(f"[TCP] ✅ Connected by {addr}")
            with conn:
                while self.running:
                    try:
                        data = conn.recv(4096)
                        if not data:
                            continue
                        self.buffer += data.decode("utf-8")

                        while "\n" in self.buffer:
                            line, self.buffer = self.buffer.split("\n", 1)
                            msg = json.loads(line.strip())
                            with self.lock:
                                self.latest_data = msg

                    except Exception as e:
                        print(f"[TCP] ❌ Error in recv: {e}")
                        break

    def reset(self):
        with self.lock:
            self.last_pose = None
            self.prev_delta = np.zeros(6)

    def add_callback(self, *args, **kwargs):
        pass  # IsaacLab compatibility placeholder

    def advance(self):
        with self.lock:
            data = self.latest_data
            if not data:
                print("[VR DEBUG] ⚠️ No data received yet.")
                return np.zeros(6), False

            joint_pose = data["joints"].get("45")
            if joint_pose is None:
                print("[VR DEBUG] ⚠️ Joint 45 not found in message.")
                return np.zeros(6), False

            # Extract current pose
            pos = np.array([joint_pose["x"], joint_pose["y"], joint_pose["z"]])
            # Quaternion gets converted into a rotation matrix rot
            rot = R.from_quat([
                joint_pose["qx"],
                joint_pose["qy"],
                joint_pose["qz"],
                joint_pose["qw"]
            ]).as_matrix()

            # First frame: initialize
            if self.last_pose is None:
                self.last_pose = (pos, rot)
                return np.zeros(6), False

            last_pos, last_rot = self.last_pose
            delta_pos = pos - last_pos
            delta_rot_mat = rot @ last_rot.T
            # This turns a 3×3 rotation matrix into a rotation vector
            delta_rot = R.from_matrix(delta_rot_mat).as_rotvec()
            self.last_pose = (pos, rot)

            # Apply scaling
            scaled_delta = np.concatenate([
                self.pos_scale * delta_pos,
                self.rot_scale * delta_rot
            ])

            # Optional smoothing (low-pass filter)
            delta_pose = 0.7 * self.prev_delta + 0.3 * scaled_delta
            self.prev_delta = delta_pose

            # Gripper control
            finger_dist = data.get("right_finger_distance", 1.0)
            gripper_closed = finger_dist < 0.03

            # Debug
            print(f"[VR DEBUG] 🧠 Pose 45: pos=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
            print(f"[VR DEBUG] ✋ Finger distance: {finger_dist:.3f}")
            print(f"[VR DEBUG] Δpos: {delta_pos}, Δrot: {delta_rot}, Gripper: {'Closed' if gripper_closed else 'Open'}")

            return delta_pose, gripper_closed

    def close(self):
        self.running = False
        self.thread.join()
