import sys
import os
import time
import numpy as np
import termios
import tty
import fcntl

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tongverse.env import Env, app
from biped_challenge.demo.DemoController import DemoController

TASK_SEED = 666

LEFT_ARM_INDICES = [0, 2, 4, 6, 8, 10, 12]
DELTA = 5

def init_keyboard():
    fd = sys.stdin.fileno()
    # save original terminal settings
    orig_term = termios.tcgetattr(fd)
    # switch to cbreak (raw) mode
    tty.setcbreak(fd)
    # re-enable echo
    attrs = termios.tcgetattr(fd)
    attrs[3] |= termios.ECHO         # LFLAG index 3, add the ECHO bit
    termios.tcsetattr(fd, termios.TCSADRAIN, attrs)
    # make stdin non-blocking
    orig_flags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, orig_flags | os.O_NONBLOCK)
    return orig_term, orig_flags

def restore_keyboard(orig_term, orig_flags):
    fd = sys.stdin.fileno()
    termios.tcsetattr(fd, termios.TCSADRAIN, orig_term)
    fcntl.fcntl(fd, fcntl.F_SETFL, orig_flags)

def read_key():
    try:
        ch = sys.stdin.read(1)
        return ch
    except IOError:
        return None

def main():
    # --- setup keyboard ---
    orig_term, orig_flags = init_keyboard()
    print("Controls: Q/A,W/S,E/D,R/F,T/G,Y/H,U/J to inc/dec joints 1–7. ESC to quit.")

    # initialize controller & sim
    controller = DemoController()
    controller.start_launch()
    env = Env(TASK_SEED, task_id="TaskTwo")
    env.reset()

    joint_angles = [0.0]*7
    pick_cmd = None  # one of "left_hand", "right_hand", or None
    action = {
        "arms": {
            "ctrl_mode": "position",
            "joint_values": [0.0] * 14,
            "stiffness": [50.0, 0, 0, 0, 50.0, 0, 0, 0, 50.0, 0, 0, 0, 0, 0],
            "dampings": [0.0] * 14,
        },
        "legs": {
            "ctrl_mode": "effort",
            "joint_values": np.zeros(12),
            "stiffness": None,
            "dampings": None,
        },
        "head": {
            "ctrl_mode": "position",
            "joint_values": np.zeros(2),
            "stiffness": None,
            "dampings": None,
        },
        "pick": pick_cmd,
    }

    key_map = {
        'q': (0, +DELTA), 'a': (0, -DELTA),
        'w': (1, +DELTA), 's': (1, -DELTA),
        'e': (2, +DELTA), 'd': (2, -DELTA),
        'r': (3, +DELTA), 'f': (3, -DELTA),
        't': (4, +DELTA), 'g': (4, -DELTA),
        'y': (5, +DELTA), 'h': (5, -DELTA),
        'u': (6, +DELTA), 'j': (6, -DELTA),
        'p': 'left_hand',  # only support pick with left hand in this script.
        'o': None,         # space to release
        '\x1b': 'exit',  # ESC
    }

    try:
        step = 0
        while app.is_running():
            # 1) read key
            ch = read_key()
            if ch:
                mapping = key_map.get(ch.lower())
                if mapping == 'exit':
                    break
                # joint update
                if isinstance(mapping, tuple):
                    idx, delta = mapping
                    joint_angles[idx] += delta
                # pick update
                elif mapping in ('left_hand', None):
                    pick_cmd = mapping

            # 2) step sim
            obs, done = env.step(action)
            action = controller.get_action(obs)

            # 3) apply joint_angles to left arm
            for li, gi in enumerate(LEFT_ARM_INDICES):
                action["arms"]["joint_values"][gi] = np.pi * joint_angles[li] / 180.0
            action["pick"] = pick_cmd

            if done:
                print("\nFinished:", obs["extras"])
                break
            step += 1

    finally:
        controller.cleanup()
        app.close()

if __name__ == "__main__":
    main()
