#!/usr/bin/env python3
"""Laptop side of the teleop: a small window that reads the keyboard
(key-down AND key-up, so driving is hold-to-move) and talks to the car.

    python teleop/client.py                 # drive the real car (SSH to adi956@pi.local)
    python teleop/client.py --sim           # practise: simulated car + arena on screen
    python teleop/client.py --host adi956@172.20.10.2
    python teleop/client.py --server-sim    # test the SSH link: the Pi runs a simulated car

Keys (the window must have focus; clicking elsewhere stops the car):
    W / S        hold to drive forward / reverse        Space   STOP (cancels anything)
    A / D        hold to steer (LH: tap = 90 deg pivot)  Esc     stop, centre, quit
    arrows       move the head (pan / tilt)              C       centre head (+ steering)
    1 2 3 4      profile HH / HL / LH / LL (when stopped)
    [ ]          slower / faster (x0.1 of the profile speed)
    Q / E        glance left / right                     H       happy wiggle
    B            surprise (profile-style reaction to the obstacle)
    M            mark the start of a leg                 R       retrace back to the mark
    sim only:    O   pull the obstacle (O0 -> O1)        P       put it back at O0, car to A
"""
import argparse
import json
import os
import queue
import subprocess
import sys
import threading
import time
import tkinter as tk

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

HOLD = {"w": "w", "a": "a", "s": "s", "d": "d", "up": "up", "down": "down", "left": "left", "right": "right"}
TAPS = {
    "space": ("stop", None), "escape": ("quit", None),
    "1": ("profile", "HH"), "2": ("profile", "HL"), "3": ("profile", "LH"), "4": ("profile", "LL"),
    "bracketleft": ("slower", None), "bracketright": ("faster", None),
    "minus": ("slower", None), "equal": ("faster", None), "plus": ("faster", None),
    "q": ("glance_left", None), "e": ("glance_right", None), "h": ("wiggle", None),
    "b": ("surprise", None), "r": ("retrace", None), "m": ("mark", None), "c": ("centre", None),
}
SIM_TAPS = {"o": "pull", "p": "reset"}
SEND_S = 0.05
RELEASE_DEBOUNCE_MS = 40        # X11 auto-repeat sends release+press pairs; ignore those
REMOTE = "cd ~/ADCS-Picar && ROBOT_HAT_GPIOCHIP=0 exec python3 -u -m teleop.server"
COLOURS = {"HH": "#2e9e6e", "HL": "#5b4fc7", "LH": "#d9622b", "LL": "#b8862f"}


# ====================================================================== links
class SshLink:
    """server.py on the car, over SSH (stdin/stdout JSON lines)."""

    def __init__(self, host, server_sim=False):
        cmd = REMOTE + (" --sim" if server_sim else "")
        self.p = subprocess.Popen(
            ["ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8",
             "-o", "ServerAliveInterval=2", "-o", "ServerAliveCountMax=3", host, cmd],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, bufsize=1)
        self.q = queue.Queue()
        threading.Thread(target=self._read, args=(self.p.stdout, False), daemon=True).start()
        threading.Thread(target=self._read, args=(self.p.stderr, True), daemon=True).start()

    def _read(self, f, err):
        for line in f:
            line = line.rstrip()
            if not line:
                continue
            if err:
                if "terminal is required" in line or "password is required" in line:
                    continue        # robot_hat's harmless `sudo chmod` without a terminal
                self.q.put({"t": "log", "msg": line})
                continue
            try:
                self.q.put(json.loads(line))
            except ValueError:
                self.q.put({"t": "log", "msg": line})
        if not err:
            self.q.put({"t": "down"})

    def send(self, obj):
        try:
            self.p.stdin.write(json.dumps(obj) + "\n")
            self.p.stdin.flush()
        except (OSError, ValueError):
            pass

    def poll(self):
        out = []
        while True:
            try:
                out.append(self.q.get_nowait())
            except queue.Empty:
                return out

    def close(self):
        try:
            self.send({"t": "cmd", "name": "quit"})
            self.p.stdin.close()
            self.p.wait(timeout=3)
        except Exception:
            self.p.kill()


class SimLink:
    """The same TeleopCore, in-process, driving ArenaSim in real time."""

    def __init__(self, profile="HH"):
        from trajectory_profiles import calibration
        from trajectory_profiles.executor import Ranger
        from trajectory_profiles.layout import DEFAULT as LAYOUT
        from trajectory_profiles.motion import Motion
        from trajectory_profiles.sim import ArenaSim
        from teleop.core import TeleopCore
        from obs_avoidance_profile_new.params import LOOP_S
        self.layout = LAYOUT
        self.res = calibration.Resolved(None)
        self.ArenaSim, self.Ranger, self.Motion, self.Core = ArenaSim, Ranger, Motion, TeleopCore
        self.loop_s = LOOP_S
        self.profile = profile
        self._build()
        self.q = [{"t": "log", "msg": "SIMULATOR: O pulls the obstacle, P resets"}]

    def _build(self):
        self.sim = self.ArenaSim(self.layout, self.res.car, tables=self.res.tables, invalid_rate=0.0, spike_rate=0.0)
        self.core = self.Core(self.sim, self.Motion(self.res.tables), self.sim, self.Ranger(self.sim, self.sim),
                              self.res.car, style=self.profile)
        self.t0 = time.monotonic()
        self.n = 0

    def send(self, obj):
        if obj.get("t") == "keys":
            self.core.keys(obj.get("held", []))
        elif obj.get("t") == "cmd":
            if obj["name"] == "pull":
                self.sim.start_pull(0.0)
            elif obj["name"] == "reset":
                self.profile = self.core.style.key
                self._build()
            else:
                self.core.command(obj["name"], obj.get("arg"))

    def poll(self):
        target = time.monotonic() - self.t0
        while self.sim.t + self.loop_s <= target:
            self.sim.sleep(self.loop_s)
            self.core.tick()
            self.n += 1
        out, self.q = self.q, []
        st = self.core.status()
        st["t"] = "status"
        p = self.sim.pose()
        st["sim"] = {"x": p.x, "y": p.y, "h": p.h, "obstacle": list(self.sim.obstacle),
                     "collided": self.sim.collided}
        out.append(st)
        if self.core.quit:
            out.append({"t": "down"})
        return out

    def close(self):
        pass


# ====================================================================== window
class App:
    SCALE = 2.5                 # px per cm in the arena view

    def __init__(self, link, sim):
        self.link, self.sim = link, sim
        self.root = tk.Tk()
        self.root.title("PiCar teleop" + (" [SIM]" if sim else ""))
        self.root.configure(bg="#14161a")
        self.held, self.down = set(), set()
        self.pending = {}
        self.status = {}
        self.trail = []
        self.alive = True
        self.logs = []

        fg, bg = "#e8e8e8", "#14161a"
        top = tk.Frame(self.root, bg=bg)
        top.pack(fill="x", padx=12, pady=(10, 4))
        self.l_prof = tk.Label(top, font=("Segoe UI", 20, "bold"), fg=fg, bg=bg, anchor="w")
        self.l_prof.pack(fill="x")
        self.l_state = tk.Label(top, font=("Segoe UI", 16), fg="#ffd166", bg=bg, anchor="w")
        self.l_state.pack(fill="x")
        self.l_nums = tk.Label(top, font=("Consolas", 13), fg=fg, bg=bg, anchor="w", justify="left")
        self.l_nums.pack(fill="x", pady=(4, 0))
        self.l_msg = tk.Label(top, font=("Segoe UI", 12), fg="#9ad1ff", bg=bg, anchor="w")
        self.l_msg.pack(fill="x")

        mid = tk.Frame(self.root, bg=bg)
        mid.pack(fill="both", expand=True, padx=12)
        size = int(150 * self.SCALE) + 20 if sim else 300
        self.cv = tk.Canvas(mid, width=size, height=size if sim else 170, bg="#1d2026", highlightthickness=0)
        self.cv.pack(side="left", pady=6)
        help_text = __doc__.split("Keys", 1)[1].split("\n", 1)[1].rstrip()
        tk.Label(mid, text=help_text, font=("Consolas", 9), fg="#b8bcc4", bg=bg, justify="left",
                 anchor="nw").pack(side="left", fill="both", padx=12, pady=6)
        self.l_log = tk.Label(self.root, font=("Consolas", 9), fg="#8a8f98", bg=bg, anchor="w", justify="left")
        self.l_log.pack(fill="x", padx=12, pady=(0, 8))

        self.root.bind("<KeyPress>", self.on_press)
        self.root.bind("<KeyRelease>", self.on_release)
        self.root.bind("<FocusOut>", self.on_focus_out)
        self.root.protocol("WM_DELETE_WINDOW", self.quit)
        self.root.after(int(SEND_S * 1000), self.send_loop)
        self.root.after(20, self.poll_loop)

    # ---------------------------------------------------------- keys
    @staticmethod
    def _name(e):
        return e.keysym.lower()

    def on_press(self, e):
        k = self._name(e)
        if k in self.pending:
            self.root.after_cancel(self.pending.pop(k))
        if k in HOLD:
            if HOLD[k] not in self.held:
                self.held.add(HOLD[k])
                self.send_keys()
            return
        if k in self.down:
            return                  # auto-repeat of a tap key
        self.down.add(k)
        if k in TAPS:
            name, arg = TAPS[k]
            self.link.send({"t": "cmd", "name": name, "arg": arg})
            if name == "quit":
                self.root.after(300, self.quit)
        elif self.sim and k in SIM_TAPS:
            self.trail = [] if SIM_TAPS[k] == "reset" else self.trail
            self.link.send({"t": "cmd", "name": SIM_TAPS[k]})

    def on_release(self, e):
        k = self._name(e)
        if k in self.pending:
            self.root.after_cancel(self.pending.pop(k))
        self.pending[k] = self.root.after(RELEASE_DEBOUNCE_MS, lambda: self._released(k))

    def _released(self, k):
        self.pending.pop(k, None)
        self.down.discard(k)
        if k in HOLD and HOLD[k] in self.held:
            self.held.discard(HOLD[k])
            self.send_keys()

    def on_focus_out(self, _e):
        if self.held:
            self.held.clear()
            self.send_keys()
            self.link.send({"t": "cmd", "name": "stop"})
        self.down.clear()

    def send_keys(self):
        self.link.send({"t": "keys", "held": sorted(self.held)})

    def send_loop(self):
        if self.alive:
            self.send_keys()
            self.root.after(int(SEND_S * 1000), self.send_loop)

    # ---------------------------------------------------------- display
    def poll_loop(self):
        if not self.alive:
            return
        for m in self.link.poll():
            if m.get("t") == "status":
                self.status = m
            elif m.get("t") == "log":
                self.logs = (self.logs + [m["msg"]])[-4:]
            elif m.get("t") == "down":
                self.logs = (self.logs + ["LINK DOWN: the car side has exited (car stopped)."])[-4:]
                self.status = dict(self.status, state="LINK DOWN")
        self.draw()
        self.root.after(30, self.poll_loop)

    def draw(self):
        s = self.status
        self.l_log.config(text="\n".join(self.logs))
        if not s:
            self.l_prof.config(text="connecting...")
            return
        col = COLOURS.get(s["profile"], "#e8e8e8")
        self.l_prof.config(text="%s  %s" % (s["profile"], s["profile_name"].split(":", 1)[-1].strip()), fg=col)
        self.l_state.config(text=s["state"].upper(),
                            fg="#ff5d5d" if any(w in s["state"] for w in ("HARD", "NO INPUT", "LINK", "BLOCKED"))
                            else "#ffd166")
        dist = "--" if s["dist"] is None or s["dist"] >= 300 else "%.0f cm" % s["dist"]
        self.l_nums.config(text="speed %+5.1f cm/s (cmd %3d)   cruise %.0f cm/s x%.1f\n"
                                "steer %+5.1f deg   head pan %+5.1f tilt %+5.1f   front %s   recorded %.1f s"
                                % (s["speed"], s["cmd"], s["cruise"], s["scale"], s["steer"], s["pan"], s["tilt"],
                                   dist, s["rec_s"]))
        self.l_msg.config(text=s["msg"])
        self.cv.delete("all")
        if self.sim and "sim" in s:
            self.draw_arena(s, col)
        else:
            self.draw_gauges(s, col)

    def draw_gauges(self, s, col):
        cv = self.cv
        cv.create_text(150, 14, text="steering", fill="#8a8f98")
        cv.create_rectangle(20, 28, 280, 44, outline="#444")
        x = 150 + s["steer"] / 30.0 * 130
        cv.create_rectangle(min(150, x), 29, max(150, x), 43, fill=col, width=0)
        cv.create_text(150, 70, text="speed", fill="#8a8f98")
        cv.create_rectangle(20, 84, 280, 100, outline="#444")
        x = 150 + s["speed"] / 45.0 * 130
        cv.create_rectangle(min(150, x), 85, max(150, x), 99, fill=col, width=0)
        cv.create_text(150, 126, text="head pan", fill="#8a8f98")
        cv.create_rectangle(20, 140, 280, 156, outline="#444")
        x = 150 + s["pan"] / 60.0 * 130
        cv.create_line(x, 138, x, 158, fill="#ddd", width=3)

    def draw_arena(self, s, col):
        from trajectory_profiles.geometry import Pose, footprint
        lay = self.link.layout
        k, o = self.SCALE, 10

        def px(x, y):
            return o + x * k, o + (lay.height - y) * k
        cv = self.cv
        cv.create_rectangle(*px(0, lay.height), *px(lay.width, 0), outline="#666", dash=(4, 3))
        for name, t in lay.targets.items():
            x, y = px(t.x, t.y)
            cv.create_line(x - 7, y - 7, x + 7, y + 7, fill="#888", width=2)
            cv.create_line(x - 7, y + 7, x + 7, y - 7, fill="#888", width=2)
            cv.create_text(x + 14, y - 10, text=name, fill="#ccc", font=("Segoe UI", 11, "bold"))
        h = lay.obstacle_half
        ox, oy = lay.o0
        cv.create_rectangle(*px(ox - h, oy + h), *px(ox + h, oy - h), outline="#555", dash=(2, 2))
        ox, oy = s["sim"]["obstacle"]
        cv.create_rectangle(*px(ox - h, oy + h), *px(ox + h, oy - h), fill="#c9c4b8", outline="")
        sp = s["sim"]
        self.trail.append(px(sp["x"], sp["y"]))
        self.trail = self.trail[-1500:]
        if len(self.trail) > 1:
            cv.create_line(*[c for p in self.trail for c in p], fill=col, width=2)
        fp = footprint(Pose(sp["x"], sp["y"], sp["h"]), self.link.res.car)
        cv.create_polygon(*[c for p in fp for c in px(*p)], fill="", outline="#fff", width=2)
        import math
        a = math.radians(sp["h"] - s["pan"])             # + pan = right = clockwise
        x0, y0 = px(sp["x"], sp["y"])
        cv.create_line(x0, y0, x0 + 40 * math.cos(a), y0 - 40 * math.sin(a), fill="#ff6b6b", dash=(3, 2))
        if sp.get("collided"):
            cv.create_text(o + 75 * k, o + 8, text="COLLISION", fill="#ff5d5d", font=("Segoe UI", 12, "bold"))

    # ---------------------------------------------------------- lifecycle
    def quit(self):
        if not self.alive:
            return
        self.alive = False
        try:
            self.link.send({"t": "keys", "held": []})
            self.link.close()
        finally:
            self.root.destroy()

    def run(self):
        self.root.focus_force()
        self.root.mainloop()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", action="store_true", help="simulated car on the laptop (no SSH)")
    ap.add_argument("--host", default="adi956@pi.local", help="SSH target (default adi956@pi.local)")
    ap.add_argument("--server-sim", action="store_true", help="real SSH link, simulated car on the Pi")
    args = ap.parse_args(argv)
    link = SimLink() if args.sim else SshLink(args.host, server_sim=args.server_sim)
    App(link, sim=args.sim).run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
