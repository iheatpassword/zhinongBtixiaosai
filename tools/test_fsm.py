#!/usr/bin/env python3
"""Host-side state-transition simulation for USER/fsm.c (tasks T4 and T5).

The script mirrors the transition logic of USER/fsm.c against mock app_if.h
hooks, so the competition sequence can be verified without hardware:

  * App_GetDistanceCm()  integrates the commanded wheel speeds
                         (counts/20 ms) into cm and returns a u16 reading,
                         exactly the way the Lead's odometry does
  * App_SetWheelSpeed()  stores the four targets (FL, FR, RL, RR)
  * App_DriveStop()      zeroes them and is recorded in the call log
  * App_SetSteerAll()/App_SetServoCenter() record the servo angle
  * Centering_Update()   returns the base speed on all four wheels (the real
                         controller adds the lateral / heading correction)

All numeric constants are parsed from USER/fsm.h, so the Python model cannot
silently drift from the firmware configuration.

Verified behaviour:
  1. configuration values (6 ridges, lengths, shift, speeds, timers)
  2. IDLE: motors stopped, servos centred, no motion before FSM_Start()
  3. nominal run: IDLE -> ENTRY -> RIDGE(1) -> SHIFT(1) -> ... -> RIDGE(6)
     -> EXIT -> PARK, 6 ridges, 5 shifts of 70 cm, serpentine directions,
     stop-before-steer ordering, no drive while the servos travel
  4. FAULT: laser invalid > 200 ms in RIDGE / SHIFT, short dropouts ignored,
     latch, restart with FSM_Start()
  5. FAULT: run time > 4.5 min (stuck car), PARK is never faulted
  6. doorway edge detector for ENTRY and EXIT (blocked -> clear)
  7. FSM_Start() accepted from IDLE / FAULT only

Run:      python tools/test_fsm.py
Exit code 0 = PASS, 1 = FAIL.
"""

import math
import os
import re
import sys
from types import SimpleNamespace

HERE = os.path.dirname(os.path.abspath(__file__))
FSM_H_PATH = os.path.normpath(os.path.join(HERE, "..", "USER", "fsm.h"))

# --- states, must match app_if.h / fsm.c ---------------------------------
ST_IDLE, ST_ENTRY, ST_RIDGE, ST_SHIFT, ST_EXIT, ST_PARK, ST_FAULT = range(7)
ST_NAMES = {
    ST_IDLE: "IDLE", ST_ENTRY: "ENTRY", ST_RIDGE: "RIDGE",
    ST_SHIFT: "SHIFT", ST_EXIT: "EXIT", ST_PARK: "PARK",
    ST_FAULT: "FAULT",
}

# --- Lead odometry geometry (main.c: 60000 counts/rev, 76.4 mm wheel) ----
COUNTS_PER_REV = 60000.0
WHEEL_DIAMETER_CM = 7.64
CM_PER_COUNT = (math.pi * WHEEL_DIAMETER_CM) / COUNTS_PER_REV  # ~0.0004


# ========================================================================
#  fsm.h configuration parser
# ========================================================================
REQUIRED = [
    "FSM_RIDGE_CNT", "FSM_RIDGE_LEN_CM_INIT", "FSM_RIDGE_PITCH_CM",
    "FSM_SHIFT_CM", "FSM_SHIFT_STEER_DEG",
    "FSM_SPEED_ENTRY", "FSM_SPEED_RIDGE", "FSM_SPEED_SHIFT", "FSM_SPEED_EXIT",
    "FSM_ENTRY_CM", "FSM_EXIT_CM",
    "FSM_BLOCK_MM", "FSM_DOOR_MM", "FSM_DOOR_TICKS",
    "FSM_TICK_MS", "FSM_STEER_MS", "FSM_LASER_FAULT_MS",
    "FSM_RUN_TIMEOUT_MS", "FSM_MAX_INC_CM",
]


def to_number(tok):
    """'220u' -> 220, '90.0f' -> 90.0."""
    tok = tok.strip()
    tok = re.sub(r"([0-9.])([uUlLfF]+)$", r"\1", tok)
    if re.fullmatch(r"-?\d+", tok):
        return int(tok)
    if re.fullmatch(r"-?\d+\.\d*", tok):
        return float(tok)
    raise ValueError("cannot parse configuration value: %r" % tok)


def parse_value(raw):
    raw = raw.strip()
    if raw.startswith("{"):
        inner = raw.strip("{} \t")
        return [to_number(t) for t in inner.split(",") if t.strip()]
    return to_number(raw)


def load_config(path=FSM_H_PATH):
    with open(path, "r", encoding="ascii") as f:
        text = f.read()
    text = re.sub(r"/\*.*?\*/", " ", text, flags=re.S)  # drop block comments
    cfg = {}
    for line in text.splitlines():
        m = re.match(r"^\s*#define\s+(FSM_[A-Z0-9_]+)\s+(.+?)\s*$", line)
        if m:
            cfg[m.group(1)] = parse_value(m.group(2))
    return cfg


# ========================================================================
#  mock app_if.h hooks
# ========================================================================
class Laser:
    def __init__(self, df=2000, dl=200, dr=200, valid=True, flags=0x07):
        self.dL = dl
        self.dR = dr
        self.dF = df
        self.flags = flags
        self.valid = valid
        self.last_ms = 0


class MockApp:
    """Mock of the Lead hooks; also the odometry model."""

    def __init__(self):
        self.spd = [0, 0, 0, 0]
        self.odom_cm_f = 0.0
        self.odom_frozen = False
        self.servo = 0.0
        self.calls = []          # (tick, name, arg)
        self.tick = -1

    def _log(self, name, arg=None):
        self.calls.append((self.tick, name, arg))

    # --- hooks ---------------------------------------------------------
    def set_wheel_speed(self, spd):
        self.spd = list(spd)
        self._log("spd", tuple(self.spd))

    def drive_stop(self):
        self.spd = [0, 0, 0, 0]
        self._log("stop")

    def set_steer_all(self, deg):
        self.servo = float(deg)
        self._log("steer_all", float(deg))

    def set_steer_rotate(self, deg, rot):
        self._log("steer_rotate", (float(deg), float(rot)))

    def set_steer4(self, fl, fr, rl, rr):
        self._log("steer4", (fl, fr, rl, rr))

    def set_servo_center(self):
        self.servo = 0.0
        self._log("center")

    def ms_now(self):
        return self.tick * 20

    def get_distance_cm(self):
        return int(self.odom_cm_f) & 0xFFFF

    # --- plant ---------------------------------------------------------
    def advance(self):
        """One 20 ms period of odometry integration."""
        if self.odom_frozen:
            return
        mean = sum(abs(v) for v in self.spd) / 4.0
        self.odom_cm_f += mean * CM_PER_COUNT


# ========================================================================
#  Python port of USER/fsm.c (same order of checks)
# ========================================================================
class FsmModel:
    def __init__(self, app, cfg):
        self.app = app
        self.cfg = cfg
        self.state = ST_IDLE
        self.entered = False
        self.state_t0_ms = 0
        self.t0_set = False
        self.t0_ms = 0
        self.ridges_done = 0
        self.shifts_done = 0
        self.fault_code = 0
        self.ph_prev_cm = 0
        self.ph_cm = 0
        self.laser_lost = False
        self.laser_lost_ms = 0
        self.door_blocked = False
        self.door_cnt = 0
        self.centering = []      # (state, base, laser.dF, yaw)
        self.phase_done_cm = 0   # instrumentation only, not in fsm.c

    # --- helpers -------------------------------------------------------
    def _phase_reset(self):
        self.ph_prev_cm = self.app.get_distance_cm()
        self.ph_cm = 0

    def _dist_tick(self):
        d = self.app.get_distance_cm()
        inc = abs(d - self.ph_prev_cm)
        if inc > self.cfg["FSM_MAX_INC_CM"]:
            inc = 0
        self.ph_cm += inc
        self.ph_prev_cm = d

    def _door_reset(self):
        self.door_blocked = False
        self.door_cnt = 0

    def _door_ahead(self, laser):
        if (not laser.valid) or ((laser.flags & 0x01) == 0) or laser.dF == 0xFFFF:
            self.door_cnt = 0
            return False
        if laser.dF < self.cfg["FSM_BLOCK_MM"]:
            self.door_blocked = True
            self.door_cnt = 0
            return False
        if laser.dF >= self.cfg["FSM_DOOR_MM"] and self.door_blocked:
            if self.door_cnt < 250:
                self.door_cnt += 1
            if self.door_cnt >= self.cfg["FSM_DOOR_TICKS"]:
                return True
            return False
        self.door_cnt = 0
        return False

    def _drive_all(self, spd):
        self.app.set_wheel_speed([spd, spd, spd, spd])

    def _ridge_base(self):
        if (self.ridges_done & 1) == 0:
            return int(self.cfg["FSM_SPEED_RIDGE"])
        return -int(self.cfg["FSM_SPEED_RIDGE"])

    def _goto(self, s):
        if s != self.state:
            # instrumentation only: distance reached by the phase that ends
            self.phase_done_cm = self.ph_cm
            self.state = s
            self.entered = False

    def _enter(self, ms):
        self.state_t0_ms = ms
        app = self.app
        s = self.state
        if s == ST_IDLE:
            app.drive_stop()
            app.set_servo_center()
        elif s == ST_ENTRY:
            app.drive_stop()
            app.set_servo_center()
            self._door_reset()
            self._phase_reset()
        elif s == ST_RIDGE:
            app.drive_stop()
            self._phase_reset()
        elif s == ST_SHIFT:
            app.drive_stop()
            app.set_steer_all(self.cfg["FSM_SHIFT_STEER_DEG"])
            self._phase_reset()
        elif s == ST_EXIT:
            app.drive_stop()
            app.set_servo_center()
            self._door_reset()
            self._phase_reset()
        elif s == ST_PARK:
            app.drive_stop()
        elif s == ST_FAULT:
            app.drive_stop()

    def _steering_busy(self, ms):
        return ((ms - self.state_t0_ms) & 0xFFFFFFFF) < self.cfg["FSM_STEER_MS"]

    def _fault_check(self, in_):
        if self.state not in (ST_RIDGE, ST_SHIFT):
            return
        if in_.laser.valid:
            self.laser_lost = False
            return
        if not self.laser_lost:
            self.laser_lost = True
            self.laser_lost_ms = in_.ms
        if ((in_.ms - self.laser_lost_ms) & 0xFFFFFFFF) > self.cfg["FSM_LASER_FAULT_MS"]:
            self.fault_code = 1
            self._goto(ST_FAULT)

    def _time_check(self, in_):
        if (not self.t0_set) or self.state in (ST_IDLE, ST_PARK, ST_FAULT):
            return
        if ((in_.ms - self.t0_ms) & 0xFFFFFFFF) > self.cfg["FSM_RUN_TIMEOUT_MS"]:
            self.fault_code = 2
            self._goto(ST_FAULT)

    # --- state bodies --------------------------------------------------
    def _run_entry(self, in_):
        self._dist_tick()
        if self._door_ahead(in_.laser) or self.ph_cm >= self.cfg["FSM_ENTRY_CM"]:
            self._goto(ST_RIDGE)
            return
        if self._steering_busy(in_.ms):
            return
        self._drive_all(int(self.cfg["FSM_SPEED_ENTRY"]))

    def _run_ridge(self, in_):
        self._dist_tick()
        if self.ph_cm >= self.cfg["FSM_RIDGE_LEN_CM_INIT"][self.ridges_done]:
            self.ridges_done += 1
            if self.ridges_done >= self.cfg["FSM_RIDGE_CNT"]:
                self._goto(ST_EXIT)
            else:
                self._goto(ST_SHIFT)
            return
        if self._steering_busy(in_.ms):
            return
        base = self._ridge_base()
        self.centering.append((self.state, base, in_.laser.dF, in_.yaw_deg))
        self.app.set_wheel_speed([base, base, base, base])

    def _run_shift(self, in_):
        self._dist_tick()
        if self.ph_cm >= self.cfg["FSM_SHIFT_CM"]:
            self.app.set_servo_center()
            self.app.drive_stop()
            self.shifts_done += 1
            self._goto(ST_RIDGE)
            return
        if self._steering_busy(in_.ms):
            return
        self._drive_all(int(self.cfg["FSM_SPEED_SHIFT"]))

    def _run_exit(self, in_):
        self._dist_tick()
        if self._door_ahead(in_.laser) or self.ph_cm >= self.cfg["FSM_EXIT_CM"]:
            self._goto(ST_PARK)
            return
        if self._steering_busy(in_.ms):
            return
        self._drive_all(int(self.cfg["FSM_SPEED_EXIT"]))

    def _run(self, in_):
        if self.state == ST_ENTRY:
            self._run_entry(in_)
        elif self.state == ST_RIDGE:
            self._run_ridge(in_)
        elif self.state == ST_SHIFT:
            self._run_shift(in_)
        elif self.state == ST_EXIT:
            self._run_exit(in_)

    # --- public --------------------------------------------------------
    def start(self):
        if self.state not in (ST_IDLE, ST_FAULT):
            return False
        self.ridges_done = 0
        self.shifts_done = 0
        self.fault_code = 0
        self.t0_set = False
        self.t0_ms = 0
        self.ph_prev_cm = 0
        self.ph_cm = 0
        self.laser_lost = False
        self.laser_lost_ms = 0
        self.door_blocked = False
        self.door_cnt = 0
        self._goto(ST_ENTRY)
        return True

    def tick(self, in_):
        if (not self.t0_set) and self.state != ST_IDLE:
            self.t0_ms = in_.ms
            self.t0_set = True
        self._fault_check(in_)
        self._time_check(in_)
        for _ in range(2):
            if not self.entered:
                self._enter(in_.ms)
                self.entered = True
            self._run(in_)
            if self.entered:
                break

    def get_state(self):
        return self.state


# ========================================================================
#  simulation harness
# ========================================================================
class Sim:
    def __init__(self, cfg):
        self.cfg = cfg
        self.app = MockApp()
        self.fsm = FsmModel(self.app, cfg)
        self.laser = Laser()
        self.ms = 0
        self.tick_no = 0
        self.transitions = []    # dicts
        self.history = [(0, 0, ST_IDLE, 0, (0, 0, 0, 0))]

    def start(self):
        before = self.fsm.state
        ok = self.fsm.start()
        if self.fsm.state != before:
            self.transitions.append({
                "tick": self.tick_no, "ms": self.ms,
                "from": before, "to": self.fsm.state,
                "ph_cm": self.fsm.phase_done_cm,
                "servo": self.app.servo,
                "spd": tuple(self.app.spd),
            })
        return ok

    def tick(self, override=None):
        if override is not None:
            override(self)
        self.app.tick = self.tick_no
        prev_state = self.fsm.state
        in_ = SimpleNamespace(laser=self.laser, yaw_deg=0.0,
                              yaw_rate_dps=0.0, ms=self.ms)
        self.fsm.tick(in_)
        if self.fsm.state != prev_state:
            self.transitions.append({
                "tick": self.tick_no, "ms": self.ms,
                "from": prev_state, "to": self.fsm.state,
                "ph_cm": self.fsm.phase_done_cm,
                "servo": self.app.servo,
                "spd": tuple(self.app.spd),
            })
        self.history.append((self.tick_no, self.ms, self.fsm.state,
                             self.fsm.ph_cm, tuple(self.app.spd)))
        self.app.advance()
        self.tick_no += 1
        self.ms += self.cfg["FSM_TICK_MS"]

    def run(self, n, override=None):
        for _ in range(n):
            self.tick(override)

    def run_until(self, pred, max_ticks=40000, override=None):
        for _ in range(max_ticks):
            self.tick(override)
            if pred(self):
                return True
        return False

    def calls_at(self, tick):
        return [(n, a) for (t, n, a) in self.app.calls if t == tick]


# ========================================================================
#  checks
# ========================================================================
class Log:
    def __init__(self):
        self.checks = 0
        self.fails = []

    def check(self, ok, desc, detail=None):
        self.checks += 1
        line = "[PASS] " if ok else "[FAIL] "
        line += desc
        if detail:
            line += "  (" + detail + ")"
        print(line)
        if not ok:
            self.fails.append(desc)
        return ok


def test_config(cfg, log):
    print("\n-- configuration parsed from USER/fsm.h --")
    missing = [n for n in REQUIRED if n not in cfg]
    log.check(not missing, "all FSM_* configuration macros present",
              "missing: %s" % missing)
    if missing:
        return

    log.check(cfg["FSM_RIDGE_CNT"] == 6, "6 ridges", str(cfg["FSM_RIDGE_CNT"]))
    lengths = cfg["FSM_RIDGE_LEN_CM_INIT"]
    log.check(lengths == [220, 220, 220, 220, 220, 180],
              "ridge lengths 220/220/220/220/220/180 cm", str(lengths))
    log.check(cfg["FSM_SHIFT_CM"] == 70, "lateral shift 70 cm",
              str(cfg["FSM_SHIFT_CM"]))
    log.check(cfg["FSM_RIDGE_PITCH_CM"] == 70, "ridge pitch 70 cm",
              str(cfg["FSM_RIDGE_PITCH_CM"]))
    log.check(abs(cfg["FSM_SHIFT_STEER_DEG"] - 90.0) < 1e-9,
              "shift steering angle 90 deg", str(cfg["FSM_SHIFT_STEER_DEG"]))
    speeds = [cfg["FSM_SPEED_ENTRY"], cfg["FSM_SPEED_RIDGE"],
              cfg["FSM_SPEED_SHIFT"], cfg["FSM_SPEED_EXIT"]]
    log.check(all(isinstance(v, int) and 0 < v < 2000 for v in speeds),
              "drive speeds are 1..1999 counts/20 ms", str(speeds))
    log.check(cfg["FSM_SPEED_SHIFT"] < cfg["FSM_SPEED_RIDGE"],
              "shift creep slower than the ridge speed", str(speeds))
    log.check(cfg["FSM_LASER_FAULT_MS"] == 200, "laser fault limit 200 ms",
              str(cfg["FSM_LASER_FAULT_MS"]))
    log.check(cfg["FSM_RUN_TIMEOUT_MS"] == 270000, "run limit 4.5 min",
              str(cfg["FSM_RUN_TIMEOUT_MS"]))
    log.check(cfg["FSM_TICK_MS"] == 20, "tick period 20 ms",
              str(cfg["FSM_TICK_MS"]))
    log.check(cfg["FSM_STEER_MS"] > 0, "steering settle time configured",
              str(cfg["FSM_STEER_MS"]) + " ms")


def test_idle(cfg, log):
    print("\n-- IDLE --")
    sim = Sim(cfg)
    sim.run(100)
    log.check(sim.fsm.state == ST_IDLE, "IDLE before FSM_Start (100 ticks)",
              ST_NAMES[sim.fsm.state])
    log.check(all(v == 0 for v in sim.app.spd), "motors stopped",
              str(sim.app.spd))
    log.check(sim.app.servo == 0.0, "servos centred", str(sim.app.servo))
    log.check(sim.app.odom_cm_f == 0.0, "no motion",
              "%.2f cm" % sim.app.odom_cm_f)


def test_nominal(cfg, log):
    print("\n-- nominal competition run --")
    sim = Sim(cfg)
    sim.tick()
    sim.start()
    log.check(sim.fsm.state == ST_ENTRY, "FSM_Start(): IDLE -> ENTRY",
              ST_NAMES[sim.fsm.state])

    ok = sim.run_until(lambda s: s.fsm.state == ST_PARK, max_ticks=40000)
    log.check(ok, "sequence reaches PARK", str(sim.tick_no) + " ticks")
    if not ok:
        return

    seq = [ST_IDLE] + [t["to"] for t in sim.transitions]
    expected = [ST_IDLE, ST_ENTRY, ST_RIDGE, ST_SHIFT, ST_RIDGE, ST_SHIFT,
                ST_RIDGE, ST_SHIFT, ST_RIDGE, ST_SHIFT, ST_RIDGE, ST_SHIFT,
                ST_RIDGE, ST_EXIT, ST_PARK]
    log.check(seq == expected,
              "state sequence IDLE->ENTRY->(RIDGE->SHIFT)x5->RIDGE6->EXIT->PARK",
              " ".join(ST_NAMES[s] for s in seq))

    print("        nominal timeline:")
    for t in sim.transitions:
        print("          t=%7.2f s  %-5s -> %-5s  phase_dist=%4d cm"
              "  servo=%3.0f  spd=%s"
              % (t["ms"] / 1000.0, ST_NAMES[t["from"]], ST_NAMES[t["to"]],
                 t["ph_cm"], t["servo"], t["spd"]))

    log.check(sim.fsm.ridges_done == 6, "6 ridges traversed",
              str(sim.fsm.ridges_done))
    log.check(sim.fsm.shifts_done == 5, "5 lateral shifts",
              str(sim.fsm.shifts_done))
    log.check(sim.fsm.fault_code == 0, "no fault during the nominal run",
              str(sim.fsm.fault_code))

    # ridge / shift distances
    ridge_out = [t for t in sim.transitions if t["from"] == ST_RIDGE]
    lengths = [t["ph_cm"] for t in ridge_out]
    exp = cfg["FSM_RIDGE_LEN_CM_INIT"]
    log.check(len(lengths) == 6 and all(abs(a - b) <= 2
                                        for a, b in zip(lengths, exp)),
              "ridge distances match the configured lengths (+-2 cm)",
              str(lengths))
    shift_out = [t for t in sim.transitions if t["from"] == ST_SHIFT]
    sl = [t["ph_cm"] for t in shift_out]
    log.check(len(sl) == 5 and all(abs(x - 70) <= 2 for x in sl),
              "each lateral shift = 70 cm (+-2 cm)", str(sl))

    # serpentine direction, taken from the last driven ridge tick
    signs = []
    for t in ridge_out:
        spd = None
        for (tk, _ms, st, _ph, spd_v) in reversed(sim.history):
            if tk < t["tick"] and st == ST_RIDGE and any(spd_v):
                spd = spd_v
                break
        signs.append(1 if (spd and spd[0] > 0) else (-1 if spd else 0))
    log.check(signs == [1, -1, 1, -1, 1, -1],
              "serpentine direction +,-,+,-,+,-", str(signs))
    log.check(all(abs(b) == cfg["FSM_SPEED_RIDGE"]
                  for (_s, b, _d, _y) in sim.fsm.centering),
              "centering is called with the ridge base speed",
              "|base|=%d" % cfg["FSM_SPEED_RIDGE"])
    log.check(any(b == -cfg["FSM_SPEED_RIDGE"] for (_s, b, _d, _y)
                  in sim.fsm.centering),
              "reverse legs pass a negative base speed")

    # shift entry: stop first, then all servos 90 deg
    shift_in = [t for t in sim.transitions if t["to"] == ST_SHIFT]
    log.check(all(abs(t["servo"] - 90.0) < 1e-9 for t in shift_in),
              "SHIFT entry: all 4 servos at 90 deg")
    log.check(all(all(v == 0 for v in t["spd"]) for t in shift_in),
              "SHIFT entry: motors stopped before steering")
    ordered = True
    for t in shift_in:
        names = [n for (n, _a) in sim.calls_at(t["tick"])]
        if ("stop" not in names or "steer_all" not in names
                or names.index("stop") > names.index("steer_all")):
            ordered = False
    log.check(ordered, "SHIFT: App_DriveStop() before App_SetSteerAll(90)")

    # shift completion: servos centred, then clean stop
    ridge_in = [t for t in sim.transitions if t["to"] == ST_RIDGE]
    centred = True
    for t in sim.transitions:
        if t["from"] != ST_SHIFT:
            continue
        names = [n for (n, _a) in sim.calls_at(t["tick"])]
        if "center" not in names or "stop" not in names:
            centred = False
        elif names.index("center") > names.index("stop"):
            centred = False
    log.check(centred, "SHIFT complete: servos centred, then clean stop")
    log.check(all(abs(t["servo"]) < 1e-9 for t in ridge_in),
              "RIDGE entry: servos centred")
    log.check(all(all(v == 0 for v in t["spd"]) for t in ridge_in),
              "RIDGE entry: motors stopped during the servo settle")

    # no wheel-speed command while the servos travel
    quiet = True
    steer_ms = cfg["FSM_STEER_MS"]
    settle_ticks = steer_ms // cfg["FSM_TICK_MS"]
    for t in sim.transitions:
        if t["to"] not in (ST_ENTRY, ST_RIDGE, ST_SHIFT, ST_EXIT):
            continue
        lo, hi = t["tick"], t["tick"] + settle_ticks
        for (tk, name, _a) in sim.app.calls:
            if lo <= tk < hi and name == "spd":
                quiet = False
    log.check(quiet, "no drive command while the steering servos travel",
              "%d ms settle" % steer_ms)

    # entry / exit fallback distances
    en = [t["ph_cm"] for t in sim.transitions if t["from"] == ST_ENTRY]
    log.check(len(en) == 1 and abs(en[0] - cfg["FSM_ENTRY_CM"]) <= 2,
              "ENTRY ends at the 40 cm fallback", str(en))
    ex = [t["ph_cm"] for t in sim.transitions if t["from"] == ST_EXIT]
    log.check(len(ex) == 1 and abs(ex[0] - cfg["FSM_EXIT_CM"]) <= 2,
              "EXIT ends at the configured fallback", str(ex))

    park_t = [t for t in sim.transitions if t["to"] == ST_PARK]
    log.check(len(park_t) == 1, "exactly one EXIT -> PARK")
    elapsed = sim.ms
    log.check(elapsed < cfg["FSM_RUN_TIMEOUT_MS"],
              "nominal run shorter than the 4.5 min limit",
              "%.1f s" % (elapsed / 1000.0))
    total_cm = sim.app.odom_cm_f
    expected_cm = (sum(exp) + 5 * cfg["FSM_SHIFT_CM"]
                   + cfg["FSM_ENTRY_CM"] + cfg["FSM_EXIT_CM"])
    log.check(abs(total_cm - expected_cm) <= 5.0,
              "odometry = 6 ridges + 5 shifts + entry + exit",
              "%.1f cm vs %.1f cm" % (total_cm, expected_cm))

    # stay parked, and PARK is exempt from the run-time limit
    sim.run(6000)                       # +120 s
    log.check(sim.ms > cfg["FSM_RUN_TIMEOUT_MS"],
              "simulated time now beyond 4.5 min",
              "%.1f s" % (sim.ms / 1000.0))
    log.check(sim.fsm.state == ST_PARK, "stays in PARK",
              ST_NAMES[sim.fsm.state])
    log.check(all(v == 0 for v in sim.app.spd), "PARK motors off",
              str(sim.app.spd))
    log.check(sim.fsm.fault_code == 0, "PARK never faults on the run-time limit",
              str(sim.fsm.fault_code))


def test_laser_fault(cfg, log):
    print("\n-- laser fault supervision --")

    # (a) RIDGE: invalid longer than 200 ms -> FAULT
    sim = Sim(cfg)
    sim.start()
    ok = sim.run_until(lambda s: s.fsm.state == ST_RIDGE and s.fsm.ph_cm >= 20)
    log.check(ok, "setup: driving on ridge 1", "ph=%d cm" % sim.fsm.ph_cm)
    sim.laser.valid = False
    t_lost = sim.ms
    ok = sim.run_until(lambda s: s.fsm.state == ST_FAULT, max_ticks=100)
    # sim.ms is already the next tick, so the fault tick is one period back
    dt = (sim.ms - cfg["FSM_TICK_MS"]) - t_lost
    log.check(ok, "RIDGE: laser invalid -> FAULT", ST_NAMES[sim.fsm.state])
    log.check(200 < dt <= 400, "FAULT after >200 ms of invalid laser",
              "%d ms" % dt)
    log.check(sim.fsm.fault_code == 1, "fault code 1 (laser)",
              str(sim.fsm.fault_code))
    log.check(all(v == 0 for v in sim.app.spd), "FAULT: motors stopped",
              str(sim.app.spd))

    # latch until FSM_Start()
    sim.laser.valid = True
    sim.run(1000)                       # 20 s
    log.check(sim.fsm.state == ST_FAULT, "FAULT latched with a valid laser",
              ST_NAMES[sim.fsm.state])
    log.check(all(v == 0 for v in sim.app.spd), "FAULT: motors stay stopped",
              str(sim.app.spd))
    log.check(sim.fsm.start() is True, "FSM_Start() accepted from FAULT")
    log.check(sim.fsm.state == ST_ENTRY, "FAULT -> ENTRY after restart",
              ST_NAMES[sim.fsm.state])
    ok = sim.run_until(lambda s: s.fsm.state == ST_RIDGE, max_ticks=2000)
    log.check(ok, "restarted sequence drives into ridge 1 again",
              "ph=%d cm" % sim.fsm.ph_cm)

    # (b) short dropout (160 ms) must not fault
    sim2 = Sim(cfg)
    sim2.start()
    sim2.run_until(lambda s: s.fsm.state == ST_RIDGE and s.fsm.ph_cm >= 20)
    sim2.laser.valid = False
    sim2.run(8)                         # 8 x 20 ms = 160 ms
    sim2.laser.valid = True
    sim2.run(200)
    log.check(sim2.fsm.state == ST_RIDGE, "160 ms laser dropout does not fault",
              ST_NAMES[sim2.fsm.state])

    # (c) SHIFT: invalid longer than 200 ms -> FAULT
    sim3 = Sim(cfg)
    sim3.start()
    ok = sim3.run_until(lambda s: s.fsm.state == ST_SHIFT, max_ticks=4000)
    log.check(ok, "setup: reached SHIFT")
    sim3.laser.valid = False
    ok = sim3.run_until(lambda s: s.fsm.state == ST_FAULT, max_ticks=100)
    log.check(ok and sim3.fsm.fault_code == 1,
              "SHIFT: laser invalid -> FAULT", ST_NAMES[sim3.fsm.state])
    log.check(all(v == 0 for v in sim3.app.spd), "SHIFT fault: motors stopped")

    # (d) ENTRY only relies on distance (spec: fault in RIDGE / SHIFT only)
    sim4 = Sim(cfg)
    sim4.start()
    sim4.laser.valid = False
    sim4.run(100)                       # 2 s
    log.check(sim4.fsm.state == ST_ENTRY,
              "ENTRY: invalid laser 2 s is not a RIDGE/SHIFT fault",
              ST_NAMES[sim4.fsm.state])


def test_run_timeout(cfg, log):
    print("\n-- run-time supervision --")
    sim = Sim(cfg)
    sim.start()
    sim.app.odom_frozen = True          # stuck / wheel spin: no progress
    ok = sim.run_until(lambda s: s.fsm.state == ST_FAULT, max_ticks=20000)
    log.check(ok, "stuck car: 4.5 min limit -> FAULT",
              ST_NAMES[sim.fsm.state])
    elapsed = sim.ms - sim.fsm.t0_ms
    log.check(sim.fsm.fault_code == 2, "fault code 2 (run time)",
              str(sim.fsm.fault_code))
    log.check(270000 < elapsed <= 270500,
              "FAULT just after 4.5 min", "%.2f s" % (elapsed / 1000.0))
    log.check(all(v == 0 for v in sim.app.spd), "FAULT: motors stopped",
              str(sim.app.spd))


def test_door_edges(cfg, log):
    print("\n-- doorway edge detection --")

    # ENTRY: blocked, then clear -> ridge 1 before the 40 cm fallback
    sim = Sim(cfg)
    sim.laser.dF = 400                  # near obstacle: gate frame
    sim.start()
    sim.run(50)                         # 1 s blocked, ~8 cm
    log.check(sim.fsm.state == ST_ENTRY, "ENTRY with a blocked front beam",
              ST_NAMES[sim.fsm.state])
    sim.laser.dF = 2000                 # open field through the door
    ok = sim.run_until(lambda s: s.fsm.state == ST_RIDGE, max_ticks=20)
    log.check(ok, "ENTRY: doorway edge ends the entry")
    d = [t["ph_cm"] for t in sim.transitions if t["from"] == ST_ENTRY][0]
    log.check(d < cfg["FSM_ENTRY_CM"],
              "ENTRY: door edge fires before the 40 cm fallback",
              "%d cm" % d)

    # (the other way round: a clear reading at power-up must not fire)
    sim0 = Sim(cfg)
    sim0.start()
    ok = sim0.run_until(lambda s: s.fsm.state == ST_RIDGE, max_ticks=1000)
    d = [t["ph_cm"] for t in sim0.transitions if t["from"] == ST_ENTRY][0]
    log.check(ok and d >= cfg["FSM_ENTRY_CM"] - 2,
              "ENTRY: clear front at power-up does not trigger the door edge",
              "entry ends on the distance fallback: %d cm" % d)

    # EXIT: blocked, then clear -> PARK before the fallback distance
    sim2 = Sim(cfg)
    sim2.start()
    ok = sim2.run_until(lambda s: s.fsm.state == ST_EXIT, max_ticks=40000)
    log.check(ok, "nominal run reaches EXIT")
    sim2.laser.dF = 400
    sim2.run(60)                        # 600 ms settle + 600 ms of creeping
    log.check(sim2.fsm.state == ST_EXIT, "EXIT with a blocked front beam",
              ST_NAMES[sim2.fsm.state])
    sim2.laser.dF = 2000
    ok = sim2.run_until(lambda s: s.fsm.state == ST_PARK, max_ticks=20)
    log.check(ok, "EXIT: doorway edge parks the car")
    d = [t["ph_cm"] for t in sim2.transitions if t["from"] == ST_EXIT][0]
    log.check(d < cfg["FSM_EXIT_CM"],
              "EXIT: door edge before the fallback distance", "%d cm" % d)


def test_start_rules(cfg, log):
    print("\n-- FSM_Start() rules --")
    sim = Sim(cfg)
    sim.start()
    ok = sim.run_until(lambda s: s.fsm.state == ST_RIDGE and s.fsm.ph_cm >= 20)
    log.check(ok, "setup: driving on a ridge")
    log.check(sim.fsm.start() is False, "FSM_Start() ignored while running")
    log.check(sim.fsm.state == ST_RIDGE, "running state unchanged",
              ST_NAMES[sim.fsm.state])

    sim2 = Sim(cfg)
    sim2.start()
    ok = sim2.run_until(lambda s: s.fsm.state == ST_PARK, max_ticks=40000)
    log.check(ok, "setup: reached PARK")
    log.check(sim2.fsm.start() is False, "FSM_Start() ignored in PARK")
    log.check(sim2.fsm.state == ST_PARK, "PARK stays PARK",
              ST_NAMES[sim2.fsm.state])


# ========================================================================
#  main
# ========================================================================
def main():
    print("FSM host simulation - USER/fsm.c (T4 lane shift, T5 state machine)")
    print("configuration source: %s" % FSM_H_PATH)
    try:
        cfg = load_config()
    except (OSError, ValueError) as exc:
        print("[FAIL] cannot read the fsm.h configuration: %s" % exc)
        return 1

    log = Log()
    test_config(cfg, log)
    test_idle(cfg, log)
    test_nominal(cfg, log)
    test_laser_fault(cfg, log)
    test_run_timeout(cfg, log)
    test_door_edges(cfg, log)
    test_start_rules(cfg, log)

    print()
    if log.fails:
        print("RESULT: FAIL - %d of %d checks failed"
              % (len(log.fails), log.checks))
        for f in log.fails:
            print("  - " + f)
        return 1
    print("RESULT: PASS - %d checks" % log.checks)
    return 0


if __name__ == "__main__":
    sys.exit(main())
