#!/usr/bin/env python3
"""
Host side simulation / unit test for USER/centering.c  (T3 / S2 centering).

stdlib only, Python 3.

The controller is re-implemented here line by line (``CenteringModel``) as a
faithful port of ``USER/centering.c``.  All gains, clamps, deadband, filters
and slew limits are *parsed out of* ``USER/centering.h`` (and the servo
sentinel out of ``cenctering.c``), so the test automatically follows the
firmware when the Lead retunes it.  A source lint at the end fails the run if
the .c/.h drift away from what the port assumes, or if a file stops being
pure ASCII.

Plant model (per 20 ms tick, "e" = laser lateral error, "psi" = IMU yaw):

    all four wheels steer by delta      -> the car translates sideways
    front wheels +Delta, rear -Delta    -> the car yaws in place

    psi_dot = 2 * v * cos(delta) * sin(Delta) / L          [deg/s]
    e_dot   = -v * sin(psi + delta) * cos(Delta)           [cm/s]

with v derived from the encoder base speed (60000 counts / wheel turn,
76.4 mm calibrated wheel diameter according to main.c).

Sign conventions checked by this test:
    e > 0  (left wall farther, car right of centre)  ->  steer LEFT  (delta>0)
    psi > 0 (nose left of reference)                 ->  toe RIGHT  (Delta<0)

Run:  python tools/test_centering.py
Exit: 0 = all checks passed, 1 = at least one failure.
"""

import math
import random
import re
import sys
from pathlib import Path
from types import SimpleNamespace

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
HEADER_PATH = ROOT / "USER" / "centering.h"
SOURCE_PATH = ROOT / "USER" / "centering.c"

# encoder / wheel constants from USER/main.c (calibrated there)
COUNTS_PER_REV = 60000.0
WHEEL_DIAMETER_MM = 76.4
# circumference in cm: 60000 counts per ~24.0 cm -> 2500 counts/cm,
# i.e. 1 count/tick (20 ms) = 0.02 cm/s  =>  V900 = 18 cm/s, 2.7 m in 15 s
COUNTS_PER_CM = COUNTS_PER_REV / (math.pi * WHEEL_DIAMETER_MM / 10.0)

# furrow / chassis geometry used by the plant
FURROW_WIDTH_CM = 40.0
WHEELBASE_CM = 30.0

RESULTS = []


# ---------------------------------------------------------------------------
# tiny test harness
# ---------------------------------------------------------------------------
def check(name, ok, detail=""):
    RESULTS.append((name, bool(ok)))
    tag = "[PASS] " if ok else "[FAIL] "
    line = tag + name
    if detail:
        line += "  " + detail
    print(line)
    return bool(ok)


def banner(text):
    print("")
    print("--- " + text + " " + "-" * max(0, 68 - len(text)))


# ---------------------------------------------------------------------------
# parse the tuning constants out of the real firmware sources
# ---------------------------------------------------------------------------
DEFINE_RE = re.compile(r"^#define\s+([A-Z][A-Z0-9_]*)\s+(.+?)\s*$")
NUM_RE = re.compile(r"^\(\s*([-+]?\d+(?:\.\d*)?)\s*[fFuU]?\s*\)$")


def parse_numeric_define(expr):
    s = expr.strip()
    m = NUM_RE.match(s)
    if m:
        s = m.group(1)
    else:
        s = s.strip("()")
        if s and s[-1] in "fFuU":
            s = s[:-1]
    try:
        return float(s)
    except ValueError:
        return None


def parse_defines(path):
    out = {}
    text = path.read_text(encoding="utf-8", errors="strict")
    for raw in text.splitlines():
        line = raw.split("/*")[0].rstrip()
        m = DEFINE_RE.match(line)
        if not m:
            continue
        name, expr = m.group(1), m.group(2)
        val = parse_numeric_define(expr)
        if val is not None:
            out[name] = val
    return out, text


class Cfg:
    """attribute style access to the parsed #defines"""

    def __init__(self, d):
        self.__dict__.update(d)


NEEDED = [
    "CENTER_E_DEADBAND_CM", "CENTER_E_FILT_ALPHA", "CENTER_E_D_ALPHA",
    "CENTER_E_KP", "CENTER_E_KI", "CENTER_E_KD", "CENTER_E_I_MAX",
    "CENTER_E_MAX_CM", "CENTER_DELTA_MAX", "CENTER_DELTA_SLEW",
    "CENTER_YAW_KP", "CENTER_YAW_KI", "CENTER_YAW_KD", "CENTER_YAW_I_MAX",
    "CENTER_YAW_MAX_DEG", "CENTER_TOE_MAX", "CENTER_TOE_SLEW",
    "CENTER_DT_S", "CENTER_SIDE_MIN_MM", "CENTER_SIDE_MAX_MM",
    "CENTER_STEER_EPS_DEG", "CENTER_SPD_DIFF_KD", "CENTER_SPD_DIFF_MAX",
    "CENTER_YAW_SIGN", "CENTER_STEER_SIGN", "CENTER_TOE_SIGN",
]


def load_config():
    hdr, hdr_text = parse_defines(HEADER_PATH)
    src, src_text = parse_defines(SOURCE_PATH)
    missing = [n for n in NEEDED if n not in hdr]
    if missing:
        raise SystemExit("centering.h is missing defines: " + ", ".join(missing))
    unknown = src.get("CF_SERVO_UNKNOWN")
    if unknown is None:
        raise SystemExit("centering.c is missing CF_SERVO_UNKNOWN")
    cfg = Cfg(hdr)
    cfg.SERVO_UNKNOWN = unknown
    return cfg, hdr_text, src_text


# ---------------------------------------------------------------------------
# controller port (mirror of USER/centering.c)
# ---------------------------------------------------------------------------
def cf_abs(x):
    return -x if x < 0.0 else x


def cf_limit(x, lim):
    if x > lim:
        return lim
    if x < -lim:
        return -lim
    return x


def cf_wrap180(d):
    while d > 180.0:
        d -= 360.0
    while d < -180.0:
        d += 360.0
    return d


def cf_side_ok(mm, c):
    if mm == 0xFFFF:
        return False
    if mm < c.CENTER_SIDE_MIN_MM:
        return False
    if mm > c.CENTER_SIDE_MAX_MM:
        return False
    return True


class FakeServo:
    """records what main.c would receive from App_SetSteer4 / App_ServoCenter"""

    def __init__(self):
        self.calls = []
        self.center_calls = 0
        self.last = (0.0, 0.0, 0.0, 0.0)

    def reset(self):
        self.calls = []
        self.center_calls = 0
        self.last = (0.0, 0.0, 0.0, 0.0)

    def set_steer4(self, fl, fr, rl, rr):
        self.calls.append((fl, fr, rl, rr))
        self.last = (fl, fr, rl, rr)

    def set_center(self):
        self.center_calls += 1
        self.calls.append((0.0, 0.0, 0.0, 0.0))
        self.last = (0.0, 0.0, 0.0, 0.0)


class CenteringModel:
    """line by line port of Centering_Init / Centering_Update"""

    def __init__(self, cfg, servo):
        self.c = cfg
        self.servo = servo
        self._reset()
        self.s_active = 0
        self.s_dir = 0
        self.s_out = [0.0, 0.0, 0.0, 0.0]
        self.dbg = SimpleNamespace(e_cm=0.0, delta_deg=0.0, toe_deg=0.0,
                                   valid=0, saturated=0)
        servo.set_center()          # Centering_Init() calls App_SetServoCenter()

    def _reset(self):
        self.s_e_filt = 0.0
        self.s_e_prev = 0.0
        self.s_e_d = 0.0
        self.s_i_e = 0.0
        self.s_delta = 0.0
        self.s_i_yaw = 0.0
        self.s_toe = 0.0

    def _forget(self):
        self.s_out = [self.c.SERVO_UNKNOWN] * 4

    def _steer(self, delta, toe):
        c = self.c
        fl = delta + toe
        fr = delta + toe
        rl = delta - toe
        rr = delta - toe
        if all(cf_abs(v - w) < c.CENTER_STEER_EPS_DEG
               for v, w in zip((fl, fr, rl, rr), self.s_out)):
            return
        self.s_out = [fl, fr, rl, rr]
        self.servo.set_steer4(fl, fr, rl, rr)

    def update(self, laser, yaw_deg, yaw_rate_dps, base_spd):
        c = self.c
        dirn = 1 if base_spd > 0 else (-1 if base_spd < 0 else 0)
        if dirn != self.s_dir:
            self.s_i_e = 0.0
            self.s_i_yaw = 0.0
            self.s_delta = 0.0
            self.s_toe = 0.0
            self.s_dir = dirn

        valid = 0
        if laser is not None and laser.valid and (laser.flags & 0x08) == 0:
            if cf_side_ok(laser.dL, c) and cf_side_ok(laser.dR, c):
                valid = 1
        self.dbg.valid = valid

        # 2. car stopped -> release the servos, accumulate nothing
        if dirn == 0:
            self._reset()
            self.s_active = 0
            self._forget()
            self.dbg.delta_deg = 0.0
            self.dbg.toe_deg = 0.0
            self.dbg.saturated = 0
            return [base_spd] * 4

        # 3. no usable side lasers -> safe straight
        if not valid:
            self._reset()
            if self.s_active:
                self.servo.set_center()
                self.s_out = [0.0, 0.0, 0.0, 0.0]
            self.s_active = 0
            self.dbg.e_cm = 0.0
            self.dbg.delta_deg = 0.0
            self.dbg.toe_deg = 0.0
            self.dbg.saturated = 0
            return [base_spd] * 4

        # 4. lateral loop: e [cm] -> delta [deg]
        e_raw = (float(laser.dL) - float(laser.dR)) * 0.05
        e_raw = cf_limit(e_raw, c.CENTER_E_MAX_CM)

        self.s_e_filt += c.CENTER_E_FILT_ALPHA * (e_raw - self.s_e_filt)

        d_e = (self.s_e_filt - self.s_e_prev) / c.CENTER_DT_S
        self.s_e_prev = self.s_e_filt
        self.s_e_d += c.CENTER_E_D_ALPHA * (d_e - self.s_e_d)

        e_use = self.s_e_filt
        if e_use > c.CENTER_E_DEADBAND_CM:
            e_use -= c.CENTER_E_DEADBAND_CM
        elif e_use < -c.CENTER_E_DEADBAND_CM:
            e_use += c.CENTER_E_DEADBAND_CM
        else:
            e_use = 0.0

        p = c.CENTER_E_KP * e_use
        d = (c.CENTER_E_KD * self.s_e_d) if e_use != 0.0 else 0.0

        i_new = self.s_i_e + (c.CENTER_E_KI * e_use) * c.CENTER_DT_S
        i_new = cf_limit(i_new, c.CENTER_E_I_MAX)

        out = p + d + i_new
        sat_e = 0
        if out > c.CENTER_DELTA_MAX:
            out = c.CENTER_DELTA_MAX
            sat_e = 1
        if out < -c.CENTER_DELTA_MAX:
            out = -c.CENTER_DELTA_MAX
            sat_e = 1
        if sat_e:
            i_new = cf_limit(out - p - d, c.CENTER_E_I_MAX)
        self.s_i_e = i_new

        if out > self.s_delta + c.CENTER_DELTA_SLEW:
            out = self.s_delta + c.CENTER_DELTA_SLEW
        if out < self.s_delta - c.CENTER_DELTA_SLEW:
            out = self.s_delta - c.CENTER_DELTA_SLEW
        self.s_delta = out

        # 5. heading loop: yaw [deg] -> toe Delta [deg]
        yaw_err = cf_wrap180(c.CENTER_YAW_SIGN * yaw_deg)
        yaw_rate = c.CENTER_YAW_SIGN * yaw_rate_dps
        yaw_err = cf_limit(yaw_err, c.CENTER_YAW_MAX_DEG)

        i_new = self.s_i_yaw + (c.CENTER_YAW_KI * yaw_err) * c.CENTER_DT_S
        i_new = cf_limit(i_new, c.CENTER_YAW_I_MAX)

        out = -(c.CENTER_YAW_KP * yaw_err + c.CENTER_YAW_KD * yaw_rate + i_new)
        sat_y = 0
        if out > c.CENTER_TOE_MAX:
            out = c.CENTER_TOE_MAX
            sat_y = 1
        if out < -c.CENTER_TOE_MAX:
            out = -c.CENTER_TOE_MAX
            sat_y = 1
        if sat_y:
            i_new = cf_limit(-out - c.CENTER_YAW_KP * yaw_err
                             - c.CENTER_YAW_KD * yaw_rate, c.CENTER_YAW_I_MAX)
        self.s_i_yaw = i_new

        if out > self.s_toe + c.CENTER_TOE_SLEW:
            out = self.s_toe + c.CENTER_TOE_SLEW
        if out < self.s_toe - c.CENTER_TOE_SLEW:
            out = self.s_toe - c.CENTER_TOE_SLEW
        self.s_toe = out

        # 6. wheel angles (mirrored when reversing)
        delta = (c.CENTER_STEER_SIGN * float(dirn)) * self.s_delta
        toe = (c.CENTER_TOE_SIGN * float(dirn)) * self.s_toe
        self._steer(delta, toe)
        self.s_active = 1

        # 7. per wheel speed targets
        dif = int(-c.CENTER_SPD_DIFF_KD * yaw_rate)
        lim = int(c.CENTER_SPD_DIFF_MAX * (base_spd if base_spd >= 0
                                           else -base_spd))
        if dif > lim:
            dif = lim
        if dif < -lim:
            dif = -lim

        spd = [int(base_spd - dif), int(base_spd + dif),
               int(base_spd - dif), int(base_spd + dif)]

        self.dbg.e_cm = self.s_e_filt
        self.dbg.delta_deg = delta
        self.dbg.toe_deg = toe
        self.dbg.saturated = int(sat_e or sat_y)
        return spd


# ---------------------------------------------------------------------------
# plant
# ---------------------------------------------------------------------------
def to_mm(cm):
    v = int(round(cm * 10.0))
    return max(1, min(65534, v))


class Plant:
    def __init__(self, e0_cm, psi0_deg=0.0,
                 width_cm=FURROW_WIDTH_CM, wheelbase_cm=WHEELBASE_CM):
        self.e = float(e0_cm)
        self.psi = float(psi0_deg)
        self.psi_rate = 0.0
        self.dist_cm = 0.0
        self.W = width_cm
        self.L = wheelbase_cm

    def laser(self, noise_cm=0.0, rng=None):
        dl = self.W / 2.0 + self.e
        dr = self.W / 2.0 - self.e
        if noise_cm > 0.0 and rng is not None:
            dl += rng.uniform(-noise_cm, noise_cm)
            dr += rng.uniform(-noise_cm, noise_cm)
        return SimpleNamespace(dL=to_mm(dl), dR=to_mm(dr), dF=1500,
                               flags=0x07, valid=1)

    def step(self, delta_deg, toe_deg, base_spd, dt):
        v = base_spd / (COUNTS_PER_CM * dt)          # cm/s, signed
        drad = math.radians(delta_deg)
        trad = math.radians(toe_deg)
        self.psi_rate = math.degrees(
            2.0 * v * math.cos(drad) * math.sin(trad) / self.L)
        y_dot = v * math.sin(math.radians(self.psi) + drad) * math.cos(trad)
        self.psi += self.psi_rate * dt
        self.e -= y_dot * dt                        # e = -(lateral position)
        self.dist_cm += v * dt


# ---------------------------------------------------------------------------
# closed loop simulation helper
# ---------------------------------------------------------------------------
def simulate(cfg, *, e0, psi0=0.0, base_spd=900, seconds=15.0, noise_cm=0.0,
             seed=1, break_at=None, break_to="invalid", break_side=None,
             hold_plant=False):
    """break_at: [s] after which the laser frame degrades (see break_to).

    hold_plant=True freezes the car (bench clamp test): the controller keeps
    seeing the same e / yaw, which is how the clamp and slew limits are
    checked without the car running out of the 40 cm corridor."""
    servo = FakeServo()
    model = CenteringModel(cfg, servo)
    servo.reset()
    plant = Plant(e0, psi0)
    rng = random.Random(seed)
    dt = cfg.CENTER_DT_S
    n = int(round(seconds / dt))
    traj = []

    for k in range(n):
        t = k * dt
        laser = plant.laser(noise_cm, rng)

        if break_at is not None and t >= break_at - 1e-9:
            if break_to == "invalid":
                laser.valid = 0
            elif break_to == "timeout":
                laser.flags |= 0x08
            elif break_to == "left_ffff":
                laser.dL = 0xFFFF
            elif break_to == "right_zero":
                laser.dR = 0
            elif break_to == "left_close":
                laser.dL = 5
            elif break_to == "left_far":
                laser.dL = 6000
            if break_side == "left":
                laser.dL = 0xFFFF

        spd = model.update(laser, plant.psi, plant.psi_rate, base_spd)
        wheels = servo.last
        traj.append(dict(t=t, e=plant.e, psi=plant.psi, yaw_rate=plant.psi_rate,
                         e_filt=model.dbg.e_cm, delta=model.dbg.delta_deg,
                         toe=model.dbg.toe_deg, valid=model.dbg.valid,
                         sat=model.dbg.saturated, wheels=wheels, spd=spd,
                         center=servo.center_calls))
        if not hold_plant:
            plant.step(model.dbg.delta_deg, model.dbg.toe_deg, base_spd, dt)

    return SimpleNamespace(model=model, servo=servo, plant=plant, traj=traj,
                           dt=dt)


def settle_time(traj, lim_cm, window_s=1.0):
    """first time |e| stays below lim for at least window_s, else None"""
    dt = traj[1]["t"] - traj[0]["t"] if len(traj) > 1 else 0.02
    need = int(round(window_s / dt))
    run = 0
    for i, p in enumerate(traj):
        if abs(p["e"]) <= lim_cm:
            run += 1
            if run >= need:
                return traj[i - need + 1]["t"]
        else:
            run = 0
    return None


def max_abs(traj, key):
    return max(abs(p[key]) for p in traj)


# ---------------------------------------------------------------------------
# tests
# ---------------------------------------------------------------------------
def test_lateral_step(cfg):
    banner("(a) lateral step response converges")
    ok_all = True
    for e0, base in ((+5.0, 900), (-4.0, 900), (+5.0, 400), (-4.0, 400)):
        sim = simulate(cfg, e0=e0, base_spd=base, seconds=15.0)
        tr = sim.traj
        e_end = tr[-1]["e"]
        ts = settle_time(tr, 2.0)
        # overshoot: how far past the centre line does it swing?
        if e0 > 0:
            overshoot = max(0.0, -min(p["e"] for p in tr))
        else:
            overshoot = max(0.0, max(p["e"] for p in tr))
        speed = base / (COUNTS_PER_CM * cfg.CENTER_DT_S)
        ok = (abs(e_end) <= 1.0) and (ts is not None) and (ts <= 12.0) \
            and (overshoot <= 1.5)
        ok_all &= ok
        check("a-e%+0.1f-%dcm/s  converge" % (e0, int(round(speed))), ok,
              "e_end=%+.2fcm settle(2cm)=%s overshoot=%.2fcm dist=%.0fcm"
              % (e_end, ("%.1fs" % ts) if ts is not None else "never",
                 overshoot, sim.plant.dist_cm))
    return ok_all


def test_heading_step(cfg):
    banner("(a2) yaw step response converges (heading, no lateral offset)")
    sim = simulate(cfg, e0=0.0, psi0=10.0, base_spd=900, seconds=15.0)
    tr = sim.traj
    psi_end = tr[-1]["psi"]
    ts = None
    for i, p in enumerate(tr):
        if abs(p["psi"]) <= 2.0 and all(abs(q["psi"]) <= 2.0 for q in tr[i:i + 50]):
            ts = p["t"]
            break
    e_worst = max_abs(tr, "e")
    ok = abs(psi_end) <= 1.0 and ts is not None and e_worst <= 3.0
    check("a2-yaw10deg  converge", ok,
          "psi_end=%+.2fdeg settle(2deg)=%s |e|max=%.2fcm"
          % (psi_end, ("%.1fs" % ts) if ts is not None else "never", e_worst))
    return ok


def slews(traj):
    """max per-tick change of delta / toe, only over the normal (laser valid)
    path: the fail-safe takeover deliberately drops the steering to zero at
    once, so it is not slew limited."""
    ds = []
    ts = []
    for i in range(1, len(traj)):
        a, b = traj[i - 1], traj[i]
        if a["valid"] and b["valid"]:
            ds.append(abs(b["delta"] - a["delta"]))
            ts.append(abs(b["toe"] - a["toe"]))
    return (max(ds) if ds else 0.0), (max(ts) if ts else 0.0)


def test_clamps(cfg):
    banner("(b) steering clamps and slew limits are respected")
    ok_all = True
    eps = 1e-4

    # bench-clamp cases: the car does not move, the controller keeps seeing a
    # saturated lateral error / yaw angle for 3 s
    cases = [
        ("held: lateral error 18cm", dict(e0=18.0, psi0=0.0), True, False),
        ("held: yaw 90deg", dict(e0=0.0, psi0=90.0), False, True),
        ("held: 18cm and 45deg", dict(e0=18.0, psi0=45.0), True, True),
    ]
    for name, kw, exp_d, exp_t in cases:
        sim = simulate(cfg, base_spd=900, seconds=3.0, hold_plant=True, **kw)
        tr = sim.traj
        dmax = max_abs(tr, "delta")
        tmax = max_abs(tr, "toe")
        wmax = max(max(abs(w) for w in p["wheels"]) for p in tr)
        slew_d, slew_t = slews(tr)
        ok = (dmax <= cfg.CENTER_DELTA_MAX + eps and
              tmax <= cfg.CENTER_TOE_MAX + eps and
              wmax <= cfg.CENTER_DELTA_MAX + cfg.CENTER_TOE_MAX + eps and
              slew_d <= cfg.CENTER_DELTA_SLEW + eps and
              slew_t <= cfg.CENTER_TOE_SLEW + eps)
        if exp_d:                       # really did hit the clamp
            ok = ok and (dmax > cfg.CENTER_DELTA_MAX * 0.99)
        if exp_t:
            ok = ok and (tmax > cfg.CENTER_TOE_MAX * 0.99)
        ok_all &= ok
        check("b-" + name, ok,
              "|delta|max=%.2f<=%.1f |toe|max=%.2f<=%.1f |wheel|max=%.2f "
              "slew=%.2f/%.2f" % (dmax, cfg.CENTER_DELTA_MAX, tmax,
                                  cfg.CENTER_TOE_MAX, wmax, slew_d, slew_t))

    # closed loop: the limits must also hold while actually driving
    sim = simulate(cfg, e0=6.0, psi0=8.0, base_spd=900, seconds=10.0)
    tr = sim.traj
    dmax = max_abs(tr, "delta")
    tmax = max_abs(tr, "toe")
    wmax = max(max(abs(w) for w in p["wheels"]) for p in tr)
    slew_d, slew_t = slews(tr)
    ok = (dmax <= cfg.CENTER_DELTA_MAX + eps and
          tmax <= cfg.CENTER_TOE_MAX + eps and
          wmax <= cfg.CENTER_DELTA_MAX + cfg.CENTER_TOE_MAX + eps and
          slew_d <= cfg.CENTER_DELTA_SLEW + eps and
          slew_t <= cfg.CENTER_TOE_SLEW + eps)
    ok_all &= ok
    check("b-driving 6cm+8deg", ok,
          "|delta|max=%.2f |toe|max=%.2f |wheel|max=%.2f slew=%.2f/%.2f"
          % (dmax, tmax, wmax, slew_d, slew_t))
    return ok_all


def test_invalid_laser(cfg):
    banner("(c) invalid laser -> steering driven to zero, no accumulation")
    ok_all = True
    kinds = ["invalid", "timeout", "left_ffff", "right_zero",
             "left_close", "left_far"]
    for kind in kinds:
        sim = simulate(cfg, e0=6.0, psi0=6.0, base_spd=900, seconds=4.0,
                       break_at=1.5, break_to=kind)
        tr = sim.traj
        k_break = int(round(1.5 / cfg.CENTER_DT_S))
        before = tr[k_break - 1]
        after = tr[k_break:]
        steer_before = max(abs(w) for w in before["wheels"])
        ok_steer = all(abs(w) < 1e-6 for p in after for w in p["wheels"])
        ok_flag = all(p["valid"] == 0 for p in after)
        ok_zero = all(abs(p["delta"]) < 1e-6 and abs(p["toe"]) < 1e-6
                      for p in after)
        center_calls = sim.servo.center_calls
        ok_center = center_calls == (1 if k_break > 0 else 0)
        ok_spd = all(p["spd"] == [900] * 4 for p in after)
        ok = (ok_steer and ok_flag and ok_zero and ok_center and ok_spd
              and steer_before > 0.5)
        ok_all &= ok
        check("c-%s" % kind, ok,
              "steer before=%0.2fdeg -> after=0 (%d ticks) center_calls=%d "
              "spd=%s" % (steer_before, len(after), center_calls,
                          "all base" if ok_spd else "WRONG"))

    # a NULL laser pointer must behave exactly like an invalid frame
    servo = FakeServo()
    model = CenteringModel(cfg, servo)
    servo.reset()
    laser = SimpleNamespace(dL=230, dR=170, dF=1500, flags=0x07, valid=1)
    model.update(laser, 0.0, 0.0, 900)
    steered = max(abs(w) for w in servo.last)
    spd = model.update(None, 0.0, 0.0, 900)
    ok = (steered > 0.5 and servo.center_calls == 1 and
          abs(max(abs(w) for w in servo.last)) < 1e-9 and
          spd == [900] * 4 and abs(model.dbg.delta_deg) < 1e-9)
    ok_all &= ok
    check("c-NULL laser pointer", ok,
          "steer=%0.2fdeg -> 0, center_calls=%d" % (steered,
                                                    servo.center_calls))
    return ok_all


def test_signs(cfg):
    banner("(d) sign conventions")
    ok_all = True

    # d1: positive e (car right of centre) must steer LEFT (delta > 0)
    sim = simulate(cfg, e0=4.0, base_spd=900, seconds=1.0)
    d0 = sim.traj[1]["delta"]
    ok = d0 > 0.1 and sim.traj[-1]["e"] < 4.0
    ok_all &= ok
    check("d1-e>0 -> delta>0 (steer left), e decreases", ok,
          "delta(1st)=%+.2fdeg e=%0.2f->%0.2fcm"
          % (d0, sim.traj[0]["e"], sim.traj[-1]["e"]))

    # d2: negative e must steer RIGHT
    sim = simulate(cfg, e0=-4.0, base_spd=900, seconds=1.0)
    d0 = sim.traj[1]["delta"]
    ok = d0 < -0.1 and sim.traj[-1]["e"] > -4.0
    ok_all &= ok
    check("d2-e<0 -> delta<0 (steer right), e increases", ok,
          "delta(1st)=%+.2fdeg e=%0.2f->%0.2fcm"
          % (d0, sim.traj[-1]["e"], sim.traj[-1]["e"]))

    # d3: positive yaw (nose left) must give a NEGATIVE toe (yaw right)
    sim = simulate(cfg, e0=0.0, psi0=6.0, base_spd=900, seconds=1.0)
    t0 = sim.traj[1]["toe"]
    ok = t0 < -0.1 and sim.traj[-1]["psi"] < 6.0
    ok_all &= ok
    check("d3-yaw>0 -> toe<0 (yaw right), psi decreases", ok,
          "toe(1st)=%+.2fdeg psi=%0.2f->%0.2fdeg"
          % (t0, sim.traj[0]["psi"], sim.traj[-1]["psi"]))

    # d4: servo wiring FL/FR = delta+Delta, RL/RR = delta-Delta
    #     (the frame we last sent may lag by up to CENTER_STEER_EPS_DEG,
    #      because identical commands are not re-sent every tick)
    sim = simulate(cfg, e0=5.0, psi0=4.0, base_spd=900, seconds=2.0)
    tol = cfg.CENTER_STEER_EPS_DEG + 1e-6
    ok = True
    for p in sim.traj:
        fl, fr, rl, rr = p["wheels"]
        if (abs(fl - fr) > 1e-9 or abs(rl - rr) > 1e-9 or
                abs((fl + rl) * 0.5 - p["delta"]) > tol or
                abs((fl - rl) * 0.5 - p["toe"]) > tol):
            ok = False
            break
    ok_all &= ok
    last = sim.traj[-1]["wheels"]
    check("d4-servo map FL=FR=delta+toe, RL=RR=delta-toe", ok,
          "last wheels=(%+.2f,%+.2f,%+.2f,%+.2f) delta=%+.2f toe=%+.2f"
          % (last + (sim.traj[-1]["delta"], sim.traj[-1]["toe"])))

    return ok_all


def test_deadband(cfg):
    banner("(e) deadband keeps the servos quiet on the centre line")
    sim = simulate(cfg, e0=0.2, base_spd=900, seconds=5.0)
    ok = len(sim.servo.calls) == 0 and all(
        abs(p["delta"]) < 1e-9 for p in sim.traj)
    check("e-|e|=0.2cm < deadband -> no servo traffic", ok,
          "servo frames=%d" % len(sim.servo.calls))
    return ok


def test_noise(cfg):
    banner("(f) sensor noise: still converges, servo bus not flooded")
    sim = simulate(cfg, e0=3.0, base_spd=900, seconds=15.0,
                   noise_cm=0.35, seed=7)
    tr = sim.traj[250:]          # ignore the initial transient
    e_worst = max_abs(tr, "e")
    e_end = tr[-1]["e"]
    span_s = tr[-1]["t"] - tr[0]["t"]
    frames_per_s = len(sim.servo.calls) / 15.0
    chatter = sum(abs(tr[i]["delta"] - tr[i - 1]["delta"])
                  for i in range(1, len(tr))) / span_s
    ok = (e_worst <= 3.0 and abs(e_end) <= 1.5 and
          frames_per_s <= 30.0 and chatter <= 10.0)
    check("f-noise +-0.35cm", ok,
          "|e|max=%0.2fcm e_end=%+.2fcm servo frames=%.1f/s "
          "delta chatter=%.1f deg/s" % (e_worst, e_end, frames_per_s, chatter))
    return ok


def test_reverse(cfg):
    banner("(g) reverse driving (base_spd < 0) mirrors correctly")
    sim = simulate(cfg, e0=4.0, base_spd=-900, seconds=6.0)
    tr = sim.traj
    d0 = tr[1]["delta"]
    ok = d0 < -0.1 and tr[-1]["e"] < 4.0
    check("g-reverse e>0 -> delta<0, e still decreases", ok,
          "delta(1st)=%+.2fdeg e=%0.2f->%0.2fcm"
          % (d0, tr[0]["e"], tr[-1]["e"]))
    return ok


def test_source_lint(cfg, hdr_text, src_text):
    banner("(h) source lint (ASCII, contract, no drift from this port)")
    ok_all = True

    for path, text in ((HEADER_PATH, hdr_text), (SOURCE_PATH, src_text)):
        bad = [(i + 1, ch) for i, line in enumerate(text.splitlines())
               for ch in line if ord(ch) > 0x7F]
        ok = not bad
        ok_all &= ok
        check("h-ascii %s" % path.name, ok,
              "pure ASCII" if ok else "non-ASCII at %r" % (bad[:3],))

    must_have = [
        "void Centering_Init(void)",
        "void Centering_Update(const LaserData *laser, float yaw_deg,",
        "s16 base_spd, s16 spd_tgt[4])",
        "App_SetSteer4(fl, fr, rl, rr);",
        "float fl = delta + toe;",
        "float rl = delta - toe;",
        "App_SetServoCenter();",
        "if ((laser != 0) && (laser->valid != 0u)",
        "0xFFFFu",
        "CENTER_DELTA_MAX",
        "CENTER_TOE_MAX",
    ]
    flat = re.sub(r"\s+", " ", src_text)
    for item in must_have:
        ok = re.sub(r"\s+", " ", item) in flat
        ok_all &= ok
        check("h-src has %r" % item, ok, "")

    hdr_has = ["void Centering_Init(void);",
               "void Centering_Update(const LaserData *laser, float yaw_deg,"
               " float yaw_rate_dps,",
               "s16 base_spd, s16 spd_tgt[4]);",
               '#include "app_if.h"']
    hflat = re.sub(r"\s+", " ", hdr_text)
    for item in hdr_has:
        ok = re.sub(r"\s+", " ", item) in hflat
        ok_all &= ok
        check("h-hdr has %r" % item, ok, "")

    # C90 style: no declarations introduced after the opening brace of a
    # for loop, and no // comments
    ok = "//" not in src_text and "//" not in hdr_text
    ok_all &= ok
    check("h-no // comments (C90)", ok, "")

    # no C99 for-loop declarations
    bad = re.findall(r"for\s*\(\s*(?:int|u8|u16|u32|s8|s16|s32|float)\b",
                     src_text)
    ok = not bad
    ok_all &= ok
    check("h-no C99 for-loop declarations", ok,
          "" if ok else "%d found" % len(bad))

    return ok_all


def main():
    print("=" * 74)
    print("T3 centering - host plant simulation / unit test")
    print("=" * 74)

    if not HEADER_PATH.exists() or not SOURCE_PATH.exists():
        print("MISSING: %s / %s" % (HEADER_PATH, SOURCE_PATH))
        return 1

    cfg, hdr_text, src_text = load_config()
    print("controller gains parsed from USER/centering.h:")
    print("  lateral : Kp=%.2f deg/cm  Ki=%.2f  Kd=%.2f  deadband=%.2f cm  "
          "LPF=%.2f  I_max=%.1f" %
          (cfg.CENTER_E_KP, cfg.CENTER_E_KI, cfg.CENTER_E_KD,
           cfg.CENTER_E_DEADBAND_CM, cfg.CENTER_E_FILT_ALPHA,
           cfg.CENTER_E_I_MAX))
    print("  heading : Kp=%.2f  Ki=%.2f  Kd=%.2f  (deg toe per deg yaw)  "
          "I_max=%.1f" %
          (cfg.CENTER_YAW_KP, cfg.CENTER_YAW_KI, cfg.CENTER_YAW_KD,
           cfg.CENTER_YAW_I_MAX))
    print("  limits  : delta +-%.1f deg (slew %.2f/tick)   toe +-%.1f deg "
          "(slew %.2f/tick)" %
          (cfg.CENTER_DELTA_MAX, cfg.CENTER_DELTA_SLEW, cfg.CENTER_TOE_MAX,
           cfg.CENTER_TOE_SLEW))
    print("  signs   : yaw=%+.0f steer=%+.0f toe=%+.0f   dt=%.3fs" %
          (cfg.CENTER_YAW_SIGN, cfg.CENTER_STEER_SIGN, cfg.CENTER_TOE_SIGN,
           cfg.CENTER_DT_S))
    print("  plant   : furrow %.0f cm, wheelbase %.0f cm, %.0f counts/cm "
          "(-> %.2f cm/s per count/tick)" %
          (FURROW_WIDTH_CM, WHEELBASE_CM, COUNTS_PER_CM,
           1.0 / (COUNTS_PER_CM * cfg.CENTER_DT_S)))

    ok = True
    ok &= test_lateral_step(cfg)
    ok &= test_heading_step(cfg)
    ok &= test_clamps(cfg)
    ok &= test_invalid_laser(cfg)
    ok &= test_signs(cfg)
    ok &= test_deadband(cfg)
    ok &= test_noise(cfg)
    ok &= test_reverse(cfg)
    ok &= test_source_lint(cfg, hdr_text, src_text)

    npass = sum(1 for _, r in RESULTS if r)
    failed = [n for n, r in RESULTS if not r]
    print("")
    print("=" * 74)
    print("%d/%d checks passed" % (npass, len(RESULTS)))
    if failed:
        print("FAILED: " + ", ".join(failed))
    print("RESULT: " + ("PASS" if ok and not failed else "FAIL"))
    print("=" * 74)
    return 0 if (ok and not failed) else 1


if __name__ == "__main__":
    sys.exit(main())
