#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""wheel_diag.py -- C30D 4-wheel balance / yaw-drift diagnostic (Sprint-1 T1).

Parses a captured C30D serial log (USART1 / USART3 / UART5, 1 line per second
per port) and reports:

  * per-wheel mean measured speed and mean PWM -- FWD / BACK lines only,
  * PWM spread (max-min)/median * 100 % and every wheel relative to the median,
  * yaw drift per metre, with the line period taken from --period-ms,
  * PASS/FAIL for the T1 acceptance criteria:
        PWM spread <= 20 %   and   |yaw drift| <= 2 deg/m,
  * optional open-loop ("OL PWM=...") speed comparison for fault isolation.

Firmware log format (USER/main.c, one line per second per port):

  SPD FWD tgt=900 1007/3278 926/3784 428/4335 998/3418 YAW=-2 C=0 SVO=0 P=5 CMD=G
  field order: FL FR RL RR, each "measured_speed/pwm"; YAW is 0.1 deg units.

Distance model (matches README: ENCODER_CPR = 60000, effective wheel diameter
76.4 mm = 7.64 cm, measured speed in counts per 20 ms = 50 Hz control period):

  counts/s     = measured_speed * 50
  distance_cm  = measured_speed * 50 * (period_ms / 1000) / 60000 * pi * 7.64

Usage:
  python tools/wheel_diag.py LOGFILE [--period-ms 1000] [--json]
  python tools/wheel_diag.py --selftest

Exit codes: 0 = both verdicts PASS, 1 = FAIL / not enough data, 2 = usage error.

Python 3 standard library only.
"""

from __future__ import annotations

import argparse
import codecs
import contextlib
import io
import json
import math
import os
import re
import statistics
import sys
import tempfile
from dataclasses import dataclass

TOOL_NAME = "wheel_diag"
TOOL_VERSION = "1.0"

WHEELS = ("FL", "FR", "RL", "RR")
# C30D wiring, see README.md section 1
CHANNELS = {
    "FL": "MotorA/E1",
    "FR": "MotorB/E2",
    "RL": "MotorC/E3",  # left rear -- current T1 suspect
    "RR": "MotorD/E4",
}

ENCODER_CPR = 60000.0
WHEEL_DIAMETER_CM = 7.64
SAMPLE_RATE_HZ = 50.0  # measured speed is counts per 20 ms control period
MIN_DISTANCE_M = 0.05  # below this a "deg per metre" number is meaningless

PWM_SPREAD_LIMIT_PCT = 20.0
YAW_DRIFT_LIMIT_DEG_PER_M = 2.0
EPS = 1e-9

# PWM feed-forward model from USER/main.c: pwm ~= 2.5 * target + 250.
# A heading correction of C counts slows one side and speeds up the other, so
# it legitimately adds about 5*|C| PWM counts of left/right difference.
FF_PWM_PER_SPEED = 2.5

# ---------------------------------------------------------------------------
# log parsing
# ---------------------------------------------------------------------------

_LINE_RE = re.compile(
    r"(?P<mode>SPD\s+(?P<dir>FWD|BACK|STOP)|OL\s+PWM=(?P<olpwm>-?\d+))"
    r"\s+tgt=(?P<tgt>-?\d+)"
    r"\s+(?P<fl_m>-?\d+)/(?P<fl_pwm>-?\d+)"
    r"\s+(?P<fr_m>-?\d+)/(?P<fr_pwm>-?\d+)"
    r"\s+(?P<rl_m>-?\d+)/(?P<rl_pwm>-?\d+)"
    r"\s+(?P<rr_m>-?\d+)/(?P<rr_pwm>-?\d+)"
    r"\s+YAW=(?P<yaw>-?\d+)"
    r"\s+C=(?P<corr>-?\d+)"
    r"\s+SVO=(?P<servo>-?\d+)"
    r"\s+P=(?P<port>-?\d+)"
    r"\s+CMD=(?P<cmd>\S*)"
)


@dataclass
class Record:
    """One parsed log line."""

    lineno: int
    kind: str  # 'FWD' | 'BACK' | 'STOP' | 'OL'
    tgt: int
    speeds: tuple  # 4 absolute measured speeds, counts/20 ms (FL, FR, RL, RR)
    pwms: tuple  # 4 PWM values
    yaw_raw: int  # 0.1 deg units
    corr: int  # heading correction applied to the wheel targets
    servo: int
    port: int
    cmd: str
    open_pwm: int | None = None


@dataclass
class ParseResult:
    records: list
    encoding: str
    total: int  # non-empty lines
    other: int  # lines that are not log lines (banner, 70CM DONE, ...)
    malformed: int  # lines that look like log lines but do not parse


def decode_log(raw):
    """Decode raw log bytes; returns (text, encoding_name)."""
    if raw.startswith(codecs.BOM_UTF8):
        return raw.decode("utf-8-sig", "replace"), "utf-8-sig"
    if raw.startswith(codecs.BOM_UTF16_LE) or raw.startswith(codecs.BOM_UTF16_BE):
        try:
            return raw.decode("utf-16"), "utf-16"
        except UnicodeDecodeError:
            pass
    for enc in ("utf-8", "gbk"):
        try:
            return raw.decode(enc), enc
        except UnicodeDecodeError:
            continue
    return raw.decode("latin-1"), "latin-1"


def parse_line(line, lineno=0):
    """Parse one line; returns Record or None."""
    m = _LINE_RE.search(line)
    if m is None:
        return None
    direction = m.group("dir")
    open_pwm = None
    if direction is None:  # OL PWM=<n>
        direction = "OL"
        open_pwm = int(m.group("olpwm"))
    return Record(
        lineno=lineno,
        kind=direction,
        tgt=int(m.group("tgt")),
        speeds=(
            int(m.group("fl_m")),
            int(m.group("fr_m")),
            int(m.group("rl_m")),
            int(m.group("rr_m")),
        ),
        pwms=(
            int(m.group("fl_pwm")),
            int(m.group("fr_pwm")),
            int(m.group("rl_pwm")),
            int(m.group("rr_pwm")),
        ),
        yaw_raw=int(m.group("yaw")),
        corr=int(m.group("corr")),
        servo=int(m.group("servo")),
        port=int(m.group("port")),
        cmd=m.group("cmd"),
        open_pwm=open_pwm,
    )


def parse_log(path):
    """Read and parse a captured log file."""
    with open(path, "rb") as fh:
        raw = fh.read()
    text, encoding = decode_log(raw)
    records = []
    total = other = malformed = 0
    for lineno, line in enumerate(text.splitlines(), 1):
        if not line.strip():
            continue
        total += 1
        rec = parse_line(line, lineno)
        if rec is not None:
            records.append(rec)
        elif ("SPD " in line) or ("OL PWM=" in line) or ("tgt=" in line):
            malformed += 1
        else:
            other += 1
    return ParseResult(records, encoding, total, other, malformed)


# ---------------------------------------------------------------------------
# statistics helpers
# ---------------------------------------------------------------------------


def _mean(values):
    return sum(values) / len(values) if values else None


def _std(values):
    if len(values) < 2:
        return None
    mean = sum(values) / len(values)
    return math.sqrt(sum((v - mean) ** 2 for v in values) / len(values))


def _spread_pct(values):
    """(max - min) / median * 100, or None when not computable."""
    if len(values) < 2:
        return None
    median = statistics.median(values)
    if median <= 0:
        return None
    return (max(values) - min(values)) / median * 100.0


def _round(value, digits=4):
    if value is None:
        return None
    rounded = round(float(value), digits)
    return 0.0 if abs(rounded) < 10 ** (-digits) else rounded


def _slope(xs, ys):
    """Least-squares slope of ys over xs, or None."""
    n = len(xs)
    if n < 2:
        return None
    mx = sum(xs) / n
    my = sum(ys) / n
    den = sum((x - mx) ** 2 for x in xs)
    if den <= 0:
        return None
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / den


def line_distance_cm(speed_counts_per_20ms, period_ms):
    """Distance travelled during one log line, in cm."""
    counts = abs(speed_counts_per_20ms) * SAMPLE_RATE_HZ * (period_ms / 1000.0)
    return counts / ENCODER_CPR * math.pi * WHEEL_DIAMETER_CM


def pwm_verdict(spread_pct, limit=PWM_SPREAD_LIMIT_PCT):
    if spread_pct is None:
        return "N/A"
    return "PASS" if spread_pct <= limit + EPS else "FAIL"


def yaw_verdict(rate_deg_per_m, limit=YAW_DRIFT_LIMIT_DEG_PER_M):
    if rate_deg_per_m is None:
        return "N/A"
    return "PASS" if abs(rate_deg_per_m) <= limit + EPS else "FAIL"


# ---------------------------------------------------------------------------
# analysis
# ---------------------------------------------------------------------------


def analyze(parse_result, source="<log>", period_ms=1000.0, ref_wheel="mean",
            direction="any", include_open_loop=False,
            pwm_limit=PWM_SPREAD_LIMIT_PCT,
            yaw_limit=YAW_DRIFT_LIMIT_DEG_PER_M):
    """Turn parsed records into a result dict (text / JSON share it)."""
    period_ms = float(period_ms)
    if period_ms <= 0:
        raise ValueError("period_ms must be > 0")
    if ref_wheel not in ("mean",) + WHEELS:
        raise ValueError("ref_wheel must be one of: mean, " + ", ".join(WHEELS))
    if direction not in ("any", "fwd", "back"):
        raise ValueError("direction must be one of: any, fwd, back")

    warnings = []
    all_spd = [r for r in parse_result.records if r.kind in ("FWD", "BACK")]
    usable = all_spd
    if direction == "fwd":
        usable = [r for r in usable if r.kind == "FWD"]
    elif direction == "back":
        usable = [r for r in usable if r.kind == "BACK"]

    stop_count = sum(1 for r in parse_result.records if r.kind == "STOP")
    ol_records = [r for r in parse_result.records if r.kind == "OL"]

    modes = []
    if any(r.kind == "FWD" for r in usable):
        modes.append("FWD")
    if any(r.kind == "BACK" for r in usable):
        modes.append("BACK")
    mode_str = "+".join(modes) if modes else None
    tgt_values = sorted({r.tgt for r in usable})

    # --- distance estimate -------------------------------------------------
    ref_index = None if ref_wheel == "mean" else WHEELS.index(ref_wheel)
    line_speeds = []
    for r in usable:
        if ref_index is None:
            line_speeds.append(sum(r.speeds) / 4.0)
        else:
            line_speeds.append(float(r.speeds[ref_index]))
    per_line_cm = [line_distance_cm(v, period_ms) for v in line_speeds]

    xs = []  # cumulative distance in metres
    cumulative = 0.0
    for dist in per_line_cm:
        cumulative += dist
        xs.append(cumulative / 100.0)
    total_m = cumulative / 100.0
    duration_s = len(usable) * period_ms / 1000.0

    # --- yaw drift ---------------------------------------------------------
    ys = [r.yaw_raw / 10.0 for r in usable]  # deg
    yaw_delta = (ys[-1] - ys[0]) if len(ys) >= 2 else None
    net_rate = None
    if yaw_delta is not None and total_m > MIN_DISTANCE_M:
        net_rate = yaw_delta / total_m
    regression = _slope(xs, ys)
    peak_rate = None
    if len(xs) >= 2:
        segment_rates = []
        for i in range(1, len(xs)):
            dx = xs[i] - xs[i - 1]
            if dx > 1e-9:
                segment_rates.append(abs((ys[i] - ys[i - 1]) / dx))
        if segment_rates:
            peak_rate = max(segment_rates)

    # --- per-wheel means ---------------------------------------------------
    wheel_rows = []
    mean_speeds = []
    mean_pwms = []
    for idx, name in enumerate(WHEELS):
        speeds = [r.speeds[idx] for r in usable]
        pwms = [r.pwms[idx] for r in usable]
        mean_speed = _mean(speeds)
        mean_pwm = _mean(pwms)
        mean_speeds.append(mean_speed)
        mean_pwms.append(mean_pwm)
        sd = _std(speeds)
        cv = (sd / mean_speed * 100.0) if (sd is not None and mean_speed) else None
        wheel_rows.append({
            "name": name,
            "channel": CHANNELS[name],
            "mean_speed": _round(mean_speed, 2),
            "speed_cv_pct": _round(cv, 2),
            "speed_rel_median_pct": None,
            "mean_pwm": _round(mean_pwm, 2),
            "pwm_rel_median_pct": None,
            "pwm_per_speed": None,
        })

    pwm_median = statistics.median(mean_pwms) if usable else None
    speed_median = statistics.median(mean_speeds) if usable else None
    spread = _spread_pct(mean_pwms) if usable else None

    for row, mean_speed, mean_pwm in zip(wheel_rows, mean_speeds, mean_pwms):
        if pwm_median:
            row["pwm_rel_median_pct"] = _round(
                (mean_pwm - pwm_median) / pwm_median * 100.0, 2)
        if speed_median:
            row["speed_rel_median_pct"] = _round(
                (mean_speed - speed_median) / speed_median * 100.0, 2)
        if mean_speed:
            row["pwm_per_speed"] = _round(mean_pwm / mean_speed, 3)

    suspect_wheel = None
    max_wheel = min_wheel = None
    if usable:
        max_wheel = WHEELS[max(range(4), key=lambda i: mean_pwms[i])]
        min_wheel = WHEELS[min(range(4), key=lambda i: mean_pwms[i])]
        # only name a suspect wheel when the imbalance is worth a hardware check
        if spread is not None and (spread > 5.0 or spread > pwm_limit):
            suspect_wheel = max_wheel

    pair_means = {
        "left": _round(_mean([mean_pwms[0], mean_pwms[2]]), 2) if usable else None,
        "right": _round(_mean([mean_pwms[1], mean_pwms[3]]), 2) if usable else None,
        "front": _round(_mean([mean_pwms[0], mean_pwms[1]]), 2) if usable else None,
        "rear": _round(_mean([mean_pwms[2], mean_pwms[3]]), 2) if usable else None,
    }

    corr_max = max((abs(r.corr) for r in usable), default=None)
    corr_mean_abs = _mean([abs(r.corr) for r in usable])

    # --- verdicts ----------------------------------------------------------
    pwm_v = pwm_verdict(spread, pwm_limit)
    yaw_v = yaw_verdict(net_rate, yaw_limit)
    overall = "PASS" if (pwm_v == "PASS" and yaw_v == "PASS") else "FAIL"

    # --- warnings ----------------------------------------------------------
    if not usable:
        warnings.append("no usable 'SPD FWD/BACK' lines -- nothing to analyse")
    else:
        if len(usable) < 3:
            warnings.append(
                "only %d usable line(s); capture a longer run for a reliable verdict"
                % len(usable))
        if total_m < 1.8:
            warnings.append(
                "estimated travel %.2f m is shorter than the 2 m acceptance run"
                % total_m)
        if len(modes) > 1:
            warnings.append(
                "capture mixes FWD and BACK lines; distance and yaw drift are only "
                "meaningful for a single direction (use --direction)")
        if len(tgt_values) > 1:
            warnings.append(
                "target speed changed during the capture: "
                + ", ".join(str(t) for t in tgt_values))
        zero_lines = sum(1 for r in usable if all(s == 0 for s in r.speeds))
        if zero_lines:
            warnings.append(
                "%d line(s) report zero speed on all wheels (car not moving); they "
                "dilute the distance estimate" % zero_lines)
        for row in wheel_rows:
            rel = row["speed_rel_median_pct"]
            if rel is not None and abs(rel) >= 20.0:
                warnings.append(
                    "%s mean speed is %+.0f%% off the median -- encoder under-count / "
                    "dropout, wrong CPR, or wheel slip" % (row["name"], rel))
        if pwm_v == "FAIL":
            blamed = suspect_wheel or max_wheel
            warnings.append(
                "PWM spread %.1f%% exceeds %.1f%% -- suspect %s (%s)"
                % (spread, pwm_limit, blamed, CHANNELS[blamed]))
        if yaw_v == "FAIL":
            warnings.append(
                "yaw drift %.2f deg/m exceeds %.2f deg/m -- check the fast/slow wheel "
                "pair and the IMU heading loop" % (net_rate, yaw_limit))
        if corr_max is not None and abs(corr_max) >= 50:
            warnings.append(
                "heading correction C reached %d counts (~%.0f PWM of deliberate "
                "left/right difference): part of the PWM spread is yaw correction, "
                "not a wheel fault" % (corr_max, 2 * FF_PWM_PER_SPEED * abs(corr_max)))
    if parse_result.malformed:
        warnings.append(
            "%d line(s) looked like log lines but could not be parsed (truncated or "
            "corrupted capture?)" % parse_result.malformed)

    # --- optional open-loop diagnostics ------------------------------------
    open_loop_info = None
    if include_open_loop and ol_records:
        groups = {}
        for rec in ol_records:
            groups.setdefault(rec.open_pwm, []).append(rec)
        group_list = []
        for pwm_level in sorted(groups, key=lambda v: (v is None, v)):
            recs = groups[pwm_level]
            means = [
                sum(r.speeds[i] for r in recs) / len(recs) for i in range(4)
            ]
            group_list.append({
                "pwm": pwm_level,
                "lines": len(recs),
                "mean_speed": {w: _round(m, 1) for w, m in zip(WHEELS, means)},
                "speed_spread_pct": _round(_spread_pct(means), 2),
                "slowest": WHEELS[min(range(4), key=lambda i: means[i])],
            })
        open_loop_info = {"groups": group_list}
    elif include_open_loop:
        open_loop_info = {"groups": []}

    return {
        "tool": TOOL_NAME,
        "version": TOOL_VERSION,
        "file": source,
        "encoding": parse_result.encoding,
        "params": {
            "period_ms": period_ms,
            "ref_wheel": ref_wheel,
            "direction": direction,
            "include_open_loop": bool(include_open_loop),
            "pwm_limit_pct": pwm_limit,
            "yaw_limit_deg_per_m": yaw_limit,
        },
        "lines": {
            "total": parse_result.total,
            "usable": len(usable),
            "stop": stop_count,
            "open_loop": len(ol_records),
            "other": parse_result.other,
            "malformed": parse_result.malformed,
            "excluded_by_direction": len(all_spd) - len(usable),
        },
        "run": {
            "mode": mode_str,
            "tgt_values": tgt_values,
            "duration_s": _round(duration_s, 2),
            "corr_max": corr_max,
            "corr_mean_abs": _round(corr_mean_abs, 2),
            "zero_speed_lines": sum(
                1 for r in usable if all(s == 0 for s in r.speeds)),
        },
        "distance": {
            "total_m": _round(total_m, 4),
            "ref_wheel": ref_wheel,
            "per_line_cm": [_round(d, 2) for d in per_line_cm],
        },
        "wheels": wheel_rows,
        "pair_means": pair_means,
        "pwm": {
            "median": _round(pwm_median, 2),
            "max": _round(max(mean_pwms), 2) if usable else None,
            "min": _round(min(mean_pwms), 2) if usable else None,
            "max_wheel": max_wheel,
            "min_wheel": min_wheel,
            "spread_pct": _round(spread, 2),
            "limit_pct": pwm_limit,
            "suspect_wheel": suspect_wheel,
            "verdict": pwm_v,
        },
        "yaw": {
            "first_deg": _round(ys[0], 1) if ys else None,
            "last_deg": _round(ys[-1], 1) if ys else None,
            "delta_deg": _round(yaw_delta, 2),
            "net_drift_deg_per_m": _round(net_rate, 3),
            "regression_slope_deg_per_m": _round(regression, 3),
            "peak_segment_deg_per_m": _round(peak_rate, 3),
            "limit_deg_per_m": yaw_limit,
            "verdict": yaw_v,
        },
        "open_loop": open_loop_info,
        "verdicts": {
            "pwm": pwm_v,
            "yaw": yaw_v,
            "overall": overall,
            "exit_code": 0 if overall == "PASS" else 1,
        },
        "warnings": warnings,
    }


def analyze_log(path, period_ms=1000.0, ref_wheel="mean", direction="any",
                include_open_loop=False, pwm_limit=PWM_SPREAD_LIMIT_PCT,
                yaw_limit=YAW_DRIFT_LIMIT_DEG_PER_M):
    """Convenience wrapper: parse path then analyze."""
    return analyze(
        parse_log(path),
        source=path,
        period_ms=period_ms,
        ref_wheel=ref_wheel,
        direction=direction,
        include_open_loop=include_open_loop,
        pwm_limit=pwm_limit,
        yaw_limit=yaw_limit,
    )


# ---------------------------------------------------------------------------
# text report
# ---------------------------------------------------------------------------


def _fmt(value, digits=1, signed=False):
    if value is None:
        return "-"
    return ("%+.*f" if signed else "%.*f") % (digits, value)


def format_text(result):
    """Human-readable report; plain ASCII so it prints on any console."""
    out = []
    add = out.append
    ln = result["lines"]
    params = result["params"]
    run = result["run"]
    dist = result["distance"]
    pwm = result["pwm"]
    yaw = result["yaw"]
    verdicts = result["verdicts"]

    add("=" * 72)
    add(" %s v%s - C30D 4-wheel balance / yaw drift (Sprint-1 T1)"
        % (TOOL_NAME, TOOL_VERSION))
    add(" file   : %s  [%s]" % (result["file"], result["encoding"]))
    add(" lines  : %d total | %d usable | %d STOP | %d OL | %d other | %d malformed"
        % (ln["total"], ln["usable"], ln["stop"], ln["open_loop"], ln["other"],
           ln["malformed"]))
    if run["mode"]:
        tgts = ", ".join(str(t) for t in run["tgt_values"])
        add(" run    : %s  tgt=%s  %d x %.0f ms = %.1f s"
            % (run["mode"], tgts, ln["usable"], params["period_ms"],
               run["duration_s"]))
    else:
        add(" run    : no usable 'SPD FWD/BACK' lines")
    add(" travel : %.2f m estimated (ref=%s; counts/s = v*50, 60000 CPR, 76.4 mm)"
        % (dist["total_m"], dist["ref_wheel"]))
    add("-" * 72)
    add(" %-6s%-12s%9s%7s%10s%9s%9s"
        % ("wheel", "channel", "mean_spd", "cv%", "mean_pwm", "vs_med", "pwm/spd"))
    for row in result["wheels"]:
        add(" %-6s%-12s%9s%7s%10s%9s%9s"
            % (row["name"], row["channel"],
               _fmt(row["mean_speed"]), _fmt(row["speed_cv_pct"]),
               _fmt(row["mean_pwm"]), _fmt(row["pwm_rel_median_pct"], signed=True),
               _fmt(row["pwm_per_speed"], 2)))
    pm = result["pair_means"]
    if pm.get("left") is not None:
        add(" pairs  : left %s / right %s ; front %s / rear %s   (mean PWM)"
            % (_fmt(pm["left"]), _fmt(pm["right"]),
               _fmt(pm["front"]), _fmt(pm["rear"])))
    if run["corr_max"] is not None:
        add(" yaw C  : max |C| = %d counts, mean |C| = %s (heading correction; "
            "~%.0f PWM of intentional left/right difference at the max)"
            % (run["corr_max"], _fmt(run["corr_mean_abs"], 1),
               2 * FF_PWM_PER_SPEED * abs(run["corr_max"])))
    add("-" * 72)

    if pwm["spread_pct"] is None:
        add(" PWM spread (max-min)/median : %8s  [%s]  limit <= %.1f%%"
            % ("N/A", pwm["verdict"], pwm["limit_pct"]))
    else:
        add(" PWM spread (max-min)/median : %7.1f%%  [%s]  limit <= %.1f%%"
            % (pwm["spread_pct"], pwm["verdict"], pwm["limit_pct"]))
        suspect = ""
        if pwm["suspect_wheel"]:
            suspect = " ; suspect %s (%s)" % (
                pwm["suspect_wheel"], CHANNELS[pwm["suspect_wheel"]])
        add("   max %s %.1f / min %s %.1f / median %.1f%s"
            % (pwm["max_wheel"], pwm["max"], pwm["min_wheel"], pwm["min"],
               pwm["median"], suspect))

    if yaw["net_drift_deg_per_m"] is None:
        add(" Yaw drift  (net deg/m)      : %8s  [%s]  limit |.| <= %.2f"
            % ("N/A", yaw["verdict"], yaw["limit_deg_per_m"]))
    else:
        add(" Yaw drift  (net deg/m)      : %7.2f   [%s]  limit |.| <= %.2f"
            % (yaw["net_drift_deg_per_m"], yaw["verdict"],
               yaw["limit_deg_per_m"]))
        add("   delta yaw %+.2f deg over %.3f m ; regression slope %s deg/m ; "
            "peak segment %s deg/m"
            % (yaw["delta_deg"], dist["total_m"],
               _fmt(yaw["regression_slope_deg_per_m"], 2),
               _fmt(yaw["peak_segment_deg_per_m"], 2)))

    olinfo = result.get("open_loop")
    if olinfo and olinfo["groups"]:
        add("-" * 72)
        add(" open-loop diagnostics (informational, NOT part of PASS/FAIL)")
        for group in olinfo["groups"]:
            speeds = group["mean_speed"]
            add("   PWM=%s (%d line(s)): FL %s FR %s RL %s RR %s  spread %s%%  "
                "slowest %s"
                % (_fmt(group["pwm"], 0), group["lines"],
                   _fmt(speeds["FL"], 0), _fmt(speeds["FR"], 0),
                   _fmt(speeds["RL"], 0), _fmt(speeds["RR"], 0),
                   _fmt(group["speed_spread_pct"], 1), group["slowest"]))
        add("   equal PWM but large speed spread -> drag / motor / encoder scaling")

    add("-" * 72)
    add(" VERDICT: %s   (PWM %s, yaw %s)"
        % (verdicts["overall"], verdicts["pwm"], verdicts["yaw"]))
    if result["warnings"]:
        add(" warnings:")
        for warning in result["warnings"]:
            add("   ! %s" % warning)
    else:
        add(" warnings: none")
    add("=" * 72)
    return "\n".join(out)


# ---------------------------------------------------------------------------
# self test
# ---------------------------------------------------------------------------

BROKEN_SAMPLE = """\
SPD FWD tgt=900 1007/3278 926/3784 428/4335 998/3418 YAW=-2 C=0 SVO=0 P=5 CMD=G
SPD FWD tgt=900 914/2207 938/2478 835/5238 927/2314 YAW=-12 C=-2 SVO=0 P=5 CMD=G
SPD FWD tgt=900 929/1864 955/2034 947/5499 885/1959 YAW=-18 C=-3 SVO=0 P=5 CMD=G
"""


def _spd_line(direction, speeds, pwms, yaw_raw, tgt=900, corr=0, cmd="G"):
    pairs = " ".join("%d/%d" % (speed, pwm) for speed, pwm in zip(speeds, pwms))
    return ("SPD %s tgt=%d %s YAW=%d C=%d SVO=0 P=5 CMD=%s"
            % (direction, tgt, pairs, yaw_raw, corr, cmd))


def _ol_line(open_pwm, speeds, yaw_raw=0):
    pairs = " ".join("%d/%d" % (speed, open_pwm) for speed in speeds)
    return ("OL PWM=%d tgt=900 %s YAW=%d C=0 SVO=0 P=5 CMD=O%d"
            % (open_pwm, pairs, yaw_raw, open_pwm))


def _healthy_log(lines=11, yaw_step=1):
    """A balanced ~2 m straight run (all wheels healthy)."""
    rows = []
    for i in range(lines):
        jitter = i % 3 - 1
        speeds = (905 + jitter, 903 + jitter, 908 + jitter, 904 + jitter)
        pwms = (2495, 2505, 2510, 2500)
        rows.append(_spd_line("FWD", speeds, pwms, yaw_step * i))
    return "\n".join(rows) + "\n"


def _boundary_log(pwm_rr):
    """Two lines whose PWM means give (pwm_rr-2000)/2000*100 % spread."""
    speeds = (900, 900, 900, 900)
    pwms = (2000, 2000, 2000, pwm_rr)
    rows = [
        _spd_line("FWD", speeds, pwms, 0),
        _spd_line("FWD", speeds, pwms, 1),
    ]
    return "\n".join(rows) + "\n"


def _yaw_log(yaw_step_raw, lines=10, speed=1000):
    """10 lines at v=1000 counts/20ms -> ~2 m, yaw advancing by yaw_step_raw."""
    rows = [
        _spd_line("FWD", (speed,) * 4, (2500,) * 4, yaw_step_raw * i)
        for i in range(lines)
    ]
    return "\n".join(rows) + "\n"


def _approx(value, expected, tol):
    return value is not None and abs(value - expected) <= tol


class _Checker:
    def __init__(self):
        self.passed = 0
        self.failed = 0
        self.lines = []

    def check(self, name, condition, detail=""):
        if condition:
            self.passed += 1
            self.lines.append("[PASS] " + name)
        else:
            self.failed += 1
            self.lines.append("[FAIL] " + name +
                              (("  (%s)" % detail) if detail else ""))


def run_selftest(stream=None):
    """Synthetic-log tests for the parser and the verdict logic."""
    out = stream if stream is not None else sys.stdout
    checker = _Checker()

    with tempfile.TemporaryDirectory(prefix="wheel_diag_selftest_") as tmp:
        def write(name, text, encoding="utf-8"):
            path = os.path.join(tmp, name)
            with open(path, "w", encoding=encoding, newline="") as fh:
                fh.write(text)
            return path

        # 1. the 2026-11-18 RL capture -------------------------------------
        broken = write("broken.log", BROKEN_SAMPLE)
        rb = analyze_log(broken)
        checker.check(
            "broken sample: 3 usable FWD lines",
            rb["lines"]["usable"] == 3,
            "usable=%r" % rb["lines"]["usable"])
        checker.check(
            "broken sample: RL (MotorC/E3) is the PWM suspect",
            rb["pwm"]["suspect_wheel"] == "RL",
            "suspect=%r" % rb["pwm"]["suspect_wheel"])
        checker.check(
            "broken sample: PWM spread ~96.6% -> FAIL",
            _approx(rb["pwm"]["spread_pct"], 96.62, 0.05)
            and rb["pwm"]["verdict"] == "FAIL",
            "spread=%r verdict=%r"
            % (rb["pwm"]["spread_pct"], rb["pwm"]["verdict"]))
        checker.check(
            "broken sample: yaw drift ~-2.994 deg/m -> FAIL",
            _approx(rb["yaw"]["net_drift_deg_per_m"], -2.994, 0.01)
            and rb["yaw"]["verdict"] == "FAIL",
            "rate=%r verdict=%r"
            % (rb["yaw"]["net_drift_deg_per_m"], rb["yaw"]["verdict"]))
        checker.check(
            "broken sample: overall FAIL, exit code 1",
            rb["verdicts"]["overall"] == "FAIL"
            and rb["verdicts"]["exit_code"] == 1,
            "%r" % (rb["verdicts"],))
        checker.check(
            "broken sample: regression -4.36 and peak segment 5.53 deg/m",
            _approx(rb["yaw"]["regression_slope_deg_per_m"], -4.362, 0.01)
            and _approx(rb["yaw"]["peak_segment_deg_per_m"], 5.534, 0.01),
            "slope=%r peak=%r"
            % (rb["yaw"]["regression_slope_deg_per_m"],
               rb["yaw"]["peak_segment_deg_per_m"]))
        checker.check(
            "broken sample: RL speed outlier warning",
            any(("RL" in w) and ("mean speed" in w) for w in rb["warnings"]),
            "warnings=%r" % (rb["warnings"],))
        checker.check(
            "broken sample: per-wheel pwm/spd shows RL ~2x",
            rb["wheels"][2]["pwm_per_speed"] > 2.0 * rb["wheels"][0]["pwm_per_speed"],
            "FL=%r RL=%r" % (rb["wheels"][0]["pwm_per_speed"],
                             rb["wheels"][2]["pwm_per_speed"]))

        # 2. healthy 2 m run -------------------------------------------------
        healthy_text = _healthy_log()
        healthy = write("healthy.log", healthy_text)
        rh = analyze_log(healthy)
        checker.check(
            "healthy 2 m log: PASS/PASS, exit 0",
            rh["verdicts"]["pwm"] == "PASS" and rh["verdicts"]["yaw"] == "PASS"
            and rh["verdicts"]["exit_code"] == 0,
            "%r" % (rh["verdicts"],))
        checker.check(
            "healthy 2 m log: 11 lines, ~1.99 m",
            rh["lines"]["usable"] == 11
            and _approx(rh["distance"]["total_m"], 1.99, 0.03),
            "lines=%r dist=%r" % (rh["lines"]["usable"],
                                  rh["distance"]["total_m"]))
        checker.check(
            "healthy 2 m log: PWM spread ~0.60%",
            _approx(rh["pwm"]["spread_pct"], 0.5994, 0.01),
            "spread=%r" % (rh["pwm"]["spread_pct"],))
        checker.check(
            "healthy 2 m log: no speed-outlier warning",
            not any("mean speed" in w for w in rh["warnings"]),
            "warnings=%r" % (rh["warnings"],))
        checker.check(
            "healthy 2 m log: no <2 m short-run warning",
            not any("shorter than the 2 m" in w for w in rh["warnings"]),
            "warnings=%r" % (rh["warnings"],))
        checker.check(
            "healthy 2 m log: no wheel is blamed (suspect is None)",
            rh["pwm"]["suspect_wheel"] is None,
            "suspect=%r" % (rh["pwm"]["suspect_wheel"],))
        r_strict = analyze_log(healthy, pwm_limit=0.05, yaw_limit=0.01)
        checker.check(
            "custom strict limits: FAIL verdicts and suspect wheel, no crash",
            r_strict["pwm"]["verdict"] == "FAIL"
            and r_strict["pwm"]["suspect_wheel"] == "RL"
            and r_strict["yaw"]["verdict"] == "FAIL",
            "%r suspect=%r" % (r_strict["verdicts"],
                              r_strict["pwm"]["suspect_wheel"]))

        # 3. STOP lines must be ignored -------------------------------------
        healthy_lines = healthy_text.splitlines()
        stop_line = _spd_line("STOP", (0, 0, 0, 0), (0, 0, 0, 0), 10, cmd="S")
        mixed_lines = (healthy_lines[:3] + [stop_line] + healthy_lines[3:7]
                       + [stop_line] + healthy_lines[7:])
        with_stop = write("stop.log", "\n".join(mixed_lines) + "\n")
        rs = analyze_log(with_stop)
        checker.check(
            "STOP lines excluded from all statistics",
            rs["lines"]["stop"] == 2 and rs["lines"]["usable"] == 11
            and _approx(rs["pwm"]["spread_pct"], rh["pwm"]["spread_pct"], 1e-9)
            and _approx(rs["yaw"]["net_drift_deg_per_m"],
                        rh["yaw"]["net_drift_deg_per_m"], 1e-9)
            and rs["verdicts"]["overall"] == "PASS",
            "stop=%r usable=%r pwm=%r yaw=%r"
            % (rs["lines"]["stop"], rs["lines"]["usable"],
               rs["pwm"]["spread_pct"], rs["yaw"]["net_drift_deg_per_m"]))

        # 4. open-loop lines: ignored by default, reported on request --------
        ol_line = _ol_line(2500, (1000, 1010, 400, 990))
        with_ol = write("ol.log", healthy_text + ol_line + "\n")
        ro = analyze_log(with_ol)
        checker.check(
            "OL lines ignored by default (still PASS)",
            ro["lines"]["open_loop"] == 1 and ro["lines"]["usable"] == 11
            and ro["verdicts"]["overall"] == "PASS"
            and ro["open_loop"] is None,
            "ol=%r usable=%r overall=%r"
            % (ro["lines"]["open_loop"], ro["lines"]["usable"],
               ro["verdicts"]["overall"]))
        roi = analyze_log(with_ol, include_open_loop=True)
        groups = (roi["open_loop"] or {}).get("groups") or []
        checker.check(
            "OL diagnostics: equal-PWM speed spread ~61.3%, slowest RL",
            len(groups) == 1
            and _approx(groups[0]["speed_spread_pct"], 61.31, 0.05)
            and groups[0]["slowest"] == "RL",
            "groups=%r" % (groups,))
        checker.check(
            "OL diagnostics do not change the closed-loop verdicts",
            roi["verdicts"]["overall"] == "PASS",
            "%r" % (roi["verdicts"],))

        # 5. threshold boundaries -------------------------------------------
        r20 = analyze_log(write("pwm20.log", _boundary_log(2400)))
        r204 = analyze_log(write("pwm204.log", _boundary_log(2408)))
        checker.check(
            "PWM spread exactly 20.0% -> PASS",
            _approx(r20["pwm"]["spread_pct"], 20.0, 1e-6)
            and r20["pwm"]["verdict"] == "PASS",
            "spread=%r verdict=%r"
            % (r20["pwm"]["spread_pct"], r20["pwm"]["verdict"]))
        checker.check(
            "PWM spread 20.4% -> FAIL",
            _approx(r204["pwm"]["spread_pct"], 20.4, 0.001)
            and r204["pwm"]["verdict"] == "FAIL",
            "spread=%r verdict=%r"
            % (r204["pwm"]["spread_pct"], r204["pwm"]["verdict"]))
        checker.check(
            "pwm_verdict helper boundary",
            pwm_verdict(20.0) == "PASS" and pwm_verdict(20.0000000001) == "PASS"
            and pwm_verdict(20.4) == "FAIL" and pwm_verdict(None) == "N/A",
            "%r %r" % (pwm_verdict(20.0), pwm_verdict(20.4)))
        checker.check(
            "yaw_verdict helper boundary (+/-2.0 PASS, 2.05 FAIL)",
            yaw_verdict(2.0) == "PASS" and yaw_verdict(-2.0) == "PASS"
            and yaw_verdict(2.05) == "FAIL" and yaw_verdict(None) == "N/A",
            "%r %r %r" % (yaw_verdict(2.0), yaw_verdict(-2.0),
                          yaw_verdict(2.05)))

        # 6. yaw drift from logs --------------------------------------------
        ry_pass = analyze_log(write("y180.log", _yaw_log(4)))
        ry_fail = analyze_log(write("y225.log", _yaw_log(5)))
        checker.check(
            "yaw 1.80 deg/m -> PASS",
            _approx(ry_pass["yaw"]["net_drift_deg_per_m"], 1.800, 0.005)
            and ry_pass["yaw"]["verdict"] == "PASS",
            "rate=%r verdict=%r"
            % (ry_pass["yaw"]["net_drift_deg_per_m"],
               ry_pass["yaw"]["verdict"]))
        checker.check(
            "yaw 2.25 deg/m -> FAIL",
            _approx(ry_fail["yaw"]["net_drift_deg_per_m"], 2.250, 0.005)
            and ry_fail["yaw"]["verdict"] == "FAIL",
            "rate=%r verdict=%r"
            % (ry_fail["yaw"]["net_drift_deg_per_m"],
               ry_fail["yaw"]["verdict"]))

        # 7. --period-ms and --ref-wheel ------------------------------------
        rp = analyze_log(broken, period_ms=2000)
        checker.check(
            "--period-ms 2000 doubles distance, halves drift rate",
            _approx(rp["distance"]["total_m"], 1.0689, 0.002)
            and _approx(rp["yaw"]["net_drift_deg_per_m"], -1.497, 0.005)
            and rp["yaw"]["verdict"] == "PASS",
            "dist=%r rate=%r"
            % (rp["distance"]["total_m"], rp["yaw"]["net_drift_deg_per_m"]))
        rr = analyze_log(broken, ref_wheel="RL")
        checker.check(
            "--ref-wheel RL uses RL speed for distance",
            rr["distance"]["ref_wheel"] == "RL"
            and _approx(rr["distance"]["total_m"], 0.4420, 0.002)
            and _approx(rr["yaw"]["net_drift_deg_per_m"], -3.620, 0.01),
            "dist=%r rate=%r"
            % (rr["distance"]["total_m"], rr["yaw"]["net_drift_deg_per_m"]))

        # 8. encodings and tolerance ----------------------------------------
        bom = write("bom.log", healthy_text, encoding="utf-8-sig")
        r_bom = analyze_log(bom)
        checker.check(
            "UTF-8 BOM log decodes",
            r_bom["encoding"] == "utf-8-sig" and r_bom["lines"]["usable"] == 11
            and r_bom["verdicts"]["overall"] == "PASS",
            "enc=%r usable=%r" % (r_bom["encoding"], r_bom["lines"]["usable"]))
        gbk = write("gbk.log", "# 采集说明：COM18 串口日志\r\n" + healthy_text,
                    encoding="gbk")
        r_gbk = analyze_log(gbk)
        checker.check(
            "GBK log with Chinese comment decodes",
            r_gbk["encoding"] == "gbk" and r_gbk["lines"]["usable"] == 11
            and r_gbk["verdicts"]["overall"] == "PASS",
            "enc=%r usable=%r warnings=%r"
            % (r_gbk["encoding"], r_gbk["lines"]["usable"], r_gbk["warnings"]))
        messy = write(
            "messy.log",
            healthy_text
            + "SPD FWD tgt=900 905/2495 903/2505\n"
            + "[14:13:01] "
            + _spd_line("FWD", (905, 903, 908, 904), (2495, 2505, 2510, 2500), 20)
            + "\n")
        r_messy = analyze_log(messy)
        checker.check(
            "truncated line counted malformed; timestamp prefix tolerated",
            r_messy["lines"]["malformed"] == 1
            and r_messy["lines"]["usable"] == 12,
            "malformed=%r usable=%r"
            % (r_messy["lines"]["malformed"], r_messy["lines"]["usable"]))

        # 9. no data / rendering --------------------------------------------
        empty = write(
            "empty.log",
            "wireless boot ok\n"
            + _spd_line("STOP", (0, 0, 0, 0), (0, 0, 0, 0), 0, cmd="S") + "\n")
        r_empty = analyze_log(empty)
        checker.check(
            "no SPD lines -> N/A verdicts, exit 1",
            r_empty["verdicts"]["pwm"] == "N/A"
            and r_empty["verdicts"]["yaw"] == "N/A"
            and r_empty["verdicts"]["exit_code"] == 1,
            "%r" % (r_empty["verdicts"],))
        try:
            empty_text = format_text(r_empty)
            text_ok = "VERDICT: FAIL" in empty_text and "N/A" in empty_text
        except Exception as exc:  # noqa: BLE001 - selftest should report anything
            text_ok = False
            empty_text = repr(exc)
        checker.check("empty result still renders a report", text_ok,
                      empty_text[:120])
        text = format_text(rh)
        checker.check(
            "healthy report renders PASS and PWM spread",
            ("VERDICT: PASS" in text) and ("PWM spread" in text)
            and ("Yaw drift" in text),
            text[:80])
        try:
            dumped = json.dumps(rb, indent=2)
            json_ok = isinstance(json.loads(dumped), dict)
        except (TypeError, ValueError):
            json_ok = False
        checker.check("JSON output serialises", json_ok)

        # 10. CLI plumbing and exit codes -----------------------------------
        sink = io.StringIO()
        with contextlib.redirect_stdout(sink), contextlib.redirect_stderr(sink):
            code_pass = run([healthy])
            code_fail = run([broken, "--json"])
            code_usage = run([])
        checker.check(
            "CLI exit codes: PASS 0 / FAIL 1 / no argument 2",
            code_pass == 0 and code_fail == 1 and code_usage == 2,
            "pass=%r fail=%r usage=%r" % (code_pass, code_fail, code_usage))

    for line in checker.lines:
        print(line, file=out)
    total = checker.passed + checker.failed
    print("", file=out)
    print("selftest: %d/%d checks passed%s"
          % (checker.passed, total,
             "" if checker.failed == 0
             else ", %d FAILED" % checker.failed), file=out)
    return 0 if checker.failed == 0 else 1


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def build_arg_parser():
    parser = argparse.ArgumentParser(
        prog="wheel_diag.py",
        description="C30D 4-wheel PWM balance / yaw drift diagnostic (Sprint-1 T1).",
        epilog="Example: python tools/wheel_diag.py run2m.log --period-ms 1000 --json",
    )
    parser.add_argument("logfile", nargs="?",
                        help="captured serial log (UTF-8 / GBK / ASCII)")
    parser.add_argument("--period-ms", type=float, default=1000.0, metavar="MS",
                        help="log line interval in ms (default: 1000)")
    parser.add_argument("--json", action="store_true",
                        help="machine-readable JSON output")
    parser.add_argument("--selftest", action="store_true",
                        help="run built-in parser/verdict tests")
    parser.add_argument("--ref-wheel", choices=("mean",) + WHEELS, default="mean",
                        help="speed source for the distance estimate (default: mean)")
    parser.add_argument("--direction", choices=("any", "fwd", "back"), default="any",
                        help="which SPD lines to analyse (default: any)")
    parser.add_argument("--include-open-loop", action="store_true",
                        help="also report informational O<n> open-loop comparison")
    parser.add_argument("--pwm-limit", type=float, default=PWM_SPREAD_LIMIT_PCT,
                        metavar="PCT", help="PWM spread limit in %% (default: 20)")
    parser.add_argument("--yaw-limit", type=float,
                        default=YAW_DRIFT_LIMIT_DEG_PER_M, metavar="DEG_M",
                        help="yaw drift limit in deg/m (default: 2.0)")
    parser.add_argument("--version", action="version",
                        version="%s %s" % (TOOL_NAME, TOOL_VERSION))
    return parser


def run(argv=None):
    args = build_arg_parser().parse_args(argv)
    if args.selftest:
        return run_selftest()
    if not args.logfile:
        print("wheel_diag: error: LOGFILE required (or use --selftest)",
              file=sys.stderr)
        return 2
    if args.period_ms <= 0:
        print("wheel_diag: error: --period-ms must be > 0", file=sys.stderr)
        return 2
    try:
        result = analyze_log(
            args.logfile,
            period_ms=args.period_ms,
            ref_wheel=args.ref_wheel,
            direction=args.direction,
            include_open_loop=args.include_open_loop,
            pwm_limit=args.pwm_limit,
            yaw_limit=args.yaw_limit,
        )
    except OSError as exc:
        print("wheel_diag: error: cannot read %s: %s" % (args.logfile, exc),
              file=sys.stderr)
        return 2

    if args.json:
        print(json.dumps(result, indent=2))
    else:
        print(format_text(result))
    return result["verdicts"]["exit_code"]


if __name__ == "__main__":
    sys.exit(run())
