/**
 * app.js — C30D 串口调参台（Web Serial + Canvas 趋势图 + 命令/参数/日志）
 *
 * 依赖：parser.js（ES module），无任何第三方库、无 CDN。
 * 浏览器要求：桌面版 Chrome / Edge 89+，通过 http://localhost 或 https 打开（Web Serial 安全上下文）。
 */

import { parseLine, WHEEL_NAMES, WHEEL_LABELS } from './parser.js';

/* ======================================================================= */
/* 常量                                                                     */
/* ======================================================================= */

const PARAMS_KEY = 'c30d.serialTuner.params.v1';
const HISTORY_KEY = 'c30d.serialTuner.history.v1';
const SERIAL_OPTIONS = {
  baudRate: 115200,
  dataBits: 8,
  stopBits: 1,
  parity: 'none',
  flowControl: 'none',
};
const CHART_MAX_POINTS = 240;
const LOG_DOM_LIMIT = 500;
const LOG_KEEP_LIMIT = 3000;
const ARM_MS = 3000;
const DEFAULT_PARAMS = { kp: 0, ki: 0, kd: 0, kmax: 600, v: 150 };

/*
 * 会让小车动作的命令首字母。A/E/F/B/G/D/P/T 是任务要求 + HANDOVER.md 第 11 节
 * 明确列出的“移动车”命令；M/O/N 也实际会驱动电机/舵机，故一并纳入安全门。
 * 这些命令默认不发送：需勾选“确认会动车”+ 3 秒内二次确认。
 */
const DANGER_KEYS = new Set(['A', 'E', 'F', 'B', 'G', 'D', 'P', 'T', 'M', 'O', 'N']);

/* ======================================================================= */
/* DOM 工具                                                                 */
/* ======================================================================= */

const el = (id) => document.getElementById(id);
const setText = (id, text) => {
  const node = el(id);
  if (node) node.textContent = text;
};

function pad2(n) {
  return String(n).padStart(2, '0');
}
function pad3(n) {
  return String(n).padStart(3, '0');
}
function fmtTime(ts, withMs = false) {
  const d = new Date(ts);
  const base = `${pad2(d.getHours())}:${pad2(d.getMinutes())}:${pad2(d.getSeconds())}`;
  return withMs ? `${base}.${pad3(d.getMilliseconds())}` : base;
}
function stamp() {
  const d = new Date();
  return (
    `${d.getFullYear()}${pad2(d.getMonth() + 1)}${pad2(d.getDate())}_` +
    `${pad2(d.getHours())}${pad2(d.getMinutes())}${pad2(d.getSeconds())}`
  );
}
function fmtInt(v) {
  return v === null || v === undefined || !Number.isFinite(Number(v)) ? '--' : String(v);
}

/* ======================================================================= */
/* 全局状态                                                                 */
/* ======================================================================= */

const state = {
  source: 'idle', // 'idle' | 'demo' | 'serial'

  // 串口
  port: null,
  writer: null,
  reader: null,
  connected: false,
  reading: false,
  closing: false,
  lineBuf: '',

  // 演示
  demo: false,
  demoTimer: null,
  demoLines: [],
  demoIndex: 0,

  // 数据
  latest: null,
  prevYaw: null,
  lastLineTime: null,
  stats: { lines: 0, errors: 0, records: 0 },
  entries: [],

  // 图表暂停（只暂停画曲线，解析/卡片照常）
  chartPaused: false,

  // 危险命令二次确认
  armed: null,
  armTimer: null,

  // 命令历史
  history: [],
  historyCursor: -1,
};

/* ======================================================================= */
/* 提示 Toast                                                               */
/* ======================================================================= */

let toastTimer = null;
function toast(message, kind = 'info') {
  const node = el('toast');
  if (!node) return;
  node.textContent = message;
  node.className = `toast show ${kind}`;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => {
    node.className = 'toast';
  }, 3400);
}

/* ======================================================================= */
/* 连接状态 / 数据源                                                         */
/* ======================================================================= */

function serialSupported() {
  return typeof navigator !== 'undefined' && 'serial' in navigator;
}

function setConnStatus(text, kind = 'idle', info) {
  const dot = el('statusDot');
  if (dot) dot.className = `dot ${kind}`;
  setText('statusText', text);
  if (info !== undefined) setText('connInfo', info);
}

function setSource(label) {
  state.source = label === '串口' ? 'serial' : label === '演示模式' ? 'demo' : 'idle';
  setText('statSource', label);
}

function updateButtons() {
  const supported = serialSupported();
  const connectBtn = el('btnConnect');
  if (connectBtn) {
    connectBtn.disabled = !supported || state.connected;
    connectBtn.title = supported ? '选择串口设备（COM18）' : '当前浏览器不支持 Web Serial';
  }
  const disconnectBtn = el('btnDisconnect');
  if (disconnectBtn) disconnectBtn.disabled = !state.connected;

  const demoBtn = el('btnDemo');
  if (demoBtn) {
    demoBtn.textContent = state.demo ? '停止演示' : '演示模式';
    demoBtn.classList.toggle('active', state.demo);
  }
}

/* ======================================================================= */
/* 趋势图（Canvas，无外部库）                                                */
/* ======================================================================= */

function niceScale(lo, hi, count = 4) {
  const span = hi - lo;
  const rawStep = span / count;
  const mag = Math.pow(10, Math.floor(Math.log10(rawStep || 1)));
  const norm = rawStep / mag;
  let step;
  if (norm <= 1) step = 1;
  else if (norm <= 2) step = 2;
  else if (norm <= 2.5) step = 2.5;
  else if (norm <= 5) step = 5;
  else step = 10;
  step *= mag;
  const nlo = Math.floor(lo / step) * step;
  const nhi = Math.ceil(hi / step) * step;
  const ticks = [];
  for (let v = nlo; v <= nhi + step * 1e-9; v += step) {
    ticks.push(Number(v.toFixed(10)));
  }
  return { lo: nlo, hi: nhi, step, ticks };
}

function formatTick(value, step) {
  if (Math.abs(value) < 1e-9) return '0';
  if (step >= 1) return String(Math.round(value));
  const decimals = step >= 0.1 ? 1 : 2;
  return value.toFixed(decimals);
}

class TrendChart {
  constructor(canvas, series, opts = {}) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.series = series;
    this.opts = opts;
    this.maxPoints = opts.maxPoints || CHART_MAX_POINTS;
    this.points = [];
    this._onResize = () => this.render();
    if (typeof ResizeObserver !== 'undefined') {
      this.ro = new ResizeObserver(this._onResize);
      this.ro.observe(canvas);
    } else {
      window.addEventListener('resize', this._onResize);
    }
  }

  add(t, values) {
    this.points.push({ t, v: values });
    const extra = this.points.length - this.maxPoints;
    if (extra > 0) this.points.splice(0, extra);
    this.render();
  }

  clear() {
    this.points.length = 0;
    this.render();
  }

  render() {
    const canvas = this.canvas;
    const ctx = this.ctx;
    const w = canvas.clientWidth;
    const h = canvas.clientHeight;
    if (!w || !h) return;

    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const bw = Math.round(w * dpr);
    const bh = Math.round(h * dpr);
    if (canvas.width !== bw || canvas.height !== bh) {
      canvas.width = bw;
      canvas.height = bh;
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);

    const padL = 46;
    const padR = 10;
    const padT = 10;
    const padB = 20;
    const plotW = Math.max(10, w - padL - padR);
    const plotH = Math.max(10, h - padT - padB);

    // 数据范围
    let lo = Infinity;
    let hi = -Infinity;
    for (const p of this.points) {
      for (const s of this.series) {
        const v = p.v[s.key];
        if (typeof v === 'number' && Number.isFinite(v)) {
          if (v < lo) lo = v;
          if (v > hi) hi = v;
        }
      }
    }
    if (!Number.isFinite(lo) || !Number.isFinite(hi)) {
      lo = 0;
      hi = 1;
    }
    if (this.opts.includeZero !== false) {
      lo = Math.min(lo, 0);
      hi = Math.max(hi, 0);
    }
    if (hi - lo < 1e-9) {
      const pad = Math.max(Math.abs(hi) * 0.1, 1);
      lo -= pad;
      hi += pad;
    }
    const scale = niceScale(lo, hi, 4);
    lo = scale.lo;
    hi = scale.hi;

    const now = Date.now();
    const t0 = this.points.length ? this.points[0].t : now - this.maxPoints * 1000;
    const t1 = this.points.length ? this.points[this.points.length - 1].t : now;
    const span = Math.max(t1 - t0, 1000);
    const X = (t) => padL + ((t - t0) / span) * plotW;
    const Y = (v) => padT + plotH - ((v - lo) / (hi - lo)) * plotH;

    // 网格 + Y 轴刻度
    ctx.font = '11px ui-monospace, Consolas, monospace';
    ctx.textBaseline = 'middle';
    for (const tv of scale.ticks) {
      const y = Y(tv);
      ctx.strokeStyle = tv === 0 ? '#31465c' : '#1d2a38';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(padL, y);
      ctx.lineTo(w - padR, y);
      ctx.stroke();
      ctx.fillStyle = '#7d93a8';
      ctx.textAlign = 'right';
      ctx.fillText(formatTick(tv, scale.step), padL - 6, y);
    }

    // 时间轴标签
    ctx.fillStyle = '#7d93a8';
    ctx.textBaseline = 'alphabetic';
    ctx.textAlign = 'left';
    ctx.fillText(fmtTime(t0), padL, h - 5);
    ctx.textAlign = 'center';
    ctx.fillText(fmtTime((t0 + t1) / 2), padL + plotW / 2, h - 5);
    ctx.textAlign = 'right';
    ctx.fillText(this.points.length ? fmtTime(t1) : '--', w - padR, h - 5);

    if (!this.points.length) return;

    // 曲线
    ctx.save();
    ctx.beginPath();
    ctx.rect(padL, padT, plotW, plotH);
    ctx.clip();
    for (const s of this.series) {
      ctx.strokeStyle = s.color;
      ctx.lineWidth = 1.6;
      ctx.lineJoin = 'round';
      ctx.beginPath();
      let pen = false;
      for (const p of this.points) {
        const v = p.v[s.key];
        if (typeof v === 'number' && Number.isFinite(v)) {
          const x = X(p.t);
          const y = Y(v);
          if (pen) ctx.lineTo(x, y);
          else {
            ctx.moveTo(x, y);
            pen = true;
          }
        } else {
          pen = false;
        }
      }
      ctx.stroke();
    }
    ctx.restore();
  }
}

const charts = {};

function initCharts() {
  charts.yaw = new TrendChart(
    el('chartYaw'),
    [{ key: 'yaw', label: 'YAW', color: '#4cc9f0' }],
    { includeZero: true },
  );
  charts.speed = new TrendChart(
    el('chartSpeed'),
    [
      { key: 'fl', label: 'FL', color: '#4cc9f0' },
      { key: 'fr', label: 'FR', color: '#f5b942' },
      { key: 'rl', label: 'RL', color: '#35d07f' },
      { key: 'rr', label: 'RR', color: '#b085ff' },
    ],
    { includeZero: true },
  );
  charts.laser = new TrendChart(
    el('chartLaser'),
    [
      { key: 'dL', label: 'dL', color: '#4cc9f0' },
      { key: 'dR', label: 'dR', color: '#f5b942' },
      { key: 'dF', label: 'dF', color: '#35d07f' },
    ],
    { includeZero: false },
  );
}

function addChartSample(t, p) {
  charts.yaw.add(t, { yaw: p.yaw });
  charts.speed.add(t, {
    fl: p.wheels[0] ? p.wheels[0].spd : null,
    fr: p.wheels[1] ? p.wheels[1].spd : null,
    rl: p.wheels[2] ? p.wheels[2].spd : null,
    rr: p.wheels[3] ? p.wheels[3].spd : null,
  });
  charts.laser.add(t, {
    dL: p.ls ? p.ls.dL : null,
    dR: p.ls ? p.ls.dR : null,
    dF: p.ls ? p.ls.dF : null,
  });
}

/* ======================================================================= */
/* 四轮卡片 / 扩展字段（动态构建）                                            */
/* ======================================================================= */

function buildWheelCards() {
  const grid = el('wheelGrid');
  if (!grid) return;
  grid.innerHTML = '';
  WHEEL_NAMES.forEach((name, i) => {
    const card = document.createElement('article');
    card.className = 'card wheel';
    card.innerHTML =
      `<div class="card-head">` +
      `<span class="card-title">${name} <small>${WHEEL_LABELS[i]}</small></span>` +
      `<span class="wheel-dir" id="wheelDir${i}">—</span>` +
      `</div>` +
      `<div class="row"><span>速度</span><b id="wheelSpd${i}">--</b><span class="unit">cnt/20ms</span></div>` +
      `<div class="bar"><i id="wheelSpdBar${i}"></i></div>` +
      `<div class="row"><span>PWM</span><b id="wheelPwm${i}">--</b></div>` +
      `<div class="bar pwm"><i id="wheelPwmBar${i}"></i></div>`;
    grid.appendChild(card);
  });
}

const EXTRAS = [
  { field: 'dd', label: 'DD 里程', fmt: (p) => (p.dd == null ? '--' : `${p.dd} cm`) },
  { field: 'rd', label: 'RD 垄侧原始', fmt: (p) => fmtInt(p.rd) },
  { field: 'df', label: 'DF 偏移', fmt: (p) => fmtInt(p.df) },
  { field: 'af', label: 'AF', fmt: (p) => fmtInt(p.af) },
  { field: 'le', label: 'LE 横向偏', fmt: (p) => fmtInt(p.le) },
  { field: 'ld', label: 'LD 横向角', fmt: (p) => fmtInt(p.ld) },
  { field: 'lc', label: 'LC 上次计数', fmt: (p) => fmtInt(p.lc) },
  { field: 'ce', label: 'CE 对中 e', fmt: (p) => fmtInt(p.ce) },
  { field: 'cd', label: 'CD 对中 Δ', fmt: (p) => fmtInt(p.cd) },
  { field: 'ct', label: 'CT 前束', fmt: (p) => fmtInt(p.ct) },
  { field: 'sy', label: 'SY 调试角', fmt: (p) => (p.sy == null ? '--' : `${p.sy}°`) },
  { field: 'c', label: 'C yaw_corr', fmt: (p) => fmtInt(p.c) },
  { field: 'svo', label: 'SVO 舵机', fmt: (p) => (p.svo == null ? '--' : `${p.svo}°`) },
  { field: 'p', label: 'P 端口', fmt: (p) => fmtInt(p.p) },
  { field: 'k', label: 'K 按键', fmt: (p) => fmtInt(p.k) },
  { field: 'r1', label: 'R1 历史', fmt: (p) => fmtHist(p.r1) },
  { field: 'r2', label: 'R2 历史', fmt: (p) => fmtHist(p.r2) },
  { field: 'r3', label: 'R3 历史', fmt: (p) => fmtHist(p.r3) },
  { field: 'st', label: 'ST 旧状态', fmt: (p) => (p.st == null ? '--' : `${p.st}（旧）`) },
  { field: 'lzn', label: 'LZN 激光帧数', fmt: (p) => fmtInt(p.lzn) },
  { field: 'lb', label: 'LB 原始帧', fmt: (p) => fmtLb(p.lb) },
];

function fmtHist(hist) {
  if (!hist || hist.count === null || hist.count === undefined) return '--';
  return `${hist.count}, ${fmtInt(hist.yaw)}`;
}

function fmtLb(bytes) {
  if (!bytes || !bytes.length) return '--';
  return bytes.map((b) => b.toString(16).toUpperCase().padStart(2, '0')).join(' ');
}

function buildExtras() {
  const grid = el('extrasGrid');
  if (!grid) return;
  grid.innerHTML = '';
  for (const item of EXTRAS) {
    const div = document.createElement('div');
    div.className = 'ext';
    const k = document.createElement('span');
    k.className = 'ext-k';
    k.textContent = item.label;
    const v = document.createElement('b');
    v.className = 'ext-v';
    v.id = `ext_${item.field}`;
    v.textContent = '--';
    div.append(k, v);
    grid.appendChild(div);
  }
}

function updateExtras(p) {
  for (const item of EXTRAS) setText(`ext_${item.field}`, item.fmt(p));
}

/* ======================================================================= */
/* 卡片刷新                                                                 */
/* ======================================================================= */

function setBar(id, value, fullScale, extraClass) {
  const bar = el(id);
  if (!bar) return;
  const n = typeof value === 'number' && Number.isFinite(value) ? value : 0;
  const ratio = Math.min(1, Math.abs(n) / fullScale);
  bar.style.width = `${(ratio * 100).toFixed(1)}%`;
  bar.className = n < 0 ? 'neg' : '';
  if (extraClass) bar.classList.add(extraClass);
}

function updateCards(p) {
  // ---- YAW ----
  const yaw = p.yaw;
  setText('yawValue', yaw == null ? '--' : yaw.toFixed(1));
  const yawCard = el('yawCard');
  yawCard.classList.toggle('pos', yaw != null && yaw > 0.05);
  yawCard.classList.toggle('neg', yaw != null && yaw < -0.05);

  const trendEl = el('yawTrend');
  let trendText = '—';
  let trendCls = '';
  if (yaw != null && state.prevYaw != null) {
    const d = yaw - state.prevYaw;
    if (Math.abs(d) < 0.05) {
      trendText = '→ 0.0°';
    } else if (d > 0) {
      trendText = `↑ +${d.toFixed(1)}°`;
      trendCls = 'up';
    } else {
      trendText = `↓ ${d.toFixed(1)}°`;
      trendCls = 'down';
    }
  }
  trendEl.textContent = trendText;
  trendEl.className = `trend ${trendCls}`;
  setText('yawSub', `SY=${fmtInt(p.sy)}° · C=${fmtInt(p.c)}`);

  // ---- F2 ----
  setText('f2Value', p.f2Name || '--');
  setText('f2Label', p.f2Label || '');
  setText('f2Code', p.f2 == null ? 'F2 = --' : `F2 = ${p.f2}`);
  el('f2Card').classList.toggle('is-fault', p.f2 === 14);

  // ---- FA ----
  setText('faValue', p.faName || '--');
  setText('faCode', p.fa == null ? 'FA = --' : `FA = ${p.fa}`);
  el('faCard').classList.toggle('is-fault', p.fa != null && p.fa !== 0);

  // ---- 电池 ----
  setText('battValue', p.vb == null ? '--' : p.vb.toFixed(1));
  setText('battSub', p.vbRaw == null ? 'VB = --' : `VB = ${p.vbRaw}（0.1 V）`);
  const battCard = el('battCard');
  battCard.classList.toggle('warn', p.vb != null && p.vb < 11.5 && p.vb >= 10.5);
  battCard.classList.toggle('is-fault', p.vb != null && p.vb < 10.5);

  // ---- 模式 / CMD ----
  setText('modeValue', p.dir || '--');
  setText('modeCmd', p.cmd == null ? 'CMD = —' : `CMD = ${p.cmd === '' ? '(空)' : p.cmd}`);
  setText('modeSub', `MR=${fmtInt(p.mr)} · CM=${fmtInt(p.cm)} · P=${fmtInt(p.p)}`);

  // ---- 目标速度 ----
  setText('tgtValue', fmtInt(p.tgt));
  setText('tgtSub', `T0=${fmtInt(p.t0)} · counts/20ms`);

  // ---- 四轮 ----
  for (let i = 0; i < 4; i += 1) {
    const w = p.wheels[i] || { spd: null, pwm: null };
    setText(`wheelSpd${i}`, fmtInt(w.spd));
    setText(`wheelPwm${i}`, fmtInt(w.pwm));
    setBar(`wheelSpdBar${i}`, w.spd, 3000);
    setBar(`wheelPwmBar${i}`, w.pwm, 12000);
    const dirEl = el(`wheelDir${i}`);
    if (dirEl) {
      const cls = w.spd == null ? '' : w.spd > 0 ? 'up' : w.spd < 0 ? 'down' : '';
      dirEl.textContent = w.spd == null ? '—' : w.spd > 0 ? '↑' : w.spd < 0 ? '↓' : '·';
      dirEl.className = `wheel-dir ${cls}`;
    }
  }

  // ---- 激光 ----
  setText('lsDl', fmtInt(p.ls ? p.ls.dL : null));
  setText('lsDr', fmtInt(p.ls ? p.ls.dR : null));
  setText('lsDf', fmtInt(p.ls ? p.ls.dF : null));
  setText('lsDb', fmtInt(p.dB));

  const lzChip = el('lzChip');
  lzChip.textContent = p.lz || '--';
  lzChip.className = `chip ${p.lz === 'OK' ? 'ok' : p.lz === 'LOST' ? 'bad' : ''}`;

  const validBits = [['validF', p.ls && p.ls.valid ? p.ls.valid.f : null],
    ['validL', p.ls && p.ls.valid ? p.ls.valid.l : null],
    ['validR', p.ls && p.ls.valid ? p.ls.valid.r : null]];
  for (const [id, ok] of validBits) {
    const node = el(id);
    if (!node) continue;
    node.className = `chip small ${ok === true ? 'ok' : ok === false ? 'bad' : 'off'}`;
  }

  setText(
    'lsFlags',
    p.ls && p.ls.flags != null
      ? `LS flags = ${p.ls.flags}（0x${p.ls.flags.toString(16).toUpperCase()}）`
      : 'LS flags = --',
  );

  let rgText = 'RG = --';
  if (p.ridge) {
    const dBText = p.ridge.dist == null ? '--' : `${p.ridge.dist} mm`;
    const validText = p.ridge.valid === true ? '有效' : p.ridge.valid === false ? '无效' : '--';
    rgText = `RG = ${dBText} · flags=${fmtInt(p.ridge.flags)} · ${validText}`;
  }
  setText('rgInfo', rgText);
  el('laserCard').classList.toggle('is-fault', p.lz === 'LOST');

  // ---- 扩展字段 ----
  updateExtras(p);

  // 记住上一帧 YAW 用于趋势
  state.prevYaw = yaw;
}

function resetCards() {
  const ids = [
    'yawValue', 'yawSub', 'f2Value', 'f2Label', 'f2Code',
    'faValue', 'faCode', 'battValue', 'battSub',
    'modeValue', 'modeSub', 'tgtValue', 'tgtSub',
    'lsDl', 'lsDr', 'lsDf', 'lsDb',
  ];
  for (const id of ids) setText(id, '--');
  setText('yawTrend', '—');
  setText('f2Label', '');
  setText('modeCmd', 'CMD = —');
  setText('battSub', 'VB = --');
  setText('tgtSub', 'T0=-- · counts/20ms');
  setText('lsFlags', 'LS flags = --');
  setText('rgInfo', 'RG = --');

  const lz = el('lzChip');
  if (lz) {
    lz.textContent = '--';
    lz.className = 'chip';
  }
  for (const id of ['validF', 'validL', 'validR']) {
    const node = el(id);
    if (node) node.className = 'chip small off';
  }
  for (const id of ['yawCard', 'f2Card', 'faCard', 'battCard', 'laserCard']) {
    el(id).classList.remove('is-fault', 'pos', 'neg', 'warn');
  }
  for (let i = 0; i < 4; i += 1) {
    setText(`wheelSpd${i}`, '--');
    setText(`wheelPwm${i}`, '--');
    const dirEl = el(`wheelDir${i}`);
    if (dirEl) {
      dirEl.textContent = '—';
      dirEl.className = 'wheel-dir';
    }
    for (const barId of [`wheelSpdBar${i}`, `wheelPwmBar${i}`]) {
      const bar = el(barId);
      if (bar) {
        bar.style.width = '0%';
        bar.className = '';
      }
    }
  }
  for (const item of EXTRAS) setText(`ext_${item.field}`, '--');
  state.prevYaw = null;
  state.latest = null;
}

/* ======================================================================= */
/* 统计 / 日志                                                              */
/* ======================================================================= */

function updateStatEls() {
  setText('statLines', String(state.stats.lines));
  setText('statErrors', String(state.stats.errors));
  const errEl = el('statErrors');
  if (errEl) errEl.classList.toggle('has-error', state.stats.errors > 0);
  setText('statLast', state.lastLineTime ? fmtTime(state.lastLineTime, true) : '—');
  setText('logCount', `${state.entries.length} 行`);
}

function entriesCap() {
  const extra = state.entries.length - LOG_KEEP_LIMIT;
  if (extra > 0) state.entries.splice(0, extra);
}

function appendLogRow(entry) {
  const list = el('logList');
  if (!list) return;
  const row = document.createElement('div');
  row.className = 'log-row';
  if (entry.type === 'sys') row.classList.add('is-sys');
  else if (entry.type === 'text') row.classList.add('is-text');
  else row.classList.add('is-data');
  if (!entry.ok) row.classList.add('is-error');

  const ts = document.createElement('span');
  ts.className = 'log-ts';
  ts.textContent = fmtTime(entry.t, true);
  const tx = document.createElement('span');
  tx.className = 'log-tx';
  tx.textContent = entry.text;
  row.append(ts, tx);
  list.appendChild(row);
  while (list.childElementCount > LOG_DOM_LIMIT) list.firstElementChild.remove();

  const pauseScroll = el('logPauseScroll');
  if (!pauseScroll || !pauseScroll.checked) list.scrollTop = list.scrollHeight;
}

function pushEntry(entry) {
  state.entries.push(entry);
  entriesCap();
  appendLogRow(entry);
}

function appendSystem(text) {
  pushEntry({ t: Date.now(), text, type: 'sys', ok: true });
  updateStatEls();
}

/* ======================================================================= */
/* 帧处理                                                                   */
/* ======================================================================= */

function handleLine(raw, when = Date.now(), opts = {}) {
  const text = String(raw === null || raw === undefined ? '' : raw)
    .replace(/\r/g, '')
    .split('\n')[0]
    .trim();
  if (!text) return;

  const parsed = parseLine(text);
  state.stats.records += 1;
  state.lastLineTime = when;

  if (!parsed.ok) state.stats.errors += 1;
  if (parsed.type === 'log' || parsed.type === 'open-loop') state.stats.lines += 1;

  pushEntry({ t: when, text, type: parsed.type, ok: parsed.ok });

  if (parsed.type === 'log' || parsed.type === 'open-loop') {
    state.latest = parsed;
    updateCards(parsed);
    if (!state.chartPaused) addChartSample(when, parsed);
  } else if (parsed.textKind === 'err' && !opts.quiet) {
    toast(`固件返回：${text}`, 'warn');
  }

  updateStatEls();
}

function onChunk(chunk) {
  state.lineBuf += chunk;
  let idx = state.lineBuf.indexOf('\n');
  while (idx >= 0) {
    const line = state.lineBuf.slice(0, idx);
    state.lineBuf = state.lineBuf.slice(idx + 1);
    handleLine(line);
    idx = state.lineBuf.indexOf('\n');
  }
  // 防御：长时间收不到换行时强制冲刷，避免缓冲无限增长
  if (state.lineBuf.length > 8192) {
    const pending = state.lineBuf;
    state.lineBuf = '';
    handleLine(pending);
  }
}

/* ======================================================================= */
/* Web Serial                                                              */
/* ======================================================================= */

function hex4(v) {
  return Number(v).toString(16).toUpperCase().padStart(4, '0');
}

async function connectSerial() {
  if (!serialSupported()) {
    toast('当前浏览器不支持 Web Serial，请使用桌面版 Chrome / Edge', 'warn');
    const banner = el('supportBanner');
    if (banner) banner.hidden = false;
    return;
  }
  if (state.connected) return;

  setConnStatus('选择串口中…', 'busy');
  let port = null;
  try {
    port = await navigator.serial.requestPort();
    await port.open(SERIAL_OPTIONS);

    stopDemo();
    state.port = port;
    state.connected = true;
    state.closing = false;
    state.reading = true;
    state.writer = port.writable.getWriter();
    state.reader = port.readable.getReader();

    resetView();
    setSource('串口');
    const info = typeof port.getInfo === 'function' ? port.getInfo() : {};
    const usb =
      info && info.usbVendorId !== undefined
        ? `USB ${hex4(info.usbVendorId)}:${hex4(info.usbProductId)}`
        : '串口';
    setConnStatus('串口已连接', 'ok', `${usb} · 115200 8N1 · 无流控`);
    appendSystem(`串口已连接（115200 8N1，无流控）：${usb}`);
    updateButtons();
    toast('串口已连接，等待 1 Hz 日志…', 'ok');
    readLoop();
  } catch (err) {
    const name = err && err.name;
    const msg = err && err.message ? err.message : String(err);
    // 如果端口已经打开但后续步骤失败，尽量释放，避免刷新前无法重连
    try {
      if (state.writer) state.writer.releaseLock();
    } catch (_e) {
      /* 忽略 */
    }
    try {
      if (state.reader) state.reader.releaseLock();
    } catch (_e) {
      /* 忽略 */
    }
    try {
      if (port) await port.close();
    } catch (_e) {
      /* 端口可能未成功打开 */
    }
    state.connected = false;
    state.port = null;
    state.writer = null;
    state.reader = null;
    state.reading = false;
    updateButtons();
    if (state.demo) {
      setConnStatus('演示模式', 'demo');
    } else {
      setConnStatus('未连接', 'idle');
    }
    if (name === 'NotFoundError') toast('未选择串口，或串口被其他程序占用', 'warn');
    else toast(`打开串口失败：${msg}`, 'error');
  }
}

async function closePort() {
  if (state.closing) return;
  state.closing = true;
  state.reading = false;
  try {
    if (state.writer) state.writer.releaseLock();
  } catch (_e) {
    /* 忽略 */
  }
  state.writer = null;
  try {
    if (state.port) await state.port.close();
  } catch (_e) {
    /* 端口可能已被拔出 */
  }
  state.port = null;
  state.connected = false;
  state.closing = false;
  updateButtons();
  setConnStatus(state.demo ? '演示模式' : '未连接', state.demo ? 'demo' : 'idle');
  if (!state.demo) {
    setSource('—');
    setText('connInfo', '115200 8N1 · 无流控');
  }
}

async function readLoop() {
  const decoder = new TextDecoder();
  try {
    while (state.reading) {
      const { value, done } = await state.reader.read();
      if (done) break;
      if (value && value.length) onChunk(decoder.decode(value, { stream: true }));
    }
  } catch (err) {
    if (state.reading) {
      const msg = err && err.message ? err.message : String(err);
      toast(`串口读取异常：${msg}`, 'error');
      appendSystem(`串口读取异常：${msg}`);
    }
  } finally {
    try {
      if (state.reader) state.reader.releaseLock();
    } catch (_e) {
      /* 忽略 */
    }
    state.reader = null;
    await closePort();
    if (!state.demo) appendSystem('串口已断开');
  }
}

async function disconnectSerial() {
  if (!state.port) return;
  state.reading = false;
  try {
    if (state.reader) await state.reader.cancel();
  } catch (_e) {
    /* 忽略 */
  }
  if (!state.reader) await closePort();
  toast('已断开串口', 'ok');
}

/* ======================================================================= */
/* 演示模式                                                                 */
/* ======================================================================= */

async function loadDemoLines() {
  try {
    const res = await fetch('sample_log.txt', { cache: 'no-store' });
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    const text = await res.text();
    const lines = text
      .split(/\r?\n/)
      .map((l) => l.trim())
      .filter(Boolean);
    if (!lines.length) throw new Error('示例文件为空');
    return lines;
  } catch (err) {
    console.warn('读取 sample_log.txt 失败，改用内置演示数据：', err);
    return syntheticDemoLines();
  }
}

/** sample_log.txt 缺失时的兜底演示数据（保证页面永远有数据可看）。 */
function syntheticDemoLines() {
  const lines = [];
  let dist = 0;
  for (let i = 0; i < 160; i += 1) {
    const moving = i > 12 && i < 148;
    const dir = moving ? 'FWD' : 'STOP';
    const f2 = i < 12 ? 0 : Math.min(13, 1 + Math.floor((i - 12) / 11));
    const yaw = Math.round(8 * Math.sin(i / 3.1));
    const spd = moving ? 2400 : 0;
    const pwm = moving ? 5600 : 0;
    const dL = Math.max(100, 470 - ((i * 7) % 360));
    const dR = Math.max(100, 460 - ((i * 9) % 350));
    const dF = Math.max(206, 400 - ((i * 5) % 200));
    const dB = 137 + ((i * 3) % 40);
    if (moving) dist += 24;
    const wheels = [0, 1, 2, 3].map(
      (w) => `${spd + ((i + w) % 7) - 3}/${pwm + ((i * 3 + w) % 200) - 100}`,
    );
    lines.push(
      `SPD ${dir} tgt=150 MR=${moving ? 1 : 0} CM=0 T0=150 ${wheels.join(' ')} ` +
        `F2=${f2} FA=0 K=5 VB=118 SY=0 R1=${100 + i},0 R2=${200 + i},0 R3=${300 + i},0 ` +
        `LE=0 LD=0 LC=175620 DD=${dist} RD=148 DF=7 AF=2 CE=12 CD=3 CT=1 ` +
        (i < 120 ? 'ST=2 ' : '') +
        `LZ=OK LS=${dL},${dR},${dF},7 RG=${dB},1,1 LZN=${i + 1} LB=AA 55 01 02 03 04 05 06 ` +
        `YAW=${yaw} C=0 SVO=0 P=1 CMD=S`,
    );
  }
  return lines;
}

function clearDemoTimer() {
  if (state.demoTimer) {
    clearInterval(state.demoTimer);
    state.demoTimer = null;
  }
}

function stopDemo() {
  const was = state.demo;
  clearDemoTimer();
  state.demo = false;
  updateButtons();
  if (was) {
    setConnStatus('未连接', 'idle');
    setSource('—');
    appendSystem('演示已停止');
  }
}

async function startDemo() {
  if (state.demo) return;
  if (state.connected) await disconnectSerial();

  clearDemoTimer();
  resetView();
  setConnStatus('演示模式', 'demo', '115200 8N1 · 演示数据');
  setSource('演示模式');
  appendSystem('演示模式：回放 sample_log.txt（1 Hz，循环）。点击「连接串口」会停止演示。');
  state.demo = true;
  updateButtons();

  let lines = [];
  try {
    lines = await loadDemoLines();
  } catch (_e) {
    lines = syntheticDemoLines();
  }
  if (!state.demo) return; // 加载期间被用户停止

  state.demoLines = lines;
  state.demoIndex = 0;
  const step = 1000;
  const start = Date.now() - lines.length * step;
  lines.forEach((line, i) => handleLine(line, start + i * step, { quiet: true }));

  state.demoTimer = setInterval(() => {
    if (!state.demo || !state.demoLines.length) return;
    const line = state.demoLines[state.demoIndex % state.demoLines.length];
    state.demoIndex += 1;
    handleLine(line, Date.now(), { quiet: true });
    if (state.demoIndex % state.demoLines.length === 0) {
      appendSystem('演示数据已回放一轮，继续循环');
    }
  }, step);
}

/* ======================================================================= */
/* 视图重置                                                                 */
/* ======================================================================= */

function resetView() {
  state.latest = null;
  state.lastLineTime = null;
  state.prevYaw = null;
  state.stats = { lines: 0, errors: 0, records: 0 };
  state.entries = [];
  state.lineBuf = '';
  const list = el('logList');
  if (list) list.innerHTML = '';
  resetCards();
  for (const chart of Object.values(charts)) chart.clear();
  updateStatEls();
}

/* ======================================================================= */
/* 危险命令二次确认                                                          */
/* ======================================================================= */

function dangerKey(cmd) {
  const text = String(cmd || '').trim().toUpperCase();
  return text ? text[0] : '';
}

function isDangerous(cmd) {
  return DANGER_KEYS.has(dangerKey(cmd));
}

function clearArm() {
  state.armed = null;
  clearTimeout(state.armTimer);
  state.armTimer = null;
  const hint = el('sendHint');
  if (hint) {
    hint.hidden = true;
    hint.textContent = '';
  }
  const btn = el('btnSend');
  if (btn) {
    btn.classList.remove('armed');
    btn.textContent = '发送';
  }
}

function armCommand(cmd) {
  state.armed = { cmd, until: Date.now() + ARM_MS };
  const hint = el('sendHint');
  if (hint) {
    hint.hidden = false;
    hint.textContent =
      `⚠ 危险命令 ${cmd} 已就绪：${ARM_MS / 1000} 秒内再次点击“发送”或按 Enter 确认。` +
      '该命令会让小车动作，请确认场地安全。';
  }
  const btn = el('btnSend');
  if (btn) {
    btn.classList.add('armed');
    btn.textContent = '再次确认发送';
  }
  clearTimeout(state.armTimer);
  state.armTimer = setTimeout(clearArm, ARM_MS);
  toast(`危险命令 ${cmd}：${ARM_MS / 1000} 秒内再次发送确认`, 'warn');
}

function flashGate() {
  const gate = el('gateLabel');
  if (!gate) return;
  gate.classList.remove('flash');
  // 触发重排以重启动画
  void gate.offsetWidth;
  gate.classList.add('flash');
  setTimeout(() => gate.classList.remove('flash'), 900);
}

/* ======================================================================= */
/* 发送                                                                     */
/* ======================================================================= */

async function doSend(cmd) {
  setText('lastSent', `${cmd} · ${fmtTime(Date.now())}`);

  if (state.source === 'demo' || !state.connected || !state.writer) {
    if (state.demo) {
      appendSystem(`→ ${cmd}（演示模式，未真正发送）`);
      toast('演示模式不会真正发送命令；连接串口后可发送', 'warn');
    } else {
      toast('串口未连接，无法发送命令', 'warn');
    }
    return false;
  }

  try {
    const bytes = new TextEncoder().encode(`${cmd}\r\n`);
    await state.writer.write(bytes);
    appendSystem(`→ ${cmd}`);
    pushHistory(cmd);
    return true;
  } catch (err) {
    const msg = err && err.message ? err.message : String(err);
    toast(`发送失败：${msg}`, 'error');
    appendSystem(`发送失败：${msg}`);
    return false;
  }
}

/**
 * @returns {Promise<boolean>} true = 已真正写入串口（或演示模式允许的模拟发送返回 false）
 */
async function requestSend(raw) {
  const cmd = String(raw === null || raw === undefined ? '' : raw).trim();
  if (!cmd) {
    toast('请输入命令', 'warn');
    return false;
  }

  // 二次确认窗口内再次发送 -> 真正发送
  if (state.armed && state.armed.cmd === cmd && Date.now() <= state.armed.until) {
    clearArm();
    return doSend(cmd);
  }

  if (isDangerous(cmd)) {
    const gate = el('dangerGate');
    if (!gate || !gate.checked) {
      toast(`已拦截危险命令 ${cmd}：请先勾选“确认会动车”`, 'warn');
      flashGate();
      return false;
    }
    armCommand(cmd);
    return false;
  }

  return doSend(cmd);
}

function pushHistory(cmd) {
  if (!cmd) return;
  state.history = [cmd, ...state.history.filter((c) => c !== cmd)].slice(0, 50);
  try {
    localStorage.setItem(HISTORY_KEY, JSON.stringify(state.history));
  } catch (_e) {
    /* localStorage 不可用时忽略 */
  }
  renderHistory();
}

function loadHistory() {
  try {
    const raw = localStorage.getItem(HISTORY_KEY);
    if (!raw) return [];
    const arr = JSON.parse(raw);
    return Array.isArray(arr) ? arr.filter((x) => typeof x === 'string').slice(0, 50) : [];
  } catch (_e) {
    return [];
  }
}

function renderHistory() {
  const box = el('cmdHistory');
  if (!box) return;
  box.innerHTML = '';
  if (!state.history.length) {
    const hint = document.createElement('span');
    hint.className = 'hint';
    hint.textContent = '暂无发送记录';
    box.appendChild(hint);
    return;
  }
  for (const cmd of state.history.slice(0, 12)) {
    const btn = document.createElement('button');
    btn.type = 'button';
    btn.className = 'history-item';
    btn.textContent = cmd;
    btn.title = '点击填回输入框（不会自动发送）';
    btn.addEventListener('click', () => {
      el('cmdInput').value = cmd;
      el('cmdInput').focus();
    });
    box.appendChild(btn);
  }
}

/* ======================================================================= */
/* 参数表单                                                                 */
/* ======================================================================= */

function numberOr(value, fallback) {
  const n = Number(value);
  return Number.isFinite(n) ? n : fallback;
}

function readKParams() {
  const params = {
    kp: Number(el('kpInput').value),
    ki: Number(el('kiInput').value),
    kd: Number(el('kdInput').value),
    kmax: Number(el('kmaxInput').value),
  };
  const problems = [];
  if (!Number.isFinite(params.kp) || params.kp < 0) problems.push('Kp 需要是 >= 0 的数字');
  if (!Number.isFinite(params.ki) || params.ki < 0) problems.push('Ki 需要是 >= 0 的数字');
  if (!Number.isFinite(params.kd) || params.kd < 0) problems.push('Kd 需要是 >= 0 的数字');
  if (!Number.isFinite(params.kmax) || params.kmax < 0 || params.kmax > 10000) {
    problems.push('max 需要是 0~10000 的数字');
  }
  return { params, problems };
}

function readVParams() {
  const v = Number(el('vInput').value);
  const problems = [];
  if (!Number.isFinite(v) || v < 0 || v > 2000) {
    problems.push('V 目标速度需要是 0~2000 的数字（固件 SPD_MAX=2000）');
  }
  return { v, problems };
}

function buildKCommand(p) {
  return `K ${Math.round(p.kp * 100)} ${Math.round(p.ki * 100)} ${Math.round(p.kd * 100)} ${Math.round(p.kmax)}`;
}

function readAllParams() {
  return {
    kp: numberOr(el('kpInput').value, DEFAULT_PARAMS.kp),
    ki: numberOr(el('kiInput').value, DEFAULT_PARAMS.ki),
    kd: numberOr(el('kdInput').value, DEFAULT_PARAMS.kd),
    kmax: numberOr(el('kmaxInput').value, DEFAULT_PARAMS.kmax),
    v: numberOr(el('vInput').value, DEFAULT_PARAMS.v),
  };
}

function applyParamsToForm(p) {
  el('kpInput').value = String(p.kp);
  el('kiInput').value = String(p.ki);
  el('kdInput').value = String(p.kd);
  el('kmaxInput').value = String(p.kmax);
  el('vInput').value = String(p.v);
  updatePreviews();
}

function updatePreviews() {
  const { params } = readKParams();
  const kPreview = el('kPreview');
  if (kPreview) {
    const ok = Number.isFinite(params.kp) && Number.isFinite(params.ki) &&
      Number.isFinite(params.kd) && Number.isFinite(params.kmax);
    kPreview.textContent = ok ? buildKCommand(params) : 'K …';
  }
  const { v } = readVParams();
  const vPreview = el('vPreview');
  if (vPreview) vPreview.textContent = Number.isFinite(v) ? `V${Math.round(v)}` : 'V…';
}

function saveParamsToStorage(params) {
  try {
    localStorage.setItem(PARAMS_KEY, JSON.stringify(params));
    return true;
  } catch (_e) {
    toast('浏览器禁止 localStorage，参数未能保存', 'warn');
    return false;
  }
}

function readParamsFromStorage() {
  try {
    const raw = localStorage.getItem(PARAMS_KEY);
    if (!raw) return null;
    const obj = JSON.parse(raw);
    if (!obj || typeof obj !== 'object') return null;
    return obj;
  } catch (_e) {
    return null;
  }
}

/* ======================================================================= */
/* 日志导出                                                                 */
/* ======================================================================= */

function exportLog() {
  if (!state.entries.length) {
    toast('没有日志可导出', 'warn');
    return;
  }
  const header = [
    '# C30D 串口调参台 日志导出',
    `# 导出时间: ${new Date().toLocaleString('zh-CN')}`,
    `# 数据源: ${state.source === 'serial' ? '串口' : state.source === 'demo' ? '演示模式' : '未连接'}`,
    `# 行数: ${state.entries.length}（解析错误 ${state.stats.errors}）`,
    '# 格式: [HH:MM:SS.mmm] 原始行',
    '',
  ].join('\r\n');
  const body = state.entries.map((e) => `[${fmtTime(e.t, true)}] ${e.text}`).join('\r\n');
  const blob = new Blob([`\ufeff${header}${body}\r\n`], { type: 'text/plain;charset=utf-8' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = `c30d_log_${stamp()}.txt`;
  document.body.appendChild(a);
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
  toast(`已导出 ${state.entries.length} 行日志`, 'ok');
}

function clearLog() {
  state.entries = [];
  state.stats.lines = 0;
  state.stats.errors = 0;
  const list = el('logList');
  if (list) list.innerHTML = '';
  updateStatEls();
  toast('日志已清空', 'ok');
}

/* ======================================================================= */
/* 事件绑定 / 启动                                                           */
/* ======================================================================= */

function bindEvents() {
  el('btnConnect').addEventListener('click', connectSerial);
  el('btnDisconnect').addEventListener('click', disconnectSerial);

  el('btnDemo').addEventListener('click', () => {
    if (state.demo) stopDemo();
    else startDemo();
  });

  // 趋势图暂停 / 清空
  el('chartPause').addEventListener('click', () => {
    state.chartPaused = !state.chartPaused;
    const btn = el('chartPause');
    btn.textContent = state.chartPaused ? '继续曲线' : '暂停曲线';
    btn.classList.toggle('active', state.chartPaused);
    btn.setAttribute('aria-pressed', String(state.chartPaused));
    toast(state.chartPaused ? '曲线已暂停（解析与卡片仍在更新）' : '曲线继续');
  });
  el('chartClear').addEventListener('click', () => {
    for (const chart of Object.values(charts)) chart.clear();
    toast('趋势图已清空');
  });

  // 日志面板
  el('logPauseScroll').addEventListener('change', (e) => {
    if (!e.target.checked) {
      const list = el('logList');
      list.scrollTop = list.scrollHeight;
    }
  });
  el('logHideText').addEventListener('change', (e) => {
    el('logList').classList.toggle('hide-text', e.target.checked);
  });
  el('btnExportLog').addEventListener('click', exportLog);
  el('btnClearLog').addEventListener('click', clearLog);

  // 命令控制台
  el('btnSend').addEventListener('click', async () => {
    const input = el('cmdInput');
    const ok = await requestSend(input.value);
    if (ok) input.value = '';
  });

  el('cmdInput').addEventListener('keydown', async (e) => {
    const input = el('cmdInput');
    if (e.key === 'Enter') {
      e.preventDefault();
      const ok = await requestSend(input.value);
      if (ok) {
        input.value = '';
        state.historyCursor = -1;
      }
    } else if (e.key === 'Escape') {
      clearArm();
    } else if (e.key === 'ArrowUp') {
      if (!state.history.length) return;
      e.preventDefault();
      state.historyCursor = Math.min(state.historyCursor + 1, state.history.length - 1);
      input.value = state.history[state.historyCursor];
    } else if (e.key === 'ArrowDown') {
      if (!state.history.length) return;
      e.preventDefault();
      state.historyCursor = Math.max(state.historyCursor - 1, -1);
      input.value = state.historyCursor < 0 ? '' : state.history[state.historyCursor];
    }
  });

  // 快捷命令按钮
  for (const btn of document.querySelectorAll('[data-cmd]')) {
    btn.addEventListener('click', async () => {
      const cmd = btn.dataset.cmd;
      const input = el('cmdInput');
      input.value = cmd;
      await requestSend(cmd);
    });
  }

  el('dangerGate').addEventListener('change', (e) => {
    if (!e.target.checked) clearArm();
  });

  el('btnClearHistory').addEventListener('click', () => {
    state.history = [];
    state.historyCursor = -1;
    try {
      localStorage.removeItem(HISTORY_KEY);
    } catch (_e) {
      /* 忽略 */
    }
    renderHistory();
  });

  // 参数表单
  el('kForm').addEventListener('submit', async (e) => {
    e.preventDefault();
    const { params, problems } = readKParams();
    if (problems.length) {
      toast(problems[0], 'warn');
      return;
    }
    const cmd = buildKCommand(params);
    const ok = await requestSend(cmd);
    if (ok) {
      saveParamsToStorage(readAllParams());
      toast(`已发送 ${cmd}`, 'ok');
    }
  });

  el('vForm').addEventListener('submit', async (e) => {
    e.preventDefault();
    const { v, problems } = readVParams();
    if (problems.length) {
      toast(problems[0], 'warn');
      return;
    }
    const cmd = `V${Math.round(v)}`;
    const ok = await requestSend(cmd);
    if (ok) {
      saveParamsToStorage(readAllParams());
      toast(`已发送 ${cmd}`, 'ok');
    }
  });

  for (const id of ['kpInput', 'kiInput', 'kdInput', 'kmaxInput', 'vInput']) {
    el(id).addEventListener('input', updatePreviews);
  }

  el('btnSaveParams').addEventListener('click', () => {
    const { problems } = readKParams();
    if (problems.length) {
      toast(problems[0], 'warn');
      return;
    }
    if (saveParamsToStorage(readAllParams())) toast('参数已保存到本机 localStorage', 'ok');
  });

  el('btnResetParams').addEventListener('click', () => {
    applyParamsToForm({ ...DEFAULT_PARAMS });
    try {
      localStorage.removeItem(PARAMS_KEY);
    } catch (_e) {
      /* 忽略 */
    }
    toast('已恢复默认参数（0 / 0 / 0 / 600，ff 由固件保持 -55，V150）', 'ok');
  });

  // 串口热插拔
  if (serialSupported()) {
    navigator.serial.addEventListener('disconnect', (e) => {
      if (e.target === state.port) {
        state.reading = false;
        try {
          if (state.reader) state.reader.cancel();
        } catch (_e) {
          /* 读循环会收尾 */
        }
      }
    });
  }
}

function initSupportUI() {
  if (!serialSupported()) {
    const banner = el('supportBanner');
    if (banner) banner.hidden = false;
  } else if (!window.isSecureContext) {
    const banner = el('supportBanner');
    if (banner) {
      banner.hidden = false;
      banner.innerHTML =
        '<strong>当前页面不是安全上下文（secure context）。</strong>' +
        ' Web Serial 只在 <code>http://localhost</code> 或 https 下可用；' +
        '请通过 <code>serve.ps1</code> 启动后打开 <code>http://localhost:8765</code>。';
    }
  }
}

function init() {
  window.__C30D_APP_READY__ = true;

  buildWheelCards();
  buildExtras();
  initCharts();
  resetCards();

  state.history = loadHistory();
  renderHistory();
  const saved = readParamsFromStorage();
  applyParamsToForm({ ...DEFAULT_PARAMS, ...(saved || {}) });

  bindEvents();
  initSupportUI();
  updateButtons();
  setConnStatus('未连接', 'idle');
  setSource('—');
  resetView();
  updateStatEls();

  if (!serialSupported()) {
    toast('浏览器不支持 Web Serial：演示模式可用，串口连接需 Chrome/Edge + localhost', 'warn');
  }

  // 打开页面即回放示例数据，保证没有串口也能看到完整 UI
  startDemo().catch((err) => {
    console.error('演示模式启动失败：', err);
    toast(`演示模式启动失败：${err && err.message ? err.message : err}`, 'error');
  });
}

init();
