/**
 * parser.js — C30D 四轮四舵小车 1 Hz 串口日志解析器
 * 纯 ES module，零第三方依赖；浏览器（Web Serial 页面）与 Node（test_parser.js）通用。
 *
 * 固件日志格式（USER/main.c 约 2015-2119 行，1 Hz 一行，USART1/3/UART5）：
 *
 *   SPD FWD|BACK|STOP tgt=.. MR=.. CM=.. T0=.. <spd0>/<pwm0> <spd1>/<pwm1>
 *       <spd2>/<pwm2> <spd3>/<pwm3> F2=.. FA=.. K=.. VB=.. SY=..
 *       R1=c,y R2=c,y R3=c,y LE=.. LD=.. LC=.. DD=.. RD=.. DF=.. AF=..
 *       CE=.. CD=.. CT=.. ST=.. LZ=OK|LOST LS=dL,dR,dF,flags
 *       RG=dB,ridge_flags,ridge_valid LZN=.. LB=aa bb ...
 *       YAW=.. C=.. SVO=.. P=.. CMD=..
 *
 * 开环诊断行（固件 open_loop != 0 时，前缀与 SPD 行不同）：
 *   OL PWM=<pwm> tgt=.. MR=.. ...
 *
 * 解析约定（以 USER/main.c 与 USER/fsm2.h 为准）：
 *  - 按 `KEY=value` 切分，不依赖字段顺序；KEY 大小写不敏感。
 *  - 缺失字段 -> null，不报错（例如 `ST=` 旧状态机字段已删除）。
 *  - 未知/多余字段 -> extras，不报错。
 *  - 已知字段的值不是整数 -> errors 记录，ok=false（界面标红），但绝不抛异常。
 *  - 四轮是无键值的位置字段 `<spd>/<pwm>`，顺序固定 FL,FR,RL,RR；用正则扫描，
 *    缺失的轮子补 {spd:null,pwm:null}，不报错（容忍字段被删）。
 *  - 激光无效值 `-1`（固件 0xFFFF）-> null，同时 LS flags 的有效位单独给出。
 *  - `dB` 不在 `LS=` 里，而是 `RG=` 的第一个字段（USER/app_if.h: dRidge，
 *    垄侧/后向 TOF；HANDOVER.md 第 8 节写作 `RG=dB,...`）。本解析器因此提供
 *    `ridge.dist` 与派生 `dB`。
 *  - `LZ=` 固件写 "OK " 或 "LOST "（后面可能跟双空格），这里归一化为 OK/LOST。
 *  - `CMD=` 是 last_cmd（最长 7 字符，可能为空或含空格，如 "K 200 5"），
 *    按 key=value 分段可完整保留到下一个 key 之前。
 */

export const WHEEL_NAMES = Object.freeze(['FL', 'FR', 'RL', 'RR']);
export const WHEEL_LABELS = Object.freeze(['左前', '右前', '左后', '右后']);

/** F2 状态码 -> 固件英文名（USER/fsm2.h Fsm2State / USER/fsm2.c Fsm2_StateName）。 */
export const F2_STATE_NAMES = Object.freeze({
  0: 'IDLE',
  1: 'ENTRY',
  2: 'CRAB_R1',
  3: 'REV_12',
  4: 'CRAB_L2',
  5: 'REV_23',
  6: 'CRAB_R3',
  7: 'REV_34',
  8: 'CRAB_L4',
  9: 'REV_45',
  10: 'CRAB_R5',
  11: 'EXIT_L',
  12: 'REV_PARK',
  13: 'DONE',
  14: 'FAULT',
});

/** F2 状态码 -> 中文短标签。 */
export const F2_STATE_LABELS = Object.freeze({
  0: '待机',
  1: '进垄',
  2: '右蟹行 R1',
  3: '倒车 12',
  4: '左蟹行 L2',
  5: '倒车 23',
  6: '右蟹行 R3',
  7: '倒车 34',
  8: '左蟹行 L4',
  9: '倒车 45',
  10: '右蟹行 R5',
  11: '左出垄',
  12: '倒车入位',
  13: '完成',
  14: '故障',
});

/** FA 故障码 -> 名称（USER/fsm2.h F2_FAULT_xxx，HANDOVER.md 第 8 节）。 */
export const FAULT_NAMES = Object.freeze({
  0: '无故障',
  1: '堵转',
  2: '前向激光过近 (<30 mm)',
  3: '后向激光过近 (<30 mm)',
  4: '左侧激光过近 (<30 mm)',
  5: '右侧激光过近 (<30 mm)',
  6: '蟹行时前后激光均无效',
  7: '状态超时',
  8: '外部中止',
});

/** F2 状态码 -> 固件英文名；未知码返回 UNKNOWN(n)，永不抛异常。 */
export function f2StateName(code) {
  if (code === null || code === undefined) return null;
  const n = Number(code);
  return Object.prototype.hasOwnProperty.call(F2_STATE_NAMES, n)
    ? F2_STATE_NAMES[n]
    : `UNKNOWN(${n})`;
}

/** F2 状态码 -> 中文标签；未知码返回“未知状态 n”。 */
export function f2StateLabel(code) {
  if (code === null || code === undefined) return null;
  const n = Number(code);
  return Object.prototype.hasOwnProperty.call(F2_STATE_LABELS, n)
    ? F2_STATE_LABELS[n]
    : `未知状态 ${n}`;
}

/** FA 故障码 -> 中文名称；未知码返回“未知故障(n)”。 */
export function faultName(code) {
  if (code === null || code === undefined) return null;
  const n = Number(code);
  return Object.prototype.hasOwnProperty.call(FAULT_NAMES, n)
    ? FAULT_NAMES[n]
    : `未知故障(${n})`;
}

const INT_RE = /^[+-]?\d+$/;
const WHEEL_PAIR_RE = /^(-?\d+)\/(-?\d+)$/;

/** 已知整数字段 KEY -> 结果对象属性名（YAW/VB 保留原始值，最后再换算）。 */
const NUMERIC_FIELDS = Object.freeze({
  TGT: 'tgt',
  MR: 'mr',
  CM: 'cm',
  T0: 't0',
  F2: 'f2',
  FA: 'fa',
  K: 'k',
  VB: 'vbRaw',
  SY: 'sy',
  LE: 'le',
  LD: 'ld',
  LC: 'lc',
  DD: 'dd',
  RD: 'rd',
  DF: 'df',
  AF: 'af',
  CE: 'ce',
  CD: 'cd',
  CT: 'ct',
  ST: 'st',
  LZN: 'lzn',
  YAW: 'yawRaw',
  C: 'c',
  SVO: 'svo',
  P: 'p',
});

function parseInteger(value) {
  if (typeof value !== 'string') return null;
  const s = value.trim();
  if (!INT_RE.test(s)) return null;
  const n = Number(s);
  return Number.isSafeInteger(n) ? n : null;
}

/** 把一个完整的单行文本切成 [{key, value}]；value 取到下一个 KEY= 之前。 */
function extractFields(text) {
  const re = /([A-Za-z][A-Za-z0-9_]*)=/g;
  const marks = [];
  let m;
  while ((m = re.exec(text)) !== null) {
    marks.push({ start: m.index, end: m.index + m[0].length, key: m[1] });
  }
  const fields = [];
  for (let i = 0; i < marks.length; i += 1) {
    const nextStart = i + 1 < marks.length ? marks[i + 1].start : text.length;
    fields.push({
      key: marks[i].key,
      value: text.slice(marks[i].end, nextStart).trim(),
    });
  }
  return fields;
}

function emptyResult(raw) {
  return {
    ok: true,
    type: 'empty', // 'log' | 'open-loop' | 'text' | 'empty'
    textKind: null, // 'ok' | 'err' | 'info'（仅 text 行）
    errors: [],
    raw,

    dir: null, // FWD / BACK / STOP / OL / 其它固件词
    openLoop: false,
    openPwm: null,

    tgt: null,
    mr: null,
    cm: null,
    t0: null,
    wheels: [
      { spd: null, pwm: null },
      { spd: null, pwm: null },
      { spd: null, pwm: null },
      { spd: null, pwm: null },
    ],

    f2: null,
    f2Name: null,
    f2Label: null,
    fa: null,
    faName: null,

    k: null,
    vbRaw: null,
    vb: null,
    sy: null,
    r1: { count: null, yaw: null },
    r2: { count: null, yaw: null },
    r3: { count: null, yaw: null },
    le: null,
    ld: null,
    lc: null,
    dd: null,
    rd: null,
    df: null,
    af: null,
    ce: null,
    cd: null,
    ct: null,
    st: null,

    lz: null, // 'OK' | 'LOST' | 其它原文
    ls: null, // {dL,dR,dF,flags,valid:{f,l,r},extra}
    ridge: null, // {dist,flags,valid,extra}
    dB: null, // 派生：ridge.dist（RG 首字段）
    lzn: null,
    lb: null, // 8 个十六进制字节

    yawRaw: null, // YAW 原始整数（0.1°）
    yaw: null, // YAW/10 = 度
    c: null,
    svo: null,
    p: null,
    cmd: null, // 可能为 ''（固件初始 last_cmd 为空）

    extras: {}, // 未知字段 key -> 原始字符串
    fieldCount: 0,
  };
}

function parseLaser(res, value) {
  const parts = value === '' ? [] : value.split(',').map((s) => s.trim());
  if (parts.length === 0) {
    res.errors.push('LS 字段为空');
    res.ls = { dL: null, dR: null, dF: null, flags: null, valid: { f: null, l: null, r: null }, extra: [] };
    return;
  }
  const nums = parts.map((p) => (p === '' ? null : parseInteger(p)));
  const ls = {
    dL: null,
    dR: null,
    dF: null,
    flags: null,
    valid: { f: null, l: null, r: null },
    extra: [],
  };
  const names = ['dL', 'dR', 'dF', 'flags'];
  const count = Math.min(parts.length, 4);
  for (let i = 0; i < count; i += 1) {
    if (nums[i] === null) {
      res.errors.push(`LS 第 ${i + 1} 项（${names[i]}）不是整数: "${parts[i]}"`);
    }
  }
  // 注意用 != null：parts 比 4 项短时 nums[i] 是 undefined
  if (nums[0] != null) ls.dL = nums[0] === -1 ? null : nums[0];
  if (nums[1] != null) ls.dR = nums[1] === -1 ? null : nums[1];
  if (nums[2] != null) ls.dF = nums[2] === -1 ? null : nums[2];
  if (nums[3] != null) {
    ls.flags = nums[3];
    // USER/main.c: vF = flags&0x01, vL = flags&0x02, vR = flags&0x04
    ls.valid.f = (nums[3] & 1) !== 0;
    ls.valid.l = (nums[3] & 2) !== 0;
    ls.valid.r = (nums[3] & 4) !== 0;
  }
  if (parts.length > 4) ls.extra = parts.slice(4);
  res.ls = ls;
}

function parseRidge(res, value) {
  const parts = value === '' ? [] : value.split(',').map((s) => s.trim());
  if (parts.length === 0) {
    res.errors.push('RG 字段为空');
    res.ridge = { dist: null, flags: null, valid: null, extra: [] };
    res.dB = null;
    return;
  }
  const nums = parts.map((p) => (p === '' ? null : parseInteger(p)));
  const ridge = { dist: null, flags: null, valid: null, extra: [] };
  const names = ['dB', 'ridge_flags', 'ridge_valid'];
  const count = Math.min(parts.length, 3);
  for (let i = 0; i < count; i += 1) {
    if (nums[i] === null) {
      res.errors.push(`RG 第 ${i + 1} 项（${names[i]}）不是整数: "${parts[i]}"`);
    }
  }
  if (nums[0] != null) ridge.dist = nums[0] === -1 ? null : nums[0];
  if (nums[1] != null) ridge.flags = nums[1];
  if (nums[2] != null) ridge.valid = nums[2] !== 0;
  if (parts.length > 3) ridge.extra = parts.slice(3);
  res.ridge = ridge;
  res.dB = ridge.dist;
}

function parseRunHist(res, key, value) {
  const parts = value.split(',').map((s) => s.trim());
  const count = parts.length > 0 ? parseInteger(parts[0]) : null;
  const yaw = parts.length > 1 ? parseInteger(parts[1]) : null;
  if (count === null && parts[0] !== undefined) {
    res.errors.push(`${key} count 不是整数: "${parts[0]}"`);
  }
  if (yaw === null && parts.length > 1) {
    res.errors.push(`${key} yaw 不是整数: "${parts[1]}"`);
  }
  const prop = key.toLowerCase(); // r1 / r2 / r3
  if (prop === 'r1' || prop === 'r2' || prop === 'r3') {
    res[prop] = { count, yaw };
  }
}

function applyField(res, field) {
  const key = field.key.toUpperCase();
  const value = field.value;

  if (/^R[123]$/.test(key)) {
    parseRunHist(res, key, value);
    return;
  }

  if (key === 'LS') {
    parseLaser(res, value);
    return;
  }
  if (key === 'RG') {
    parseRidge(res, value);
    return;
  }
  if (key === 'LZ') {
    if (value === '') {
      res.errors.push('LZ 字段为空');
    } else {
      // 固件写 "OK " / "LOST "，后面可能是双空格；只取第一个词，保留原文大写
      res.lz = value.split(/\s+/)[0].toUpperCase();
    }
    return;
  }
  if (key === 'LB') {
    const bytes = [];
    for (const tok of value.split(/\s+/)) {
      if (/^[0-9A-Fa-f]{2}$/.test(tok)) bytes.push(parseInt(tok, 16));
    }
    if (bytes.length > 0) {
      res.lb = bytes.slice(0, 8);
    } else if (value !== '') {
      res.errors.push(`LB 无法解析为十六进制字节: "${value}"`);
    }
    return;
  }
  if (key === 'CMD') {
    // last_cmd 可能为空、含空格；extractFields 已按下一个 KEY= 截断
    res.cmd = value;
    return;
  }
  if (key === 'PWM' && res.type === 'open-loop') {
    const n = parseInteger(value);
    if (n === null) res.errors.push(`PWM 不是整数: "${value}"`);
    else res.openPwm = n;
    return;
  }

  if (Object.prototype.hasOwnProperty.call(NUMERIC_FIELDS, key)) {
    const n = parseInteger(value);
    if (n === null) {
      res.errors.push(`字段 ${key} 不是整数: "${value}"`);
    } else {
      res[NUMERIC_FIELDS[key]] = n;
    }
    return;
  }

  // 未知字段：容忍，保留原文
  res.extras[key] = value;
}

/**
 * 解析一行串口日志。
 *
 * @param {string|null|undefined} line 单行原始文本（可含结尾 \r\n）
 * @returns {ReturnType<typeof emptyResult>} 结构化结果；ok=false 表示该行是损坏的日志行。
 */
export function parseLine(line) {
  let text = '';
  try {
    text = line === null || line === undefined ? '' : String(line);
  } catch (_e) {
    text = '';
  }
  text = text.replace(/\r/g, '').split('\n')[0].trim();

  const res = emptyResult(text);
  if (text === '') {
    res.type = 'empty';
    return res;
  }

  try {
    // 前缀判定：SPD 日志行 / OL 开环诊断行 / 其它文本（命令回显、帮助等）
    if (/^SPD\b/i.test(text)) {
      res.type = 'log';
      const dm = /^SPD\s+([A-Za-z][A-Za-z0-9_]*)/i.exec(text);
      if (dm) {
        res.dir = dm[1].toUpperCase();
      } else {
        res.errors.push('SPD 行缺少运行方向（FWD/BACK/STOP）');
      }
    } else if (/^OL\b/i.test(text)) {
      res.type = 'open-loop';
      res.dir = 'OL';
      res.openLoop = true;
    } else {
      res.type = 'text';
      if (/^OK\b/i.test(text)) res.textKind = 'ok';
      else if (/^ERR\b/i.test(text)) res.textKind = 'err';
      else res.textKind = 'info';
      return res;
    }

    // 先把无键值的位置字段 <spd>/<pwm> 摘出来（否则会被上一个 key 的值吞掉，
    // 例如 T0=150 后面紧跟 2380/4123）。顺序固定 FL,FR,RL,RR。
    const wheelTokens = [];
    const keyedTokens = [];
    for (const tok of text.split(/\s+/)) {
      const m = WHEEL_PAIR_RE.exec(tok);
      if (m) wheelTokens.push({ spd: Number(m[1]), pwm: Number(m[2]) });
      else keyedTokens.push(tok);
    }
    for (let i = 0; i < 4; i += 1) {
      res.wheels[i] = wheelTokens[i] || { spd: null, pwm: null };
    }

    const payload = keyedTokens.join(' ');
    const fields = extractFields(payload);
    res.fieldCount = fields.length;
    for (const field of fields) applyField(res, field);

    if (fields.length === 0) {
      res.errors.push('日志行中未找到任何 key=value 字段');
    }
    if (res.type === 'open-loop' && res.openPwm === null && fields.length > 0) {
      res.errors.push('开环行缺少 PWM= 字段');
    }
  } catch (err) {
    // 防御性兜底：任何意外都只记错误，不把异常抛给调用方
    res.errors.push(`解析异常: ${err && err.message ? err.message : String(err)}`);
  }

  // 单位换算与派生名称（以 USER/main.c、USER/fsm2.h 为准）
  res.yaw = res.yawRaw === null ? null : res.yawRaw / 10;
  res.vb = res.vbRaw === null ? null : res.vbRaw / 10;
  res.f2Name = res.f2 === null ? null : f2StateName(res.f2);
  res.f2Label = res.f2 === null ? null : f2StateLabel(res.f2);
  res.faName = res.fa === null ? null : faultName(res.fa);
  res.ok = res.errors.length === 0;

  return res;
}

export default { parseLine, f2StateName, f2StateLabel, faultName, WHEEL_NAMES, WHEEL_LABELS };
