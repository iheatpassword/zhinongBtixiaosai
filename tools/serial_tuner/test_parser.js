/**
 * test_parser.js — parser.js 的 Node 单元测试（无浏览器、无第三方依赖）
 *
 * 运行： node test_parser.js
 * 退出码：0 = 全部通过；1 = 有失败（便于 CI / 手工验收）。
 */

import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

import { parseLine, f2StateName, f2StateLabel, faultName } from './parser.js';

const HERE = path.dirname(fileURLToPath(import.meta.url));

/* 固件真实字段顺序（USER/main.c 2015-2119 行）构造的完整行 */
const FULL =
  'SPD FWD tgt=150 MR=1 CM=0 T0=150 2380/4123 2390/4150 2370/4090 2385/4110 ' +
  'F2=2 FA=0 K=5 VB=118 SY=3 R1=120,5 R2=340,-2 R3=560,8 LE=0 LD=1 LC=175620 ' +
  'DD=123 RD=148 DF=7 AF=2 CE=12 CD=3 CT=1 ST=4 LZ=OK LS=450,600,206,7 ' +
  'RG=137,1,1 LZN=1234 LB=AA 55 01 02 03 04 05 06 YAW=12 C=3 SVO=0 P=1 CMD=F';

const tests = [];
function test(name, fn) {
  tests.push({ name, fn });
}

/* ------------------------------------------------------------------ */
/* 基础解析                                                            */
/* ------------------------------------------------------------------ */

test('完整日志行：类型/方向/基础字段', () => {
  const r = parseLine(FULL);
  assert.equal(r.ok, true);
  assert.equal(r.type, 'log');
  assert.equal(r.errors.length, 0);
  assert.equal(r.dir, 'FWD');
  assert.equal(r.openLoop, false);
  assert.equal(r.tgt, 150);
  assert.equal(r.mr, 1);
  assert.equal(r.cm, 0);
  assert.equal(r.t0, 150);
  assert.equal(r.fieldCount > 20, true);
  assert.deepEqual(r.extras, {});
});

test('完整日志行：四轮 spd/pwm（顺序 FL,FR,RL,RR）', () => {
  const r = parseLine(FULL);
  assert.deepEqual(r.wheels, [
    { spd: 2380, pwm: 4123 },
    { spd: 2390, pwm: 4150 },
    { spd: 2370, pwm: 4090 },
    { spd: 2385, pwm: 4110 },
  ]);
});

test('完整日志行：F2/FA 状态名（fsm2.h）', () => {
  const r = parseLine(FULL);
  assert.equal(r.f2, 2);
  assert.equal(r.f2Name, 'CRAB_R1');
  assert.equal(r.f2Label, '右蟹行 R1');
  assert.equal(r.fa, 0);
  assert.equal(r.faName, '无故障');
});

test('完整日志行：单位换算 YAW/10、VB/10', () => {
  const r = parseLine(FULL);
  assert.equal(r.yawRaw, 12);
  assert.equal(r.yaw, 1.2);
  assert.equal(r.vbRaw, 118);
  assert.equal(r.vb, 11.8);
});

test('完整日志行：R1/R2/R3、里程/调试字段', () => {
  const r = parseLine(FULL);
  assert.deepEqual(r.r1, { count: 120, yaw: 5 });
  assert.deepEqual(r.r2, { count: 340, yaw: -2 });
  assert.deepEqual(r.r3, { count: 560, yaw: 8 });
  assert.equal(r.le, 0);
  assert.equal(r.ld, 1);
  assert.equal(r.lc, 175620);
  assert.equal(r.dd, 123);
  assert.equal(r.rd, 148);
  assert.equal(r.df, 7);
  assert.equal(r.af, 2);
  assert.equal(r.ce, 12);
  assert.equal(r.cd, 3);
  assert.equal(r.ct, 1);
  assert.equal(r.st, 4);
  assert.equal(r.sy, 3);
  assert.equal(r.c, 3);
  assert.equal(r.svo, 0);
  assert.equal(r.p, 1);
  assert.equal(r.k, 5);
});

test('完整日志行：LZ / LS / RG / LB', () => {
  const r = parseLine(FULL);
  assert.equal(r.lz, 'OK');
  assert.equal(r.ls.dL, 450);
  assert.equal(r.ls.dR, 600);
  assert.equal(r.ls.dF, 206);
  assert.equal(r.ls.flags, 7);
  assert.deepEqual(r.ls.valid, { f: true, l: true, r: true });
  assert.equal(r.ridge.dist, 137);
  assert.equal(r.ridge.flags, 1);
  assert.equal(r.ridge.valid, true);
  assert.equal(r.dB, 137); // 后向/垄侧 TOF：来自 RG 首字段
  assert.equal(r.lzn, 1234);
  assert.deepEqual(r.lb, [0xaa, 0x55, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06]);
  assert.equal(r.cmd, 'F');
});

test('字段顺序任意打乱仍可解析', () => {
  const line =
    'SPD BACK YAW=-15 CMD=B LS=300,400,500,7 tgt=120 MR=-1 CM=0 T0=120 ' +
    '-2400/-4800 -2390/-4790 -2380/-4760 -2405/-4810 F2=5 FA=0 VB=117 ' +
    'LZ=OK RG=-1,1,0 ST=2';
  const r = parseLine(line);
  assert.equal(r.ok, true);
  assert.equal(r.type, 'log');
  assert.equal(r.dir, 'BACK');
  assert.equal(r.yaw, -1.5);
  assert.equal(r.cmd, 'B');
  assert.equal(r.ls.dL, 300);
  assert.equal(r.ls.dF, 500);
  assert.equal(r.tgt, 120);
  assert.deepEqual(r.wheels[0], { spd: -2400, pwm: -4800 });
  assert.equal(r.f2Name, 'REV_23');
  assert.equal(r.ridge.dist, null);
  assert.equal(r.ridge.valid, false);
  assert.equal(r.dB, null);
  assert.equal(r.st, 2);
});

test('缺失字段容忍：只保留基础字段也不报错', () => {
  const r = parseLine('SPD STOP tgt=150 MR=0 CM=0 T0=150 0/0 0/0 0/0 0/0 F2=0 FA=0');
  assert.equal(r.ok, true);
  assert.equal(r.type, 'log');
  assert.equal(r.dir, 'STOP');
  assert.equal(r.lz, null);
  assert.equal(r.ls, null);
  assert.equal(r.ridge, null);
  assert.equal(r.lb, null);
  assert.equal(r.cmd, null);
  assert.equal(r.yaw, null);
  assert.equal(r.vb, null);
  assert.equal(r.f2Name, 'IDLE');
  assert.deepEqual(r.wheels[3], { spd: 0, pwm: 0 });
});

test('ST= 旧状态机字段缺失/存在都能解析（已删除的字段）', () => {
  assert.equal(parseLine('SPD STOP F2=0 FA=0').st, null);
  assert.equal(parseLine('SPD STOP F2=0 FA=0 ST=7').st, 7);
});

test('未知/多余字段进入 extras，不视为错误', () => {
  const r = parseLine('SPD STOP FOO=9 F2=0 FA=0 BAR=hello');
  assert.equal(r.ok, true);
  assert.equal(r.extras.FOO, '9');
  assert.equal(r.extras.BAR, 'hello');
  assert.equal(r.f2Name, 'IDLE');
});

test('CMD= 为空、含空格都能保留', () => {
  const empty = parseLine('SPD STOP F2=0 FA=0 CMD=');
  assert.equal(empty.ok, true);
  assert.equal(empty.cmd, '');

  const spaced = parseLine('SPD FWD F2=2 CMD=K 200 5');
  assert.equal(spaced.ok, true);
  assert.equal(spaced.cmd, 'K 200 5');

  // CMD 不在行尾时也能被截断到下一个 KEY=
  const mid = parseLine('SPD FWD CMD=Q YAW=7');
  assert.equal(mid.cmd, 'Q');
  assert.equal(mid.yaw, 0.7);
});

test('键名/前缀大小写不敏感', () => {
  const r = parseLine('spd fwd tgt=100 yaw=5 lz=ok ls=1,2,3,7 f2=1 fa=0');
  assert.equal(r.ok, true);
  assert.equal(r.dir, 'FWD');
  assert.equal(r.tgt, 100);
  assert.equal(r.yaw, 0.5);
  assert.equal(r.lz, 'OK');
  assert.equal(r.f2Name, 'ENTRY');
});

test('CRLF 结尾的行', () => {
  const r = parseLine(FULL + '\r\n');
  assert.equal(r.ok, true);
  assert.equal(r.yaw, 1.2);
  assert.equal(r.raw.endsWith('\r'), false);
});

/* ------------------------------------------------------------------ */
/* 激光 / 链路                                                         */
/* ------------------------------------------------------------------ */

test('LS 无效值 -1 -> null，flags 有效位正确', () => {
  const r = parseLine('SPD STOP F2=1 FA=0 LZ=LOST LS=-1,-1,-1,0');
  assert.equal(r.ok, true);
  assert.equal(r.lz, 'LOST');
  assert.equal(r.ls.dL, null);
  assert.equal(r.ls.dR, null);
  assert.equal(r.ls.dF, null);
  assert.equal(r.ls.flags, 0);
  assert.deepEqual(r.ls.valid, { f: false, l: false, r: false });
});

test('LS flags 位映射：bit0=F bit1=L bit2=R', () => {
  const r = parseLine('SPD STOP LS=100,200,300,2');
  assert.deepEqual(r.ls.valid, { f: false, l: true, r: false });
});

test('LS 非法值 -> ok=false 并给出错误', () => {
  const r = parseLine('SPD STOP F2=0 LS=abc,1,2,3');
  assert.equal(r.ok, false);
  assert.equal(r.ls.dL, null);
  assert.equal(r.type, 'log');
  assert.ok(r.errors.some((e) => e.includes('LS')));
});

test('RG 无效值 -1 -> null，valid=false', () => {
  const r = parseLine('SPD STOP F2=0 RG=-1,0,0');
  assert.equal(r.ok, true);
  assert.equal(r.ridge.dist, null);
  assert.equal(r.ridge.valid, false);
  assert.equal(r.dB, null);
});

test('RG 缺字段容忍', () => {
  const r = parseLine('SPD STOP F2=0 RG=137');
  assert.equal(r.ok, true);
  assert.equal(r.ridge.dist, 137);
  assert.equal(r.ridge.flags, null);
  assert.equal(r.ridge.valid, null);
});

test('LS 缺字段容忍', () => {
  const r = parseLine('SPD STOP F2=0 LS=100,200');
  assert.equal(r.ok, true);
  assert.equal(r.ls.dL, 100);
  assert.equal(r.ls.dF, null);
  assert.equal(r.ls.flags, null);
  assert.deepEqual(r.ls.valid, { f: null, l: null, r: null });
});

test('LS/RG 空字段 -> ok=false', () => {
  assert.equal(parseLine('SPD STOP F2=0 LS=').ok, false);
  assert.equal(parseLine('SPD STOP F2=0 RG=').ok, false);
});

test('LB 非法 -> ok=false；空值不报错', () => {
  const bad = parseLine('SPD STOP F2=0 LB=ZZ');
  assert.equal(bad.ok, false);
  assert.ok(bad.errors.some((e) => e.includes('LB')));
  const empty = parseLine('SPD STOP F2=0 LB=');
  assert.equal(empty.ok, true);
  assert.equal(empty.lb, null);
});

/* ------------------------------------------------------------------ */
/* 开环行 / 文本行 / 空行 / 损坏行                                      */
/* ------------------------------------------------------------------ */

test('开环诊断行 OL PWM=', () => {
  const r = parseLine(
    'OL PWM=2500 tgt=0 MR=1 CM=0 1200/2500 1210/2510 1190/2490 1205/2505 ' +
      'F2=0 FA=0 VB=118 YAW=0 CMD=O2500',
  );
  assert.equal(r.ok, true);
  assert.equal(r.type, 'open-loop');
  assert.equal(r.openLoop, true);
  assert.equal(r.openPwm, 2500);
  assert.equal(r.dir, 'OL');
  assert.equal(r.cmd, 'O2500');
  assert.deepEqual(r.wheels[1], { spd: 1210, pwm: 2510 });
});

test('命令回显/帮助等文本行不当作日志，也不报错', () => {
  const texts = [
    'OK FWD',
    'OK STOP',
    'ERR BUSY (RSM: A/Q)',
    'ERR: V 0..600',
    'TIMEOUT -> STOP',
    'LOW BATT STOP',
    'SAFE STOP',
    '=== C30D speed-loop command ===',
    'F       : forward (speed closed loop)',
    'T / T-  : servos +90 / -90 deg (broadcast)',
    '===============================',
  ];
  for (const line of texts) {
    const r = parseLine(line);
    assert.equal(r.ok, true, `文本行不应报错: ${line}`);
    assert.equal(r.type, 'text', `应识别为 text: ${line}`);
    assert.ok(['ok', 'err', 'info'].includes(r.textKind));
  }
  assert.equal(parseLine('OK FWD').textKind, 'ok');
  assert.equal(parseLine('ERR BUSY').textKind, 'err');
  assert.equal(parseLine('TIMEOUT -> STOP').textKind, 'info');
});

test('空行 / null / undefined 不抛异常', () => {
  for (const v of ['', '   ', '\r\n', null, undefined]) {
    const r = parseLine(v);
    assert.equal(r.ok, true);
    assert.equal(r.type, 'empty');
  }
});

test('损坏日志行 -> ok=false（界面标红）但不抛异常', () => {
  const r1 = parseLine('SPD FWD tgt=abc 100/200');
  assert.equal(r1.type, 'log');
  assert.equal(r1.ok, false);
  assert.ok(r1.errors.some((e) => e.includes('TGT')));
  assert.deepEqual(r1.wheels[0], { spd: 100, pwm: 200 });
  assert.equal(r1.wheels[3].spd, null);

  const r2 = parseLine('SPD FWD');
  assert.equal(r2.ok, false);
  assert.ok(r2.errors.length > 0);

  const r3 = parseLine('SPD');
  assert.equal(r3.type, 'log');
  assert.equal(r3.ok, false);

  const r4 = parseLine('SPD FWD F2=0 R1=x,2');
  assert.equal(r4.ok, false);
  assert.ok(r4.errors.some((e) => e.includes('R1')));

  const r5 = parseLine('OL tgt=0');
  assert.equal(r5.ok, false);
  assert.ok(r5.errors.some((e) => e.includes('PWM')));
});

test('极端/畸形输入不抛异常', () => {
  const weird = [42, {}, [], 'SPD ' + 'x'.repeat(5000), '='.repeat(50), 'a=1 b'];
  for (const v of weird) {
    assert.doesNotThrow(() => parseLine(v));
  }
  const obj = parseLine(42);
  assert.equal(obj.type, 'text');
});

/* ------------------------------------------------------------------ */
/* 辅助函数                                                            */
/* ------------------------------------------------------------------ */

test('F2 / FA 名称映射与未知码', () => {
  assert.equal(f2StateName(0), 'IDLE');
  assert.equal(f2StateName(14), 'FAULT');
  assert.equal(f2StateLabel(13), '完成');
  assert.equal(f2StateName(99), 'UNKNOWN(99)');
  assert.equal(f2StateLabel(99), '未知状态 99');
  assert.equal(faultName(0), '无故障');
  assert.ok(faultName(7).includes('超时'));
  assert.ok(faultName(2).includes('前向'));
  assert.equal(faultName(99), '未知故障(99)');
  assert.equal(f2StateName(null), null);
  assert.equal(faultName(undefined), null);
});

test('parseLine 结果不共享可变对象（两次调用互不影响）', () => {
  const a = parseLine(FULL);
  const b = parseLine(FULL);
  a.wheels[0].spd = 99999;
  a.ls.dL = 99999;
  a.r1.count = 99999;
  a.extras.X = '1';
  assert.equal(b.wheels[0].spd, 2380);
  assert.equal(b.ls.dL, 450);
  assert.equal(b.r1.count, 120);
  assert.deepEqual(b.extras, {});
});

/* ------------------------------------------------------------------ */
/* sample_log.txt（演示数据）                                          */
/* ------------------------------------------------------------------ */

test('sample_log.txt：每行都能解析、包含 F2/FA/LS/RG/YAW/LZ', () => {
  const file = path.join(HERE, 'sample_log.txt');
  assert.ok(fs.existsSync(file), '缺少 sample_log.txt');
  const raw = fs.readFileSync(file, 'utf8');
  assert.ok(raw.includes('F2='), '样例缺少 F2=');
  assert.ok(raw.includes('FA='), '样例缺少 FA=');
  assert.ok(raw.includes('LS='), '样例缺少 LS=');
  assert.ok(raw.includes('RG='), '样例缺少 RG=');
  assert.ok(raw.includes('YAW='), '样例缺少 YAW=');
  assert.ok(raw.includes('LZ='), '样例缺少 LZ=');

  const lines = raw.split(/\r?\n/).filter((l) => l.trim() !== '');
  assert.ok(lines.length >= 100, `样例行数应 >= 100，实际 ${lines.length}`);

  let logCount = 0;
  let textCount = 0;
  let sawFault = false;
  let sawLost = false;
  let sawInvalidLaser = false;
  let sawWithSt = false;
  let sawWithoutSt = false;
  const failures = [];

  for (const line of lines) {
    const r = parseLine(line);
    if (!r.ok) {
      if (failures.length < 5) failures.push({ line, errors: r.errors });
      continue;
    }
    if (r.type === 'log') {
      logCount += 1;
      if (r.f2Name === 'FAULT' && r.faName && r.faName !== '无故障') sawFault = true;
      if (r.lz === 'LOST') sawLost = true;
      if (r.ls && r.ls.dL === null) sawInvalidLaser = true;
      if (r.st === null) sawWithoutSt = true;
      else sawWithSt = true;
    } else if (r.type === 'text') {
      textCount += 1;
    } else {
      failures.push({ line, errors: ['不应出现空行（已过滤）'] });
    }
  }

  assert.deepEqual(failures, [], `样例中存在解析失败行: ${JSON.stringify(failures.slice(0, 3))}`);
  assert.ok(logCount >= 100, `日志行应 >= 100，实际 ${logCount}`);
  assert.ok(textCount >= 3, `文本/回显行应 >= 3，实际 ${textCount}`);
  assert.equal(sawFault, true, '样例应包含 F2=FAULT + 非 0 FA 的行');
  assert.equal(sawLost, true, '样例应包含 LZ=LOST 的行');
  assert.equal(sawInvalidLaser, true, '样例应包含激光无效（-1）的行');
  assert.equal(sawWithSt, true, '样例应有带 ST= 的旧行');
  assert.equal(sawWithoutSt, true, '样例应有不带 ST= 的行（ST 已删除）');
});

/* ------------------------------------------------------------------ */
/* 运行                                                                */
/* ------------------------------------------------------------------ */

let passed = 0;
const failed = [];
for (const t of tests) {
  try {
    t.fn();
    passed += 1;
    console.log(`  PASS  ${t.name}`);
  } catch (err) {
    failed.push({ name: t.name, err });
    console.log(`  FAIL  ${t.name}`);
    console.log(`        ${err && err.message ? err.message : err}`);
  }
}

console.log('');
console.log(`共 ${tests.length} 项：通过 ${passed}，失败 ${failed.length}`);
if (failed.length > 0) {
  console.log('');
  console.log('失败详情：');
  for (const f of failed) {
    console.log(`- ${f.name}`);
    console.log(f.err && f.err.stack ? f.err.stack : String(f.err));
  }
  process.exitCode = 1;
} else {
  console.log('parser.js 全部测试通过。');
}
