#include <Adafruit_VL53L7CX.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>

// 28BYJ-48 + ULN2003, same pins as the working motor test.
const int MOTOR_IN1 = 4;
const int MOTOR_IN2 = 5;
const int MOTOR_IN3 = 6;
const int MOTOR_IN4 = 7;

// VL53L7CX I2C pins.
const int I2C_SDA = 8;
const int I2C_SCL = 9;
const int VL53_LPN_PIN = 10;  // SHUT/LPn -> GPIO10, used for a real hardware reset.

const long MOTOR_STEPS_PER_REV = 4096;
const int RING_TEETH = 100;
const int PINION_TEETH = 14;
const long TURNTABLE_ONE_REV_STEPS =
    ((long)MOTOR_STEPS_PER_REV * RING_TEETH + (PINION_TEETH / 2)) / PINION_TEETH;

const char *AP_SSID = "VL53L7CX_MODEL";
const char *AP_PASS = "12345678";

WebServer server(80);
Adafruit_VL53L7CX vl53l7cx;
VL53L7CX_ResultsData results;

uint16_t distanceMm[64];
uint8_t statusCode[64];
uint32_t frameId = 0;
uint32_t lastFrameMs = 0;
bool sensorOk = false;
bool i2cFound29 = false;
uint8_t i2cDeviceCount = 0;
String sensorMessage = "not initialized";
bool lastFrameHasObject = false;
uint8_t lastObjectZones = 0;
uint16_t lastNearestMm = 0;

bool motorRunContinuous = false;
long motorStepsRemaining = 0;
long turntableStep = 0;
int halfStepIndex = 0;
uint32_t lastStepUs = 0;
uint32_t stepIntervalUs = 5000;
const uint32_t EMPTY_REGION_STEP_US = 2600;
const uint16_t OBJECT_MIN_DISTANCE_MM = 20;
const uint16_t OBJECT_NEAR_LIMIT_MM = 165;
const uint8_t OBJECT_MIN_ZONES = 5;
bool adaptiveScanSpeed = false;
long pendingSectorSteps = 0;
uint32_t pendingSectorStepUs = 7000;
bool pendingSectorAdaptive = false;
uint32_t lastSerialDebugMs = 0;

const uint8_t SCAN_FRAME_RING_SIZE = 96;
struct ScanFrame {
  uint32_t id;
  uint16_t angleCdeg;
  uint16_t pinionAngleCdeg;
  long turntableStep;
  uint8_t objectZones;
  uint8_t hasObject;
  uint16_t nearestMm;
  uint16_t dist[64];
  uint8_t status[64];
};

ScanFrame scanFrames[SCAN_FRAME_RING_SIZE];
uint32_t scanFrameCounter = 0;
uint8_t scanFrameWriteIndex = 0;
bool scanCaptureEnabled = false;

const int halfStepSeq[8][4] = {
    {1, 0, 0, 0},
    {1, 1, 0, 0},
    {0, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 0},
    {0, 0, 1, 1},
    {0, 0, 0, 1},
    {1, 0, 0, 1},
};

const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="zh-Hant">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>VL53L7CX Fixed Lineframe Server</title>
  <style>
    :root { color-scheme: light; font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; }
    body { margin: 0; background: #f3f2ee; color: #1d232b; }
    header { padding: 12px 14px; border-bottom: 1px solid #d6d3ca; background: #fff; display: flex; align-items: center; justify-content: space-between; gap: 12px; flex-wrap: wrap; }
    h1 { margin: 0; font-size: 18px; }
    main { max-width: 1180px; margin: 0 auto; padding: 14px; display: grid; grid-template-columns: 390px minmax(320px, 1fr); gap: 14px; }
    .controls { display: flex; gap: 8px; flex-wrap: wrap; align-items: center; }
    button { border: 1px solid #a9aaa2; background: #fff; color: #1d232b; border-radius: 6px; padding: 8px 10px; font-size: 14px; cursor: pointer; }
    button.primary { background: #1f7a5a; border-color: #1f7a5a; color: #fff; }
    button.danger { background: #b84235; border-color: #b84235; color: #fff; }
    button.active { outline: 3px solid rgba(31, 122, 90, .22); }
    label { font-size: 13px; color: #515963; }
    input, select { width: 84px; padding: 7px 8px; border: 1px solid #b8b7ae; border-radius: 6px; font-size: 14px; background: #fff; }
    select { width: 112px; }
    .stats { display: grid; grid-template-columns: repeat(2, minmax(120px, 1fr)); gap: 8px; margin-bottom: 10px; }
    .stat { background: #fff; border: 1px solid #dad8cf; border-radius: 8px; padding: 9px; min-width: 0; }
    .stat b { display: block; color: #626a74; font-size: 12px; margin-bottom: 3px; }
    .stat span { font-size: 17px; font-variant-numeric: tabular-nums; word-break: break-word; }
    .grid { display: grid; grid-template-columns: repeat(8, 1fr); gap: 4px; padding: 4px; background: #d8d5cc; border-radius: 8px; }
    .cell { aspect-ratio: 1; border-radius: 4px; display: flex; align-items: center; justify-content: center; font-size: clamp(9px, 2.4vw, 12px); font-variant-numeric: tabular-nums; overflow: hidden; color: #111; }
    canvas { display: block; width: 100%; height: min(72vh, 610px); border: 1px solid #dad8cf; border-radius: 8px; background: #fff; touch-action: none; }
    .hint { color: #59616b; font-size: 13px; line-height: 1.45; margin-top: 10px; }
    .mono { font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; }
    .debuglog { height: 170px; overflow: auto; margin-top: 10px; padding: 8px; border-radius: 6px; background: #111827; color: #d1d5db; font-size: 11px; line-height: 1.35; white-space: pre-wrap; }
    @media (max-width: 830px) { main { grid-template-columns: 1fr; padding: 10px; } canvas { height: 430px; } }
  </style>
</head>
<body>
  <header>
    <h1>VL53L7CX 表面點雲建模</h1>
    <div class="controls">
      <button id="startBtn" class="primary" onclick="startModel()">開始建模</button>
      <button onclick="calibrateBackground()">空掃校正一圈</button>
      <button onclick="oneRevModel()">轉一圈建模</button>
      <button onclick="slowRefineScan()">慢速補掃一圈</button>
      <button onclick="smartRefineScan()">智慧補洞</button>
      <button class="danger" onclick="stopModel()">停止</button>
      <button onclick="clearCalibration()">清校正</button>
      <button onclick="cmd('/zero')">角度歸零</button>
      <button onclick="clearModel()">清圖</button>
      <button onclick="exportPLY()">匯出 PLY</button>
      <button onclick="exportCSV()">匯出 CSV</button>
      <button onclick="exportDebugLog()">匯出 Debug Log</button>
      <button onclick="exportRawScanLog()">匯出 Raw Scan</button>
      <button onclick="replayRawScan()">用 Raw 重算</button>
      <label>模式 <select id="mode"><option value="rawhits" selected>即時命中點雲</option><option value="surface3d">幾何表面</option><option value="lineframe">長方形診斷</option></select></label>
      <label>固定模型方向 <select id="dir" onchange="setRebuildNeeded()"><option value="-1">-轉盤角度</option><option value="1">+轉盤角度</option></select></label>
      <label>距離投影 <select id="distmodel" onchange="setRebuildNeeded()"><option value="perpendicular" selected>perpendicular</option><option value="radial">legacy radial</option></select></label>
      <label>角度倍率 <input id="angscale" type="number" min="0.100" max="1.500" step="0.001" value="1.000" onchange="setRebuildNeeded()"></label>
      <label>中心距 <input id="centerd" type="number" min="20" max="500" step="0.5" value="50" onchange="setRebuildNeeded()"></label>
      <label>近遮擋排除 <input id="mindist" type="number" min="5" max="120" step="1" value="20" onchange="setRebuildNeeded()"></label>
      <label>最遠距離 <input id="maxdist" type="number" min="80" max="300" step="1" value="165" onchange="setRebuildNeeded()"></label>
      <label>Zone mapping <select id="zonemap" onchange="setRebuildNeeded()"><option value="horizontal_cw" selected>horizontal 90 CW</option><option value="horizontal_ccw">horizontal 90 CCW</option><option value="sensor">sensor x+y</option><option value="exported">exported grid</option><option value="flip_x">flip X</option><option value="flip_y">flip Y</option><option value="rot180">rot 180</option><option value="transpose">transpose</option></select></label>
      <label>軸心X校正 <input id="offx" type="number" min="-80" max="80" step="1" value="0" onchange="setRebuildNeeded()"></label>
      <label>軸心Y校正 <input id="offy" type="number" min="-80" max="80" step="1" value="0" onchange="setRebuildNeeded()"></label>
      <label>感測器Z <input id="offz" type="number" min="-80" max="80" step="1" value="0" onchange="setRebuildNeeded()"></label>
      <label>Yaw Z <input id="sensyaw" type="number" min="-45" max="45" step="0.5" value="0" onchange="setRebuildNeeded()"></label>
      <label>Pitch X <input id="senspitch" type="number" min="-45" max="45" step="0.5" value="0" onchange="setRebuildNeeded()"></label>
      <label>Roll Y <input id="sensroll" type="number" min="-45" max="45" step="0.5" value="0" onchange="setRebuildNeeded()"></label>
      <label>步進 us <input id="speed" type="number" min="1500" max="20000" step="100" value="5000" onchange="setSpeed()"></label>
    </div>
  </header>
  <main>
    <section>
      <div class="stats">
        <div class="stat"><b>感測器</b><span id="sensor">--</span></div>
        <div class="stat"><b>角度</b><span id="angle">--</span></div>
        <div class="stat"><b>最近距離</b><span id="min">--</span></div>
        <div class="stat"><b>點雲 / 原始 / 命中</b><span id="pts">0 / 0 / 0</span></div>
        <div class="stat"><b>覆蓋率</b><span id="cover">0%</span></div>
        <div class="stat"><b>掃描品質</b><span id="quality">--</span></div>
      </div>
      <div id="grid" class="grid"></div>
      <div class="hint">
        Calibration-first：先按「空掃校正一圈」建立同角度/同 zone 的背景模型，再放物體掃描。raw scan 完整記錄 64 區距離/status；即時命中點雲保留 foreground raw hit，幾何表面再由支撐線交集重建。
      </div>
      <div class="hint mono" id="msg"></div>
      <pre class="debuglog" id="debuglog">debug log waiting...</pre>
    </section>
    <section>
      <canvas id="plot" width="760" height="760"></canvas>
    </section>
  </main>
  <script>
    const SENSOR_Y = 50;
    const SENSOR_Z = 61;
    const HFOV_DEG = 60;
    const VFOV_DEG = 60;
    const DRAW_POINT_LIMIT = 22000;
    const RAW_SURFACE_POINT_LIMIT = 36000;
    const RAW_SURFACE_MIN_FRAME_POINTS = 1;
    const ANGLE_BINS = 72;
    const BACKGROUND_BINS = ANGLE_BINS;
    const BACKGROUND_MIN_SAMPLES = 3;
    const BACKGROUND_FOREGROUND_MARGIN_MM = 8;
    const BACKGROUND_NEIGHBOR_BINS = 1;
    const BACKGROUND_VALID_MAX_MM = 300;
    const SUPPORT_SAMPLE_QUANTILE = 0.35;
    const VISUAL_HULL_MAX_RADIUS_MM = 90;
    const VISUAL_HULL_MIN_SUPPORTS = 12;
    const VISUAL_HULL_EDGE_STEP_MM = 2.0;
    const VISUAL_HULL_Z_STEP_MM = 6.0;
    const SHAPE_FIT_MIN_SUPPORTS = 60;
    const BOX_FIT_MAX_ERR = 4.0;
    const ELLIPSE_MIN_CIRCULARITY = 0.80;
    const ELLIPSE_MAX_ASPECT = 3.0;
    const NO_BG_SUPPORT_QUANTILES = [0.30, 0.40, 0.50, 0.60, 0.70];
    const NO_BG_DISPLAY_QUANTILE = 0.50;
    const NO_BG_UNION_MIN_SUPPORTS = 60;
    const NO_BG_UNION_PHI_STEP_DEG = 3;
    const NO_BG_UNION_HUBER_MM = 5;
    const QUALITY_MAX_SUPPORT_GAP_DEG = 120;
    const QUALITY_MIN_ANGULAR_COVERAGE_PCT = 45;
    const QUALITY_MAX_EDGE_RATIO = 0.72;
    const QUALITY_MIN_CENTER_RATIO = 0.12;

    const grid = document.getElementById('grid');
    const plot = document.getElementById('plot');
    const ctx = plot.getContext('2d');
    const cells = [];
    let modeling = false;
    let lastFrame = -1;
    let lastServerFrame = 0;
    let latestServerFrame = 0;
    let currentAngleDeg = 0;
    let rebuildNeeded = false;
    let lastSurfaceCount = 0;
    let yaw = -0.72;
    let pitch = 0.72;
    let zoom = 2.35;
    let drag = null;
    let calibrationMode = 'none';
    let backgroundReady = false;
    let backgroundFrameCount = 0;
    let backgroundSampleCount = 0;
    let backgroundBins = [];

    for (let i = 0; i < 64; i++) {
      const cell = document.createElement('div');
      cell.className = 'cell';
      cell.textContent = '--';
      grid.appendChild(cell);
      cells.push(cell);
    }

    function vnorm(v) {
      const n = Math.hypot(v.x, v.y, v.z) || 1;
      return { x: v.x / n, y: v.y / n, z: v.z / n };
    }
    function degToRad(deg) {
      return (Number(deg) || 0) * Math.PI / 180;
    }
    function rotX(v, a) {
      const c = Math.cos(a), s = Math.sin(a);
      return { x: v.x, y: c * v.y - s * v.z, z: s * v.y + c * v.z };
    }
    function rotY(v, a) {
      const c = Math.cos(a), s = Math.sin(a);
      return { x: c * v.x + s * v.z, y: v.y, z: -s * v.x + c * v.z };
    }
    function rotZ(v, a) {
      const c = Math.cos(a), s = Math.sin(a);
      return { x: c * v.x - s * v.y, y: s * v.x + c * v.y, z: v.z };
    }
    function zoneRowCol(zone, cal = null) {
      const baseRow = Math.floor(zone / 8);
      const baseCol = zone % 8;
      const map = (cal && cal.zoneMap) || 'exported';
      if (map === 'sensor') return { row: 7 - baseCol, col: 7 - baseRow };
      if (map === 'horizontal_cw') return { row: 7 - baseRow, col: baseCol };
      if (map === 'horizontal_ccw') return { row: baseRow, col: 7 - baseCol };
      if (map === 'flip_x') return { row: baseRow, col: 7 - baseCol };
      if (map === 'flip_y') return { row: 7 - baseRow, col: baseCol };
      if (map === 'rot180') return { row: 7 - baseRow, col: 7 - baseCol };
      if (map === 'transpose') return { row: baseCol, col: baseRow };
      return { row: baseRow, col: baseCol };
    }
    function rayForZone(zone, cal = null) {
      const rc = zoneRowCol(zone, cal);
      const yawOff = ((rc.col + 0.5) / 8 - 0.5) * HFOV_DEG * Math.PI / 180;
      const pitchOff = (0.5 - (rc.row + 0.5) / 8) * VFOV_DEG * Math.PI / 180;
      let ray = {
        x: Math.sin(yawOff) * Math.cos(pitchOff),
        y: -Math.cos(yawOff) * Math.cos(pitchOff),
        z: Math.sin(pitchOff)
      };
      if (cal) {
        ray = rotX(ray, degToRad(cal.sensorPitchDeg));
        ray = rotY(ray, degToRad(cal.sensorRollDeg));
        ray = rotZ(ray, degToRad(cal.sensorYawDeg));
      }
      ray = vnorm(ray);
      return { x: ray.x, y: ray.y, z: ray.z, row: rc.row, col: rc.col };
    }
    function sensorForwardAxis(cal = null) {
      let axis = { x: 0, y: -1, z: 0 };
      if (cal) {
        axis = rotX(axis, degToRad(cal.sensorPitchDeg));
        axis = rotY(axis, degToRad(cal.sensorRollDeg));
        axis = rotZ(axis, degToRad(cal.sensorYawDeg));
      }
      return vnorm(axis);
    }
    function sensorOrigin(cal) {
      return {
        x: Number(cal && cal.offx) || 0,
        y: (Number(cal && cal.centerD) || SENSOR_Y) + (Number(cal && cal.offy) || 0),
        z: SENSOR_Z + (Number(cal && cal.offz) || 0)
      };
    }
    function localPointFromDistance(zone, distance, cal) {
      const ray = rayForZone(zone, cal);
      const origin = sensorOrigin(cal);
      let scale = distance;
      if (!cal || (cal.distanceModel || 'perpendicular') === 'perpendicular') {
        const axis = sensorForwardAxis(cal);
        const axial = ray.x * axis.x + ray.y * axis.y + ray.z * axis.z;
        scale = distance / Math.max(0.12, axial);
      }
      return {
        xLocal: origin.x + ray.x * scale,
        yLocal: origin.y + ray.y * scale,
        zLocal: origin.z + ray.z * scale,
        row: ray.row,
        col: ray.col,
        zone,
        projectionScale: scale
      };
    }
    function colorForDistance(mm) {
      if (!mm || mm < 30 || mm > 1500) return '#e5e2da';
      const t = Math.max(0, Math.min(1, (mm - 40) / 700));
      return `hsl(${10 + t * 215}, 76%, 58%)`;
    }
    function colorForZ(z) {
      const t = Math.max(0, Math.min(1, (Number(z) - 20) / 75));
      return `hsl(${255 - t * 205}, 82%, ${38 + t * 12}%)`;
    }
    function validDistance(v) {
      return v && v > 30 && v < 1500;
    }
    function createBackgroundBins() {
      return Array.from({ length: BACKGROUND_BINS }, () =>
        Array.from({ length: 64 }, () => ({ n: 0, mean: 0, m2: 0, min: Infinity, max: 0 }))
      );
    }
    backgroundBins = createBackgroundBins();
    function bgBinForAngle(angle) {
      return Math.floor((((Number(angle) || 0) % 360 + 360) % 360) / (360 / BACKGROUND_BINS)) % BACKGROUND_BINS;
    }
    function resetBackgroundCalibration() {
      backgroundBins = createBackgroundBins();
      backgroundReady = false;
      backgroundFrameCount = 0;
      backgroundSampleCount = 0;
    }
    function addBackgroundSample(bin, zone, distance) {
      const s = backgroundBins[bin][zone];
      s.n++;
      const delta = distance - s.mean;
      s.mean += delta / s.n;
      s.m2 += delta * (distance - s.mean);
      s.min = Math.min(s.min, distance);
      s.max = Math.max(s.max, distance);
      backgroundSampleCount++;
    }
    function addBackgroundFrame(frame) {
      if (!frame || !Array.isArray(frame.dist)) return 0;
      const bin = bgBinForAngle(frame.angle);
      let added = 0;
      for (let i = 0; i < 64; i++) {
        const d = Number(frame.dist[i]) || 0;
        if (!(d > 0 && d < BACKGROUND_VALID_MAX_MM)) continue;
        addBackgroundSample(bin, i, d);
        added++;
      }
      backgroundFrameCount++;
      return added;
    }
    function combinedBackgroundStat(angle, zone) {
      let n = 0;
      let sum = 0;
      let sumSq = 0;
      const centerBin = bgBinForAngle(angle);
      for (let dBin = -BACKGROUND_NEIGHBOR_BINS; dBin <= BACKGROUND_NEIGHBOR_BINS; dBin++) {
        const stat = backgroundBins[(centerBin + dBin + BACKGROUND_BINS) % BACKGROUND_BINS][zone];
        if (!stat || !stat.n) continue;
        n += stat.n;
        sum += stat.mean * stat.n;
        sumSq += stat.m2 + stat.mean * stat.mean * stat.n;
      }
      if (!n) return null;
      const mean = sum / n;
      const variance = Math.max(0, sumSq / n - mean * mean);
      return { n, mean, std: Math.sqrt(variance) };
    }
    function isForegroundMeasurement(frame, zone, distance, cal) {
      if (!backgroundReady) return true;
      const bg = combinedBackgroundStat(frame && frame.angle, zone);
      if (!bg || bg.n < BACKGROUND_MIN_SAMPLES) {
        return Array.isArray(frame && frame.status) && usableTargetStatus(frame.status[zone]);
      }
      const margin = Math.max(BACKGROUND_FOREGROUND_MARGIN_MM, bg.std * 2 + 4);
      return distance < bg.mean - margin;
    }
    function backgroundSummary() {
      let populated = 0;
      let stable = 0;
      for (let b = 0; b < BACKGROUND_BINS; b++) {
        for (let z = 0; z < 64; z++) {
          const s = backgroundBins[b][z];
          if (!s.n) continue;
          populated++;
          const std = s.n > 1 ? Math.sqrt(s.m2 / Math.max(1, s.n - 1)) : 0;
          if (s.n >= BACKGROUND_MIN_SAMPLES && std <= BACKGROUND_FOREGROUND_MARGIN_MM) stable++;
        }
      }
      return { ready: backgroundReady, frames: backgroundFrameCount, samples: backgroundSampleCount, populated, stable };
    }
    function backgroundCalibrationModel() {
      const entries = [];
      for (let b = 0; b < BACKGROUND_BINS; b++) {
        for (let z = 0; z < 64; z++) {
          const s = backgroundBins[b][z];
          if (!s.n) continue;
          const std = s.n > 1 ? Math.sqrt(s.m2 / Math.max(1, s.n - 1)) : 0;
          entries.push({
            bin: b,
            zone: z,
            n: s.n,
            mean: numRound(s.mean, 2),
            std: numRound(std, 2),
            min: Number.isFinite(s.min) ? numRound(s.min, 1) : null,
            max: Number.isFinite(s.max) ? numRound(s.max, 1) : null
          });
        }
      }
      return {
        schema: 'background_model_v1',
        angle_bins: BACKGROUND_BINS,
        min_samples: BACKGROUND_MIN_SAMPLES,
        foreground_margin_mm: BACKGROUND_FOREGROUND_MARGIN_MM,
        ready: backgroundReady,
        frames: backgroundFrameCount,
        samples: backgroundSampleCount,
        entries
      };
    }
    function finishBackgroundCalibration() {
      calibrationMode = 'none';
      modeling = false;
      document.getElementById('startBtn').classList.remove('active');
      const summary = backgroundSummary();
      backgroundReady = backgroundFrameCount >= 20 && backgroundSampleCount >= 100;
      debugLog('background_calibration_done', Object.assign(backgroundSummary(), { min_samples: BACKGROUND_MIN_SAMPLES, foreground_margin_mm: BACKGROUND_FOREGROUND_MARGIN_MM }));
      document.getElementById('msg').textContent = backgroundReady
        ? `空掃校正完成：frames=${backgroundFrameCount}, samples=${backgroundSampleCount}, populated=${summary.populated}`
        : `空掃校正不足：frames=${backgroundFrameCount}, samples=${backgroundSampleCount}`;
    }
    async function calibrateBackground() {
      clearModel();
      resetBackgroundCalibration();
      calibrationMode = 'background';
      modeling = true;
      beginRawScanSession('background_empty_360');
      document.getElementById('startBtn').classList.add('active');
      debugLog('cmd_background_calibration', { bins: BACKGROUND_BINS, min_samples: BACKGROUND_MIN_SAMPLES, margin_mm: BACKGROUND_FOREGROUND_MARGIN_MM });
      document.getElementById('msg').textContent = '空掃校正中：請保持轉盤上沒有物體。';
      await cmd('/one_rev');
    }
    function clearCalibration() {
      resetBackgroundCalibration();
      calibrationMode = 'none';
      debugLog('background_calibration_clear');
      document.getElementById('msg').textContent = '背景校正已清除；建議先空掃校正一圈。';
      updatePointStats();
      drawPointCloud();
    }
    function isModelZone(zone) {
      if (backgroundReady) return true;
      return zone < 56;
    }
    function calValidDistance(v, cal) {
      const minD = Number(cal && cal.minDist) || 20;
      const maxD = Number(cal && cal.maxDist) || 165;
      return v && v > minD && v < maxD;
    }
    function countModelObjectZones(frame, cal) {
      if (!frame || !Array.isArray(frame.dist)) return 0;
      let good = 0;
      const hasStatus = Array.isArray(frame.status);
      for (let i = 0; i < 64; i++) {
        if (!isModelZone(i)) continue;
        if (!calValidDistance(frame.dist[i], cal)) continue;
        if (!backgroundReady && hasStatus && !usableTargetStatus(frame.status[i])) continue;
        if (!isForegroundMeasurement(frame, i, frame.dist[i], cal)) continue;
        good++;
      }
      return good;
    }
    async function cmd(path) {
      try {
        const r = await fetch(path, { cache: 'no-store' });
        const data = await r.json();
        if (typeof data.scan_frame === 'number') {
          latestServerFrame = data.scan_frame;
          lastServerFrame = data.scan_frame;
        }
      } catch (e) {}
      await update();
    }
    async function setSpeed() {
      await cmd('/speed?us=' + encodeURIComponent(document.getElementById('speed').value));
    }
    function getCal() {
      return {
        mode: document.getElementById('mode').value,
        dir: Number(document.getElementById('dir').value),
        angleScale: Number(document.getElementById('angscale').value) || 1,
        distanceModel: document.getElementById('distmodel').value || 'perpendicular',
        centerD: Number(document.getElementById('centerd').value) || SENSOR_Y,
        minDist: Number(document.getElementById('mindist').value) || 20,
        maxDist: Number(document.getElementById('maxdist').value) || 165,
        zoneMap: document.getElementById('zonemap').value || 'exported',
        offx: Number(document.getElementById('offx').value) || 0,
        offy: Number(document.getElementById('offy').value) || 0,
        offz: Number(document.getElementById('offz').value) || 0,
        sensorYawDeg: Number(document.getElementById('sensyaw').value) || 0,
        sensorPitchDeg: Number(document.getElementById('senspitch').value) || 0,
        sensorRollDeg: Number(document.getElementById('sensroll').value) || 0
      };
    }
    function setRebuildNeeded() {
      rebuildNeeded = true;
      document.getElementById('msg').textContent = '幾何參數已改，可按「用 Raw 重算」用目前 raw frame 重投影。';
    }
    function addFrameWithGapFill(frame) {
      addFramePoints(frame);
    }
    function usableTargetStatus(status) {
      return status === 5 || status === 6 || status === 9;
    }
    function project(p) {
      const cy = Math.cos(yaw), sy = Math.sin(yaw);
      const cp = Math.cos(pitch), sp = Math.sin(pitch);
      const x1 = p.x * cy - p.y * sy;
      const y1 = p.x * sy + p.y * cy;
      const z1 = p.z - 35;
      const y2 = y1 * cp - z1 * sp;
      const z2 = y1 * sp + z1 * cp;
      return { x: plot.width / 2 + x1 * zoom, y: plot.height / 2 - z2 * zoom, depth: y2 };
    }
    function drawAxes() {
      const base = [];
      for (let a = 0; a < Math.PI * 2; a += Math.PI / 48) base.push({ x: Math.cos(a) * 60, y: Math.sin(a) * 60, z: 0 });
      ctx.strokeStyle = '#d7d4ca';
      ctx.lineWidth = 1;
      ctx.beginPath();
      base.forEach((p, i) => {
        const q = project(p);
        if (i === 0) ctx.moveTo(q.x, q.y); else ctx.lineTo(q.x, q.y);
      });
      ctx.closePath();
      ctx.stroke();

      const axes = [
        [{ x: -80, y: 0, z: 0 }, { x: 80, y: 0, z: 0 }, '#b84235'],
        [{ x: 0, y: -80, z: 0 }, { x: 0, y: 80, z: 0 }, '#1f7a5a'],
        [{ x: 0, y: 0, z: 0 }, { x: 0, y: 0, z: 110 }, '#2d6f9f']
      ];
      for (const [a, b, c] of axes) {
        const pa = project(a), pb = project(b);
        ctx.strokeStyle = c;
        ctx.beginPath();
        ctx.moveTo(pa.x, pa.y);
        ctx.lineTo(pb.x, pb.y);
        ctx.stroke();
      }
    }
    function download(name, text, type) {
      const a = document.createElement('a');
      a.href = URL.createObjectURL(new Blob([text], { type }));
      a.download = name;
      a.click();
      URL.revokeObjectURL(a.href);
    }
    async function update() {
      try {
        const r = await fetch('/frames?after=' + encodeURIComponent(lastServerFrame), { cache: 'no-store' });
        const data = await r.json();
        const sensorEl = document.getElementById('sensor');
        sensorEl.textContent = data.sensor ? 'OK' : 'ERROR';
        sensorEl.title = data.sensor_msg || '';
        if (!rebuildNeeded) {
          const zeroText = lfZeroAngle === null ? '--' : lfZeroAngle.toFixed(1);
          const modelZones = countModelObjectZones(data, getCal());
          const qText = lfSupports.size ? ` | ${lfQualityText(lfQuality || lfScanQuality())}` : '';
          document.getElementById('msg').textContent = `${data.sensor_msg || ''} | server object zones=${data.object_zones || 0} | model zones=${modelZones} | surface zones=${lastSurfaceCount} | model zero=${zeroText} deg${qText}`;
        }
        const pinionAngle = Number.isFinite(Number(data.pinion_angle)) ? Number(data.pinion_angle) : 0;
        document.getElementById('angle').textContent = data.angle.toFixed(1) + ' deg / pinion ' + pinionAngle.toFixed(1) + ' deg';
        currentAngleDeg = data.angle;
        document.getElementById('speed').value = data.step_us;
        if (typeof data.scan_frame === 'number') latestServerFrame = data.scan_frame;

        let min = 99999;
        for (let i = 0; i < 64; i++) {
          const v = data.dist[i] || 0;
          cells[i].textContent = validDistance(v) ? v : '--';
          cells[i].style.background = colorForDistance(v);
          if (validDistance(v)) min = Math.min(min, v);
        }
        document.getElementById('min').textContent = min < 99999 ? min + ' mm' : '--';
        if (Array.isArray(data.frames) && data.frames.length) {
          for (const frame of data.frames) {
            recordRawFrame(frame, data);
            if (modeling) addFrameWithGapFill(frame);
            lastServerFrame = Math.max(lastServerFrame, frame.id);
            lastFrame = data.frame;
          }
          const now = performance.now();
          if (now - lastBatchLogMs > 1000 || !data.running) {
            lastBatchLogMs = now;
            debugLog('server_batch', {
              frames: data.frames.length,
              scan_frame: data.scan_frame,
              angle: round2(data.angle),
              pinion: round2(data.pinion_angle),
              running: !!data.running,
              object_zones: data.object_zones || 0,
              nearest_mm: data.nearest_mm || 0,
              step_us: data.step_us,
              one_rev_steps: data.one_rev_steps
            });
          }
        } else if (data.frame !== lastFrame) {
          lastFrame = data.frame;
        }
        if (calibrationMode === 'background') {
          const summary = backgroundSummary();
          document.getElementById('msg').textContent = `空掃校正中：frames=${summary.frames}, samples=${summary.samples}, populated=${summary.populated}`;
          if (!data.running && backgroundFrameCount > 0) finishBackgroundCalibration();
        }
        drawPointCloud();
      } catch (e) {
        document.getElementById('sensor').textContent = 'OFFLINE';
      }
    }
    plot.addEventListener('pointerdown', e => { drag = { x: e.clientX, y: e.clientY, yaw, pitch }; plot.setPointerCapture(e.pointerId); });
    plot.addEventListener('pointermove', e => {
      if (!drag) return;
      yaw = drag.yaw + (e.clientX - drag.x) * 0.01;
      pitch = Math.max(-1.2, Math.min(1.2, drag.pitch + (e.clientY - drag.y) * 0.01));
      drawPointCloud();
    });
    plot.addEventListener('pointerup', () => { drag = null; });
    plot.addEventListener('wheel', e => {
      e.preventDefault();
      zoom = Math.max(0.8, Math.min(8, zoom * (e.deltaY > 0 ? 0.9 : 1.1)));
      drawPointCloud();
    }, { passive: false });

    // Calibration-first model: raw frames are kept for debug/replay; visible
    // geometry is rebuilt from foreground support lines in object coordinates.
    const LF_CENTER_D_MM = 50;
    const LF_MAX_DIST_MM = 165;
    const LF_MIN_DIST_MM = 20;
    const LF_RADIUS_MM = 75;
    const LF_Z_FLOOR_MM = 18;
    const LF_Z_CEIL_MM = 150;
    const LF_SUPPORT_BIN_DEG = 3;
    const LF_REBUILD_MS = 450;
    const LF_DEFAULT_Z0 = 18;
    const LF_DEFAULT_Z1 = 135;
    const LF_MIN_FALLBACK_PTS = 3;
    const LF_MIN_OBJECT_ZONES = 5;
    const EXPECTED_BOX_SHORT_MM = 28;
    const EXPECTED_BOX_LONG_MM = 43;
    const RAW_SCAN_LOG_LIMIT = 20000;

    let lfSupports = new Map();
    let lfOutline = [];
    let lfFit = null;
    let visualHullOutline = [];
    let visualHullPoints = [];
    let surfaceShapeKind = 'none';
    let lfZeroAngle = null;
    let lfZMin = Infinity;
    let lfZMax = -Infinity;
    let lfFrameCount = 0;
    let lfDirty = false;
    let lfLastRebuildMs = 0;
    let lfTrace = [];
    let lfCornerMarks = [];
    let debugSeq = 0;
    let debugLines = [];
    let rawFrameLines = [];
    let supportFrameLines = [];
    let rawFrameSeq = 0;
    let rawScanSessionId = 0;
    let rawScanMode = 'page';
    let rawFrameDropped = 0;
    let supportFrameDropped = 0;
    let lastBatchLogMs = 0;
    let lastNoHitLogMs = 0;
    let lastFitLogMs = 0;
    let rawSurfacePoints = [];
    let rawSurfaceDropped = 0;
    let lfFgCellCounts = Array(64).fill(0);
    let lfFgTotal = 0;
    let lfFgEdge = 0;
    let lfFgCenter = 0;
    let lfQuality = null;

    function debugLog(event, data = null) {
      let payload = '';
      if (data !== null) {
        try {
          payload = ' ' + JSON.stringify(data);
        } catch (e) {
          payload = ' [unserializable]';
        }
      }
      const line = `${String(debugSeq++).padStart(4, '0')} ${(performance.now() / 1000).toFixed(2)}s ${event}${payload}`;
      debugLines.push(line);
      if (debugLines.length > 900) debugLines.splice(0, debugLines.length - 900);
      const el = document.getElementById('debuglog');
      if (el) {
        el.textContent = debugLines.slice(-140).join('\n');
        el.scrollTop = el.scrollHeight;
      }
      console.log('[vl53-scan-debug]', event, data || '');
    }
    function exportDebugLog() {
      const lines = [];
      lines.push('raw_meta ' + JSON.stringify(rawScanMeta()));
      lines.push('');
      lines.push('--- debug_summary ---');
      lines.push(...debugLines);
      lines.push('');
      lines.push('--- support_frame_jsonl ---');
      lines.push(...supportFrameLines);
      lines.push('');
      lines.push('--- background_calibration_json ---');
      lines.push('background_model ' + JSON.stringify(backgroundCalibrationModel()));
      lines.push('');
      lines.push('--- raw_frame_jsonl ---');
      lines.push(...rawFrameLines);
      download('vl53l7cx_debug_bundle.txt', lines.join('\n') + '\n', 'text/plain');
    }
    function exportRawScanLog() {
      const lines = ['raw_meta ' + JSON.stringify(rawScanMeta()), ...rawFrameLines];
      download('vl53l7cx_raw_scan.jsonl', lines.join('\n') + '\n', 'application/x-ndjson');
    }
    function frameFromRawPayload(payload) {
      return {
        id: Number(payload.frame ?? payload.seq) || null,
        angle: Number(payload.server_angle) || 0,
        pinion_angle: Number(payload.pinion_angle) || 0,
        turntable_step: Number(payload.turntable_step) || 0,
        dist: copy64(payload.dist),
        status: copy64(payload.status)
      };
    }
    function replayRawScan() {
      const savedRaw = rawFrameLines.slice();
      if (!savedRaw.length) {
        document.getElementById('msg').textContent = '目前沒有 raw frame 可重算。';
        return;
      }
      const wasModeling = modeling;
      modeling = false;
      clearModel();
      rawFrameLines = savedRaw;
      supportFrameLines = [];
      supportFrameDropped = 0;
      for (const line of savedRaw) {
        if (!line.startsWith('raw_frame ')) continue;
        try {
          const payload = JSON.parse(line.slice('raw_frame '.length));
          addFramePoints(frameFromRawPayload(payload));
        } catch (e) {
          debugLog('raw_replay_parse_error', { message: String(e && e.message || e) });
        }
      }
      lfRequestRebuild(true);
      modeling = wasModeling;
      updatePointStats();
      drawPointCloud();
      const quality = lfQuality || lfScanQuality();
      debugLog('raw_replay_done', { frames: savedRaw.length, raw_surface_points: rawSurfacePoints.length, supports: lfSupports.size, hull_vertices: visualHullOutline.length, hull_points: visualHullPoints.length, quality, background: backgroundSummary() });
      document.getElementById('msg').textContent = `Raw 重算完成：frames=${savedRaw.length}, raw=${rawSurfacePoints.length}, supports=${lfSupports.size}, hull=${visualHullOutline.length} vertices, ${lfQualityText(quality)}`;
    }
    function round1(v) {
      return Number.isFinite(Number(v)) ? Number(v).toFixed(1) : null;
    }
    function round2(v) {
      return Number.isFinite(Number(v)) ? Number(v).toFixed(2) : null;
    }
    function numRound(v, digits = 2) {
      const n = Number(v);
      return Number.isFinite(n) ? Number(n.toFixed(digits)) : null;
    }
    function copy64(src) {
      const out = [];
      for (let i = 0; i < 64; i++) out.push(Number(src && src[i]) || 0);
      return out;
    }
    function rawScanMeta() {
      const cal = lfCal();
      return {
        schema: 'vl53l7cx_debug_bundle_v2',
        raw_frame_schema: 'raw_frame_v1',
        support_frame_schema: 'support_frame_v1',
        raw_frames: rawFrameLines.length,
        raw_frames_dropped: rawFrameDropped,
        support_frames: supportFrameLines.length,
        support_frames_dropped: supportFrameDropped,
        zone_order: 'raw payload is unchanged 0..63. Geometry mapping is current_cal.zoneMap. horizontal_cw is default for the sensor mounted sideways: row=7-floor(i/8), col=i%8. horizontal_ccw: row=floor(i/8), col=7-(i%8). sensor: row=7-(i%8), col=7-floor(i/8). exported: row=floor(i/8), col=i%8.',
        constants: {
          sensor_z_mm: SENSOR_Z,
          hfov_deg: HFOV_DEG,
          vfov_deg: VFOV_DEG,
          lf_support_bin_deg: LF_SUPPORT_BIN_DEG,
          lf_z_floor_mm: LF_Z_FLOOR_MM,
          lf_z_ceil_mm: LF_Z_CEIL_MM,
          lf_min_fallback_pts: LF_MIN_FALLBACK_PTS,
          lf_min_object_zones: LF_MIN_OBJECT_ZONES,
          support_sample_quantile: SUPPORT_SAMPLE_QUANTILE,
          visual_hull_max_radius_mm: VISUAL_HULL_MAX_RADIUS_MM,
          shape_fit_min_supports: SHAPE_FIT_MIN_SUPPORTS,
          box_fit_max_err: BOX_FIT_MAX_ERR,
          ellipse_min_circularity: ELLIPSE_MIN_CIRCULARITY,
          ellipse_max_aspect: ELLIPSE_MAX_ASPECT,
          no_bg_support_quantiles: NO_BG_SUPPORT_QUANTILES,
          no_bg_display_quantile: NO_BG_DISPLAY_QUANTILE,
          no_bg_union_min_supports: NO_BG_UNION_MIN_SUPPORTS,
          quality_max_support_gap_deg: QUALITY_MAX_SUPPORT_GAP_DEG,
          quality_min_angular_coverage_pct: QUALITY_MIN_ANGULAR_COVERAGE_PCT,
          quality_max_edge_ratio: QUALITY_MAX_EDGE_RATIO,
          quality_min_center_ratio: QUALITY_MIN_CENTER_RATIO
        },
        point_cloud: {
          raw_surface_points: rawSurfacePoints.length,
          raw_surface_dropped: rawSurfaceDropped,
          raw_surface_limit: RAW_SURFACE_POINT_LIMIT,
          distance_model: cal.distanceModel
        },
        physical_setup: {
          measured_center_distance_mm: SENSOR_Y,
          nearest_surface_if_43mm_box_centered_mm: numRound(SENSOR_Y - EXPECTED_BOX_LONG_MM / 2, 1),
          note: 'If foreground stays on edge cells or support coverage is partial, move/aim sensor before trusting shape fit.'
        },
        expected_box_mm: {
          short: EXPECTED_BOX_SHORT_MM,
          long: EXPECTED_BOX_LONG_MM
        },
        session_id: rawScanSessionId,
        session_mode: rawScanMode,
        current_cal: cal,
        scan_quality: lfScanQuality(),
        background_calibration: backgroundSummary(),
        lf_zero_angle: lfZeroAngle === null ? null : numRound(lfZeroAngle),
        current_angle: numRound(currentAngleDeg),
        raw_scan_log_limit: RAW_SCAN_LOG_LIMIT
      };
    }
    function pushRawFrameLine(line) {
      rawFrameLines.push(line);
      if (rawFrameLines.length > RAW_SCAN_LOG_LIMIT) {
        rawFrameLines.shift();
        rawFrameDropped++;
      }
    }
    function pushSupportFrameLine(line) {
      supportFrameLines.push(line);
      if (supportFrameLines.length > RAW_SCAN_LOG_LIMIT) {
        supportFrameLines.shift();
        supportFrameDropped++;
      }
    }
    function recordRawFrame(frame, batch) {
      if (!frame || !Array.isArray(frame.dist)) return;
      const payload = {
        schema: 'raw_frame_v1',
        seq: rawFrameSeq++,
        session_id: rawScanSessionId,
        session_mode: rawScanMode,
        frame: Number(frame.id) || null,
        scan_frame: Number(batch.scan_frame) || null,
        server_angle: numRound(frame.angle),
        pinion_angle: numRound(frame.pinion_angle ?? batch.pinion_angle),
        turntable_step: Number(frame.turntable_step ?? batch.turntable_step) || null,
        one_rev_steps: Number(batch.one_rev_steps) || null,
        running: !!batch.running,
        capture: !!batch.capture,
        step_us: Number(batch.step_us) || 0,
        dist: copy64(frame.dist),
        status: copy64(frame.status)
      };
      pushRawFrameLine('raw_frame ' + JSON.stringify(payload));
    }
    function recordSupportFrame(event, data) {
      const payload = Object.assign({ schema: 'support_frame_v1', event }, data || {});
      payload.session_id = rawScanSessionId;
      payload.session_mode = rawScanMode;
      pushSupportFrameLine('support_frame ' + JSON.stringify(payload));
    }
    function beginRawScanSession(mode) {
      rawScanSessionId++;
      rawScanMode = mode || 'scan';
      rawFrameLines = [];
      supportFrameLines = [];
      rawFrameSeq = 0;
      rawFrameDropped = 0;
      supportFrameDropped = 0;
      debugSeq = 0;
      debugLines = [];
      debugLog('raw_session_start', rawScanMeta());
    }

    function lfNum(id, fallback) {
      const el = document.getElementById(id);
      const v = el ? Number(el.value) : NaN;
      return Number.isFinite(v) ? v : fallback;
    }
    function lfCal() {
      const scale = lfNum('angscale', 1);
      return {
        dir: lfNum('dir', -1),
        angleScale: scale > 0 ? scale : 1,
        distanceModel: (document.getElementById('distmodel') && document.getElementById('distmodel').value) || 'perpendicular',
        centerD: lfNum('centerd', LF_CENTER_D_MM),
        minDist: lfNum('mindist', LF_MIN_DIST_MM),
        maxDist: lfNum('maxdist', LF_MAX_DIST_MM),
        zoneMap: (document.getElementById('zonemap') && document.getElementById('zonemap').value) || 'exported',
        offx: lfNum('offx', 0),
        offy: lfNum('offy', 0),
        offz: lfNum('offz', 0),
        sensorYawDeg: lfNum('sensyaw', 0),
        sensorPitchDeg: lfNum('senspitch', 0),
        sensorRollDeg: lfNum('sensroll', 0),
        radius: LF_RADIUS_MM,
        zfloor: LF_Z_FLOOR_MM,
        zceil: LF_Z_CEIL_MM
      };
    }
    function lfResetQualityCounters() {
      lfFgCellCounts = Array(64).fill(0);
      lfFgTotal = 0;
      lfFgEdge = 0;
      lfFgCenter = 0;
      lfQuality = null;
    }
    function lfRecordForegroundZones(points) {
      if (!Array.isArray(points)) return;
      for (const p of points) {
        if (!p || !Number.isFinite(p.row) || !Number.isFinite(p.col)) continue;
        const row = Math.max(0, Math.min(7, Math.round(p.row)));
        const col = Math.max(0, Math.min(7, Math.round(p.col)));
        lfFgCellCounts[row * 8 + col]++;
        lfFgTotal++;
        if (row === 0 || row === 7 || col === 0 || col === 7) lfFgEdge++;
        if (row >= 2 && row <= 5 && col >= 2 && col <= 5) lfFgCenter++;
      }
    }
    function lfLargestSupportGapDeg() {
      const angles = [...lfSupports.values()]
        .filter(s => Number.isFinite(s.angle))
        .map(s => lfNormDeg(s.angle))
        .sort((a, b) => a - b);
      if (angles.length < 2) return 360;
      let largest = 0;
      for (let i = 0; i < angles.length; i++) {
        const a = angles[i];
        const b = i === angles.length - 1 ? angles[0] + 360 : angles[i + 1];
        largest = Math.max(largest, b - a);
      }
      return largest;
    }
    function lfScanQuality() {
      const supports = lfSupports.size;
      const largestGap = lfLargestSupportGapDeg();
      const angularCoverage = supports >= 2 ? Math.max(0, Math.min(100, Math.round((360 - largestGap) * 100 / 360))) : 0;
      const edgeRatio = lfFgTotal ? lfFgEdge / lfFgTotal : 0;
      const centerRatio = lfFgTotal ? lfFgCenter / lfFgTotal : 0;
      const activeCells = lfFgCellCounts.filter(v => v > 0).length;
      let status = 'no-data';
      let reason = 'no foreground';
      if (supports > 0 || lfFgTotal > 0) {
        status = 'partial';
        reason = 'insufficient angular support';
        if (!backgroundReady) {
          status = supports >= NO_BG_UNION_MIN_SUPPORTS ? 'no-bg-envelope' : 'raw-only';
          reason = supports >= NO_BG_UNION_MIN_SUPPORTS
            ? 'estimated from raw hits with rectangle-plus-round-clutter model'
            : 'no background calibration; raw hits are diagnostic only';
        } else if (supports < VISUAL_HULL_MIN_SUPPORTS) {
          status = 'no-hull';
          reason = 'too few support angles';
        } else if (edgeRatio > QUALITY_MAX_EDGE_RATIO && centerRatio < QUALITY_MIN_CENTER_RATIO) {
          status = 'needs-geometry';
          reason = 'object is mostly on sensor edge zones';
        } else if (supports < SHAPE_FIT_MIN_SUPPORTS || angularCoverage < QUALITY_MIN_ANGULAR_COVERAGE_PCT || largestGap > QUALITY_MAX_SUPPORT_GAP_DEG) {
          status = 'partial';
          reason = 'angle coverage not enough';
        } else {
          status = 'good';
          reason = 'enough angular and zone coverage';
        }
      }
      return {
        status,
        reason,
        supports,
        angular_coverage_pct: angularCoverage,
        largest_gap_deg: numRound(largestGap, 1),
        foreground_points: lfFgTotal,
        active_cells: activeCells,
        edge_ratio: numRound(edgeRatio, 3),
        center_ratio: numRound(centerRatio, 3),
        background_ready: backgroundReady,
        estimated_fit_allowed: !backgroundReady && supports >= NO_BG_UNION_MIN_SUPPORTS,
        fit_allowed: backgroundReady && status === 'good'
      };
    }
    function lfQualityText(q = lfScanQuality()) {
      if (!q || q.status === 'no-data') return '--';
      return `${q.status} cover=${q.angular_coverage_pct}% gap=${q.largest_gap_deg}° edge=${Math.round((q.edge_ratio || 0) * 100)}%`;
    }
    function lfValidDistance(d) {
      const cal = lfCal();
      return d && d > cal.minDist && d < cal.maxDist;
    }
    function lfUsableStatus(s) {
      return s === 5 || s === 6 || s === 9;
    }
    function lfNorm(v) {
      const n = Math.hypot(v.x, v.y, v.z) || 1;
      return { x: v.x / n, y: v.y / n, z: v.z / n };
    }
    function lfRotate2(x, y, a) {
      const c = Math.cos(a), s = Math.sin(a);
      return { x: c * x - s * y, y: s * x + c * y };
    }
    function lfClockwiseDeltaDeg(from, to) {
      return ((to - from + 360) % 360);
    }
    function lfFixedAngleDeg(frameAngle, cal) {
      if (lfZeroAngle === null) lfZeroAngle = frameAngle;
      return cal.dir * lfClockwiseDeltaDeg(lfZeroAngle, frameAngle) * cal.angleScale;
    }
    function lfFixedAngleRad(frameAngle, cal) {
      return lfFixedAngleDeg(frameAngle, cal) * Math.PI / 180;
    }
    function lfNormDeg(a) {
      return ((a % 360) + 360) % 360;
    }
    function lfRayDirFromZone(row, col) {
      const h = ((col + 0.5) / 8 - 0.5) * HFOV_DEG * Math.PI / 180;
      const v = (0.5 - (row + 0.5) / 8) * VFOV_DEG * Math.PI / 180;
      return lfNorm({
        x: Math.sin(h) * Math.cos(v),
        y: -Math.cos(h) * Math.cos(v),
        z: Math.sin(v)
      });
    }
    function lfLabPoint(frame, zone, cal) {
      const d = frame.dist[zone] || 0;
      if (!isModelZone(zone)) return null;
      if (!lfValidDistance(d)) return null;
      if (!isForegroundMeasurement(frame, zone, d, cal)) return null;
      const p = localPointFromDistance(zone, d, cal);
      const hasStatus = Array.isArray(frame.status);
      const status = hasStatus ? frame.status[zone] : null;
      return {
        x: p.xLocal,
        y: p.yLocal,
        z: p.zLocal,
        d, row: p.row, col: p.col, zone, status,
        statusOk: !hasStatus || lfUsableStatus(status)
      };
    }
    function lfPointInScanBounds(p, cal) {
      return p &&
        p.z >= cal.zfloor && p.z <= cal.zceil &&
        Math.abs(p.x) <= cal.radius + 35 &&
        p.y >= -cal.radius - 25 && p.y <= cal.radius + 25;
    }
    function lfFrameStatusCount(frame) {
      if (!Array.isArray(frame.status)) return null;
      const cal = lfCal();
      let good = 0;
      for (let i = 0; i < 64; i++) {
        if (!isModelZone(i)) continue;
        const d = frame.dist[i] || 0;
        if (!lfValidDistance(d)) continue;
        if (!backgroundReady && !lfUsableStatus(frame.status[i])) continue;
        if (!isForegroundMeasurement(frame, i, d, cal)) continue;
        good++;
      }
      return good;
    }
    function lfFrameHasObjectEvidence(frame) {
      const modelCount = lfFrameStatusCount(frame);
      if (modelCount !== null) return modelCount >= LF_MIN_OBJECT_ZONES;
      if (frame && typeof frame.object === 'boolean') return frame.object && (Number(frame.object_zones) || 0) >= LF_MIN_OBJECT_ZONES;
      return false;
    }
    function lfFrameHits(frame, cal) {
      const hasStatus = Array.isArray(frame.status);
      const pts = [];
      const trusted = [];
      for (let i = 0; i < 64; i++) {
        if (!isModelZone(i)) continue;
        if (!lfValidDistance(frame.dist[i])) continue;
        const p = lfLabPoint(frame, i, cal);
        if (!lfPointInScanBounds(p, cal)) continue;
        pts.push(p);
        if (p.statusOk) trusted.push(p);
      }
      if (backgroundReady) return pts.length >= 2 ? pts : [];
      if (hasStatus && trusted.length >= 2) return trusted;
      if (hasStatus && pts.length < LF_MIN_FALLBACK_PTS) return [];
      if (!hasStatus && pts.length < LF_MIN_FALLBACK_PTS) return [];
      return pts;
    }
    function lfAddRawSurfacePoints(pts, frame, cal) {
      if (!Array.isArray(pts) || pts.length < RAW_SURFACE_MIN_FRAME_POINTS) return;
      const aDeg = lfFixedAngleDeg(frame.angle, cal);
      const aRad = aDeg * Math.PI / 180;
      for (const p of pts) {
        const q = lfRotate2(p.x, p.y, aRad);
        rawSurfacePoints.push({
          x: q.x,
          y: q.y,
          z: p.z,
          mm: p.d,
          angle: lfNormDeg(aDeg),
          zone: p.zone,
          row: p.row,
          col: p.col,
          n: 1,
          free: 0,
          raw: true,
          statusOk: !!p.statusOk
        });
      }
      if (rawSurfacePoints.length > RAW_SURFACE_POINT_LIMIT) {
        const excess = rawSurfacePoints.length - RAW_SURFACE_POINT_LIMIT;
        rawSurfacePoints.splice(0, excess);
        rawSurfaceDropped += excess;
      }
    }
    function lfPercentile(values, q) {
      const a = values.slice().filter(Number.isFinite).sort((x, y) => x - y);
      if (!a.length) return NaN;
      const idx = Math.max(0, Math.min(a.length - 1, Math.round((a.length - 1) * q)));
      return a[idx];
    }
    function lfSupportY(values) {
      const a = values.slice().filter(Number.isFinite).sort((x, y) => x - y);
      if (!a.length) return NaN;
      const keep = a.length >= 5 ? 2 : 1;
      let sum = 0;
      for (let i = 0; i < keep; i++) sum += a[a.length - 1 - i];
      return sum / keep;
    }
    function lfRobustHigh(samples) {
      const a = samples.slice().filter(Number.isFinite).sort((x, y) => x - y);
      if (!a.length) return 0;
      const q = SUPPORT_SAMPLE_QUANTILE;
      return a[Math.max(0, Math.min(a.length - 1, Math.round((a.length - 1) * q)))];
    }
    function lfNoBgKey(q) {
      return Number(q).toFixed(2);
    }
    function lfStoreNoBgQuantileSamples(target, supportByQ) {
      if (!target || !Array.isArray(supportByQ)) return;
      if (!target.noBgSamples) target.noBgSamples = {};
      if (!target.noBgH) target.noBgH = {};
      for (const item of supportByQ) {
        if (!Number.isFinite(item.h)) continue;
        const key = lfNoBgKey(item.q);
        const arr = target.noBgSamples[key] || [];
        arr.push(item.h);
        if (arr.length > 24) arr.shift();
        target.noBgSamples[key] = arr;
        target.noBgH[key] = lfRobustHigh(arr);
      }
    }
    function lfNoBgSupportSetForQuantile(q) {
      const key = lfNoBgKey(q);
      return [...lfSupports.values()]
        .map(s => ({
          angle: s.angle,
          n: s.n,
          count: s.count || 1,
          h: s.noBgH && Number.isFinite(s.noBgH[key]) ? s.noBgH[key] : s.h
        }))
        .filter(s => Number.isFinite(s.h));
    }
    function lfAddSupportFromFrame(frame) {
      const cal = lfCal();
      const pts = lfFrameHits(frame, cal);
      lastSurfaceCount = pts.length;
      const hasObjectEvidence = lfFrameHasObjectEvidence(frame);
      if (pts.length && (hasObjectEvidence || lfZeroAngle !== null)) {
        lfRecordForegroundZones(pts);
        lfAddRawSurfacePoints(pts, frame, cal);
      }
      if (!hasObjectEvidence) {
        recordSupportFrame('skip_no_object', {
          frame: frame.id || null,
          server_angle: numRound(frame.angle),
          object: typeof frame.object === 'boolean' ? frame.object : null,
          object_zones: Number(frame.object_zones) || 0,
          nearest_mm: Number(frame.nearest_mm) || 0,
          status_ok: lfFrameStatusCount(frame),
          distanceModel: cal.distanceModel,
          min_required_zones: LF_MIN_OBJECT_ZONES
        });
        return false;
      }
      if (pts.length < 2) {
        recordSupportFrame('skip_no_surface', {
          frame: frame.id || null,
          server_angle: numRound(frame.angle),
          centerD: cal.centerD,
          distanceModel: cal.distanceModel,
          minDist: cal.minDist,
          maxDist: cal.maxDist,
          valid_raw: Array.isArray(frame.dist) ? frame.dist.filter(lfValidDistance).length : 0,
          status_ok: lfFrameStatusCount(frame)
        });
        const now = performance.now();
        if (now - lastNoHitLogMs > 1000) {
          lastNoHitLogMs = now;
          debugLog('frame_skip_no_surface', {
            id: frame.id || null,
            server_angle: round2(frame.angle),
            centerD: cal.centerD,
            valid_raw: Array.isArray(frame.dist) ? frame.dist.filter(lfValidDistance).length : 0,
            status_ok: lfFrameStatusCount(frame)
          });
        }
        return false;
      }

      let mid = backgroundReady ? pts.filter(p => p.row >= 2 && p.row <= 5) : pts;
      if (mid.length < 2) mid = pts;
      const midYs = mid.map(p => p.y).filter(Number.isFinite);
      const noBgSupportByQ = backgroundReady ? null : NO_BG_SUPPORT_QUANTILES.map(q => ({ q, h: lfPercentile(midYs, q) }));
      const displayNoBg = noBgSupportByQ ? noBgSupportByQ.reduce((best, cur) => Math.abs(cur.q - NO_BG_DISPLAY_QUANTILE) < Math.abs(best.q - NO_BG_DISPLAY_QUANTILE) ? cur : best, noBgSupportByQ[0]) : null;
      const frontY = backgroundReady ? lfSupportY(midYs) : (displayNoBg ? displayNoBg.h : NaN);
      if (!Number.isFinite(frontY)) {
        recordSupportFrame('skip_bad_frontY', {
          frame: frame.id || null,
          server_angle: numRound(frame.angle),
          pts: pts.length,
          mid: mid.length,
          centerD: cal.centerD,
          minDist: cal.minDist,
          maxDist: cal.maxDist
        });
        debugLog('frame_skip_bad_frontY', {
          id: frame.id || null,
          server_angle: round2(frame.angle),
          pts: pts.length,
          mid: mid.length
        });
        return false;
      }

      const surface = pts.filter(p => frontY - p.y <= 18);
      for (const p of surface) {
        lfZMin = Math.min(lfZMin, p.z);
        lfZMax = Math.max(lfZMax, p.z);
      }

      const aRad = lfFixedAngleRad(frame.angle, cal);
      const aDeg = lfFixedAngleDeg(frame.angle, cal);
      const bin = Math.round(aDeg / LF_SUPPORT_BIN_DEG);
      const n = lfRotate2(0, 1, aRad);
      lfTrace.push({
        angle: lfNormDeg(aDeg),
        fixedAngle: aDeg,
        serverAngle: frame.angle,
        h: frontY,
        zones: pts.length
      });
      if (lfTrace.length > 1200) lfTrace.splice(0, lfTrace.length - 1200);
      const ds = pts.map(p => p.d).filter(Number.isFinite);
      const rows = pts.map(p => p.row).filter(Number.isFinite);
      const cols = pts.map(p => p.col).filter(Number.isFinite);
      const ySpread = midYs.length ? Math.max(...midYs) - Math.min(...midYs) : NaN;
      const supportPayload = {
        frame: frame.id || null,
        server_angle: numRound(frame.angle),
        fixed_angle: numRound(aDeg),
        bin,
        h: numRound(frontY, 1),
        pts: pts.length,
        mid: mid.length,
        status_ok: pts.filter(p => p.statusOk).length,
        y_spread: numRound(ySpread, 1),
        min_d: ds.length ? Math.min(...ds) : null,
        max_d: ds.length ? Math.max(...ds) : null,
        row_min: rows.length ? Math.min(...rows) : null,
        row_max: rows.length ? Math.max(...rows) : null,
        col_min: cols.length ? Math.min(...cols) : null,
        col_max: cols.length ? Math.max(...cols) : null,
        edge_hits: pts.filter(p => p.row === 0 || p.row === 7 || p.col === 0 || p.col === 7).length,
        center_hits: pts.filter(p => p.row >= 2 && p.row <= 5 && p.col >= 2 && p.col <= 5).length,
        centerD: cal.centerD,
        distanceModel: cal.distanceModel,
        angleScale: cal.angleScale,
        dir: cal.dir,
        minDist: cal.minDist,
        maxDist: cal.maxDist,
        support_mode: backgroundReady ? 'front_high_mean_keep_1_or_2' : `no_bg_auto_quantiles_${NO_BG_SUPPORT_QUANTILES.join('_')}`,
        no_bg_support_by_q: noBgSupportByQ ? noBgSupportByQ.map(v => ({ q: v.q, h: numRound(v.h, 1) })) : null,
        robust_mode: `sample_percentile_${SUPPORT_SAMPLE_QUANTILE}`
      };
      recordSupportFrame('support_add', supportPayload);
      if (lfFrameCount < 8 || lfFrameCount % 8 === 0) {
        debugLog('support_add', {
          frame: frame.id || null,
          server_angle: round2(frame.angle),
          fixed_angle: round2(aDeg),
          bin,
          h: round1(frontY),
          pts: pts.length,
          mid: mid.length,
          status_ok: pts.filter(p => p.statusOk).length,
          y_spread: round1(ySpread),
          min_d: ds.length ? Math.min(...ds) : null,
          max_d: ds.length ? Math.max(...ds) : null,
          rows: rows.length ? `${Math.min(...rows)}-${Math.max(...rows)}` : null,
          cols: cols.length ? `${Math.min(...cols)}-${Math.max(...cols)}` : null,
          centerD: cal.centerD,
          distanceModel: cal.distanceModel,
          angleScale: cal.angleScale,
          minDist: cal.minDist,
          maxDist: cal.maxDist
        });
      }
      const old = lfSupports.get(bin);
      if (!old) {
        const entry = { angle: aDeg, h: frontY, n, samples: [frontY], count: 1 };
        lfStoreNoBgQuantileSamples(entry, noBgSupportByQ);
        lfSupports.set(bin, entry);
      } else {
        old.samples.push(frontY);
        if (old.samples.length > 24) old.samples.shift();
        old.h = lfRobustHigh(old.samples);
        old.n = n;
        old.angle = aDeg;
        old.count++;
        lfStoreNoBgQuantileSamples(old, noBgSupportByQ);
      }
      lfFrameCount++;
      lfRequestRebuild(false);
      return true;
    }
    function lfSolve4(A, b) {
      const m = A.map((r, i) => r.concat([b[i]]));
      for (let col = 0; col < 4; col++) {
        let piv = col;
        for (let r = col + 1; r < 4; r++) if (Math.abs(m[r][col]) > Math.abs(m[piv][col])) piv = r;
        if (Math.abs(m[piv][col]) < 1e-9) return null;
        if (piv !== col) [m[piv], m[col]] = [m[col], m[piv]];
        const div = m[col][col];
        for (let c = col; c <= 4; c++) m[col][c] /= div;
        for (let r = 0; r < 4; r++) {
          if (r === col) continue;
          const f = m[r][col];
          for (let c = col; c <= 4; c++) m[r][c] -= f * m[col][c];
        }
      }
      return [m[0][4], m[1][4], m[2][4], m[3][4]];
    }
    function lfRectangleFromFit(cx, cy, hx, hy, phi) {
      const e1 = { x: Math.cos(phi), y: Math.sin(phi) };
      const e2 = { x: -Math.sin(phi), y: Math.cos(phi) };
      return [
        { x: cx - hx * e1.x - hy * e2.x, y: cy - hx * e1.y - hy * e2.y },
        { x: cx + hx * e1.x - hy * e2.x, y: cy + hx * e1.y - hy * e2.y },
        { x: cx + hx * e1.x + hy * e2.x, y: cy + hx * e1.y + hy * e2.y },
        { x: cx - hx * e1.x + hy * e2.x, y: cy - hx * e1.y + hy * e2.y }
      ];
    }
    function lfClipPolygonBySupport(poly, n, h) {
      if (!poly.length) return [];
      const out = [];
      const eps = 1e-9;
      for (let i = 0; i < poly.length; i++) {
        const a = poly[i];
        const b = poly[(i + 1) % poly.length];
        const da = n.x * a.x + n.y * a.y - h;
        const db = n.x * b.x + n.y * b.y - h;
        const inA = da <= eps;
        const inB = db <= eps;
        if (inA && inB) {
          out.push(b);
        } else if (inA !== inB) {
          const t = Math.abs(da - db) > eps ? da / (da - db) : 0;
          const p = { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t };
          out.push(p);
          if (inB) out.push(b);
        }
      }
      return out;
    }
    function lfPolygonObb(poly) {
      if (!poly || poly.length < 3) return null;
      let best = null;
      for (let deg = 0; deg < 180; deg++) {
        const a = deg * Math.PI / 180;
        const c = Math.cos(a), s = Math.sin(a);
        let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
        for (const p of poly) {
          const x = c * p.x + s * p.y;
          const y = -s * p.x + c * p.y;
          minX = Math.min(minX, x);
          maxX = Math.max(maxX, x);
          minY = Math.min(minY, y);
          maxY = Math.max(maxY, y);
        }
        const w = maxX - minX;
        const h = maxY - minY;
        const area = w * h;
        if (!best || area < best.area) best = { area, w, h, deg, minX, maxX, minY, maxY };
      }
      if (!best) return null;
      const a = best.deg * Math.PI / 180;
      const c = Math.cos(a), s = Math.sin(a);
      const u = (best.minX + best.maxX) / 2;
      const v = (best.minY + best.maxY) / 2;
      return {
        short: Math.min(best.w, best.h),
        long: Math.max(best.w, best.h),
        angle: best.deg,
        cx: c * u - s * v,
        cy: s * u + c * v,
        rx: best.w / 2,
        ry: best.h / 2,
        phi: a
      };
    }
    function lfPolygonArea(poly) {
      let area = 0;
      for (let i = 0; i < poly.length; i++) {
        const a = poly[i], b = poly[(i + 1) % poly.length];
        area += a.x * b.y - b.x * a.y;
      }
      return Math.abs(area) * 0.5;
    }
    function lfPolygonPerimeter(poly) {
      let p = 0;
      for (let i = 0; i < poly.length; i++) {
        const a = poly[i], b = poly[(i + 1) % poly.length];
        p += Math.hypot(b.x - a.x, b.y - a.y);
      }
      return p;
    }
    function lfCircularity(poly) {
      const perim = lfPolygonPerimeter(poly);
      if (perim <= 0) return 0;
      return 4 * Math.PI * lfPolygonArea(poly) / (perim * perim);
    }
    function lfShouldEllipseFit(poly, obb) {
      if (!poly || poly.length < 8 || !obb || obb.short <= 0) return false;
      const aspect = obb.long / obb.short;
      return aspect <= ELLIPSE_MAX_ASPECT && lfCircularity(poly) >= ELLIPSE_MIN_CIRCULARITY;
    }
    function lfBuildVisualHullOutline() {
      const r = VISUAL_HULL_MAX_RADIUS_MM;
      let poly = [
        { x: -r, y: -r },
        { x: r, y: -r },
        { x: r, y: r },
        { x: -r, y: r }
      ];
      const supports = [...lfSupports.values()]
        .filter(s => Number.isFinite(s.h) && s.count >= 1)
        .sort((a, b) => lfNormDeg(a.angle) - lfNormDeg(b.angle));
      if (supports.length < VISUAL_HULL_MIN_SUPPORTS) return [];
      for (const s of supports) {
        poly = lfClipPolygonBySupport(poly, s.n, s.h);
        if (poly.length < 3) return [];
      }
      return poly;
    }
    function lfRebuildVisualHull() {
      visualHullOutline = lfBuildVisualHullOutline();
      visualHullPoints = [];
      surfaceShapeKind = 'none';
      lfQuality = lfScanQuality();
      const hullObb = lfPolygonObb(visualHullOutline);
      const enoughSupports = !!(lfQuality && lfQuality.fit_allowed);
      const useBox = enoughSupports && lfFit && lfFit.err <= BOX_FIT_MAX_ERR && lfOutline.length >= 3;
      const useNoBgEstimate = !backgroundReady && lfFit && lfFit.estimated && lfOutline.length >= 3 && lfSupports.size >= NO_BG_UNION_MIN_SUPPORTS;
      const useEllipse = enoughSupports && !useBox && lfShouldEllipseFit(visualHullOutline, hullObb);
      const renderOutline = (useBox || useNoBgEstimate) ? lfOutline : visualHullOutline;
      if (useEllipse) {
        surfaceShapeKind = 'ellipse-fit';
      } else if (renderOutline.length >= 3) {
        surfaceShapeKind = useBox ? 'box-fit' : (useNoBgEstimate ? 'no-bg-rect-estimate' : (!backgroundReady ? 'no-bg-envelope' : (enoughSupports ? 'hull' : (lfQuality ? lfQuality.status : 'partial-hull'))));
      } else {
        return;
      }
      const z0 = Number.isFinite(lfZMin) ? Math.max(4, lfZMin - 2) : LF_DEFAULT_Z0;
      const z1 = Number.isFinite(lfZMax) ? Math.min(160, lfZMax + 2) : LF_DEFAULT_Z1;
      const zSteps = Math.max(2, Math.ceil(Math.max(1, z1 - z0) / VISUAL_HULL_Z_STEP_MM));
      if (useEllipse) {
        const segs = Math.max(36, Math.ceil(Math.PI * (Math.abs(hullObb.rx) + Math.abs(hullObb.ry)) / VISUAL_HULL_EDGE_STEP_MM));
        const c = Math.cos(hullObb.phi), s = Math.sin(hullObb.phi);
        for (let e = 0; e < segs; e++) {
          const t = e * Math.PI * 2 / segs;
          const u = Math.cos(t) * hullObb.rx;
          const v = Math.sin(t) * hullObb.ry;
          const x = hullObb.cx + c * u - s * v;
          const y = hullObb.cy + s * u + c * v;
          for (let zStep = 0; zStep <= zSteps; zStep++) {
            const z = z0 + (z1 - z0) * zStep / zSteps;
            visualHullPoints.push({ x, y, z, mm: 70, angle: 0, zone: -1, n: 1, free: 0, hull: true });
          }
        }
        return;
      }
      for (let i = 0; i < renderOutline.length; i++) {
        const a = renderOutline[i];
        const b = renderOutline[(i + 1) % renderOutline.length];
        const edgeLen = Math.hypot(b.x - a.x, b.y - a.y);
        const edgeSteps = Math.max(1, Math.ceil(edgeLen / VISUAL_HULL_EDGE_STEP_MM));
        for (let e = 0; e <= edgeSteps; e++) {
          const t = e / edgeSteps;
          const x = a.x + (b.x - a.x) * t;
          const y = a.y + (b.y - a.y) * t;
          for (let zStep = 0; zStep <= zSteps; zStep++) {
            const z = z0 + (z1 - z0) * zStep / zSteps;
            visualHullPoints.push({ x, y, z, mm: 70, angle: 0, zone: -1, n: 1, free: 0, hull: true });
          }
        }
      }
    }
    function surfaceVisiblePoints() {
      const mode = document.getElementById('mode').value;
      if (mode === 'rawhits') return rawSurfacePoints;
      if (mode === 'surface3d' && visualHullPoints.length) return visualHullPoints;
      return [];
    }
    function lfSortedSupports() {
      return [...lfSupports.values()]
        .filter(s => Number.isFinite(s.h))
        .map(s => ({ angle: lfNormDeg(s.angle), h: s.h, count: s.count || 1 }))
        .sort((a, b) => a.angle - b.angle);
    }
    function lfAngularDelta(a, b) {
      let d = b - a;
      while (d <= 0) d += 360;
      return d;
    }
    function lfMeasuredCornerMarks() {
      const s = lfSortedSupports();
      if (s.length < 8) return [];
      const candidates = [];
      for (let i = 0; i < s.length; i++) {
        const p = s[(i + s.length - 1) % s.length];
        const c = s[i];
        const n = s[(i + 1) % s.length];
        const prevAngle = i === 0 ? p.angle - 360 : p.angle;
        const nextAngle = i === s.length - 1 ? n.angle + 360 : n.angle;
        const left = (c.h - p.h) / Math.max(1, c.angle - prevAngle);
        const right = (n.h - c.h) / Math.max(1, nextAngle - c.angle);
        const score = Math.abs(right - left) * Math.sqrt(Math.max(1, c.count));
        candidates.push({ angle: c.angle, h: c.h, score });
      }
      candidates.sort((a, b) => b.score - a.score);
      const marks = [];
      for (const c of candidates) {
        if (c.score < 0.03) continue;
        if (marks.some(m => Math.min(Math.abs(m.angle - c.angle), 360 - Math.abs(m.angle - c.angle)) < 28)) continue;
        marks.push(c);
        if (marks.length >= 4) break;
      }
      return marks.sort((a, b) => a.angle - b.angle);
    }
    function lfFitCornerMarks() {
      if (!lfFit) return [];
      const base = lfNormDeg(lfFit.phi * 180 / Math.PI);
      return [0, 90, 180, 270].map(add => ({ angle: lfNormDeg(base + add), h: NaN, score: 0 }));
    }
    function lfFitRectangle() {
      const cal = lfCal();
      const cs = [...lfSupports.values()].filter(c => Number.isFinite(c.h));
      if (cs.length < 8) return null;
      let best = null;
      const fitStepDeg = modeling ? 4 : 1;
      for (let deg = 0; deg < 180; deg += fitStepDeg) {
        const phi = deg * Math.PI / 180;
        const e1 = { x: Math.cos(phi), y: Math.sin(phi) };
        const e2 = { x: -Math.sin(phi), y: Math.cos(phi) };
        const M = Array.from({ length: 4 }, () => [0, 0, 0, 0]);
        const B = [0, 0, 0, 0];
        let weightSum = 0;
        for (const c of cs) {
          const nx = c.n.x, ny = c.n.y;
          const a1 = Math.abs(nx * e1.x + ny * e1.y);
          const a2 = Math.abs(nx * e2.x + ny * e2.y);
          const row = [nx, ny, a1, a2];
          const w = Math.min(4, 1 + Math.log2((c.count || 1) + 1));
          weightSum += w;
          for (let i = 0; i < 4; i++) {
            B[i] += w * row[i] * c.h;
            for (let j = 0; j < 4; j++) M[i][j] += w * row[i] * row[j];
          }
        }
        M[0][0] += 0.02; M[1][1] += 0.02; M[2][2] += 0.02; M[3][3] += 0.02;
        const x = lfSolve4(M, B);
        if (!x) continue;
        let [cx, cy, hx, hy] = x;
        hx = Math.abs(hx);
        hy = Math.abs(hy);
        if (hx < 8 || hy < 8 || hx > cal.radius * 1.35 || hy > cal.radius * 1.35) continue;
        if (Math.hypot(cx, cy) > cal.radius * 0.70) continue;
        let err = 0;
        for (const c of cs) {
          const nx = c.n.x, ny = c.n.y;
          const pred = cx * nx + cy * ny + hx * Math.abs(nx * e1.x + ny * e1.y) + hy * Math.abs(nx * e2.x + ny * e2.y);
          const r = pred - c.h;
          const ar = Math.abs(r);
          err += ar < 6 ? r * r : 36 + (ar - 6) * 6;
        }
        err /= Math.max(1, weightSum);
        if (!best || err < best.err) best = { cx, cy, hx, hy, phi, err };
      }
      return best;
    }
    function lfNoBgUnionLoss(cs, phi, p) {
      const e1 = { x: Math.cos(phi), y: Math.sin(phi) };
      const e2 = { x: -Math.sin(phi), y: Math.cos(phi) };
      const cx = p[0], cy = p[1], hx = Math.abs(p[2]), hy = Math.abs(p[3]), r = Math.abs(p[4]);
      if (hx < 2 || hy < 2 || hx > 80 || hy > 80 || r > 80) return Infinity;
      let loss = 0;
      for (const c of cs) {
        const nx = c.n.x, ny = c.n.y;
        const trans = cx * nx + cy * ny;
        const rect = trans + hx * Math.abs(nx * e1.x + ny * e1.y) + hy * Math.abs(nx * e2.x + ny * e2.y);
        const roundClutter = trans + r;
        const residual = Math.max(rect, roundClutter) - c.h;
        const ar = Math.abs(residual);
        loss += ar < NO_BG_UNION_HUBER_MM
          ? residual * residual
          : NO_BG_UNION_HUBER_MM * NO_BG_UNION_HUBER_MM + (ar - NO_BG_UNION_HUBER_MM) * NO_BG_UNION_HUBER_MM;
      }
      return loss / Math.max(1, cs.length) + r * r * 0.0005;
    }
    function lfImproveNoBgUnion(cs, phi, start) {
      let p = start.slice();
      let steps = [4, 4, 4, 4, 4];
      let best = lfNoBgUnionLoss(cs, phi, p);
      const clamp = (idx, v) => {
        if (idx < 2) return Math.max(-50, Math.min(50, v));
        return Math.max(idx === 4 ? 0 : 2, Math.min(80, Math.abs(v)));
      };
      for (let iter = 0; iter < 36; iter++) {
        let improved = false;
        for (let i = 0; i < p.length; i++) {
          for (const dir of [-1, 1]) {
            const cand = p.slice();
            cand[i] = clamp(i, cand[i] + dir * steps[i]);
            const loss = lfNoBgUnionLoss(cs, phi, cand);
            if (loss < best) {
              p = cand;
              best = loss;
              improved = true;
            }
          }
        }
        if (!improved) {
          steps = steps.map(v => v * 0.55);
          if (Math.max(...steps) < 0.04) break;
        }
      }
      return { p, loss: best };
    }
    function lfFitNoBgUnion() {
      const rough = lfFitRectangle();
      const rhx = rough ? Math.max(2, Math.abs(rough.hx)) : 18;
      const rhy = rough ? Math.max(2, Math.abs(rough.hy)) : 22;
      const rcx = rough ? rough.cx : 0;
      const rcy = rough ? rough.cy : 0;
      const r0 = Math.max(8, Math.min(rhx, rhy) * 1.25);
      let best = null;
      const candidateSummary = [];
      for (const supportQ of NO_BG_SUPPORT_QUANTILES) {
        const cs = lfNoBgSupportSetForQuantile(supportQ);
        if (cs.length < NO_BG_UNION_MIN_SUPPORTS) continue;
        const starts = [
          [rcx, rcy, rhx, rhy, r0],
          [rcx, rcy, Math.min(rhx, rhy) * 0.86, Math.max(rhx, rhy) * 1.02, r0],
          [rcx, rcy, Math.max(rhx, rhy) * 1.02, Math.min(rhx, rhy) * 0.86, r0],
          [0, 0, Math.min(rhx, rhy) * 0.9, Math.max(rhx, rhy), r0],
          [0, 0, Math.max(rhx, rhy), Math.min(rhx, rhy) * 0.9, r0]
        ];
        let bestForQ = null;
        for (let deg = 0; deg < 180; deg += NO_BG_UNION_PHI_STEP_DEG) {
          const phi = deg * Math.PI / 180;
          for (const start of starts) {
            const fit = lfImproveNoBgUnion(cs, phi, start);
            const p = fit.p;
            const hx = Math.abs(p[2]), hy = Math.abs(p[3]), r = Math.abs(p[4]);
            if (hx < 5 || hy < 5 || hx > 60 || hy > 60 || r < 4 || r > 60) continue;
            const result = {
              cx: p[0],
              cy: p[1],
              hx,
              hy,
              nuisanceR: r,
              phi,
              err: fit.loss,
              supportQ,
              estimated: true,
              model: 'rect_plus_round_clutter'
            };
            if (!bestForQ || result.err < bestForQ.err) bestForQ = result;
            if (!best || result.err < best.err) best = result;
          }
        }
        if (bestForQ) {
          candidateSummary.push({
            q: supportQ,
            size: `${Math.min(bestForQ.hx * 2, bestForQ.hy * 2).toFixed(1)}x${Math.max(bestForQ.hx * 2, bestForQ.hy * 2).toFixed(1)}`,
            round: numRound(bestForQ.nuisanceR, 1),
            err: numRound(bestForQ.err, 3)
          });
        }
      }
      if (best) best.candidates = candidateSummary;
      return best;
    }
    function lfRebuildOutline() {
      const cal = lfCal();
      lfFit = backgroundReady ? lfFitRectangle() : lfFitNoBgUnion();
      lfOutline = lfFit ? lfRectangleFromFit(lfFit.cx, lfFit.cy, lfFit.hx, lfFit.hy, lfFit.phi) : [];
      lfRebuildVisualHull();
      lfCornerMarks = lfMeasuredCornerMarks();
      const hullObb = lfPolygonObb(visualHullOutline);
      const quality = lfQuality || lfScanQuality();
      const now = performance.now();
      if (now - lastFitLogMs > 700 || !modeling) {
        lastFitLogMs = now;
        if (lfFit) {
          const sx = Math.min(lfFit.hx * 2, lfFit.hy * 2);
          const sy = Math.max(lfFit.hx * 2, lfFit.hy * 2);
          debugLog('fit_update', {
            supports: lfSupports.size,
            size: `${sx.toFixed(1)}x${sy.toFixed(1)}`,
            cx: round1(lfFit.cx),
            cy: round1(lfFit.cy),
            phi: round1(lfFit.phi * 180 / Math.PI),
            err: round2(lfFit.err),
            model: lfFit.model || (backgroundReady ? 'rectangle' : 'no_bg'),
            nuisanceR: Number.isFinite(lfFit.nuisanceR) ? round1(lfFit.nuisanceR) : null,
            supportQ: Number.isFinite(lfFit.supportQ) ? lfFit.supportQ : null,
            candidates: lfFit.candidates || null,
            estimated: !!lfFit.estimated,
            validation: lfFit.estimated ? 'model estimate only; raw hull OBB is not trusted without background/cross-scan validation' : 'calibrated',
            corners: lfCornerMarks.map(m => round1(m.angle)),
            centerD: cal.centerD,
            angleScale: cal.angleScale,
            minDist: cal.minDist,
            maxDist: cal.maxDist,
            hull: hullObb ? `${hullObb.short.toFixed(1)}x${hullObb.long.toFixed(1)}` : 'none',
            quality,
            warn_squareish: Math.abs(sx - sy) < 4 && sx > 45
          });
        } else {
          debugLog('fit_failed', {
            supports: lfSupports.size,
            centerD: cal.centerD,
            angleScale: cal.angleScale,
            minDist: cal.minDist,
            maxDist: cal.maxDist,
            quality,
            corner_candidates: lfCornerMarks.length
          });
        }
      }
      lfDirty = false;
      lfLastRebuildMs = performance.now();
      updatePointStats();
    }
    function lfRequestRebuild(force) {
      lfDirty = true;
      const now = performance.now();
      if (force || now - lfLastRebuildMs >= LF_REBUILD_MS) lfRebuildOutline();
    }
    function line3(a, b, color, width = 2) {
      const pa = project(a), pb = project(b);
      ctx.strokeStyle = color;
      ctx.lineWidth = width;
      ctx.beginPath();
      ctx.moveTo(pa.x, pa.y);
      ctx.lineTo(pb.x, pb.y);
      ctx.stroke();
    }
    function lfFitSupportAt(angleDeg) {
      if (!lfFit) return NaN;
      const a = angleDeg * Math.PI / 180;
      const n = lfRotate2(0, 1, a);
      const e1 = { x: Math.cos(lfFit.phi), y: Math.sin(lfFit.phi) };
      const e2 = { x: -Math.sin(lfFit.phi), y: Math.cos(lfFit.phi) };
      return lfFit.cx * n.x + lfFit.cy * n.y +
        lfFit.hx * Math.abs(n.x * e1.x + n.y * e1.y) +
        lfFit.hy * Math.abs(n.x * e2.x + n.y * e2.y);
    }
    function drawCornerTrace() {
      const chartW = Math.min(plot.width - 28, 600);
      const chartH = 150;
      const x0 = 14;
      const y0 = plot.height - chartH - 34;
      const samples = lfSortedSupports();

      ctx.save();
      ctx.fillStyle = 'rgba(255,255,255,0.94)';
      ctx.fillRect(x0 - 8, y0 - 24, chartW + 16, chartH + 54);
      ctx.strokeStyle = '#d5d1c6';
      ctx.lineWidth = 1;
      ctx.strokeRect(x0, y0, chartW, chartH);
      ctx.fillStyle = '#374151';
      ctx.font = '12px system-ui';
      ctx.fillText('轉角折線: 物體角度 0~360 deg vs 支撐距離 h(mm)', x0, y0 - 8);

      for (let a = 0; a <= 360; a += 90) {
        const x = x0 + chartW * a / 360;
        ctx.strokeStyle = a === 0 || a === 360 ? '#9ca3af' : '#e5e7eb';
        ctx.beginPath();
        ctx.moveTo(x, y0);
        ctx.lineTo(x, y0 + chartH);
        ctx.stroke();
        ctx.fillStyle = '#6b7280';
        ctx.fillText(`${a}`, x + 2, y0 + chartH + 14);
      }

      if (samples.length) {
        let minH = Math.min(...samples.map(s => s.h));
        let maxH = Math.max(...samples.map(s => s.h));
        if (lfFit) {
          for (let a = 0; a <= 360; a += 4) {
            const h = lfFitSupportAt(a);
            if (Number.isFinite(h)) {
              minH = Math.min(minH, h);
              maxH = Math.max(maxH, h);
            }
          }
        }
        if (maxH - minH < 8) {
          const mid = (maxH + minH) / 2;
          minH = mid - 4;
          maxH = mid + 4;
        }
        const pad = (maxH - minH) * 0.12;
        minH -= pad;
        maxH += pad;
        const sx = a => x0 + chartW * lfNormDeg(a) / 360;
        const sy = h => y0 + chartH - (h - minH) * chartH / (maxH - minH);

        ctx.strokeStyle = '#d7d4ca';
        ctx.setLineDash([3, 3]);
        for (let i = 0; i <= 4; i++) {
          const h = minH + (maxH - minH) * i / 4;
          const y = sy(h);
          ctx.beginPath();
          ctx.moveTo(x0, y);
          ctx.lineTo(x0 + chartW, y);
          ctx.stroke();
          ctx.fillStyle = '#6b7280';
          ctx.fillText(`${h.toFixed(0)}`, x0 + chartW + 4, y + 4);
        }
        ctx.setLineDash([]);

        if (lfFit) {
          ctx.strokeStyle = '#2563eb';
          ctx.lineWidth = 1.8;
          ctx.beginPath();
          for (let a = 0; a <= 360; a += 3) {
            const x = sx(a);
            const y = sy(lfFitSupportAt(a));
            if (a === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
          }
          ctx.stroke();
        }

        ctx.strokeStyle = '#111827';
        ctx.lineWidth = 2.2;
        ctx.beginPath();
        samples.forEach((p, i) => {
          const x = sx(p.angle);
          const y = sy(p.h);
          if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
        });
        ctx.stroke();

        ctx.fillStyle = '#111827';
        for (const p of samples) {
          ctx.beginPath();
          ctx.arc(sx(p.angle), sy(p.h), 2.2, 0, Math.PI * 2);
          ctx.fill();
        }

        const fitMarks = lfFitCornerMarks();
        ctx.strokeStyle = 'rgba(37,99,235,0.45)';
        ctx.lineWidth = 1.2;
        for (const m of fitMarks) {
          const x = sx(m.angle);
          ctx.beginPath();
          ctx.moveTo(x, y0);
          ctx.lineTo(x, y0 + chartH);
          ctx.stroke();
        }

        ctx.strokeStyle = '#dc2626';
        ctx.fillStyle = '#dc2626';
        ctx.lineWidth = 2;
        for (const m of lfCornerMarks) {
          const x = sx(m.angle);
          ctx.beginPath();
          ctx.moveTo(x, y0);
          ctx.lineTo(x, y0 + chartH);
          ctx.stroke();
          ctx.fillText(`${m.angle.toFixed(0)}°`, Math.min(x + 3, x0 + chartW - 28), y0 + 13);
        }

        const cal = lfCal();
        const fixedNow = lfZeroAngle === null ? currentAngleDeg : lfNormDeg(cal.dir * lfClockwiseDeltaDeg(lfZeroAngle, currentAngleDeg) * cal.angleScale);
        const curX = sx(fixedNow);
        ctx.strokeStyle = '#16a34a';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(curX, y0);
        ctx.lineTo(curX, y0 + chartH);
        ctx.stroke();
        ctx.fillStyle = '#16a34a';
        ctx.fillText(`now ${fixedNow.toFixed(1)}°`, Math.min(curX + 4, x0 + chartW - 66), y0 + chartH - 8);

        ctx.fillStyle = '#374151';
        const cornerText = lfCornerMarks.length ? lfCornerMarks.map(m => `${m.angle.toFixed(1)}°`).join(', ') : '--';
        ctx.fillText(`黑線=實測折線  藍線=擬合預測  紅線=偵測折點: ${cornerText}`, x0, y0 + chartH + 30);
      } else {
        ctx.fillStyle = '#6b7280';
        ctx.fillText('尚未收到支撐線資料；按「轉一圈建模」後這裡會出現折線。', x0 + 10, y0 + 72);
      }
      ctx.restore();
    }
    function drawLineframeModel() {
      if (lfDirty && performance.now() - lfLastRebuildMs >= LF_REBUILD_MS) lfRebuildOutline();
      ctx.clearRect(0, 0, plot.width, plot.height);
      ctx.fillStyle = '#fff';
      ctx.fillRect(0, 0, plot.width, plot.height);
      drawAxes();

      const z0 = Number.isFinite(lfZMin) ? Math.max(4, lfZMin - 3) : LF_DEFAULT_Z0;
      const z1 = Number.isFinite(lfZMax) ? Math.min(160, lfZMax + 3) : LF_DEFAULT_Z1;
      if (lfOutline.length >= 3) {
        for (let i = 0; i < lfOutline.length; i++) {
          const a = lfOutline[i], b = lfOutline[(i + 1) % lfOutline.length];
          line3({x:a.x,y:a.y,z:z0}, {x:b.x,y:b.y,z:z0}, '#222', 2.2);
          line3({x:a.x,y:a.y,z:z1}, {x:b.x,y:b.y,z:z1}, '#111', 2.6);
          line3({x:a.x,y:a.y,z:z0}, {x:a.x,y:a.y,z:z1}, '#333', 1.6);
        }
      }

      ctx.fillStyle = '#4b5563';
      ctx.font = '13px system-ui';
      const size = lfFit ? ` size=${(lfFit.hx * 2).toFixed(0)}x${(lfFit.hy * 2).toFixed(0)}mm err=${lfFit.err.toFixed(2)}` : '';
      const cal = lfCal();
      ctx.fillText(`fixed lineframe supports=${lfSupports.size} frames=${lfFrameCount} center=${cal.centerD}mm depth=150mm dir=${cal.dir} scale=${cal.angleScale.toFixed(3)}${size}`, 12, 22);
      ctx.fillText(`quality=${lfQualityText(lfQuality || lfScanQuality())}`, 12, 40);
      drawCornerTrace();
      ctx.fillText('drag: rotate, wheel: zoom', 12, plot.height - 14);
    }
    function clearModel() {
      lastSurfaceCount = 0;
      lfZeroAngle = null;
      lfSupports = new Map();
      lfOutline = [];
      lfFit = null;
      visualHullOutline = [];
      visualHullPoints = [];
      surfaceShapeKind = 'none';
      lfZMin = Infinity;
      lfZMax = -Infinity;
      lfFrameCount = 0;
      lfTrace = [];
      lfCornerMarks = [];
      rawSurfacePoints = [];
      rawSurfaceDropped = 0;
      lfResetQualityCounters();
      lfDirty = false;
      lastFrame = -1;
      lastServerFrame = latestServerFrame;
      updatePointStats();
      drawPointCloud();
    }
    function updatePointStats() {
      const mode = document.getElementById('mode').value;
      lfQuality = lfScanQuality();
      const qualityEl = document.getElementById('quality');
      if (qualityEl) qualityEl.textContent = lfQualityText(lfQuality);
      if (mode === 'lineframe') {
        const size = lfFit ? `${Math.round(lfFit.hx * 2)}x${Math.round(lfFit.hy * 2)}mm` : '--';
        document.getElementById('pts').textContent = `${lfSupports.size} supports / ${lfCornerMarks.length} corners / ${size}`;
        const lineCovered = Math.min(100, Math.round(lfSupports.size * LF_SUPPORT_BIN_DEG * 100 / 360));
        document.getElementById('cover').textContent = `${lineCovered}%`;
        return;
      }
      const visible = surfaceVisiblePoints();
      const bg = backgroundSummary();
      const bgText = bg.ready ? `bg ${bg.frames}f` : 'bg none';
      const hullText = visualHullOutline.length ? ` / hull ${visualHullOutline.length}v` : '';
      const fitText = lfFit ? ` / fit ${Math.min(lfFit.hx * 2, lfFit.hy * 2).toFixed(1)}x${Math.max(lfFit.hx * 2, lfFit.hy * 2).toFixed(1)}mm` : '';
      const rawText = document.getElementById('mode').value === 'rawhits' ? ` / raw drop ${rawSurfaceDropped}` : '';
      document.getElementById('pts').textContent = `${visible.length} pts / ${lfSupports.size} supports / ${bgText}${hullText}${fitText}${rawText}`;
      const surfaceCovered = Math.min(100, Math.round(lfSupports.size * LF_SUPPORT_BIN_DEG * 100 / 360));
      document.getElementById('cover').textContent = `${surfaceCovered}%`;
    }
    function addFramePoints(data) {
      if (calibrationMode === 'background') {
        lastSurfaceCount = addBackgroundFrame(data);
        updatePointStats();
        return;
      }
      const mode = document.getElementById('mode').value;
      if (mode === 'lineframe') {
        lfAddSupportFromFrame(data);
        updatePointStats();
        return;
      }
      if (mode === 'surface3d' || mode === 'rawhits') {
        lfAddSupportFromFrame(data);
        updatePointStats();
        return;
      }
    }
    function drawPointCloud() {
      if (document.getElementById('mode').value === 'lineframe') {
        drawLineframeModel();
        return;
      }
      ctx.clearRect(0, 0, plot.width, plot.height);
      ctx.fillStyle = '#fff';
      ctx.fillRect(0, 0, plot.width, plot.height);
      drawAxes();
      const visible = surfaceVisiblePoints();
      const drawStep = Math.max(1, Math.floor(visible.length / DRAW_POINT_LIMIT));
      const drawSource = drawStep === 1 ? visible : visible.filter((_, i) => i % drawStep === 0);
      const sorted = drawSource.map(p => ({ p, q: project(p) })).sort((a, b) => a.q.depth - b.q.depth);
      for (const item of sorted) {
        ctx.fillStyle = item.p.raw && !item.p.statusOk ? '#9ca3af' : (item.p.hull ? colorForZ(item.p.z) : colorForDistance(item.p.mm));
        ctx.beginPath();
        ctx.arc(item.q.x, item.q.y, item.p.hull ? 2.0 : Math.min(4.2, 1.6 + Math.log2((item.p.n || 1) + 1) * 0.45), 0, Math.PI * 2);
        ctx.fill();
      }
      const cal = getCal();
      const bg = backgroundSummary();
      ctx.fillStyle = '#4b5563';
      ctx.font = '13px system-ui';
      const hullObb = lfPolygonObb(visualHullOutline);
      const hullText = hullObb ? ` hull=${hullObb.short.toFixed(1)}x${hullObb.long.toFixed(1)}mm/${visualHullOutline.length}v` : '';
      const fitText = lfFit ? ` fit=${Math.min(lfFit.hx * 2, lfFit.hy * 2).toFixed(1)}x${Math.max(lfFit.hx * 2, lfFit.hy * 2).toFixed(1)}mm${lfFit.nuisanceR ? '/round=' + lfFit.nuisanceR.toFixed(1) : ''}${Number.isFinite(lfFit.supportQ) ? '/q=' + lfFit.supportQ.toFixed(2) : ''}` : '';
      const mode = document.getElementById('mode').value;
      const shapeText = mode === 'rawhits' ? `raw-hit-cloud dropped=${rawSurfaceDropped}` : `shape=${surfaceShapeKind}${hullText}${fitText}`;
      ctx.fillText(`points=${visible.length} supports=${lfSupports.size} ${shapeText}`, 12, 22);
      ctx.fillText(`center=${cal.centerD}mm projection=${cal.distanceModel} scale=${cal.angleScale.toFixed(3)} map=${cal.zoneMap} bg=${bg.ready ? bg.frames + 'f' : 'none'}`, 12, 40);
      ctx.fillText(`quality=${lfQualityText(lfQuality || lfScanQuality())}`, 12, 58);
      ctx.fillText('drag: rotate, wheel: zoom', 12, plot.height - 14);
    }
    function exportCSV() {
      if (document.getElementById('mode').value !== 'lineframe') {
        const visible = surfaceVisiblePoints();
        let out = 'x_mm,y_mm,z_mm,distance_mm,turntable_angle_deg,zone,row,col,hits,status_ok,source\n';
        for (const p of visible) out += `${p.x.toFixed(2)},${p.y.toFixed(2)},${p.z.toFixed(2)},${p.mm.toFixed(1)},${p.angle.toFixed(2)},${p.zone},${p.row ?? ''},${p.col ?? ''},${p.n || 1},${p.statusOk === false ? 0 : 1},${p.raw ? 'raw_hit' : 'visual_hull'}\n`;
        download('vl53l7cx_surface_point_cloud.csv', out, 'text/csv');
        return;
      }
      let out = 'support_bin,angle_deg,h_mm,nx,ny,count\n';
      for (const [bin, s] of lfSupports.entries()) {
        out += `${bin},${s.angle.toFixed(2)},${s.h.toFixed(2)},${s.n.x.toFixed(6)},${s.n.y.toFixed(6)},${s.count}\n`;
      }
      download('vl53l7cx_fixed_lineframe_supports.csv', out, 'text/csv');
    }
    function exportPLY() {
      if (document.getElementById('mode').value !== 'lineframe') {
        const visible = surfaceVisiblePoints();
        let out = 'ply\nformat ascii 1.0\n';
        out += `element vertex ${visible.length}\n`;
        out += 'property float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n';
        for (const p of visible) {
          const t = Math.max(0, Math.min(1, (p.z + 10) / 150));
          const r = Math.round(220 - 120 * t), g = Math.round(70 + 130 * t), b = Math.round(55 + 150 * t);
          const hitBoost = Math.min(45, Math.log2((p.n || 1) + 1) * 18);
          out += `${p.x.toFixed(2)} ${p.y.toFixed(2)} ${p.z.toFixed(2)} ${Math.min(255, r + hitBoost)} ${Math.min(255, g + hitBoost)} ${Math.min(255, b + hitBoost)}\n`;
        }
        download('vl53l7cx_surface_point_cloud.ply', out, 'application/octet-stream');
        return;
      }
      const poly = lfOutline;
      const z0 = Number.isFinite(lfZMin) ? Math.max(4, lfZMin - 3) : LF_DEFAULT_Z0;
      const z1 = Number.isFinite(lfZMax) ? Math.min(160, lfZMax + 3) : LF_DEFAULT_Z1;
      let out = 'ply\nformat ascii 1.0\n';
      if (poly.length < 3) {
        out += 'element vertex 0\nproperty float x\nproperty float y\nproperty float z\nend_header\n';
        download('vl53l7cx_fixed_lineframe_empty.ply', out, 'application/octet-stream');
        return;
      }
      out += `element vertex ${poly.length * 2}\n`;
      out += 'property float x\nproperty float y\nproperty float z\n';
      out += `element face ${poly.length + 2}\n`;
      out += 'property list uchar int vertex_indices\nend_header\n';
      for (const p of poly) out += `${p.x.toFixed(2)} ${p.y.toFixed(2)} ${z0.toFixed(2)}\n`;
      for (const p of poly) out += `${p.x.toFixed(2)} ${p.y.toFixed(2)} ${z1.toFixed(2)}\n`;
      out += `${poly.length}`;
      for (let i = poly.length - 1; i >= 0; i--) out += ` ${i}`;
      out += '\n';
      out += `${poly.length}`;
      for (let i = 0; i < poly.length; i++) out += ` ${i + poly.length}`;
      out += '\n';
      for (let i = 0; i < poly.length; i++) {
        const j = (i + 1) % poly.length;
        out += `4 ${i} ${j} ${j + poly.length} ${i + poly.length}\n`;
      }
      download('vl53l7cx_fixed_lineframe_box.ply', out, 'application/octet-stream');
    }
    async function startModel() {
      clearModel();
      modeling = true;
      beginRawScanSession('continuous');
      document.getElementById('startBtn').classList.add('active');
      const cal = lfCal();
      debugLog('cmd_start_continuous', { centerD: cal.centerD, distanceModel: cal.distanceModel, angleScale: cal.angleScale, minDist: cal.minDist, maxDist: cal.maxDist, dir: cal.dir });
      await cmd('/start');
    }
    async function oneRevModel() {
      clearModel();
      modeling = true;
      beginRawScanSession('one_rev_360');
      document.getElementById('startBtn').classList.add('active');
      const cal = lfCal();
      debugLog('cmd_one_rev', { centerD: cal.centerD, distanceModel: cal.distanceModel, angleScale: cal.angleScale, minDist: cal.minDist, maxDist: cal.maxDist, dir: cal.dir });
      await cmd('/one_rev');
    }
    async function slowRefineScan() {
      modeling = true;
      if (!lfSupports.size) lfZeroAngle = null;
      beginRawScanSession('slow_one_rev_360');
      document.getElementById('startBtn').classList.add('active');
      const cal = lfCal();
      debugLog('cmd_slow_scan', { centerD: cal.centerD, distanceModel: cal.distanceModel, angleScale: cal.angleScale, minDist: cal.minDist, maxDist: cal.maxDist, dir: cal.dir });
      await cmd('/slow_scan');
    }
    async function smartRefineScan() {
      modeling = true;
      document.getElementById('startBtn').classList.add('active');
      if (lfSupports.size < 12 || lfZeroAngle === null) {
        document.getElementById('msg').textContent = '支撐線不足，先執行慢速全圈。';
        debugLog('cmd_smart_fallback_slow', { supports: lfSupports.size, zero: lfZeroAngle });
        await cmd('/slow_scan');
        return;
      }
      const angles = [...lfSupports.values()].map(s => lfNormDeg(s.angle)).sort((a, b) => a - b);
      let bestGap = -1;
      let targetFixed = angles[0];
      for (let i = 0; i < angles.length; i++) {
        const a = angles[i];
        const b = i === angles.length - 1 ? angles[0] + 360 : angles[i + 1];
        const gap = b - a;
        if (gap > bestGap) {
          bestGap = gap;
          targetFixed = lfNormDeg(a + gap / 2);
        }
      }
      const cal = lfCal();
      const targetDelta = lfNormDeg(cal.dir >= 0 ? targetFixed : -targetFixed) / cal.angleScale;
      const serverTarget = lfNormDeg(lfZeroAngle + targetDelta);
      document.getElementById('msg').textContent = `補掃最大缺口 ${bestGap.toFixed(1)} deg，目標角 ${serverTarget.toFixed(1)} deg`;
      debugLog('cmd_smart_sector', {
        supports: lfSupports.size,
        best_gap: round1(bestGap),
        fixed_target: round1(targetFixed),
        server_target: round1(serverTarget),
        angleScale: cal.angleScale
      });
      await cmd(`/smart_sector?angle=${serverTarget.toFixed(1)}&deg=${Math.max(12, Math.min(36, bestGap + 6)).toFixed(1)}&us=7000`);
    }
    async function stopModel() {
      modeling = false;
      document.getElementById('startBtn').classList.remove('active');
      await cmd('/stop');
      if (calibrationMode === 'background' && backgroundFrameCount > 0) finishBackgroundCalibration();
      if (document.getElementById('mode').value === 'lineframe') lfRequestRebuild(true);
      debugLog('cmd_stop_final', {
        mode: document.getElementById('mode').value,
        surface_points: surfaceVisiblePoints().length,
        raw_surface_points: rawSurfacePoints.length,
        raw_surface_dropped: rawSurfaceDropped,
        supports: lfSupports.size,
        shape: surfaceShapeKind,
        quality: lfQuality || lfScanQuality(),
        corners: lfCornerMarks.map(m => round1(m.angle)),
        size: lfFit ? `${Math.min(lfFit.hx * 2, lfFit.hy * 2).toFixed(1)}x${Math.max(lfFit.hx * 2, lfFit.hy * 2).toFixed(1)}` : null
      });
      drawPointCloud();
    }

    setInterval(update, 140);
    const initialCal = lfCal();
    debugLog('page_loaded', {
      html: 'raw_hit_point_cloud_default',
      centerD: initialCal.centerD,
      distanceModel: initialCal.distanceModel,
      angleScale: initialCal.angleScale,
      dir: initialCal.dir,
      minDist: initialCal.minDist,
      maxDist: initialCal.maxDist
    });
    drawPointCloud();
    update();
  </script>
</body>
</html>
)HTML";

void writeMotorStep(int idx) {
  digitalWrite(MOTOR_IN1, halfStepSeq[idx][0]);
  digitalWrite(MOTOR_IN2, halfStepSeq[idx][1]);
  digitalWrite(MOTOR_IN3, halfStepSeq[idx][2]);
  digitalWrite(MOTOR_IN4, halfStepSeq[idx][3]);
}

void motorOff() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW);
  digitalWrite(MOTOR_IN4, LOW);
}

long degreesToSteps(float deg) {
  if (deg < 0) deg = 0;
  return (long)((deg * (float)TURNTABLE_ONE_REV_STEPS / 360.0f) + 0.5f);
}

long clockwiseStepsToAngle(float targetDeg) {
  while (targetDeg < 0) targetDeg += 360.0f;
  while (targetDeg >= 360.0f) targetDeg -= 360.0f;

  long targetStep = (long)((targetDeg * (float)TURNTABLE_ONE_REV_STEPS / 360.0f) + 0.5f);
  targetStep %= TURNTABLE_ONE_REV_STEPS;
  long cur = turntableStep % TURNTABLE_ONE_REV_STEPS;
  if (cur < 0) cur += TURNTABLE_ONE_REV_STEPS;
  long delta = targetStep - cur;
  if (delta < 0) delta += TURNTABLE_ONE_REV_STEPS;
  return delta;
}

void startPendingSectorIfNeeded() {
  if (pendingSectorSteps <= 0) {
    motorOff();
    scanCaptureEnabled = false;
    adaptiveScanSpeed = false;
    return;
  }

  stepIntervalUs = pendingSectorStepUs;
  adaptiveScanSpeed = pendingSectorAdaptive;
  scanCaptureEnabled = true;
  motorStepsRemaining = pendingSectorSteps;
  pendingSectorSteps = 0;
}

void serviceMotor() {
  if (!motorRunContinuous && motorStepsRemaining <= 0) return;

  uint32_t now = micros();
  uint32_t activeIntervalUs = stepIntervalUs;
  if (adaptiveScanSpeed && scanCaptureEnabled && !lastFrameHasObject) {
    activeIntervalUs = EMPTY_REGION_STEP_US;
  }
  if ((uint32_t)(now - lastStepUs) < activeIntervalUs) return;
  lastStepUs = now;

  halfStepIndex = (halfStepIndex + 1) & 7;
  writeMotorStep(halfStepIndex);
  turntableStep++;

  if (motorStepsRemaining > 0) {
    motorStepsRemaining--;
    if (motorStepsRemaining == 0 && !motorRunContinuous) {
      startPendingSectorIfNeeded();
    }
  }
}

float currentAngleDeg() {
  long step = turntableStep % TURNTABLE_ONE_REV_STEPS;
  if (step < 0) step += TURNTABLE_ONE_REV_STEPS;
  return (float)step * 360.0f / (float)TURNTABLE_ONE_REV_STEPS;
}

float currentPinionAngleDeg() {
  long step = turntableStep % MOTOR_STEPS_PER_REV;
  if (step < 0) step += MOTOR_STEPS_PER_REV;
  return (float)step * 360.0f / (float)MOTOR_STEPS_PER_REV;
}

uint16_t currentAngleCdeg() {
  long step = turntableStep % TURNTABLE_ONE_REV_STEPS;
  if (step < 0) step += TURNTABLE_ONE_REV_STEPS;
  return (uint16_t)((step * 36000L) / TURNTABLE_ONE_REV_STEPS);
}

ScanFrame *findScanFrame(uint32_t id) {
  for (uint8_t i = 0; i < SCAN_FRAME_RING_SIZE; i++) {
    if (scanFrames[i].id == id) return &scanFrames[i];
  }
  return nullptr;
}

void pushScanFrame() {
  if (!scanCaptureEnabled) return;

  ScanFrame &f = scanFrames[scanFrameWriteIndex];
  f.id = ++scanFrameCounter;
  f.angleCdeg = currentAngleCdeg();
  f.pinionAngleCdeg = (uint16_t)(currentPinionAngleDeg() * 100.0f + 0.5f);
  f.turntableStep = turntableStep;
  f.objectZones = lastObjectZones;
  f.hasObject = lastFrameHasObject ? 1 : 0;
  f.nearestMm = lastNearestMm;
  memcpy(f.dist, distanceMm, sizeof(distanceMm));
  memcpy(f.status, statusCode, sizeof(statusCode));

  scanFrameWriteIndex++;
  if (scanFrameWriteIndex >= SCAN_FRAME_RING_SIZE) scanFrameWriteIndex = 0;
}

String jsonEscape(const String &src) {
  String out;
  out.reserve(src.length() + 8);
  for (size_t i = 0; i < src.length(); i++) {
    char c = src[i];
    if (c == '"' || c == '\\') {
      out += '\\';
      out += c;
    } else if (c == '\n' || c == '\r') {
      out += ' ';
    } else {
      out += c;
    }
  }
  return out;
}

bool usableObjectStatus(uint8_t status) {
  return status == 5 || status == 6 || status == 9;
}

void scanI2CBus() {
  i2cDeviceCount = 0;
  i2cFound29 = false;
  Serial.println("I2C scan start");
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();
    if (error == 0) {
      i2cDeviceCount++;
      if (addr == VL53L7CX_DEFAULT_ADDRESS) i2cFound29 = true;
      Serial.print("  found 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
    }
  }
  if (i2cDeviceCount == 0) Serial.println("  no I2C device found");
  Serial.println("I2C scan done");
}

bool initSensor() {
  sensorOk = false;
  frameId = 0;
  lastFrameMs = 0;

  Serial.println("\n--- Initializing VL53L7CX ---");

  if (VL53_LPN_PIN >= 0) {
    Serial.println("Resetting sensor via LPn pin (GPIO10)...");
    pinMode(VL53_LPN_PIN, OUTPUT);
    digitalWrite(VL53_LPN_PIN, LOW);
    delay(10);
    digitalWrite(VL53_LPN_PIN, HIGH);
    delay(10);
  }

  Serial.println("Setting I2C clock to 400kHz...");
  Wire.setClock(400000);
  scanI2CBus();
  if (!i2cFound29) {
    sensorMessage = "I2C did not find 0x29. Check wiring.";
    Serial.println(sensorMessage);
    return false;
  }

  Serial.println("Uploading ~84KB firmware (can take up to 10s)...");
  if (!vl53l7cx.begin(VL53L7CX_DEFAULT_ADDRESS, &Wire)) {
    sensorMessage = "Firmware upload failed!";
    Serial.println(sensorMessage);
    return false;
  }
  if (!vl53l7cx.setResolution(64)) {
    sensorMessage = "setResolution(64) failed.";
    Serial.println(sensorMessage);
    return false;
  }
  if (!vl53l7cx.setRangingFrequency(10)) {
    sensorMessage = "setRangingFrequency(10) failed.";
    Serial.println(sensorMessage);
    return false;
  }
  if (!vl53l7cx.startRanging()) {
    sensorMessage = "startRanging() failed.";
    Serial.println(sensorMessage);
    return false;
  }

  sensorMessage = "OK";
  sensorOk = true;
  Serial.println("VL53L7CX Found and Initialized Successfully!");
  return true;
}

void serviceSensor() {
  if (!sensorOk) return;
  if (vl53l7cx.isDataReady()) {
    if (vl53l7cx.getRangingData(&results)) {
      int out = 0;
      uint8_t objectZones = 0;
      uint16_t nearest = 65535;
      for (int x = 7; x >= 0; x--) {
        for (int y = 56; y >= 0; y -= 8) {
          int idx = x + y;
          distanceMm[out] = results.distance_mm[idx];
          statusCode[out] = results.target_status[idx];
          uint16_t d = distanceMm[out];
          bool modelZone = out < 56;  // Exported zones 56..63 are the fixed empty-scan side reflection band.
          if (modelZone && usableObjectStatus(statusCode[out]) && d > OBJECT_MIN_DISTANCE_MM && d < OBJECT_NEAR_LIMIT_MM) {
            objectZones++;
            if (d < nearest) nearest = d;
          }
          out++;
        }
      }
      lastObjectZones = objectZones;
      lastNearestMm = nearest == 65535 ? 0 : nearest;
      lastFrameHasObject = objectZones >= OBJECT_MIN_ZONES;
      frameId++;
      lastFrameMs = millis();
      pushScanFrame();
      uint32_t nowMs = millis();
      if (nowMs - lastSerialDebugMs >= 1000) {
        lastSerialDebugMs = nowMs;
        Serial.printf(
            "DBG frame=%lu scan_frame=%lu obj_angle=%.2f pinion_angle=%.2f object_zones=%u nearest=%u step=%ld running=%u capture=%u step_us=%lu\n",
            (unsigned long)frameId,
            (unsigned long)scanFrameCounter,
            currentAngleDeg(),
            currentPinionAngleDeg(),
            (unsigned int)lastObjectZones,
            (unsigned int)lastNearestMm,
            turntableStep,
            (unsigned int)(motorRunContinuous || motorStepsRemaining > 0),
            (unsigned int)scanCaptureEnabled,
            (unsigned long)stepIntervalUs);
      }
    }
  }
}

void sendJson() {
  String json;
  json.reserve(1900);
  json += "{";
  json += "\"sensor\":";
  json += sensorOk ? "true" : "false";
  json += ",\"sensor_msg\":\"";
  json += jsonEscape(sensorMessage);
  json += "\"";
  json += ",\"i2c_found_29\":";
  json += i2cFound29 ? "true" : "false";
  json += ",\"i2c_count\":";
  json += String((int)i2cDeviceCount);
  json += ",\"running\":";
  json += (motorRunContinuous || motorStepsRemaining > 0) ? "true" : "false";
  json += ",\"capture\":";
  json += scanCaptureEnabled ? "true" : "false";
  json += ",\"object\":";
  json += lastFrameHasObject ? "true" : "false";
  json += ",\"object_zones\":";
  json += String((int)lastObjectZones);
  json += ",\"nearest_mm\":";
  json += lastNearestMm;
  json += ",\"angle\":";
  json += String(currentAngleDeg(), 2);
  json += ",\"pinion_angle\":";
  json += String(currentPinionAngleDeg(), 2);
  json += ",\"turntable_step\":";
  json += turntableStep;
  json += ",\"one_rev_steps\":";
  json += TURNTABLE_ONE_REV_STEPS;
  json += ",\"step_us\":";
  json += stepIntervalUs;
  json += ",\"frame\":";
  json += frameId;
  json += ",\"scan_frame\":";
  json += scanFrameCounter;
  json += ",\"age_ms\":";
  json += (lastFrameMs == 0 ? 0 : millis() - lastFrameMs);
  json += ",\"dist\":[";
  for (int i = 0; i < 64; i++) {
    if (i) json += ",";
    json += distanceMm[i];
  }
  json += "],\"status\":[";
  for (int i = 0; i < 64; i++) {
    if (i) json += ",";
    json += statusCode[i];
  }
  json += "]}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void appendLatestDistances(String &json) {
  json += ",\"dist\":[";
  for (int i = 0; i < 64; i++) {
    if (i) json += ",";
    json += distanceMm[i];
  }
  json += "],\"status\":[";
  for (int i = 0; i < 64; i++) {
    if (i) json += ",";
    json += statusCode[i];
  }
  json += "]";
}

void sendFramesJson() {
  uint32_t after = 0;
  if (server.hasArg("after")) after = strtoul(server.arg("after").c_str(), nullptr, 10);

  uint32_t oldest = 1;
  if (scanFrameCounter >= SCAN_FRAME_RING_SIZE) {
    oldest = scanFrameCounter - SCAN_FRAME_RING_SIZE + 1;
  }
  uint32_t start = after + 1;
  if (start < oldest) start = oldest;

  String json;
  json.reserve(56000);
  json += "{";
  json += "\"sensor\":";
  json += sensorOk ? "true" : "false";
  json += ",\"sensor_msg\":\"";
  json += jsonEscape(sensorMessage);
  json += "\"";
  json += ",\"running\":";
  json += (motorRunContinuous || motorStepsRemaining > 0) ? "true" : "false";
  json += ",\"capture\":";
  json += scanCaptureEnabled ? "true" : "false";
  json += ",\"object\":";
  json += lastFrameHasObject ? "true" : "false";
  json += ",\"object_zones\":";
  json += String((int)lastObjectZones);
  json += ",\"nearest_mm\":";
  json += lastNearestMm;
  json += ",\"angle\":";
  json += String(currentAngleDeg(), 2);
  json += ",\"pinion_angle\":";
  json += String(currentPinionAngleDeg(), 2);
  json += ",\"turntable_step\":";
  json += turntableStep;
  json += ",\"one_rev_steps\":";
  json += TURNTABLE_ONE_REV_STEPS;
  json += ",\"step_us\":";
  json += stepIntervalUs;
  json += ",\"frame\":";
  json += frameId;
  json += ",\"scan_frame\":";
  json += scanFrameCounter;
  json += ",\"age_ms\":";
  json += (lastFrameMs == 0 ? 0 : millis() - lastFrameMs);
  appendLatestDistances(json);
  json += ",\"frames\":[";

  bool firstFrame = true;
  for (uint32_t id = start; id <= scanFrameCounter; id++) {
    ScanFrame *f = findScanFrame(id);
    if (!f) continue;
    if (!firstFrame) json += ",";
    firstFrame = false;

    json += "{\"id\":";
    json += f->id;
    json += ",\"angle\":";
    json += String((float)f->angleCdeg / 100.0f, 2);
    json += ",\"pinion_angle\":";
    json += String((float)f->pinionAngleCdeg / 100.0f, 2);
    json += ",\"turntable_step\":";
    json += f->turntableStep;
    json += ",\"object\":";
    json += f->hasObject ? "true" : "false";
    json += ",\"object_zones\":";
    json += String((int)f->objectZones);
    json += ",\"nearest_mm\":";
    json += f->nearestMm;
    json += ",\"dist\":[";
    for (int i = 0; i < 64; i++) {
      if (i) json += ",";
      json += f->dist[i];
    }
    json += "],\"status\":[";
    for (int i = 0; i < 64; i++) {
      if (i) json += ",";
      json += f->status[i];
    }
    json += "]}";
  }

  json += "]}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void handleRoot() {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

void handleStart() {
  pendingSectorSteps = 0;
  adaptiveScanSpeed = true;
  scanCaptureEnabled = true;
  motorRunContinuous = true;
  motorStepsRemaining = 0;
  Serial.printf("DBG cmd=start angle=%.2f pinion=%.2f step=%ld\n", currentAngleDeg(), currentPinionAngleDeg(), turntableStep);
  sendJson();
}

void handleOneRev() {
  pendingSectorSteps = 0;
  adaptiveScanSpeed = false;
  scanCaptureEnabled = true;
  motorRunContinuous = false;
  motorStepsRemaining = TURNTABLE_ONE_REV_STEPS;
  Serial.printf("DBG cmd=one_rev steps=%ld angle=%.2f pinion=%.2f step=%ld\n", TURNTABLE_ONE_REV_STEPS, currentAngleDeg(), currentPinionAngleDeg(), turntableStep);
  sendJson();
}

void handleSlowScan() {
  stepIntervalUs = 7000;
  pendingSectorSteps = 0;
  adaptiveScanSpeed = true;
  scanCaptureEnabled = true;
  motorRunContinuous = false;
  motorStepsRemaining = TURNTABLE_ONE_REV_STEPS;
  Serial.printf("DBG cmd=slow_scan steps=%ld angle=%.2f pinion=%.2f step=%ld\n", TURNTABLE_ONE_REV_STEPS, currentAngleDeg(), currentPinionAngleDeg(), turntableStep);
  sendJson();
}

void handleSmartSector() {
  float targetAngle = server.hasArg("angle") ? server.arg("angle").toFloat() : currentAngleDeg();
  float sectorDeg = server.hasArg("deg") ? server.arg("deg").toFloat() : 18.0f;
  long us = server.hasArg("us") ? server.arg("us").toInt() : 7000;
  if (sectorDeg < 3) sectorDeg = 3;
  if (sectorDeg > 90) sectorDeg = 90;
  if (us < 2500) us = 2500;
  if (us > 20000) us = 20000;

  long moveSteps = clockwiseStepsToAngle(targetAngle);
  pendingSectorSteps = degreesToSteps(sectorDeg);
  pendingSectorStepUs = (uint32_t)us;
  pendingSectorAdaptive = false;
  adaptiveScanSpeed = false;
  scanCaptureEnabled = false;
  motorRunContinuous = false;
  stepIntervalUs = EMPTY_REGION_STEP_US;

  if (moveSteps <= 0) {
    startPendingSectorIfNeeded();
  } else {
    motorStepsRemaining = moveSteps;
  }
  Serial.printf("DBG cmd=smart_sector target=%.2f sector=%.2f move_steps=%ld pending=%ld angle=%.2f pinion=%.2f\n",
                targetAngle, sectorDeg, moveSteps, pendingSectorSteps, currentAngleDeg(), currentPinionAngleDeg());
  sendJson();
}

void handleStop() {
  scanCaptureEnabled = false;
  adaptiveScanSpeed = false;
  pendingSectorSteps = 0;
  motorRunContinuous = false;
  motorStepsRemaining = 0;
  motorOff();
  Serial.printf("DBG cmd=stop angle=%.2f pinion=%.2f step=%ld scan_frame=%lu\n",
                currentAngleDeg(), currentPinionAngleDeg(), turntableStep, (unsigned long)scanFrameCounter);
  sendJson();
}

void handleZero() {
  turntableStep = 0;
  scanFrameCounter = 0;
  scanFrameWriteIndex = 0;
  pendingSectorSteps = 0;
  adaptiveScanSpeed = false;
  memset(scanFrames, 0, sizeof(scanFrames));
  Serial.println("DBG cmd=zero step=0 scan_frame=0");
  sendJson();
}

void handleSpeed() {
  if (server.hasArg("us")) {
    long us = server.arg("us").toInt();
    if (us < 1500) us = 1500;
    if (us > 20000) us = 20000;
    stepIntervalUs = (uint32_t)us;
  }
  sendJson();
}

void handleRetrySensor() {
  initSensor();
  sendJson();
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  pinMode(MOTOR_IN4, OUTPUT);
  motorOff();

  for (int i = 0; i < 64; i++) {
    distanceMm[i] = 0;
    statusCode[i] = 0;
  }

  Wire.begin(I2C_SDA, I2C_SCL);
  initSensor();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  IPAddress ip = WiFi.softAPIP();

  server.on("/", handleRoot);
  server.on("/data", sendJson);
  server.on("/frames", sendFramesJson);
  server.on("/start", handleStart);
  server.on("/one_rev", handleOneRev);
  server.on("/slow_scan", handleSlowScan);
  server.on("/smart_sector", handleSmartSector);
  server.on("/stop", handleStop);
  server.on("/zero", handleZero);
  server.on("/speed", handleSpeed);
  server.on("/retry_sensor", handleRetrySensor);
  server.begin();

  Serial.print("WiFi SSID: ");
  Serial.println(AP_SSID);
  Serial.print("WiFi PASS: ");
  Serial.println(AP_PASS);
  Serial.print("Open: http://");
  Serial.println(ip);
  Serial.print("VL53L7CX: ");
  Serial.println(sensorOk ? "OK" : "ERROR");
  Serial.print("Turntable one revolution steps: ");
  Serial.println(TURNTABLE_ONE_REV_STEPS);
}

void loop() {
  serviceMotor();
  serviceSensor();
  server.handleClient();
  yield();
}
