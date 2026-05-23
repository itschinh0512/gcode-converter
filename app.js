const $ = (id) => document.getElementById(id);

const state = {
  image: null,
  bitmap: null,
  paths: [],
  gcode: [],
  sender: null,
  isSending: false,
};

const els = {
  imageInput: $("imageInput"),
  fileName: $("fileName"),
  threshold: $("threshold"),
  thresholdValue: $("thresholdValue"),
  invert: $("invert"),
  centerline: $("centerline"),
  maxProcessSize: $("maxProcessSize"),
  workWidth: $("workWidth"),
  workHeight: $("workHeight"),
  keepAspect: $("keepAspect"),
  invertY: $("invertY"),
  simplifyTolerance: $("simplifyTolerance"),
  lineSmoothing: $("lineSmoothing"),
  lineSmoothingValue: $("lineSmoothingValue"),
  travelFeed: $("travelFeed"),
  drawFeed: $("drawFeed"),
  penUp: $("penUp"),
  penDown: $("penDown"),
  wsUrl: $("wsUrl"),
  wsMode: $("wsMode"),
  previewCanvas: $("previewCanvas"),
  gcodeOutput: $("gcodeOutput"),
  jobStats: $("jobStats"),
  connectionStatus: $("connectionStatus"),
  connectBtn: $("connectBtn"),
  disconnectBtn: $("disconnectBtn"),
  sendBtn: $("sendBtn"),
  pauseBtn: $("pauseBtn"),
  resumeBtn: $("resumeBtn"),
  statusBtn: $("statusBtn"),
  resetBtn: $("resetBtn"),
  regenerateBtn: $("regenerateBtn"),
  downloadBtn: $("downloadBtn"),
  copyBtn: $("copyBtn"),
  clearLogBtn: $("clearLogBtn"),
  logOutput: $("logOutput"),
};

function readSettings() {
  return {
    threshold: Number(els.threshold.value),
    invert: els.invert.checked,
    centerline: els.centerline.checked,
    maxProcessSize: Number(els.maxProcessSize.value),
    workWidthMm: Number(els.workWidth.value),
    workHeightMm: Number(els.workHeight.value),
    keepAspectRatio: els.keepAspect.checked,
    yAxisInverted: els.invertY.checked,
    simplifyToleranceMm: Number(els.simplifyTolerance.value),
    lineSmoothingIterations: Number(els.lineSmoothing.value),
    minPathLengthMm: 0.8,
    travelFeedMmMin: Number(els.travelFeed.value),
    drawFeedMmMin: Number(els.drawFeed.value),
    penUpCommand: els.penUp.value.trim(),
    penDownCommand: els.penDown.value.trim(),
  };
}

function log(message, kind = "info") {
  const stamp = new Date().toLocaleTimeString();
  const prefix = kind === "error" ? "!" : kind === "rx" ? "<" : kind === "tx" ? ">" : "-";
  els.logOutput.textContent += `[${stamp}] ${prefix} ${message}\n`;
  els.logOutput.scrollTop = els.logOutput.scrollHeight;
}

function setConnectionStatus(label, mode = "idle") {
  els.connectionStatus.textContent = label;
  els.connectionStatus.classList.toggle("is-connected", mode === "connected");
  els.connectionStatus.classList.toggle("is-error", mode === "error");
  const connected = mode === "connected";
  els.connectBtn.disabled = connected;
  els.disconnectBtn.disabled = !connected;
  els.pauseBtn.disabled = !connected;
  els.resumeBtn.disabled = !connected;
  els.statusBtn.disabled = !connected;
  els.resetBtn.disabled = !connected;
  updateActionButtons();
}

function updateActionButtons() {
  const hasJob = state.gcode.length > 0;
  const connected = state.sender?.isConnected() ?? false;
  els.sendBtn.disabled = !hasJob || !connected || state.isSending;
  els.downloadBtn.disabled = !hasJob;
  els.copyBtn.disabled = !hasJob;
}

function debounce(fn, delay = 180) {
  let timer = 0;
  return (...args) => {
    clearTimeout(timer);
    timer = window.setTimeout(() => fn(...args), delay);
  };
}

async function loadImage(file) {
  const url = URL.createObjectURL(file);
  try {
    const bitmap = await createImageBitmap(file);
    state.bitmap = bitmap;
    state.image = { name: file.name, url, width: bitmap.width, height: bitmap.height };
    els.fileName.textContent = `${file.name} (${bitmap.width} x ${bitmap.height})`;
    regenerate();
  } catch (error) {
    URL.revokeObjectURL(url);
    log(`Could not load image: ${error.message}`, "error");
  }
}

function getImageData(bitmap, maxSize) {
  const scale = Math.min(1, maxSize / Math.max(bitmap.width, bitmap.height));
  const width = Math.max(1, Math.round(bitmap.width * scale));
  const height = Math.max(1, Math.round(bitmap.height * scale));
  const canvas = document.createElement("canvas");
  canvas.width = width;
  canvas.height = height;
  const ctx = canvas.getContext("2d", { willReadFrequently: true });
  ctx.drawImage(bitmap, 0, 0, width, height);
  return ctx.getImageData(0, 0, width, height);
}

function thresholdImage(imageData, threshold, invert) {
  const { data, width, height } = imageData;
  const binary = new Uint8Array(width * height);
  for (let i = 0; i < width * height; i += 1) {
    const offset = i * 4;
    const gray = data[offset] * 0.299 + data[offset + 1] * 0.587 + data[offset + 2] * 0.114;
    const dark = gray < threshold;
    binary[i] = invert ? Number(!dark) : Number(dark);
  }
  return { binary, width, height };
}

function thinBinaryImage(binaryImage) {
  const { width, height } = binaryImage;
  const pixels = new Uint8Array(binaryImage.binary);
  const index = (x, y) => y * width + x;
  const neighbor = (x, y, dx, dy) => pixels[index(x + dx, y + dy)] ? 1 : 0;
  let changed = true;
  let iterations = 0;

  const transitions = (neighbors) => {
    let count = 0;
    for (let i = 0; i < neighbors.length; i += 1) {
      const current = neighbors[i];
      const next = neighbors[(i + 1) % neighbors.length];
      if (current === 0 && next === 1) count += 1;
    }
    return count;
  };

  while (changed && iterations < 80) {
    changed = false;
    iterations += 1;

    for (const pass of [0, 1]) {
      const deleteList = [];
      for (let y = 1; y < height - 1; y += 1) {
        for (let x = 1; x < width - 1; x += 1) {
          if (!pixels[index(x, y)]) continue;

          const p2 = neighbor(x, y, 0, -1);
          const p3 = neighbor(x, y, 1, -1);
          const p4 = neighbor(x, y, 1, 0);
          const p5 = neighbor(x, y, 1, 1);
          const p6 = neighbor(x, y, 0, 1);
          const p7 = neighbor(x, y, -1, 1);
          const p8 = neighbor(x, y, -1, 0);
          const p9 = neighbor(x, y, -1, -1);
          const neighbors = [p2, p3, p4, p5, p6, p7, p8, p9];
          const count = neighbors.reduce((sum, value) => sum + value, 0);
          if (count < 2 || count > 6) continue;
          if (transitions(neighbors) !== 1) continue;

          const shouldDelete =
            pass === 0
              ? p2 * p4 * p6 === 0 && p4 * p6 * p8 === 0
              : p2 * p4 * p8 === 0 && p2 * p6 * p8 === 0;

          if (shouldDelete) deleteList.push(index(x, y));
        }
      }

      for (const pixelIndex of deleteList) {
        pixels[pixelIndex] = 0;
        changed = true;
      }
    }
  }

  return { binary: pixels, width, height };
}

function isOn(binaryImage, x, y) {
  if (x < 0 || y < 0 || x >= binaryImage.width || y >= binaryImage.height) return false;
  return binaryImage.binary[y * binaryImage.width + x] === 1;
}

function extractEdgePixels(binaryImage) {
  const { width, height, binary } = binaryImage;
  const edges = new Uint8Array(width * height);
  for (let y = 1; y < height - 1; y += 1) {
    for (let x = 1; x < width - 1; x += 1) {
      const idx = y * width + x;
      if (!binary[idx]) continue;
      if (
        !isOn(binaryImage, x - 1, y) ||
        !isOn(binaryImage, x + 1, y) ||
        !isOn(binaryImage, x, y - 1) ||
        !isOn(binaryImage, x, y + 1)
      ) {
        edges[idx] = 1;
      }
    }
  }
  return { edges, width, height };
}

function binaryToTracePixels(binaryImage) {
  return {
    edges: new Uint8Array(binaryImage.binary),
    width: binaryImage.width,
    height: binaryImage.height,
  };
}

function traceEdgePaths(edgeImage) {
  const { edges, width, height } = edgeImage;
  const visited = new Uint8Array(width * height);
  const paths = [];
  const dirs = [
    [1, 0],
    [1, 1],
    [0, 1],
    [-1, 1],
    [-1, 0],
    [-1, -1],
    [0, -1],
    [1, -1],
  ];

  const findNearestNeighbor = (x, y) => {
    let best = null;
    let bestDist = Infinity;
    for (let radius = 1; radius <= 8; radius += 1) {
      for (let dy = -radius; dy <= radius; dy += 1) {
        for (let dx = -radius; dx <= radius; dx += 1) {
          if (Math.max(Math.abs(dx), Math.abs(dy)) !== radius) continue;
          const nx = x + dx;
          const ny = y + dy;
          if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue;
          const idx = ny * width + nx;
          if (!edges[idx] || visited[idx]) continue;
          const dist = dx * dx + dy * dy;
          if (dist < bestDist) {
            best = [nx, ny];
            bestDist = dist;
          }
        }
      }
      if (best) return best;
    }
    return null;
  };

  for (let y = 0; y < height; y += 1) {
    for (let x = 0; x < width; x += 1) {
      const startIdx = y * width + x;
      if (!edges[startIdx] || visited[startIdx]) continue;

      const path = [];
      let cx = x;
      let cy = y;
      visited[startIdx] = 1;

      while (true) {
        path.push({ x: cx, y: cy });
        let next = null;
        for (const [dx, dy] of dirs) {
          const nx = cx + dx;
          const ny = cy + dy;
          if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue;
          const idx = ny * width + nx;
          if (edges[idx] && !visited[idx]) {
            next = [nx, ny];
            break;
          }
        }
        if (!next) next = findNearestNeighbor(cx, cy);
        if (!next) break;
        [cx, cy] = next;
        visited[cy * width + cx] = 1;
      }

      if (path.length > 3) paths.push(path);
    }
  }
  return paths;
}

function simplifyPath(points, tolerance) {
  if (points.length <= 2 || tolerance <= 0) return points;
  const sqTolerance = tolerance * tolerance;

  const sqSegmentDistance = (p, a, b) => {
    let x = a.x;
    let y = a.y;
    let dx = b.x - x;
    let dy = b.y - y;
    if (dx !== 0 || dy !== 0) {
      const t = Math.max(0, Math.min(1, ((p.x - x) * dx + (p.y - y) * dy) / (dx * dx + dy * dy)));
      x += dx * t;
      y += dy * t;
    }
    dx = p.x - x;
    dy = p.y - y;
    return dx * dx + dy * dy;
  };

  const simplifyDps = (start, end, output) => {
    let maxSqDist = sqTolerance;
    let index = -1;
    for (let i = start + 1; i < end; i += 1) {
      const sqDist = sqSegmentDistance(points[i], points[start], points[end]);
      if (sqDist > maxSqDist) {
        index = i;
        maxSqDist = sqDist;
      }
    }
    if (index > -1) {
      if (index - start > 1) simplifyDps(start, index, output);
      output.push(points[index]);
      if (end - index > 1) simplifyDps(index, end, output);
    }
  };

  const simplified = [points[0]];
  simplifyDps(0, points.length - 1, simplified);
  simplified.push(points[points.length - 1]);
  return simplified;
}

function mapPathsToMachine(pixelPaths, imageSize, settings) {
  const scaleX = settings.workWidthMm / imageSize.width;
  const scaleY = settings.workHeightMm / imageSize.height;
  const scale = settings.keepAspectRatio ? Math.min(scaleX, scaleY) : null;
  const usedWidth = settings.keepAspectRatio ? imageSize.width * scale : settings.workWidthMm;
  const usedHeight = settings.keepAspectRatio ? imageSize.height * scale : settings.workHeightMm;
  const offsetX = (settings.workWidthMm - usedWidth) / 2;
  const offsetY = (settings.workHeightMm - usedHeight) / 2;

  return pixelPaths
    .map((path) => {
      const points = path.map((point) => {
        const x = offsetX + point.x * (scale ?? scaleX);
        const sourceY = settings.yAxisInverted ? imageSize.height - point.y : point.y;
        const y = offsetY + sourceY * (scale ?? scaleY);
        return { x, y };
      });
      const simplified = simplifyPath(points, settings.simplifyToleranceMm);
      return { points: simplified };
    })
    .filter((path) => path.points.length > 1 && pathLength(path.points) >= settings.minPathLengthMm);
}

function pathLength(points) {
  let total = 0;
  for (let i = 1; i < points.length; i += 1) {
    total += Math.hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y);
  }
  return total;
}

function smoothPath(points, iterations) {
  if (iterations <= 0 || points.length < 3) return points;
  let smoothed = points;

  for (let iteration = 0; iteration < iterations; iteration += 1) {
    const next = [smoothed[0]];
    for (let i = 0; i < smoothed.length - 1; i += 1) {
      const current = smoothed[i];
      const target = smoothed[i + 1];
      next.push({
        x: current.x * 0.75 + target.x * 0.25,
        y: current.y * 0.75 + target.y * 0.25,
      });
      next.push({
        x: current.x * 0.25 + target.x * 0.75,
        y: current.y * 0.25 + target.y * 0.75,
      });
    }
    next.push(smoothed[smoothed.length - 1]);
    smoothed = next;
  }

  return smoothed;
}

function smoothPaths(paths, settings) {
  if (settings.lineSmoothingIterations <= 0) return paths;
  return paths.map((path) => ({
    points: smoothPath(path.points, settings.lineSmoothingIterations),
  }));
}

function orderPaths(paths) {
  const remaining = [...paths];
  const ordered = [];
  let cursor = { x: 0, y: 0 };

  while (remaining.length) {
    let bestIndex = 0;
    let bestReverse = false;
    let bestDistance = Infinity;
    for (let i = 0; i < remaining.length; i += 1) {
      const points = remaining[i].points;
      const first = points[0];
      const last = points[points.length - 1];
      const firstDist = Math.hypot(first.x - cursor.x, first.y - cursor.y);
      const lastDist = Math.hypot(last.x - cursor.x, last.y - cursor.y);
      if (firstDist < bestDistance) {
        bestIndex = i;
        bestReverse = false;
        bestDistance = firstDist;
      }
      if (lastDist < bestDistance) {
        bestIndex = i;
        bestReverse = true;
        bestDistance = lastDist;
      }
    }
    const [path] = remaining.splice(bestIndex, 1);
    if (bestReverse) path.points.reverse();
    ordered.push(path);
    cursor = path.points[path.points.length - 1];
  }

  return ordered;
}

function generateGcode(paths, settings) {
  const lines = [
    "; Image to G-Code ESP Sender",
    "; Units: millimeters",
    "G21",
    "G90",
    "G94",
    settings.penUpCommand,
    `G0 F${settings.travelFeedMmMin}`,
    `G1 F${settings.drawFeedMmMin}`,
  ];

  for (const path of paths) {
    const [start, ...rest] = path.points;
    lines.push(`G0 X${fmt(start.x)} Y${fmt(start.y)}`);
    if (settings.penDownCommand) lines.push(settings.penDownCommand);
    for (const point of rest) {
      lines.push(`G1 X${fmt(point.x)} Y${fmt(point.y)} F${settings.drawFeedMmMin}`);
    }
    if (settings.penUpCommand) lines.push(settings.penUpCommand);
  }

  lines.push("G0 X0 Y0");
  lines.push("M2");
  return lines;
}

function fmt(value) {
  return Number(value).toFixed(3).replace(/\.?0+$/, "");
}

function regenerate() {
  els.thresholdValue.textContent = els.threshold.value;
  els.lineSmoothingValue.textContent = els.lineSmoothing.value;
  if (!state.bitmap) {
    drawEmptyPreview();
    return;
  }

  const settings = readSettings();
  const imageData = getImageData(state.bitmap, settings.maxProcessSize);
  const binary = thresholdImage(imageData, settings.threshold, settings.invert);
  const traceInput = settings.centerline ? binaryToTracePixels(thinBinaryImage(binary)) : extractEdgePixels(binary);
  const pixelPaths = traceEdgePaths(traceInput);
  const machinePaths = orderPaths(smoothPaths(mapPathsToMachine(pixelPaths, imageData, settings), settings));
  state.paths = machinePaths;
  state.gcode = generateGcode(machinePaths, settings);
  els.gcodeOutput.value = state.gcode.join("\n");

  const totalLength = machinePaths.reduce((sum, path) => sum + pathLength(path.points), 0);
  els.jobStats.textContent = `${machinePaths.length} paths, ${state.gcode.length} lines, ${totalLength.toFixed(1)} mm draw length`;
  drawPreview(machinePaths, settings);
  updateActionButtons();
}

function drawEmptyPreview() {
  const canvas = els.previewCanvas;
  const ctx = canvas.getContext("2d");
  resizeCanvasToDisplaySize(canvas);
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = "#65746d";
  ctx.font = "18px system-ui";
  ctx.textAlign = "center";
  ctx.fillText("Upload an image to preview toolpaths", canvas.width / 2, canvas.height / 2);
}

function drawPreview(paths, settings) {
  const canvas = els.previewCanvas;
  const ctx = canvas.getContext("2d");
  resizeCanvasToDisplaySize(canvas);
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  const padding = 34;
  const scale = Math.min(
    (canvas.width - padding * 2) / settings.workWidthMm,
    (canvas.height - padding * 2) / settings.workHeightMm,
  );
  const offsetX = (canvas.width - settings.workWidthMm * scale) / 2;
  const offsetY = (canvas.height - settings.workHeightMm * scale) / 2;

  const toCanvas = (point) => ({
    x: offsetX + point.x * scale,
    y: offsetY + (settings.workHeightMm - point.y) * scale,
  });

  ctx.fillStyle = "#ffffff";
  ctx.strokeStyle = "#d7dfda";
  ctx.lineWidth = 1;
  ctx.fillRect(offsetX, offsetY, settings.workWidthMm * scale, settings.workHeightMm * scale);
  ctx.strokeRect(offsetX, offsetY, settings.workWidthMm * scale, settings.workHeightMm * scale);

  ctx.strokeStyle = "#167c80";
  ctx.lineWidth = Math.max(1, scale * 0.12);
  ctx.lineCap = "round";
  ctx.lineJoin = "round";

  for (const path of paths) {
    if (path.points.length < 2) continue;
    ctx.beginPath();
    const start = toCanvas(path.points[0]);
    ctx.moveTo(start.x, start.y);
    for (let i = 1; i < path.points.length; i += 1) {
      const point = toCanvas(path.points[i]);
      ctx.lineTo(point.x, point.y);
    }
    ctx.stroke();
  }

  ctx.fillStyle = "#c65331";
  ctx.beginPath();
  ctx.arc(offsetX, offsetY + settings.workHeightMm * scale, 4, 0, Math.PI * 2);
  ctx.fill();
}

function resizeCanvasToDisplaySize(canvas) {
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const width = Math.round(rect.width * dpr);
  const height = Math.round(rect.height * dpr);
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
}

class GrblWebSocketSender {
  constructor(onMessage, onClose) {
    this.socket = null;
    this.pending = null;
    this.onMessage = onMessage;
    this.onClose = onClose;
  }

  connect(url, protocols = []) {
    return new Promise((resolve, reject) => {
      this.socket = protocols.length ? new WebSocket(url, protocols) : new WebSocket(url);
      this.socket.addEventListener("open", () => resolve(this.socket.protocol || "none"), { once: true });
      this.socket.addEventListener("error", () => reject(new Error("WebSocket connection failed")), {
        once: true,
      });
      this.socket.addEventListener("message", (event) => this.handleMessage(String(event.data)));
      this.socket.addEventListener("close", () => this.onClose());
    });
  }

  isConnected() {
    return this.socket?.readyState === WebSocket.OPEN;
  }

  handleMessage(raw) {
    const messages = raw.split(/\r?\n/).map((line) => line.trim()).filter(Boolean);
    for (const message of messages) {
      this.onMessage(message);
      const lower = message.toLowerCase();
      if (!this.pending) continue;
      if (lower === "ok") {
        this.pending.resolve("ok");
        this.pending = null;
      } else if (lower.startsWith("error:") || lower.startsWith("alarm:")) {
        this.pending.reject(new Error(message));
        this.pending = null;
      }
    }
  }

  sendLine(line) {
    if (!this.isConnected()) return Promise.reject(new Error("Not connected"));
    if (this.pending) return Promise.reject(new Error("Sender is waiting for controller response"));
    return new Promise((resolve, reject) => {
      const timeout = window.setTimeout(() => {
        this.pending = null;
        reject(new Error(`Timeout waiting for ok after: ${line}`));
      }, 10000);
      this.pending = {
        resolve: (value) => {
          clearTimeout(timeout);
          resolve(value);
        },
        reject: (error) => {
          clearTimeout(timeout);
          reject(error);
        },
      };
      this.socket.send(`${line}\n`);
      log(line, "tx");
    });
  }

  sendRealtime(command) {
    if (this.isConnected()) {
      this.socket.send(command);
      log(command === "\x18" ? "Ctrl-X reset" : command, "tx");
    }
  }

  disconnect() {
    this.socket?.close();
    this.socket = null;
  }
}

class MayVeHttpSender {
  constructor(onMessage, onClose) {
    this.socket = null;
    this.baseHttpUrl = "";
    this.onMessage = onMessage;
    this.onClose = onClose;
  }

  connect(wsUrl) {
    const parsed = new URL(wsUrl);
    this.baseHttpUrl = `${parsed.protocol === "wss:" ? "https:" : "http:"}//${parsed.hostname}${parsed.port ? `:${parsed.port}` : ""}`;
    return new Promise((resolve, reject) => {
      this.socket = new WebSocket(wsUrl);
      this.socket.addEventListener("open", () => resolve("may-ve-http"), { once: true });
      this.socket.addEventListener("error", () => reject(new Error("Máy Vẽ status WebSocket failed")), {
        once: true,
      });
      this.socket.addEventListener("message", (event) => this.handleMessage(String(event.data)));
      this.socket.addEventListener("close", () => this.onClose());
    });
  }

  isConnected() {
    return this.socket?.readyState === WebSocket.OPEN;
  }

  handleMessage(raw) {
    try {
      const data = JSON.parse(raw);
      const parts = [];
      if (data.x !== undefined && data.y !== undefined) {
        parts.push(`X${Number(data.x).toFixed(2)} Y${Number(data.y).toFixed(2)}`);
      }
      if (data.status) parts.push(data.status);
      if (data.ver) parts.push(`Firmware ${data.ver}`);
      if (parts.length) this.onMessage(parts.join(" | "));
    } catch (error) {
      this.onMessage(raw);
    }
  }

  async post(path, body = "") {
    if (!this.baseHttpUrl) throw new Error("HTTP endpoint is not ready");
    await fetch(`${this.baseHttpUrl}${path}`, {
      method: "POST",
      mode: "no-cors",
      body,
    });
  }

  async sendLine(line) {
    log(line, "tx");
    await this.post("/cmd", line);
    return "ok";
  }

  async sendJob(lines) {
    const body = lines.join("\n");
    log(`POST /gcode (${lines.length} lines)`, "tx");
    await this.post("/gcode", body);
    return "ok";
  }

  sendRealtime(command) {
    if (command === "!" || command === "\x18") {
      if (command === "\x18") {
        this.post("/cmd", "G28").then(() => log("RESET -> G28 home", "tx"));
        return;
      }
      this.post("/stop").then(() => log("STOP", "tx"));
      return;
    }
    if (command === "?") {
      log("Status is received from /ws automatically");
      return;
    }
    if (command === "~") {
      log("Resume is not exposed by this Máy Vẽ HTTP API");
    }
  }

  disconnect() {
    this.socket?.close();
    this.socket = null;
  }
}

function buildConnectionCandidates(rawUrl, mode) {
  const base = rawUrl.trim();
  const candidates = [];
  const add = (url, protocols = [], label = url, type = "grbl-ws") => {
    if (!url) return;
    const key = `${type}|${url}|${protocols.join(",")}`;
    if (!candidates.some((candidate) => candidate.key === key)) {
      candidates.push({ key, url, protocols, label, type });
    }
  };

  let parsed = null;
  try {
    parsed = new URL(base);
  } catch (error) {
    add(base, mode === "webui-v3" ? ["webui-v3"] : [], base);
    return candidates;
  }

  const host = parsed.hostname;
  const path = parsed.pathname === "/" ? "/" : parsed.pathname;
  const protocol = parsed.protocol === "wss:" ? "wss:" : "ws:";
  const makeUrl = (port) => `${protocol}//${host}${port ? `:${port}` : ""}${path}`;
  const makeMayVeUrl = () => `${protocol}//${host}/ws`;

  if (mode === "may-ve-http") {
    add(makeMayVeUrl(), [], "Máy Vẽ HTTP API status socket", "may-ve-http");
    return candidates;
  }

  if (mode === "raw") {
    add(base, [], "Raw GRBL WebSocket");
    return candidates;
  }

  if (mode === "webui-v3") {
    add(base, ["webui-v3"], "FluidNC WebUI v3");
    if (parsed.port !== "82") add(makeUrl("82"), ["webui-v3"], "FluidNC WebUI v3 on port 82");
    return candidates;
  }

  add(makeMayVeUrl(), [], "Máy Vẽ HTTP API status socket", "may-ve-http");
  add(base, [], "Raw GRBL WebSocket");
  add(base, ["webui-v3"], "FluidNC WebUI v3 at entered URL");
  add(makeUrl("82"), ["webui-v3"], "FluidNC WebUI v3 on port 82");
  add(makeUrl("81"), [], "FluidNC WebUI v2/raw on port 81");
  add(makeUrl("80"), ["webui-v3"], "FluidNC WebSocket on port 80 with webui-v3");
  add(makeUrl("80"), [], "Raw WebSocket on port 80");
  return candidates;
}

async function connect() {
  const candidates = buildConnectionCandidates(els.wsUrl.value, els.wsMode.value);
  let lastError = null;
  try {
    for (const candidate of candidates) {
      const sender = candidate.type === "may-ve-http" ? new MayVeHttpSender(
        (message) => log(message, "rx"),
        () => {
          state.sender = null;
          setConnectionStatus("Disconnected");
          log("Connection closed");
        },
      ) : new GrblWebSocketSender(
        (message) => log(message, "rx"),
        () => {
          state.sender = null;
          setConnectionStatus("Disconnected");
          log("Connection closed");
        },
      );

      try {
        const protocolText = candidate.protocols.length ? ` (${candidate.protocols.join(", ")})` : "";
        log(`Trying ${candidate.label}: ${candidate.url}${protocolText}`);
        const acceptedProtocol = candidate.type === "may-ve-http"
          ? await sender.connect(candidate.url)
          : await sender.connect(candidate.url, candidate.protocols);
        state.sender = sender;
        els.wsUrl.value = candidate.url;
        setConnectionStatus("Connected", "connected");
        log(`Connected using ${candidate.label}; protocol: ${acceptedProtocol}`);
        return;
      } catch (error) {
        sender.disconnect();
        lastError = error;
        log(`${candidate.label} failed: ${error.message}`, "error");
      }
    }
    throw lastError ?? new Error("No WebSocket candidates were available");
  } catch (error) {
    setConnectionStatus("Connection error", "error");
    log(`Unable to connect. Try opening the ESP WebUI in the browser and check whether it uses port 81, 82, or a v3 WebSocket. Last error: ${error.message}`, "error");
  }
}

async function sendJob() {
  if (!state.sender?.isConnected() || !state.gcode.length) return;
  state.isSending = true;
  updateActionButtons();
  try {
    const lines = state.gcode.filter((line) => {
      const trimmed = line.trim();
      return trimmed && !trimmed.startsWith(";");
    });
    if (typeof state.sender.sendJob === "function") {
      await state.sender.sendJob(lines);
      els.jobStats.textContent = `Posted ${lines.length} lines to ESP`;
    } else {
      for (let i = 0; i < lines.length; i += 1) {
        await state.sender.sendLine(lines[i]);
        els.jobStats.textContent = `Sending ${i + 1} / ${lines.length} lines`;
      }
    }
    log("Job complete");
  } catch (error) {
    log(`Job stopped: ${error.message}`, "error");
  } finally {
    state.isSending = false;
    updateActionButtons();
    regenerate();
  }
}

function downloadGcode() {
  const blob = new Blob([state.gcode.join("\n")], { type: "text/plain" });
  const url = URL.createObjectURL(blob);
  const link = document.createElement("a");
  link.href = url;
  link.download = "image-job.gcode";
  link.click();
  URL.revokeObjectURL(url);
}

function applyPreset(preset) {
  if (preset === "laser") {
    els.penUp.value = "M5";
    els.penDown.value = "M3 S1000";
  } else {
    els.penUp.value = "G0 Z5";
    els.penDown.value = "G1 Z0 F300";
  }
  regenerate();
}

els.imageInput.addEventListener("change", (event) => {
  const [file] = event.target.files;
  if (file) loadImage(file);
});

const debouncedRegenerate = debounce(regenerate);
[
  els.threshold,
  els.invert,
  els.centerline,
  els.maxProcessSize,
  els.workWidth,
  els.workHeight,
  els.keepAspect,
  els.invertY,
  els.simplifyTolerance,
  els.lineSmoothing,
  els.travelFeed,
  els.drawFeed,
  els.penUp,
  els.penDown,
].forEach((el) => el.addEventListener("input", debouncedRegenerate));

document.querySelectorAll("[data-preset]").forEach((button) => {
  button.addEventListener("click", () => applyPreset(button.dataset.preset));
});

els.regenerateBtn.addEventListener("click", regenerate);
els.downloadBtn.addEventListener("click", downloadGcode);
els.copyBtn.addEventListener("click", async () => {
  await navigator.clipboard.writeText(state.gcode.join("\n"));
  log("G-code copied to clipboard");
});
els.clearLogBtn.addEventListener("click", () => {
  els.logOutput.textContent = "";
});
els.connectBtn.addEventListener("click", connect);
els.disconnectBtn.addEventListener("click", () => state.sender?.disconnect());
els.sendBtn.addEventListener("click", sendJob);
els.pauseBtn.addEventListener("click", () => state.sender?.sendRealtime("!"));
els.resumeBtn.addEventListener("click", () => state.sender?.sendRealtime("~"));
els.statusBtn.addEventListener("click", () => state.sender?.sendRealtime("?"));
els.resetBtn.addEventListener("click", () => state.sender?.sendRealtime("\x18"));
window.addEventListener("resize", debounce(() => drawPreview(state.paths, readSettings()), 100));

drawEmptyPreview();
setConnectionStatus("Disconnected");
