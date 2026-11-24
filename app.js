import { forwardKinematics, inverseKinematics, interpolatePoint, quinticBlend, DEG, wrapAngle } from './kinematics.js';

const canvas = document.getElementById('robotCanvas');
const ctx = canvas.getContext('2d');
const lengths = [220, 220, 220];
let joints = [-25 * DEG, 35 * DEG, 20 * DEG];
let velocities = [0, 0, 0];
let target = { x: 420, y: 260 };
let targetAngles = [...joints];
let startAngles = [...joints];
let plannedPoints = [];
let running = false;
let planStart = 0;
let planDuration = 3;
let lastTime = performance.now();

const ui = {
  x: document.getElementById('xTarget'), y: document.getElementById('yTarget'),
  duration: document.getElementById('duration'), kp: document.getElementById('kp'), kd: document.getElementById('kd'),
  xOut: document.getElementById('xOut'), yOut: document.getElementById('yOut'), timeOut: document.getElementById('timeOut'),
  kpOut: document.getElementById('kpOut'), kdOut: document.getElementById('kdOut'),
  error: document.getElementById('errorMetric'), reach: document.getElementById('reachMetric'), ik: document.getElementById('ikMetric'),
  status: document.getElementById('status')
};

function toCanvas(point) {
  return { x: canvas.width / 2 + point.x, y: canvas.height * 0.82 - point.y };
}

function drawGrid() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.strokeStyle = 'rgba(128, 168, 195, 0.12)';
  ctx.lineWidth = 1;
  for (let r = 100; r <= 600; r += 100) {
    ctx.beginPath(); ctx.arc(canvas.width / 2, canvas.height * 0.82, r, Math.PI, 2 * Math.PI); ctx.stroke();
  }
  ctx.beginPath(); ctx.moveTo(0, canvas.height * 0.82); ctx.lineTo(canvas.width, canvas.height * 0.82); ctx.stroke();
}

function drawCircle(point, radius, color) {
  const p = toCanvas(point); ctx.beginPath(); ctx.arc(p.x, p.y, radius, 0, Math.PI * 2); ctx.fillStyle = color; ctx.fill();
}

function drawArm() {
  const fk = forwardKinematics(joints, lengths);
  const points = fk.points.map(toCanvas);
  ctx.lineJoin = 'round'; ctx.lineCap = 'round';
  ctx.strokeStyle = '#58d7e8'; ctx.lineWidth = 18;
  ctx.beginPath(); ctx.moveTo(points[0].x, points[0].y);
  for (let i = 1; i < points.length; i++) ctx.lineTo(points[i].x, points[i].y);
  ctx.stroke();
  ctx.strokeStyle = '#dffaff'; ctx.lineWidth = 3; ctx.stroke();
  points.forEach((p, i) => { ctx.beginPath(); ctx.arc(p.x, p.y, i === points.length - 1 ? 9 : 12, 0, Math.PI * 2); ctx.fillStyle = i === points.length - 1 ? '#58d7e8' : '#0d2032'; ctx.fill(); ctx.strokeStyle = '#a8eaf3'; ctx.stroke(); });
  drawCircle(target, 10, '#ff9f43');
}

function drawPath() {
  if (plannedPoints.length < 2) return;
  ctx.beginPath(); plannedPoints.forEach((point, i) => { const p = toCanvas(point); if (i === 0) ctx.moveTo(p.x, p.y); else ctx.lineTo(p.x, p.y); });
  ctx.strokeStyle = 'rgba(175, 195, 210, 0.55)'; ctx.lineWidth = 2; ctx.setLineDash([7, 7]); ctx.stroke(); ctx.setLineDash([]);
}

function planMotion() {
  const startFk = forwardKinematics(joints, lengths);
  const start = { x: startFk.x, y: startFk.y };
  target = { x: Number(ui.x.value), y: Number(ui.y.value) };
  planDuration = Number(ui.duration.value);
  startAngles = [...joints];

  const result = inverseKinematics(target, joints, lengths, { tolerance: 1.2, damping: 0.0008 });
  targetAngles = result.angles;
  ui.ik.textContent = result.reached ? `Reached (λ=${result.iterations})` : `${result.error.toFixed(1)} mm`;

  plannedPoints = [];
  for (let i = 0; i <= 80; i++) plannedPoints.push(interpolatePoint(start, target, i / 80));

  running = true;
  planStart = performance.now();
  ui.status.textContent = 'Trajectory active';
}

function update(dt) {
  if (running) {
    const elapsed = (performance.now() - planStart) / 1000;
    const fraction = Math.min(elapsed / planDuration, 1);
    const blend = quinticBlend(fraction);

    const kp = Number(ui.kp.value);
    const kd = Number(ui.kd.value);
    for (let i = 0; i < joints.length; i++) {
      const desired = wrapAngle(startAngles[i] + wrapAngle(targetAngles[i] - startAngles[i]) * blend);
      const error = wrapAngle(desired - joints[i]);
      const acceleration = kp * error - kd * velocities[i];
      velocities[i] += acceleration * dt;
      velocities[i] *= 0.995;
      joints[i] = wrapAngle(joints[i] + velocities[i] * dt);
    }

    if (fraction >= 1) running = false;
  }
}

function animate(now) {
  const dt = Math.min((now - lastTime) / 1000, 0.032); lastTime = now;
  update(dt); drawGrid(); drawPath(); drawArm();
  const fk = forwardKinematics(joints, lengths);
  ui.error.textContent = `${Math.hypot(target.x - fk.x, target.y - fk.y).toFixed(1)} mm`;
  if (!running) ui.status.textContent = 'Simulation ready';
  requestAnimationFrame(animate);
}

function syncControls() {
  ui.xOut.textContent = ui.x.value; ui.yOut.textContent = ui.y.value; ui.timeOut.textContent = Number(ui.duration.value).toFixed(1);
  ui.kpOut.textContent = Number(ui.kp.value).toFixed(1); ui.kdOut.textContent = Number(ui.kd.value).toFixed(1);
}

[ui.x, ui.y, ui.duration, ui.kp, ui.kd].forEach((input) => input.addEventListener('input', syncControls));
document.getElementById('planBtn').addEventListener('click', planMotion);
document.getElementById('stopBtn').addEventListener('click', () => { running = false; plannedPoints = []; ui.status.textContent = 'Stopped'; });
canvas.addEventListener('click', (event) => {
  const rect = canvas.getBoundingClientRect();
  ui.x.value = String(Math.round(Math.max(-430, Math.min(430, event.clientX - rect.left - canvas.width / 2))));
  ui.y.value = String(Math.round(Math.max(-100, Math.min(520, canvas.height * 0.82 - (event.clientY - rect.top)))));
  syncControls(); planMotion();
});

syncControls(); requestAnimationFrame(animate);
