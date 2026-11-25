import test from 'node:test';
import assert from 'node:assert/strict';
import { forwardKinematics, inverseKinematics, quinticBlend, interpolatePoint, stepJointController, wrapAngle } from '../kinematics.js';

const lengths = [220, 220, 220];

test('forward kinematics of zero angles matches workspace origin + reach', () => {
  const result = forwardKinematics([0, 0, 0], lengths);
  assert.equal(result.x, 660);
  assert.equal(result.y, 0);
});

test('inverse kinematics reaches a representative target', () => {
  const target = { x: 420, y: 260 };
  const result = inverseKinematics(target, [-25 * Math.PI / 180, 35 * Math.PI / 180, 20 * Math.PI / 180], lengths);
  assert.equal(result.reached, true);
  assert.ok(result.error < 1.2);
});

test('quintic profile has zero endpoints', () => {
  assert.equal(quinticBlend(0), 0);
  assert.equal(quinticBlend(1), 1);
});

test('interpolation reaches the exact target', () => {
  const point = interpolatePoint({ x: 10, y: 20 }, { x: 110, y: 220 }, 1);
  assert.deepEqual(point, { x: 110, y: 220 });
});

test('joint controller settles the tool at its target after the trajectory ends', () => {
  const target = { x: -38, y: 348 };
  let angles = [-25, 35, 20].map((angle) => angle * Math.PI / 180);
  let velocities = [0, 0, 0];
  const startAngles = [...angles];
  const result = inverseKinematics(target, angles, lengths, { tolerance: 1.2, damping: 0.0008 });
  const dt = 1 / 60;
  let tipError = Infinity;

  for (let frame = 1; frame <= 360; frame++) {
    const fraction = Math.min(frame * dt / 3, 1);
    const blend = quinticBlend(fraction);
    const desiredAngles = startAngles.map((start, index) =>
      wrapAngle(start + wrapAngle(result.angles[index] - start) * blend));
    ({ angles, velocities } = stepJointController(angles, velocities, desiredAngles, dt, 18, 2.2));
  }

  for (let frame = 0; frame < 180; frame++) {
    ({ angles, velocities } = stepJointController(angles, velocities, result.angles, dt, 18, 2.2));
    const fk = forwardKinematics(angles, lengths);
    tipError = Math.hypot(target.x - fk.x, target.y - fk.y);
    if (tipError <= 1.5 && Math.max(...velocities.map(Math.abs)) <= 0.02) break;
  }

  assert.ok(tipError <= 1.5, `expected tip to settle within 1.5 mm, got ${tipError.toFixed(2)} mm`);
});
