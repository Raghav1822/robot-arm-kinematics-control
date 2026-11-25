import test from 'node:test';
import assert from 'node:assert/strict';
import { forwardKinematics, inverseKinematics, quinticBlend, interpolatePoint } from '../kinematics.js';

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
