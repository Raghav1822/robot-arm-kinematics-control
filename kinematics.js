export const DEG = Math.PI / 180;

export function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

export function wrapAngle(rad) {
  let a = rad;
  while (a > Math.PI) a -= 2 * Math.PI;
  while (a < -Math.PI) a += 2 * Math.PI;
  return a;
}

export function forwardKinematics(angles, lengths) {
  let x = 0;
  let y = 0;
  let heading = 0;
  const points = [{ x, y }];

  for (let i = 0; i < angles.length; i++) {
    heading += angles[i];
    x += lengths[i] * Math.cos(heading);
    y += lengths[i] * Math.sin(heading);
    points.push({ x, y });
  }

  return { x, y, heading, points };
}

export function jacobian(angles, lengths) {
  const joints = forwardKinematics(angles, lengths).points;
  const tool = joints[joints.length - 1];

  return angles.map((_, i) => {
    let heading = 0;
    for (let j = 0; j <= i; j++) heading += angles[j];
    void heading;
    let dx = 0;
    let dy = 0;
    for (let j = i; j < angles.length; j++) {
      let theta = 0;
      for (let k = 0; k <= j; k++) theta += angles[k];
      dx += -lengths[j] * Math.sin(theta);
      dy += lengths[j] * Math.cos(theta);
    }
    return [dx, dy];
  }).map((joint) => [joint[0], joint[1]]);
}

export function inverseKinematics(target, startAngles, lengths, options = {}) {
  const maxIterations = options.maxIterations ?? 240;
  const tolerance = options.tolerance ?? 0.5;
  const damping = options.damping ?? 0.001;
  const angles = startAngles.map(wrapAngle);

  for (let iteration = 0; iteration < maxIterations; iteration++) {
    const fk = forwardKinematics(angles, lengths);
    const ex = target.x - fk.x;
    const ey = target.y - fk.y;
    const error = Math.hypot(ex, ey);
    if (error < tolerance) return { angles, error, iterations: iteration, reached: true };

    const J = jacobian(angles, lengths);
    const jx = J.map((column) => column[0]);
    const jy = J.map((column) => column[1]);
    const a = jx.reduce((sum, value, i) => sum + value * jx[i], 0);
    const b = jx.reduce((sum, value, i) => sum + value * jy[i], 0);
    const c = jy.reduce((sum, value, i) => sum + value * jy[i], 0);

    const aa = a + damping;
    const bb = b;
    const cc = c + damping;
    const determinant = aa * cc - bb * bb;
    if (Math.abs(determinant) < 1e-10) continue;

    const lambdaX = (cc * ex - bb * ey) / determinant;
    const lambdaY = (-bb * ex + aa * ey) / determinant;

    for (let i = 0; i < angles.length; i++) {
      angles[i] = wrapAngle(angles[i] + jx[i] * lambdaX + jy[i] * lambdaY);
      angles[i] = clamp(angles[i], -170 * DEG, 170 * DEG);
    }
  }

  const finalFk = forwardKinematics(angles, lengths);
  const error = Math.hypot(target.x - finalFk.x, target.y - finalFk.y);
  return { angles, error, iterations: maxIterations, reached: error < tolerance };
}

export function quinticBlend(s) {
  const t = clamp(s, 0, 1);
  return 10 * t ** 3 - 15 * t ** 4 + 6 * t ** 5;
}

export function interpolatePoint(start, end, fraction) {
  const s = quinticBlend(fraction);
  return { x: start.x + (end.x - start.x) * s, y: start.y + (end.y - start.y) * s };
}

export function stepJointController(angles, velocities, desiredAngles, dt, kp, kd) {
  const nextAngles = [];
  const nextVelocities = [];

  for (let i = 0; i < angles.length; i++) {
    const error = wrapAngle(desiredAngles[i] - angles[i]);
    const acceleration = kp * error - kd * velocities[i];
    const velocity = (velocities[i] + acceleration * dt) * 0.995;
    nextVelocities.push(velocity);
    nextAngles.push(wrapAngle(angles[i] + velocity * dt));
  }

  return { angles: nextAngles, velocities: nextVelocities };
}
