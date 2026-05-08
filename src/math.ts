export type Vec2 = { x: number; y: number };

export const vec2 = (x = 0, y = 0): Vec2 => ({ x, y });

export function vAdd(a: Vec2, b: Vec2): Vec2 {
  return { x: a.x + b.x, y: a.y + b.y };
}

export function vSub(a: Vec2, b: Vec2): Vec2 {
  return { x: a.x - b.x, y: a.y - b.y };
}

export function vScale(a: Vec2, s: number): Vec2 {
  return { x: a.x * s, y: a.y * s };
}

export function vDot(a: Vec2, b: Vec2): number {
  return a.x * b.x + a.y * b.y;
}

/** 2D cross product (scalar z of a x b) */
export function vCross(a: Vec2, b: Vec2): number {
  return a.x * b.y - a.y * b.x;
}

/** Perpendicular (left normal): (-y, x) */
export function vPerp(a: Vec2): Vec2 {
  return { x: -a.y, y: a.x };
}

export function vLenSq(a: Vec2): number {
  return a.x * a.x + a.y * a.y;
}

export function vLen(a: Vec2): number {
  return Math.hypot(a.x, a.y);
}

export function vNormalize(a: Vec2): Vec2 {
  const l = vLen(a);
  if (l < 1e-12) return { x: 0, y: 0 };
  return { x: a.x / l, y: a.y / l };
}

export function vRotate(v: Vec2, angle: number): Vec2 {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return { x: c * v.x - s * v.y, y: s * v.x + c * v.y };
}

export function vLerp(a: Vec2, b: Vec2, t: number): Vec2 {
  return { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t };
}

export function vNeg(a: Vec2): Vec2 {
  return { x: -a.x, y: -a.y };
}
