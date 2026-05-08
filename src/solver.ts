import { type RigidBody } from "./body";
import { type Vec2, vCross, vDot, vPerp, vScale, vSub } from "./math";
import type { Manifold } from "./collision/contacts";

function tangentFromNormal(normal: Vec2): Vec2 {
  return vPerp(normal);
}

function velocityAt(body: RigidBody, contact: Vec2): Vec2 {
  const r = vSub(contact, body.position);
  return {
    x: body.linearVelocity.x - body.angularVelocity * r.y,
    y: body.linearVelocity.y + body.angularVelocity * r.x,
  };
}

function linearPositionalCorrection(
  manifold: Manifold,
  beta: number,
  slop: number,
): void {
  const { a, b, penetration, normal } = manifold;
  const invMa = a.inverseMass;
  const invMb = b.inverseMass;
  const invSum = invMa + invMb;
  if (invSum <= 0) return;
  const corr = penetration - slop;
  if (corr <= 0) return;

  let clampedMag = beta * corr;
  const maxCorr = 0.25;
  if (clampedMag > maxCorr) clampedMag = maxCorr;

  const scaleA = -(invMa / invSum) * clampedMag;
  const scaleB = (invMb / invSum) * clampedMag;
  if (invMa > 0) {
    a.position.x += scaleA * normal.x;
    a.position.y += scaleA * normal.y;
  }
  if (invMb > 0) {
    b.position.x += scaleB * normal.x;
    b.position.y += scaleB * normal.y;
  }
}

function applyLinAng(body: RigidBody, impulse: Vec2, r: Vec2): void {
  if (body.inverseMass <= 0) return;
  body.linearVelocity.x += body.inverseMass * impulse.x;
  body.linearVelocity.y += body.inverseMass * impulse.y;
  const rz = body.inverseInertia * (r.x * impulse.y - r.y * impulse.x);
  body.angularVelocity += rz;
}

function applyImpulses(
  a: RigidBody,
  b: RigidBody,
  normal: Vec2,
  contact: Vec2,
  tangent: Vec2,
): void {
  const ra = vSub(contact, a.position);
  const rb = vSub(contact, b.position);

  const va = velocityAt(a, contact);
  const vb = velocityAt(b, contact);
  const rv = vSub(vb, va);

  const restitution =
    Math.max(
      Math.min(a.restitution, b.restitution),
      0,
    );
  const mu = Math.max(Math.min(a.friction, b.friction), 0);

  const vn = vDot(rv, normal);

  /** small bias allows resting stacking without jittering bounce */
  if (vn > 8e-3) return;

  const rnA = vCross(ra, normal);
  const rnB = vCross(rb, normal);
  const denomNormal =
    a.inverseMass +
    b.inverseMass +
    rnA * rnA * a.inverseInertia +
    rnB * rnB * b.inverseInertia;

  const jnNumer = -(1 + restitution) * vn;
  if (denomNormal <= 1e-14 || !Number.isFinite(denomNormal)) return;
  let jn = jnNumer / denomNormal;

  const rtA = vCross(ra, tangent);
  const rtB = vCross(rb, tangent);
  const denomTangent =
    a.inverseMass +
    b.inverseMass +
    rtA * rtA * a.inverseInertia +
    rtB * rtB * b.inverseInertia;

  const vt = vDot(rv, tangent);
  const jtFree = denomTangent <= 1e-14 ? 0 : -vt / denomTangent;
  const maxT = mu * Math.abs(jn);
  let jt = jtFree;
  if (jt > maxT) jt = maxT;
  if (jt < -maxT) jt = -maxT;

  const impulseB = {
    x: normal.x * jn + tangent.x * jt,
    y: normal.y * jn + tangent.y * jt,
  };
  applyLinAng(b, impulseB, rb);
  applyLinAng(a, vScale(impulseB, -1), ra);
}

export function sequentialImpulses(
  manifolds: Manifold[],
  iterations: number,
  betaCorrection: number,
  slop: number,
): void {
  for (const manifold of manifolds) {
    linearPositionalCorrection(manifold, betaCorrection, slop);
  }

  for (let iter = 0; iter < iterations; iter++) {
    for (const manifold of manifolds) {
      const tang = tangentFromNormal(manifold.normal);
      for (const contact of manifold.contacts) {
        applyImpulses(manifold.a, manifold.b, manifold.normal, contact, tang);
      }
    }
  }
}
