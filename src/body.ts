import { type Vec2, vAdd, vCross, vDot as vDot2, vLen, vRotate, vSub } from "./math";

export type Shape =
  | { kind: "circle"; radius: number }
  | { kind: "polygon"; vertsLocal: Vec2[] };

let nextId = 1;

export class RigidBody {
  readonly id = nextId++;

  position: Vec2;
  angle: number;
  linearVelocity: Vec2;
  angularVelocity: number;
  inverseMass: number;
  inverseInertia: number;
  friction = 0.35;
  restitution = 0.05;
  shape: Shape;

  constructor(shape: Shape, position: Vec2, mass: number) {
    this.shape = shape;
    this.position = { ...position };
    this.angle = 0;
    this.linearVelocity = { x: 0, y: 0 };
    this.angularVelocity = 0;
    if (!Number.isFinite(mass) || mass <= 0) {
      this.inverseMass = 0;
      this.inverseInertia = 0;
    } else {
      this.inverseMass = 1 / mass;
      const I = inertiaFor(shape, mass);
      this.inverseInertia = I > 0 ? 1 / I : 0;
    }
  }

  getWorldVertices(out: Vec2[] = []): Vec2[] {
    out.length = 0;
    if (this.shape.kind === "polygon") {
      for (const v of this.shape.vertsLocal) {
        out.push(vAdd(this.position, vRotate(v, this.angle)));
      }
    }
    return out;
  }

  worldPoint(local: Vec2): Vec2 {
    return vAdd(this.position, vRotate(local, this.angle));
  }

  velocityAt(worldPoint: Vec2): Vec2 {
    const r = vSub(worldPoint, this.position);
    const tangential = {
      x: -this.angularVelocity * r.y,
      y: this.angularVelocity * r.x,
    };
    return vAdd(this.linearVelocity, tangential);
  }

  /** Axis-aligned bounding box in world space (conservative). */
  aabb(min: Vec2, max: Vec2): void {
    if (this.shape.kind === "circle") {
      const r = this.shape.radius;
      min.x = this.position.x - r;
      min.y = this.position.y - r;
      max.x = this.position.x + r;
      max.y = this.position.y + r;
      return;
    }
    const verts = this.getWorldVertices();
    let x0 = Infinity,
      y0 = Infinity,
      x1 = -Infinity,
      y1 = -Infinity;
    for (const p of verts) {
      x0 = Math.min(x0, p.x);
      y0 = Math.min(y0, p.y);
      x1 = Math.max(x1, p.x);
      y1 = Math.max(y1, p.y);
    }
    min.x = x0;
    min.y = y0;
    max.x = x1;
    max.y = y1;
  }

  boundsRadius(): number {
    if (this.shape.kind === "circle") return this.shape.radius;
    let m = 0;
    for (const v of this.shape.vertsLocal) {
      m = Math.max(m, vLen(v));
    }
    return m;
  }
}

function inertiaFor(shape: Shape, mass: number): number {
  if (shape.kind === "circle") {
    return 0.5 * mass * shape.radius * shape.radius;
  }
  const verts = shape.vertsLocal;
  if (verts.length < 3) return 0;
  let twiceArea = 0;
  let num = 0;
  const n = verts.length;
  for (let i = 0; i < n; i++) {
    const v1 = verts[i]!;
    const v2 = verts[(i + 1) % n]!;
    const cross = vCross(v1, v2);
    twiceArea += cross;
    num += cross * (vDot2(v1, v1) + vDot2(v2, v2) + vDot2(v1, v2));
  }
  const absTwiceArea = Math.abs(twiceArea);
  if (absTwiceArea < 1e-12) return 0;
  const area = absTwiceArea * 0.5;
  const term = Math.abs(num);
  return ((mass / (6 * area)) * term);
}

export function makeBox(halfW: number, halfH: number): Vec2[] {
  return [
    { x: -halfW, y: -halfH },
    { x: halfW, y: -halfH },
    { x: halfW, y: halfH },
    { x: -halfW, y: halfH },
  ];
}

export function createCircle(
  radius: number,
  position: Vec2,
  mass: number,
): RigidBody {
  return new RigidBody({ kind: "circle", radius }, position, mass);
}

export function createPolygon(
  vertsLocal: Vec2[],
  position: Vec2,
  mass: number,
): RigidBody {
  return new RigidBody({ kind: "polygon", vertsLocal: [...vertsLocal] }, position, mass);
}
