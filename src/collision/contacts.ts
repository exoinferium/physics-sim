import { type RigidBody } from "../body";
import {
  type Vec2,
  vAdd,
  vCross,
  vDot,
  vLenSq,
  vNormalize,
  vPerp,
  vScale,
  vSub,
} from "../math";
import { polygonAreaSigned, satConvexConvex } from "./sat";

export type Manifold = {
  a: RigidBody;
  b: RigidBody;
  /** Unit normal from A toward B */
  normal: Vec2;
  penetration: number;
  contacts: Vec2[];
};

function midpoint(p: Vec2, q: Vec2): Vec2 {
  return { x: (p.x + q.x) * 0.5, y: (p.y + q.y) * 0.5 };
}

function edgeFromIndex(verts: Vec2[], i: number): [Vec2, Vec2] {
  return [verts[i]!, verts[(i + 1) % verts.length]!];
}

function projectOntoSegment(p: Vec2, a: Vec2, b: Vec2): Vec2 {
  const ab = vSub(b, a);
  const denom = vLenSq(ab);
  if (denom < 1e-12) return { ...a };
  let t = vDot(vSub(p, a), ab) / denom;
  t = Math.max(0, Math.min(1, t));
  return vAdd(a, vScale(ab, t));
}

function dedupeContacts(pts: Vec2[], eps: number): Vec2[] {
  const out: Vec2[] = [];
  for (const p of pts) {
    let dup = false;
    for (const q of out) {
      if (vLenSq(vSub(p, q)) < eps * eps) {
        dup = true;
        break;
      }
    }
    if (!dup) out.push(p);
  }
  return out;
}

/**
 * Find incident edge on `incVerts` — outward normal minimizes dot(..., manifoldNormalFromAtoB).
 */
function findIncidentEdgeIndex(incVerts: Vec2[], normalAtoB: Vec2): number {
  const n = incVerts.length;
  let best = Infinity;
  let idx = 0;
  for (let i = 0; i < n; i++) {
    const a = incVerts[i]!;
    const b = incVerts[(i + 1) % n]!;
    const edge = vSub(b, a);
    const inward = vNormalize(vPerp(edge));
    const outward = vScale(inward, -1);
    const d = vDot(outward, normalAtoB);
    if (d < best) {
      best = d;
      idx = i;
    }
  }
  return idx;
}

/** Two contact candidates by projecting incident edge onto reference segment */
function polyPolyContactsWorld(
  refVerts: Vec2[],
  refEdgeIdx: number,
  incVerts: Vec2[],
  manifoldNormalAtoB: Vec2,
): Vec2[] {
  const [R1, R2] = edgeFromIndex(refVerts, refEdgeIdx);
  const iEdge = findIncidentEdgeIndex(incVerts, manifoldNormalAtoB);
  const I1 = incVerts[iEdge]!;
  const I2 = incVerts[(iEdge + 1) % incVerts.length]!;
  const p1 = projectOntoSegment(I1, R1, R2);
  const p2 = projectOntoSegment(I2, R1, R2);
  return dedupeContacts([p1, p2], 1e-2);
}

export function collideConvexPolygons(
  a: RigidBody,
  b: RigidBody,
  vertsAW: Vec2[],
  vertsBW: Vec2[],
): Manifold | null {
  const sat = satConvexConvex(vertsAW, vertsBW, a.position, b.position);
  if (!sat) return null;
  const normal = sat.normal;

  let refVerts: Vec2[];
  let incVerts: Vec2[];
  const refEdgeIdx = sat.refEdgeIdx;

  if (sat.refOnA) {
    refVerts = vertsAW;
    incVerts = vertsBW;
  } else {
    refVerts = vertsBW;
    incVerts = vertsAW;
  }

  const contacts = polyPolyContactsWorld(refVerts, refEdgeIdx, incVerts, normal);
  if (contacts.length === 0) {
    /** Fallback deep point on reference edge */
    const [R1, R2] = edgeFromIndex(refVerts, refEdgeIdx);
    contacts.push(midpoint(R1, R2));
  }

  return {
    a,
    b,
    normal,
    penetration: sat.depth,
    contacts,
  };
}

export function collideCircles(a: RigidBody, b: RigidBody): Manifold | null {
  if (a.shape.kind !== "circle" || b.shape.kind !== "circle") return null;
  const ra = a.shape.radius;
  const rb = b.shape.radius;
  const d = vSub(b.position, a.position);
  const distSq = vLenSq(d);
  const rt = ra + rb;
  if (distSq > rt * rt) return null;
  const dist = Math.sqrt(distSq);
  let normal: Vec2;
  let penetration: number;
  if (dist > 1e-8) {
    normal = vScale(d, 1 / dist);
    penetration = rt - dist;
  } else {
    normal = { x: 1, y: 0 };
    penetration = rt;
  }
  const pa = vAdd(a.position, vScale(normal, ra));
  const pb = vSub(b.position, vScale(normal, rb));
  return {
    a,
    b,
    normal,
    penetration,
    contacts: [midpoint(pa, pb)],
  };
}

/** Closest features circle vs convex CCW polygon in world verts */
export function collideCirclePolygon(
  circ: RigidBody,
  poly: RigidBody,
  vertsW: Vec2[],
): Manifold | null {
  if (circ.shape.kind !== "circle" || poly.shape.kind !== "polygon") return null;
  const r = circ.shape.radius;
  const c = circ.position;
  const inside = pointInConvexPolygon(c, vertsW);
  let bestDist = Infinity;
  let bestPoint: Vec2 | null = null;

  const n = vertsW.length;
  for (let i = 0; i < n; i++) {
    const a = vertsW[i]!;
    const b = vertsW[(i + 1) % n]!;
    const p = closestPointSegment(c, a, b);
    const dSq = vLenSq(vSub(c, p));
    if (dSq < bestDist) {
      bestDist = dSq;
      bestPoint = p;
    }
  }

  if (!bestPoint) return null;

  let normal: Vec2;
  let penetration: number;

  if (!inside) {
    const dist = Math.sqrt(bestDist);
    if (dist > r + 1e-8) return null;
    /** Normal from circle (A) toward polygon (B): contact → center would leave B inward; separation uses A→B ~= toward poly interior → from center toward surface */
    normal =
      dist > 1e-8 ? vNormalize(vSub(bestPoint, c)) : { x: 1, y: 0 };
    penetration = r - dist;
  } else {
    /** Center inside polygon: push circle out along outward normal with minimal translation */
    const areaPoly = polygonAreaSigned(vertsW);
    let minExit = Infinity;
    normal = { x: 1, y: 0 };
    let contact: Vec2 = { ...c };
    for (let i = 0; i < n; i++) {
      const va = vertsW[i]!;
      const vb = vertsW[(i + 1) % n]!;
      const edge = vSub(vb, va);
      const inwardLeft = vPerp(edge);
      const inward = vNormalize(
        areaPoly >= 0 ? inwardLeft : vScale(inwardLeft, -1),
      );
      const distInside = -vDot(vSub(c, va), inward); /** inside if negative */
      if (distInside < minExit) {
        minExit = distInside;
        normal = inward;
        /** Contact on polygon boundary along outward from poly (opposite inward) — approximate */
        const out = vScale(normal, -1);
        contact = vAdd(c, vScale(out, Math.max(0.0001, r + distInside)));
      }
    }
    penetration = r + Math.max(minExit, 0);
    bestPoint = contact;
  }

  /** Normal from rigid body circ (a) toward poly (b) */
  return {
    a: circ,
    b: poly,
    normal,
    penetration: Math.max(penetration, 0),
    contacts: [bestPoint ?? c],
  };
}

function closestPointSegment(p: Vec2, a: Vec2, b: Vec2): Vec2 {
  return projectOntoSegment(p, a, b);
}

function pointInConvexPolygon(p: Vec2, verts: Vec2[]): boolean {
  if (verts.length < 3) return false;
  const areaSign = polygonAreaSigned(verts) >= 0 ? 1 : -1;
  const n = verts.length;
  for (let i = 0; i < n; i++) {
    const a = verts[i]!;
    const b = verts[(i + 1) % n]!;
    const cross = vCross(vSub(b, a), vSub(p, a));
    if (areaSign * cross < -1e-6) return false;
  }
  return true;
}

export function collide(a: RigidBody, b: RigidBody): Manifold | null {
  if (a === b) return null;
  if (a.inverseMass === 0 && b.inverseMass === 0) return null;

  if (a.shape.kind === "circle" && b.shape.kind === "circle") return collideCircles(a, b);
  const vertsScratchA: Vec2[] = [];
  const vertsScratchB: Vec2[] = [];
  if (a.shape.kind === "polygon") vertsScratchA.push(...a.getWorldVertices());
  if (b.shape.kind === "polygon") vertsScratchB.push(...b.getWorldVertices());

  /** circle/poly permutations — always pass manifold with `a` as first rigid body caller order */
  if (a.shape.kind === "circle" && b.shape.kind === "polygon") {
    return collideCirclePolygon(a, b, vertsScratchB);
  }
  if (a.shape.kind === "polygon" && b.shape.kind === "circle") {
    const m = collideCirclePolygon(b, a, vertsScratchA);
    if (!m) return null;
    return {
      a: m.b,
      b: m.a,
      normal: { x: -m.normal.x, y: -m.normal.y },
      penetration: m.penetration,
      contacts: m.contacts.map((p) => p),
    };
  }

  if (vertsScratchA.length >= 3 && vertsScratchB.length >= 3) {
    return collideConvexPolygons(a, b, vertsScratchA, vertsScratchB);
  }
  return null;
}
