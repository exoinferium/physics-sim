import { type Vec2, vCross, vDot, vLen, vPerp, vSub } from "../math";

export function projectVerts(verts: Vec2[], axis: Vec2): { min: number; max: number } {
  let min = Infinity;
  let max = -Infinity;
  for (const v of verts) {
    const p = vDot(v, axis);
    min = Math.min(min, p);
    max = Math.max(max, p);
  }
  return { min, max };
}

function overlap(
  aMin: number,
  aMax: number,
  bMin: number,
  bMax: number,
): number {
  return Math.min(aMax, bMax) - Math.max(aMin, bMin);
}

export type SatPolyResult = {
  depth: number;
  normal: Vec2;
  refOnA: boolean;
  refEdgeIdx: number;
};

/**
 * Separating-axis test between two convex CCW polygons in world space.
 * Returns penetration depth and a normal pointing from body A toward body B.
 */
export function satConvexConvex(
  vertsA: Vec2[],
  vertsB: Vec2[],
  centerA: Vec2,
  centerB: Vec2,
): SatPolyResult | null {
  let bestDepth = Infinity;
  let bestNormal: Vec2 = { x: 1, y: 0 };
  let refOnA = true;
  let refEdgeIdx = 0;

  const centerDelta = vSub(centerB, centerA);

  const tryPoly = (verts: Vec2[], onA: boolean): boolean => {
    const nVerts = verts.length;
    for (let i = 0; i < nVerts; i++) {
      const v0 = verts[i]!;
      const v1 = verts[(i + 1) % nVerts]!;
      const axis = vPerp(vSub(v1, v0));
      const axisLen = vLen(axis);
      if (axisLen < 1e-12) continue;
      const n = { x: axis.x / axisLen, y: axis.y / axisLen };

      let normal = n;
      if (vDot(normal, centerDelta) < 0) {
        normal = { x: -n.x, y: -n.y };
      }

      const pa = projectVerts(vertsA, normal);
      const pb = projectVerts(vertsB, normal);
      const o = overlap(pa.min, pa.max, pb.min, pb.max);
      if (o < 0) return false;

      if (o < bestDepth - 1e-9) {
        bestDepth = o;
        bestNormal = normal;
        refOnA = onA;
        refEdgeIdx = i;
      }
    }
    return true;
  };

  if (!tryPoly(vertsA, true)) return null;
  if (!tryPoly(vertsB, false)) return null;

  if (!(bestDepth < Infinity)) return null;

  const normal = normalizeNormalTowardB(bestNormal, centerA, centerB);
  return { depth: bestDepth, normal, refOnA, refEdgeIdx };
}

export function normalizeNormalTowardB(n: Vec2, centerA: Vec2, centerB: Vec2): Vec2 {
  const d = vSub(centerB, centerA);
  let out = n;
  if (vDot(out, d) < 0) out = { x: -n.x, y: -n.y };
  const len = vLen(out);
  if (len < 1e-12) return { x: 1, y: 0 };
  return { x: out.x / len, y: out.y / len };
}

/** Polygon area (>0 when CCW) */
export function polygonAreaSigned(verts: Vec2[]): number {
  let sum = 0;
  const n = verts.length;
  for (let i = 0; i < n; i++) {
    const v0 = verts[i]!;
    const v1 = verts[(i + 1) % n]!;
    sum += vCross(v0, v1);
  }
  return sum * 0.5;
}
