import { type RigidBody } from "../body";
import type { Vec2 } from "../math";

export type PotentialPair = { a: RigidBody; b: RigidBody };

/**
 * Spatial hash keyed by overlapping grid cells.
 * Bodies overlapping multiple cells duplicate into those buckets for local pairing;
 * unordered pairs globally dedupe by `(minId,maxId)`.
 */
export class SpatialGrid {
  cellSize: number;

  constructor(cellSize: number) {
    this.cellSize = Math.max(cellSize, 1e-3);
  }

  queryPairs(bodies: RigidBody[]): PotentialPair[] {
    const buckets = new Map<string, RigidBody[]>();
    const inv = 1 / this.cellSize;
    const minTmp: Vec2 = { x: 0, y: 0 };
    const maxTmp: Vec2 = { x: 0, y: 0 };

    const key = (ix: number, iy: number) => `${ix},${iy}`;

    for (const b of bodies) {
      b.aabb(minTmp, maxTmp);
      const ix0 = Math.floor(minTmp.x * inv);
      const iy0 = Math.floor(minTmp.y * inv);
      const ix1 = Math.floor(maxTmp.x * inv);
      const iy1 = Math.floor(maxTmp.y * inv);
      for (let gx = ix0; gx <= ix1; gx++) {
        for (let gy = iy0; gy <= iy1; gy++) {
          const k = key(gx, gy);
          let arr = buckets.get(k);
          if (!arr) {
            arr = [];
            buckets.set(k, arr);
          }
          arr.push(b);
        }
      }
    }

    const seen = new Set<string>();
    const pairs: PotentialPair[] = [];

    const pairStr = (aA: RigidBody, bB: RigidBody) =>
      aA.id < bB.id ? `${aA.id}_${bB.id}` : `${bB.id}_${aA.id}`;

    for (const group of buckets.values()) {
      const n = group.length;
      for (let ii = 0; ii < n; ii++) {
        const A = group[ii]!;
        for (let jj = ii + 1; jj < n; jj++) {
          const B = group[jj]!;
          const ps = pairStr(A, B);
          if (seen.has(ps)) continue;

          /** conservative AABB test */
          A.aabb(minTmp, maxTmp);
          const aminX = minTmp.x,
            aminY = minTmp.y,
            amaxX = maxTmp.x,
            amaxY = maxTmp.y;
          B.aabb(minTmp, maxTmp);
          if (
            amaxX < minTmp.x ||
            amaxY < minTmp.y ||
            maxTmp.x < aminX ||
            maxTmp.y < aminY
          ) {
            continue;
          }

          seen.add(ps);
          pairs.push({ a: A, b: B });
        }
      }
    }

    return pairs;
  }
}
