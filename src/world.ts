import { type RigidBody } from "./body";
import type { Vec2 } from "./math";
import { collide as narrowCollide, type Manifold } from "./collision/contacts";
import { SpatialGrid } from "./collision/broadphase";
import { sequentialImpulses } from "./solver";

export type WorldConfig = {
  gravity: Vec2;
  /** Fixed integration step (seconds) */
  timeStep: number;
  solverIterations: number;
  positionalBeta: number;
  positionalSlop: number;
};

const defaultConfig: WorldConfig = {
  gravity: { x: 0, y: -12 },
  timeStep: 1 / 120,
  solverIterations: 14,
  positionalBeta: 0.32,
  positionalSlop: 0.006,
};

export class PhysicsWorld {
  bodies: RigidBody[] = [];
  config: WorldConfig = { ...defaultConfig };

  readonly broadphase: SpatialGrid = new SpatialGrid(1.8);

  addBody(b: RigidBody): void {
    this.bodies.push(b);
  }

  reset(): void {
    this.bodies.length = 0;
  }

  /** Integrate velocities with gravity without changing positions — used before positional solve */
  integrateVelocities(dt: number): void {
    const { gravity } = this.config;
    for (const body of this.bodies) {
      if (body.inverseMass <= 0) continue;
      body.linearVelocity.x += gravity.x * dt;
      body.linearVelocity.y += gravity.y * dt;
    }
  }

  integratePositions(dt: number): void {
    for (const body of this.bodies) {
      if (body.inverseMass <= 0) continue;
      body.position.x += body.linearVelocity.x * dt;
      body.position.y += body.linearVelocity.y * dt;
      body.angle += body.angularVelocity * dt;
    }
  }

  /** Full narrowphase + impulses for current configuration */
  private gatherManifolds(): Manifold[] {
    let span = this.broadphase.cellSize;
    for (const b of this.bodies) {
      span = Math.max(span, b.boundsRadius() * 1.05 + 0.25);
    }
    this.broadphase.cellSize = span;

    const pairs = this.broadphase.queryPairs(this.bodies);
    const result: Manifold[] = [];
    for (const { a, b } of pairs) {
      const manifold = narrowCollide(a, b);
      if (manifold != null && manifold.penetration > 1e-4) result.push(manifold);
    }
    return result;
  }

  stepSubstep(dt: number): void {
    /** Predict motion, solve contacts on the predicted poses (sequential impulses + corrective projection). */
    this.integrateVelocities(dt);
    this.integratePositions(dt);
    const contacts = this.gatherManifolds();
    sequentialImpulses(
      contacts,
      this.config.solverIterations,
      this.config.positionalBeta,
      this.config.positionalSlop,
    );
  }
}
