import {
  createCircle,
  createPolygon,
  makeBox,
  type RigidBody,
} from "./body";
import { PhysicsWorld } from "./world";
import { bodyCount, drawWorld, resizeCanvas, type RendererConfig } from "./render/canvas";

const FIXED_DT = 1 / 120;
const MAX_STEPS_PER_FRAME = 10;

function makeGround(centerYWorld: number): RigidBody {
  const hw = 18;
  const hh = 0.75;
  return createPolygon(makeBox(hw, hh), { x: 0, y: centerYWorld }, 0);
}

function addPyramid(
  world: PhysicsWorld,
  layers: number,
  halfW: number,
  halfH: number,
  massPerBox: number,
  surfaceTopY: number,
): void {
  /** surfaceTopY world y of resting plane */
  let row = layers;
  for (let ly = 0; ly < layers; ly++, row--) {
    const rowY =
      surfaceTopY + halfH + ly * (2 * halfH + 2e-2);
    const span = row * halfW * 2;
    for (let k = 0; k < row; k++) {
      const lx = k * halfW * 2 - span / 2 + halfW;
      world.addBody(
        createPolygon(
          makeBox(halfW, halfH),
          { x: lx, y: rowY },
          massPerBox,
        ),
      );
    }
  }
}

function addCirclesAround(world: PhysicsWorld): void {
  const rSmall = 0.28;
  const mass = 0.95;
  for (let i = 0; i < 14; i++) {
    world.addBody(
      createCircle(
        rSmall,
        {
          x: -12 + ((i % 7) / 7) * 26,
          y: 4.5 + (i >>> 3) * 0.95,
        },
        mass + i * 0.02,
      ),
    );
  }
}

function buildDemo(world: PhysicsWorld): void {
  world.reset();
  const groundCy = -5.85;
  world.addBody(makeGround(groundCy));
  /** top surface of ground slab */
  const top = groundCy + 0.75;
  addPyramid(world, 9, 0.52, 0.35, 1.85, top);
  addCirclesAround(world);
}

const canvas = document.getElementById("c") as HTMLCanvasElement | null;
const hud = document.getElementById("hud");
const btnReset = document.getElementById("btn-reset");
const btnDrop = document.getElementById("btn-drop");

if (!canvas || !hud || !btnReset || !btnDrop) {
  throw new Error("Missing DOM nodes for physics demo");
}

const canvasEl = canvas;
const hudEl = hud;

const maybeCtx = canvasEl.getContext("2d");
if (!maybeCtx) throw new Error("2D canvas unsupported");
const renderCtx = maybeCtx;

const world = new PhysicsWorld();
world.config.timeStep = FIXED_DT;
world.config.solverIterations = 14;
world.config.gravity = { x: 0, y: -18 };

const renderCfg: RendererConfig = { ppm: 52, floorPaddingPx: 56 };

let acc = 0;
let last = performance.now();

let fpsFrames = 0;
let fpsAccTime = 0;
let fpsDisplay = 0;

let viewWidth = canvas.clientWidth;
let viewHeight = canvas.clientHeight;
let viewDpr = 1;

function syncCanvasSize(): void {
  viewDpr = resizeCanvas(canvasEl);
  viewWidth = canvasEl.clientWidth;
  viewHeight = canvasEl.clientHeight;
}

syncCanvasSize();
window.addEventListener("resize", syncCanvasSize);

buildDemo(world);

btnReset.addEventListener("click", () => {
  buildDemo(world);
});

btnDrop.addEventListener("click", () => {
  /** push the stack slightly so it collapses */
  for (const b of world.bodies) {
    if (b.inverseMass <= 0) continue;
    b.linearVelocity.y -= 0.75;
    b.angularVelocity += (Math.random() - 0.5) * 1.2;
  }
});

function frame(now: number): void {
  const rawDt = Math.min(0.05, (now - last) / 1000);
  last = now;
  acc += rawDt;
  fpsAccTime += rawDt;
  fpsFrames++;
  if (fpsAccTime >= 0.5) {
    fpsDisplay = Math.round(fpsFrames / fpsAccTime);
    fpsFrames = 0;
    fpsAccTime = 0;
  }

  let steps = 0;
  while (acc >= FIXED_DT && steps < MAX_STEPS_PER_FRAME) {
    world.stepSubstep(FIXED_DT);
    acc -= FIXED_DT;
    steps++;
  }

  drawWorld(world, renderCtx, viewWidth, viewHeight, viewDpr, renderCfg);
  hudEl.textContent = `FPS ${fpsDisplay} | bodies ${bodyCount(world)} | substeps ${steps} | solver iters ${world.config.solverIterations} | dt ${FIXED_DT.toFixed(4)}s`;

  requestAnimationFrame(frame);
}

requestAnimationFrame(frame);
