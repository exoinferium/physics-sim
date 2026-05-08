import { type PhysicsWorld } from "../world";
import type { Vec2 } from "../math";

export type RendererConfig = {
  /** pixels per world meter */
  ppm: number;
  /** Margin from bottom of canvas for world-origin offset (pixels) */
  floorPaddingPx: number;
};

export function resizeCanvas(canvas: HTMLCanvasElement): number {
  const dpr = window.devicePixelRatio || 1;
  const parent = canvas.parentElement;
  const w = Math.max(320, parent?.clientWidth ?? 800);
  const h = Math.max(260, parent?.clientHeight ?? 520);
  canvas.width = Math.floor(w * dpr);
  canvas.height = Math.floor(h * dpr);
  canvas.style.width = `${w}px`;
  canvas.style.height = `${h}px`;
  return dpr;
}

function applyCamera(
  ctx: CanvasRenderingContext2D,
  widthCss: number,
  heightCss: number,
  cfg: RendererConfig,
  dpr: number,
): void {
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.scale(dpr, dpr);
  ctx.clearRect(0, 0, widthCss, heightCss);
  ctx.fillStyle = "#0f1218";
  ctx.fillRect(0, 0, widthCss, heightCss);

  const ox = widthCss * 0.5;
  const oy = heightCss - cfg.floorPaddingPx;
  ctx.setTransform(cfg.ppm, 0, 0, -cfg.ppm, ox, oy);
}

function drawPolygonPath(
  ctx: CanvasRenderingContext2D,
  verts: Vec2[],
  closed: boolean,
): void {
  if (verts.length === 0) return;
  ctx.beginPath();
  ctx.moveTo(verts[0]!.x, verts[0]!.y);
  for (let i = 1; i < verts.length; i++) {
    ctx.lineTo(verts[i]!.x, verts[i]!.y);
  }
  if (closed) ctx.closePath();
}

export function drawWorld(
  world: PhysicsWorld,
  ctx: CanvasRenderingContext2D,
  widthCss: number,
  heightCss: number,
  dpr: number,
  cfg: RendererConfig,
): void {
  applyCamera(ctx, widthCss, heightCss, cfg, dpr);

  const scratch: Vec2[] = [];
  for (const body of world.bodies) {
    if (body.shape.kind === "circle") {
      const r = body.shape.radius;
      ctx.beginPath();
      ctx.arc(body.position.x, body.position.y, r, 0, Math.PI * 2);
      ctx.fillStyle = body.inverseMass <= 0 ? "#2a3344" : "rgba(120, 170, 255, 0.22)";
      ctx.fill();
      ctx.strokeStyle = body.inverseMass <= 0 ? "#5a6a88" : "#9db4ff";
      ctx.lineWidth = 2 / cfg.ppm;
      ctx.stroke();
      continue;
    }

    const verts = body.getWorldVertices(scratch);
    drawPolygonPath(ctx, verts, true);
    ctx.fillStyle = body.inverseMass <= 0 ? "#2a3344" : "rgba(120, 170, 255, 0.18)";
    ctx.fill();
    ctx.strokeStyle = body.inverseMass <= 0 ? "#5a6a88" : "#9db4ff";
    ctx.lineWidth = 2 / cfg.ppm;
    ctx.stroke();
  }
}

export function bodyCount(world: PhysicsWorld): number {
  return world.bodies.length;
}
