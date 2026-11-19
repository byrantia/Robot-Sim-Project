// Simulation Engine (draft 10.2, 10.3): virtual movement, obstacle sensing, and the clock that drives it.
// No network code here: the clock only calls engine.tick(seconds), so movement can be tested by calling tick directly.
const TURN_RATE = 180;     // degrees per second; the robot turns to face the next point before driving to it
const LIFT_SECONDS = 3;    // time spent in a lift between levels (robot Waiting, task Waiting)
const ROBOT_RADIUS = 0.3;  // robot footprint (a circle); a path is blocked where the footprint would touch an obstacle
const STOP_GAP = 0.2;      // a robot waiting at an obstacle stops this far short of touching it
const SLOW_FACTOR = 0.5;   // speed while any obstacle is within the robot's sensor range

const norm = (deg) => ((deg + 540) % 360) - 180;
// distance from point p to the segment a-b
const segDist = (p, a, b) => {
  const dx = b.x - a.x, dy = b.y - a.y, len2 = dx * dx + dy * dy;
  const t = len2 ? Math.max(0, Math.min(1, ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2)) : 0;
  return Math.hypot(p.x - a.x - t * dx, p.y - a.y - t * dy);
};
// does obstacle o block the straight drive a-b (would the robot's footprint touch it)?
const blocks = (o, a, b) => segDist(o, a, b) < o.radius + ROBOT_RADIUS;
// distance from the robot's centre to the obstacle's edge
const edgeDist = (r, o) => Math.hypot(o.x - r.position.x, o.y - r.position.y) - o.radius;

// Moves robot r along r.route for dt seconds. m holds the movement bookkeeping: { travelled, total, lift }.
// maxDist caps how far it may drive this tick (to stop short of an obstacle); factor scales its speed (slow near obstacles).
// Points on another level are reached by lift: the robot waits LIFT_SECONDS, then comes out at that point, once
// canExit(point) says the way out is clear (it stays in the lift until then).
// Returns true once the last route point is reached.
function advance(r, m, dt, maxDist = Infinity, factor = 1, canExit = () => true) {
  let left = dt, allow = maxDist;
  const speed = r.speed * factor;
  while (left > 0 && r.routeIndex < r.route.length) {
    const p = r.route[r.routeIndex];
    if (p.level !== r.level) {
      if (m.lift == null) { m.lift = LIFT_SECONDS; r.state = 'Waiting'; }
      const used = Math.min(left, m.lift); m.lift -= used; left -= used;
      if (m.lift > 1e-9) break;
      if (!canExit(p)) { m.lift = 0; break; }
      m.lift = null; r.level = p.level; r.position = { x: p.x, y: p.y, z: p.z }; r.routeIndex++; r.state = 'Moving';
      continue;
    }
    const dx = p.x - r.position.x, dy = p.y - r.position.y, d = Math.hypot(dx, dy);
    if (d < 1e-9) { r.routeIndex++; continue; }
    if (allow <= 1e-9) break;
    const turn = norm(Math.atan2(dy, dx) * 180 / Math.PI - r.orientation);
    if (Math.abs(turn) > 1e-6) {
      if (Math.abs(turn) > TURN_RATE * left) { r.orientation = norm(r.orientation + Math.sign(turn) * TURN_RATE * left); left = 0; break; }
      r.orientation = norm(r.orientation + turn); left -= Math.abs(turn) / TURN_RATE;
    }
    const step = Math.min(speed * left, allow);
    if (step >= d) { r.position.x = p.x; r.position.y = p.y; m.travelled += d; allow -= d; left -= d / speed; r.routeIndex++; }
    else { r.position.x += (dx / d) * step; r.position.y += (dy / d) * step; m.travelled += step; allow -= step; left = 0; }
  }
  const done = r.routeIndex >= r.route.length;
  r.routeProgress = done ? 1 : m.total > 0 ? Math.min(1, m.travelled / m.total) : 0;
  return done;
}

// Driving distance of a route from the robot's position (lift rides count as 0).
function routeLength(r) {
  let d = 0, at = { level: r.level, x: r.position.x, y: r.position.y };
  for (const p of r.route) { if (p.level === at.level) d += Math.hypot(p.x - at.x, p.y - at.y); at = p; }
  return d;
}

// The first obstacle in the robot's way on its current level: { o, at }, where at is how far it can still drive
// along its route before it must stop (STOP_GAP short of touching o). null when the way ahead is clear.
function firstBlock(r, obstacles) {
  const pts = [{ x: r.position.x, y: r.position.y }];
  for (let i = r.routeIndex; i < r.route.length && r.route[i].level === r.level; i++) pts.push(r.route[i]);
  let walked = 0;
  for (let i = 1; i < pts.length; i++) {
    const a = pts[i - 1], b = pts[i], len = Math.hypot(b.x - a.x, b.y - a.y);
    let best = null;
    for (const o of obstacles) {
      if (o.level !== r.level || !blocks(o, a, b)) continue;
      const at = walked + entry(a, b, o, o.radius + ROBOT_RADIUS + STOP_GAP) * len;
      if (!best || at < best.at) best = { o, at };
    }
    if (best) return best;
    walked += len;
  }
  return null;
}
// how far along a-b (0..1) the point first comes within R of o (0 if a already is)
function entry(a, b, o, R) {
  const dx = b.x - a.x, dy = b.y - a.y, fx = a.x - o.x, fy = a.y - o.y;
  const A = dx * dx + dy * dy, B = 2 * (fx * dx + fy * dy), Cc = fx * fx + fy * fy - R * R;
  if (Cc <= 0 || !A) return 0;
  const disc = B * B - 4 * A * Cc;
  if (disc < 0) return 1;
  const t = (-B - Math.sqrt(disc)) / (2 * A);
  return Math.max(0, Math.min(1, t));
}
// how far point p is from the path (a list of points; only parts on p's level count)
function distToPath(p, path) {
  let best = Infinity;
  for (let i = 0; i < path.length; i++) {
    const a = path[i], b = path[i + 1] && path[i + 1].level === a.level ? path[i + 1] : a;
    if (a.level === p.level) best = Math.min(best, segDist(p, a, b));
  }
  return best;
}

// The clock: start / pause / resume (draft 8.4). Ticks every tickMs while running.
class Simulation {
  constructor(engine, { tickMs = 100 } = {}) { this.engine = engine; this.tickMs = tickMs; this.timer = null; this.state = 'paused'; }
  start() {
    if (!this.timer) {
      let last = Date.now();
      this.timer = setInterval(() => { const now = Date.now(); this.engine.tick(Math.min((now - last) / 1000, 0.5)); last = now; }, this.tickMs);
      this.state = 'running'; this.engine.emit('sim:state', 'running');
    }
    return { state: this.state };
  }
  pause() {
    if (this.timer) { clearInterval(this.timer); this.timer = null; this.state = 'paused'; this.engine.emit('sim:state', 'paused'); }
    return { state: this.state };
  }
}
module.exports = { advance, routeLength, firstBlock, blocks, edgeDist, distToPath, segDist, entry, norm, Simulation, TURN_RATE,
  LIFT_SECONDS, ROBOT_RADIUS, STOP_GAP, SLOW_FACTOR };
