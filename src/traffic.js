// Traffic Manager: multi-robot collision avoidance by path reservations (one central planner for every robot).
// No network code and no movement here: each tick the engine describes every robot (position, route ahead, speed,
// priority) and plan() answers how far along its route each robot on a task may drive. The engine then moves the
// robots with the usual movement code, capped at that distance.
//
// How it works:
// - Every robot always holds its own footprint. A robot on a task also holds a reservation: the stretch of its route
//   just ahead of it. Reservations are kept from tick to tick (sticky) and never come within SAFE_DIST of another
//   robot's reservation or footprint, so robots that only drive inside their reservations cannot collide.
// - Each tick, in priority order, every robot tries to extend its reservation to LOOKAHEAD_SECONDS of driving at its
//   speed. It gets as far as it can before it would come within SAFE_DIST of something another robot holds.
// - Keep out of the way (shared intersections and narrow paths): a robot may not end its reservation on the path a
//   higher-priority robot will drive in the next YIELD_SECONDS, unless both go the same way (following is fine). So
//   at a crossing, or at the mouth of a corridor with a robot coming the other way, the lower-priority robot stops
//   short and waits. A robot already standing in such a path may only drive on if that gets it clear of the path.
// - Priority: the robot whose job started first (robot:navigate order) goes first. It is fixed for the job, so two
//   robots never take turns giving way to each other.
// A robot whose reservation cannot grow is waiting for the robots in waitFor. The engine turns that into the Waiting
// state and resumes it once it can reserve RESUME_GAP again; when robots block each other, one gives way (see
// engine._yield).
// - Narrow corridors (too narrow to pass, on the floor plan): a robot in one holds it in its direction of travel.
//   Others may follow it in, but nobody enters from the other end until it is out; they wait SAFE_DIST before it.
const { ROBOT_RADIUS, segDist } = require('./simulation');

const SAFETY_GAP = 0.4;                          // m kept clear between two robots' footprints
const SAFE_DIST = 2 * ROBOT_RADIUS + SAFETY_GAP; // so robot centres stay at least this far apart (1 m)
const LOOKAHEAD_SECONDS = 3;                     // a robot reserves this much driving ahead (at its speed)
const MIN_LOOKAHEAD = 1;                         // m: the least it tries to reserve, however slow it is
const YIELD_SECONDS = 6;                         // lower-priority robots keep out of this much of a robot's path ahead
const RESUME_GAP = 0.3;                          // m: a robot waiting for traffic moves again once it can reserve this far
const SAMPLE = 0.05;                             // m between the points checked along a path
const ALIGN = 0.7;                               // paths whose directions agree this much (cos 45°) go the same way

// A path as a polyline with the distance along it at each point.
function polyline(pts) {
  const cum = [0];
  for (let i = 1; i < pts.length; i++) cum.push(cum[i - 1] + Math.hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y));
  return { pts, cum, len: cum[cum.length - 1] };
}
// the point s metres along the path, with the direction of travel there
function pointAt(pl, s) {
  const { pts, cum } = pl;
  if (pts.length < 2) return { x: pts[0].x, y: pts[0].y, dx: 0, dy: 0 };
  let i = 1;
  while (i < pts.length - 1 && cum[i] < s) i++;
  const a = pts[i - 1], b = pts[i], len = cum[i] - cum[i - 1];
  const f = len > 1e-12 ? Math.max(0, Math.min(1, (s - cum[i - 1]) / len)) : 1;
  return { x: a.x + (b.x - a.x) * f, y: a.y + (b.y - a.y) * f, dx: len > 1e-12 ? (b.x - a.x) / len : 0, dy: len > 1e-12 ? (b.y - a.y) / len : 0 };
}
// the first s metres of the path
function cut(pl, s) {
  if (s >= pl.len) return pl;
  const out = [pl.pts[0]];
  for (let i = 1; i < pl.pts.length && pl.cum[i] < s; i++) out.push(pl.pts[i]);
  const e = pointAt(pl, s); out.push({ x: e.x, y: e.y });
  return polyline(out);
}
// distance from point p to the path, and the path's direction at the nearest place
function nearest(p, pl) {
  if (pl.pts.length < 2) return { d: Math.hypot(p.x - pl.pts[0].x, p.y - pl.pts[0].y), dx: 0, dy: 0 };
  let best = { d: Infinity, dx: 0, dy: 0 };
  for (let i = 1; i < pl.pts.length; i++) {
    const a = pl.pts[i - 1], b = pl.pts[i], d = segDist(p, a, b), len = pl.cum[i] - pl.cum[i - 1];
    if (d < best.d && (len > 1e-12 || best.d === Infinity)) best = { d, dx: len > 1e-12 ? (b.x - a.x) / len : 0, dy: len > 1e-12 ? (b.y - a.y) / len : 0 };
  }
  return best;
}

class TrafficManager {
  // robots: [{ id, level, x, y, prio, speed, path, held, corridors, paused }]
  //   path: the route ahead on its level, from its position ([{x, y}, ...]); only for a robot on a task.
  //   paused: on a task but paused (prio Infinity, holds nothing): others still keep out of its path.
  //   prio: lower goes first (Infinity for a robot that is not driving a route). held: metres of path it already holds.
  //   corridors: the narrow corridors on its path (see corridorMap) as { key, dir, from, to } metres along it.
  // Returns Map id -> { allow, waitFor: [ids], len } for every robot with a path: allow is how far it may drive now
  // (and what it now holds), len how much path it has left on this level.
  plan(robots) {
    const P = robots.map((r) => ({ ...r, path: polyline(r.path && r.path.length ? r.path : [{ x: r.x, y: r.y }]) }));
    P.sort((a, b) => a.prio - b.prio || (a.id < b.id ? -1 : 1));
    for (const p of P) { p.held = Math.max(0, Math.min(p.held || 0, p.path.len)); p.claim = cut(p.path, p.held); }
    const foot = (q) => polyline([{ x: q.x, y: q.y }]);
    const out = new Map();

    // How far p can go from s0 towards s1 without coming within SAFE_DIST of what others hold (higher priority:
    // their reservation; all others: lower is their reservation too, unless onlyFoot). Already closer than that
    // (two robots placed close together)? Then it may not get any closer, but it may move away.
    const reach = (p, s0, s1, other) => {
      const start = { x: p.x, y: p.y }, near0 = new Map();
      for (const q of P) if (q !== p && q.level === p.level) near0.set(q, nearest(start, other(q)).d);
      let last = s0;
      for (let s = s0; ; s = Math.min(s1, s + SAMPLE)) {
        const pt = pointAt(p.path, s);
        for (const [q, d0] of near0) if (nearest(pt, other(q)).d < Math.min(SAFE_DIST, d0 - 1e-6)) return { s: last, by: q.id };
        last = s;
        if (s >= s1) return { s: s1, by: null };
      }
    };
    // 1. Held reservations stay, but trimmed if they now clash (a robot was placed there, or one ahead rerouted).
    P.forEach((p, i) => {
      if (!p.held) return;
      const r = reach(p, 0, p.held, (q) => (P.indexOf(q) < i ? q.claim : foot(q)));
      if (r.s < p.held) { p.held = r.s; p.claim = cut(p.path, p.held); }
    });
    // Narrow corridors (corridors: [{ key, dir, from, to }] along a robot's path; from < 0 when it is already in it).
    // The way in starts SAFE_DIST before the corridor, so a robot waiting to go in leaves room for one coming out.
    // A robot in a corridor, or with a reservation into its way in, holds it in its direction; nobody may go into it
    // the other way.
    const locks = new Map();   // corridor key -> Map(robot id -> dir)
    const inside = (p, c) => c.from < 0 || c.from - SAFE_DIST < p.held - 1e-9;
    const hold = (p) => { for (const c of p.corridors || []) if (inside(p, c)) {
      if (!locks.has(c.key)) locks.set(c.key, new Map());
      locks.get(c.key).set(p.id, c.dir);
    } };
    P.forEach(hold);
    // how far p may reserve (up to s) without going into a corridor someone holds the other way
    const corridorStop = (p, s, waitFor) => {
      for (const c of p.corridors || []) {
        if (inside(p, c) || c.from - SAFE_DIST >= s) continue;
        const against = [...(locks.get(c.key) || [])].filter(([id, d]) => id !== p.id && d !== c.dir).map(([id]) => id);
        if (against.length) { against.forEach((id) => waitFor.add(id)); s = Math.max(p.held, Math.min(s, c.from - SAFE_DIST)); }
      }
      return s;
    };
    // 2. In priority order, each robot on a route extends its reservation.
    const yieldPath = (q) => cut(q.path, q.held + Math.max(MIN_LOOKAHEAD, q.speed * YIELD_SECONDS));
    // A paused robot reserves nothing, but keeps its right of way whatever its priority: nobody stops in its path, so
    // it can carry on when it is resumed (and anybody may still cross its path in one go, as it is held up).
    const paused = P.filter((q) => q.paused && q.path.len > 1e-6);
    for (const q of paused) { q.yp = yieldPath(q); q.waiting = true; }
    P.forEach((p, i) => {
      if (p.prio === Infinity) return;
      const higher = [...P.slice(0, i).filter((q) => q.level === p.level && q.yp && q.path.len > 1e-6),
        ...paused.filter((q) => q.level === p.level)];
      // the higher-priority robots whose way a point (going in direction d) is in
      const inWay = (pt) => higher.filter((q) => {
        const n = nearest(pt, q.yp);
        return n.d < SAFE_DIST && !(pt.dx * n.dx + pt.dy * n.dy >= ALIGN);
      }).map((q) => q.id);
      let target = Math.min(p.path.len, Math.max(p.held, p.speed * LOOKAHEAD_SECONDS, MIN_LOOKAHEAD));
      const waitFor = new Set(), here = inWay(pointAt(p.path, p.held));
      if (here.length) {
        // already in someone's way: drive on only to get clear of it (if the path gets clear before it ends)
        let exit = null;
        for (let s = p.held; s <= p.path.len + 1e-9; s += SAMPLE) if (!inWay(pointAt(p.path, Math.min(s, p.path.len))).length) { exit = Math.min(s, p.path.len); break; }
        const go = exit == null ? null : reach(p, p.held, exit, (q) => q.claim);
        if (!go || go.s < exit - 1e-9) {   // cannot get clear (say, head-on in a corridor): stay put, so it is a clean deadlock
          target = p.held; here.forEach((id) => waitFor.add(id)); if (go && go.by) waitFor.add(go.by);
        } else target = Math.max(target, exit);
      }
      const r = reach(p, p.held, target, (q) => q.claim);
      let s = corridorStop(p, r.s, waitFor);
      if (r.by) waitFor.add(r.by);
      if (!here.length && s < p.path.len - 1e-9) {
        // do not stop in a higher-priority robot's way: stop short of it instead. If every robot whose way it is in
        // is held up itself, it may cross that way, but only if it can reserve all the way across it.
        const blockers = inWay(pointAt(p.path, s));
        if (blockers.length) {
          let crossed = false;
          if (blockers.every((id) => P.find((q) => q.id === id).waiting)) {
            let exit = null;
            for (let e = s; e <= p.path.len + 1e-9; e += SAMPLE) if (!inWay(pointAt(p.path, Math.min(e, p.path.len))).length) { exit = Math.min(e, p.path.len); break; }
            if (exit != null && reach(p, p.held, exit, (q) => q.claim).s >= exit - 1e-9 && corridorStop(p, exit, new Set()) >= exit - 1e-9) { s = exit; crossed = true; }
          }
          if (!crossed) {
            blockers.forEach((id) => waitFor.add(id));
            let e = s;
            while (e > p.held + 1e-9 && inWay(pointAt(p.path, e)).length) e = Math.max(p.held, e - SAMPLE);
            s = e;
          }
        }
      }
      p.held = s; p.claim = cut(p.path, s); hold(p);
      // held up (cannot get RESUME_GAP further): it keeps its right of way, but others may cross its path in one go
      p.waiting = p.path.len > 1e-6 && s < Math.min(RESUME_GAP, p.path.len) - 1e-9;
      p.yp = yieldPath(p);
      out.set(p.id, { allow: s, waitFor: s >= target - 1e-9 && !here.length ? [] : [...waitFor], len: p.path.len });
    });
    return out;
  }
}

// Narrow corridors on a level: chains of nav graph links too narrow for two robots to pass each other on the floor
// plan (nowhere SAFE_DIST to either side of the link for a robot to stand), joined through nodes with only those two
// links. Returns Map "a|b" -> { chain, dir } for every link in a corridor, both ways round: dir +1 one way along the
// chain, -1 the other. A level without a floor plan is open floor: no corridors. Worked out once per nav graph.
const corridorCache = new WeakMap();
function corridorMap(nav, level) {
  let byLevel = corridorCache.get(nav);
  if (!byLevel) corridorCache.set(nav, (byLevel = new Map()));
  if (byLevel.has(level)) return byLevel.get(level);
  const out = new Map(), g = nav.graph[level], occ = nav.occupancy && nav.occupancy[level];
  byLevel.set(level, out);
  if (!g || !occ) return out;
  const narrow = new Map(), key = (a, b) => `${a}|${b}`, deg = (n) => (g.adj.get(n) || []).length;
  const isNarrow = (a, b) => {
    const A = g.nodes.get(a), B = g.nodes.get(b), len = Math.hypot(B.x - A.x, B.y - A.y);
    // not right at a junction (3 or more links), where there is room where the links meet
    const m0 = deg(a) >= 3 ? SAFE_DIST : 0, m1 = deg(b) >= 3 ? SAFE_DIST : 0;
    if (len < 1e-6 || len - m0 - m1 < 0) return false;
    const nx = -(B.y - A.y) / len, ny = (B.x - A.x) / len;
    for (let s = m0; s <= len - m1 + 1e-9; s += 0.25) {
      const x = A.x + ((B.x - A.x) * s) / len, y = A.y + ((B.y - A.y) * s) / len;
      if (!occ.free(x + nx * SAFE_DIST, y + ny * SAFE_DIST, ROBOT_RADIUS) && !occ.free(x - nx * SAFE_DIST, y - ny * SAFE_DIST, ROBOT_RADIUS)) return true;
    }
    return false;
  };
  for (const [a, ns] of g.adj) for (const b of ns) if (a < b && g.nodes.has(b)) { const n = isNarrow(a, b); narrow.set(key(a, b), n); narrow.set(key(b, a), n); }
  // the narrow link that carries on from node n (arriving from m) when n only joins two links
  const onward = (m, n) => { if (deg(n) !== 2) return null; const x = g.adj.get(n).find((o) => o !== m); return x && narrow.get(key(n, x)) ? x : null; };
  let chains = 0;
  for (const [k, n] of narrow) {
    if (!n || out.has(k)) continue;
    let [a, b] = k.split('|');
    // walk back to one end of the chain, then along it to the other, numbering the direction
    const seen = new Set([k]);
    for (let x; (x = onward(b, a)) && !seen.has(key(x, a)); ) { seen.add(key(x, a)); b = a; a = x; }
    const chain = `${level}:${++chains}`;
    for (let x = a, y = b; y && !out.has(key(x, y)); ) {
      out.set(key(x, y), { chain, dir: 1 }); out.set(key(y, x), { chain, dir: -1 });
      const z = onward(x, y); x = y; y = z;
    }
  }
  return out;
}

// the closest two paths come to each other (checked every SAMPLE along a)
function pathGap(a, b) {
  let best = Infinity;
  for (let s = 0; ; s = Math.min(a.len, s + SAMPLE)) { best = Math.min(best, nearest(pointAt(a, s), b).d); if (s >= a.len) return best; }
}

module.exports = { TrafficManager, pathGap, corridorMap, SAFE_DIST, SAFETY_GAP, RESUME_GAP, LOOKAHEAD_SECONDS, YIELD_SECONDS, polyline, pointAt, cut, nearest };
