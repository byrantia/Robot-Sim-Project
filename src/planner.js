// Route planning (draft section 10): the nav graph is a graph of nodes and two-way links, searched with BFS.
// Same level: BFS from the robot's nearest node to the destination.
// Other level: BFS to a lift on the robot's level, take the lift, then BFS from where it lands to the destination.
// BFS gives the route with the fewest nodes; it can be swapped for Dijkstra or A* without changing the callers.
const { distToPath } = require('./simulation');

function bfs(g, from, isGoal, pass = () => true) {
  const prev = new Map([[from, null]]), q = [from];
  for (let i = 0; i < q.length; i++) {
    const n = q[i];
    if (isGoal(n)) { const p = []; for (let c = n; c != null; c = prev.get(c)) p.unshift(c); return p; }
    for (const m of g.adj.get(n) || []) if (!prev.has(m) && pass(n, m)) { prev.set(m, n); q.push(m); }
  }
  return null;
}
function nearestNode(g, p) {
  let best = null, bd = Infinity;
  for (const [id, n] of g.nodes) { const d = (n.x - p.x) ** 2 + (n.y - p.y) ** 2; if (d < bd) { bd = d; best = id; } }
  return best;
}

// from: { level, x, y } (the robot), to: { level, node }, lift: optional lift id to use when changing level.
// opts.start: node to start from (default: the node nearest the robot).
// opts.blocked(level, a, b): true when the link a-b ({x, y} points) must not be used (a reroute around obstacles).
// Returns { steps, route } or { error }. steps: drive / lift / drive. route: every node to drive through, in order,
// as { level, nodeID, x, y }; the node where the robot boards a lift also has { lift }.
function planRoute(nav, from, to, lift, opts = {}) {
  const A = nav.graph[from.level], B = nav.graph[to.level];
  if (!A || !A.nodes.size) return { error: 'The robot is not on a level with nodes.' };
  if (!B || !B.nodes.has(to.node)) return { error: 'The destination is not on the nav graph.' };
  const start = opts.start ?? nearestNode(A, from);
  const pass = (level) => { const g = nav.graph[level]; return (a, b) => !opts.blocked || !opts.blocked(level, g.nodes.get(a), g.nodes.get(b)); };
  let steps;
  if (from.level === to.level) {
    const p = bfs(A, start, (n) => n === to.node, pass(from.level));
    if (!p) return { error: 'No route to destination.' };
    steps = [{ type: 'drive', level: from.level, path: p }];
  } else {
    // Ways to the other level: the chosen lift; else a real lift (liftConfig: its lift point on each level), and only
    // if none of those works, any other link between the levels (a plain branch to a node on the other level). Some
    // maps also link the lift hold points across levels (where the robot calls the lift): those are not the ride.
    const onTo = (o) => o.level === to.level && B.nodes.has(o.id);
    const ways = lift ? [(o) => onTo(o) && o.lift === lift] : [(o) => onTo(o) && o.lift != null, onTo];
    let p1 = null, p2 = null, hold = null, link = null, reached = false;
    for (const via of ways) {
      p1 = bfs(A, start, (n) => (A.out.get(n) || []).some(via), pass(from.level));
      if (!p1) continue;
      reached = true; hold = p1[p1.length - 1]; link = A.out.get(hold).find(via);
      p2 = bfs(B, link.id, (n) => n === to.node, pass(to.level));
      if (p2) break;
    }
    if (!reached) return { error: lift ? `No route to lift ${lift} on ${from.level}, or it does not go to ${to.level}.` : `No route to a lift on ${from.level}.` };
    if (!p2) return { error: `No route from the lift to the destination on ${to.level}.` };
    steps = [{ type: 'drive', level: from.level, path: p1 }, { type: 'lift', lift: link.lift, at: hold, from: from.level, to: to.level },
      { type: 'drive', level: to.level, path: p2 }];
  }
  const route = [];
  steps.forEach((s, i) => {
    if (s.type !== 'drive') return;
    for (const id of s.path) { const n = nav.graph[s.level].nodes.get(id); route.push({ level: s.level, nodeID: id, x: n.x, y: n.y }); }
    const next = steps[i + 1];
    if (next && next.type === 'lift') route[route.length - 1].lift = next.lift;
  });
  return { steps, route };
}
// Detour round obstacles, off the nav graph (draft 10.3 "recalculate a route around the blocked segment").
// A* on a GRID-metre grid centred on the robot. A spot is usable when the robot's footprint there is clear of every
// obstacle, on free floor in the floor plan, and within maxDetour of the planned path. Spots within TRUST of the
// planned path skip the floor plan check: the nav graph already says that line is drivable.
// The detour ends where the route ahead is clear of obstacles again, and the robot carries on along its route from there.
// o: { level, from: {x, y}, ahead: [route points still to drive on this level], obstacles, occupancy (or null: no floor plan),
//      planned: [the planned path's points], maxDetour, clearance (robot radius) }
// Returns { points: [{x, y}, ...] (ending at the rejoin point), rejoin: index in ahead of the first point after it, dev }
// or { why }.
const GRID = 0.1, TRUST = 0.2, MARGIN = 0.05, REJOIN_GAP = 0.3, MAX_EXPANSIONS = 80000;
function planDetour(o) {
  const { level, from, ahead, obstacles, occupancy, planned, maxDetour, clearance } = o;
  const near = (p, extra) => obstacles.some((b) => Math.hypot(p.x - b.x, p.y - b.y) < b.radius + clearance + extra);
  // rejoin: the first point along the route ahead, past the obstacle, where the robot is clear of it again
  const pts = [from, ...ahead];
  let goal = null, rejoin = -1, seen = false;
  for (let k = 1; k < pts.length && !goal; k++) {
    const a = pts[k - 1], b = pts[k], len = Math.hypot(b.x - a.x, b.y - a.y), n = Math.max(1, Math.ceil(len / GRID));
    for (let s = 0; s <= n; s++) {
      const p = { level, x: a.x + ((b.x - a.x) * s) / n, y: a.y + ((b.y - a.y) * s) / n };
      if (near(p, REJOIN_GAP)) seen = true;
      else if (seen) { goal = p; rejoin = k - 1; break; }
    }
  }
  if (!seen) return { why: 'nothing is in the way' };
  if (!goal) return { why: 'it is on the destination' };
  const dev = (p) => distToPath({ level, x: p.x, y: p.y }, planned);
  const usable = (p) => {
    const d = dev(p);
    if (d > maxDetour || near(p, MARGIN)) return false;
    return !occupancy || d <= TRUST || occupancy.free(p.x, p.y, clearance);
  };
  // A* over grid cells (i, j) = from + (i, j) * GRID
  const gi = Math.round((goal.x - from.x) / GRID), gj = Math.round((goal.y - from.y) / GRID);
  const reach = Math.hypot(gi, gj) + (2 * maxDetour + 2) / GRID;
  const key = (i, j) => (i + 32768) * 65536 + (j + 32768), cache = new Map();
  const cellOk = (i, j) => {
    const k = key(i, j);
    if (!cache.has(k)) cache.set(k, (i === 0 && j === 0) || (Math.hypot(i, j) <= reach && usable({ x: from.x + i * GRID, y: from.y + j * GRID })));
    return cache.get(k);
  };
  if (!usable(goal)) return { why: `no space to go round it within ${maxDetour} m of the planned path` };
  const heap = [[Math.hypot(gi, gj), 0, 0, 0]], best = new Map([[key(0, 0), 0]]), prev = new Map();
  const push = (e) => { heap.push(e); let c = heap.length - 1; while (c && heap[(c - 1) >> 1][0] > heap[c][0]) { [heap[c], heap[(c - 1) >> 1]] = [heap[(c - 1) >> 1], heap[c]]; c = (c - 1) >> 1; } };
  const pop = () => {
    const top = heap[0], last = heap.pop();
    if (heap.length) { heap[0] = last; let c = 0; for (;;) { const l = 2 * c + 1, r = l + 1; let m = c;
      if (l < heap.length && heap[l][0] < heap[m][0]) m = l; if (r < heap.length && heap[r][0] < heap[m][0]) m = r;
      if (m === c) break; [heap[c], heap[m]] = [heap[m], heap[c]]; c = m; } }
    return top;
  };
  let found = false;
  for (let n = 0; heap.length && n < MAX_EXPANSIONS; n++) {
    const [, g, i, j] = pop();
    if (g > best.get(key(i, j))) continue;
    if (i === gi && j === gj) { found = true; break; }
    for (const [di, dj] of [[1, 0], [-1, 0], [0, 1], [0, -1], [1, 1], [1, -1], [-1, 1], [-1, -1]]) {
      const ni = i + di, nj = j + dj;
      if (!(ni === gi && nj === gj) && !cellOk(ni, nj)) continue;
      const ng = g + Math.hypot(di, dj), k = key(ni, nj);
      if (ng < (best.get(k) ?? Infinity)) { best.set(k, ng); prev.set(k, [i, j]); push([ng + Math.hypot(gi - ni, gj - nj), ng, ni, nj]); }
    }
  }
  if (!found) return { why: `no space to go round it within ${maxDetour} m of the planned path` };
  const cells = [];
  for (let c = [gi, gj]; c; c = prev.get(key(c[0], c[1]))) cells.unshift(c);
  const path = cells.map(([i, j]) => ({ x: from.x + i * GRID, y: from.y + j * GRID }));
  path[0] = { x: from.x, y: from.y }; path[path.length - 1] = { x: goal.x, y: goal.y };
  // straighten: from each kept point, jump to the farthest later point it can drive to in a straight line
  const straight = (a, b) => {
    const len = Math.hypot(b.x - a.x, b.y - a.y), n = Math.max(1, Math.ceil(len / (GRID / 2)));
    for (let s = 1; s < n; s++) {
      const p = { x: a.x + ((b.x - a.x) * s) / n, y: a.y + ((b.y - a.y) * s) / n };
      if (Math.hypot(p.x - from.x, p.y - from.y) > GRID && !usable(p)) return false;
    }
    return true;
  };
  const out = [];
  for (let a = 0; a < path.length - 1;) {
    let b = path.length - 1;
    while (b > a + 1 && !straight(path[a], path[b])) b--;
    out.push(path[b]); a = b;
  }
  return { points: out, rejoin, dev: Math.max(...path.map(dev)) };
}
module.exports = { planRoute, planDetour };
