// Reads a Robohub nav graph (plain, {navGraph}, or the "sites" backup format).
// Keeps levels, points of interest, lifts, and a graph per level (nodes, two-way links, links to other levels) for route planning.
const { occupancyOf } = require('./occupancy');

function parseNavGraph(json) {
  let root = json && json.navGraph ? json.navGraph : json, site = '';
  if (root && root.sites && typeof root.sites === 'object') {
    const has = (x) => x && x.graphs && Object.keys(x.graphs).length;
    let s = root.sites[root.activeSiteId];
    if (!has(s)) s = Object.values(root.sites).find(has);
    if (!s) throw new Error('The file has no site with graphs.');
    site = s.name || ''; root = s;
  }
  if (!root || !root.graphs || typeof root.graphs !== 'object' || !Object.keys(root.graphs).length)
    throw new Error('The file has no graphs.');
  const levels = {}, pois = [], ids = {};
  for (const [name, nodes] of Object.entries(root.graphs)) {
    let count = 0;
    for (const [id, n] of Object.entries(nodes || {})) {
      const p = n && n.pose;
      if (!p || typeof p.x !== 'number' || typeof p.y !== 'number')
        throw new Error(`Level "${name}", node "${id}": pose.x and pose.y must be numbers.`);
      count++;
      if (n.locationName) pois.push({ name: n.locationName, level: name, node: id, x: p.x, y: p.y });
    }
    levels[name] = { nodes: count }; ids[name] = new Set(Object.keys(nodes || {}));
  }
  const graph = graphOf(root, ids);
  // floor plans (the file's overlays, beside "sites"): level -> occupancy map, used to keep detours on free floor
  const occupancy = {};
  for (const o of (json && json.overlays) || []) {
    if (o && levels[o.level] && !occupancy[o.level]) { try { occupancy[o.level] = occupancyOf(o); } catch { occupancy[o.level] = null; } }
  }
  const levelNames = Object.keys(levels);
  // lifts and the levels they stop at (a stop counts only if its lift point is a node on that level)
  const lifts = [];
  for (const l of (root.liftConfig && root.liftConfig.lifts) || []) {
    if (!l || l.id == null) continue;
    const stops = (l.details || []).filter((d) => d && ids[d.mapName] && ids[d.mapName].has(d.liftPoint));
    lifts.push({ id: String(l.id), name: l.name || String(l.id), levels: stops.map((d) => d.mapName) });
    for (const a of stops) for (const b of stops) if (a.mapName !== b.mapName)
      graph[a.mapName].out.get(a.liftPoint).push({ level: b.mapName, id: b.liftPoint, lift: String(l.id) });
  }
  return {
    site, levels, levelNames, pois, lifts, graph, occupancy,
    // a destination is a point of interest name, or any node id on the graph
    findDestination: (name, level) => {
      const poi = pois.find((p) => p.name === name && (!level || p.level === level));
      if (poi) return poi;
      const lv = level ? (ids[level] && ids[level].has(name) ? level : null) : levelNames.find((l) => ids[l].has(name));
      return lv ? { name, level: lv, node: name } : null;
    },
    summary: () => ({
      site,
      levels: levelNames.map((n) => ({ name: n, nodes: levels[n].nodes, pois: pois.filter((p) => p.level === n).length, floorPlan: !!occupancy[n] })),
      lifts,
    }),
  };
}
// level -> { nodes: id -> {x, y}, adj: id -> [ids] (two-way links on the level), out: id -> [{ level, id, lift }] (to other levels) }
// A branch to a node on another level is a link without a lift; branches to nodes that do not exist are skipped.
function graphOf(root, ids) {
  const graph = {};
  for (const [name, nodes] of Object.entries(root.graphs)) {
    const g = graph[name] = { nodes: new Map(), adj: new Map(), out: new Map() };
    for (const [id, n] of Object.entries(nodes || {})) { g.nodes.set(id, { x: n.pose.x, y: n.pose.y }); g.adj.set(id, []); g.out.set(id, []); }
  }
  for (const [name, nodes] of Object.entries(root.graphs)) {
    const g = graph[name];
    for (const [id, n] of Object.entries(nodes || {})) for (const b of n.branches || []) {
      if (g.nodes.has(b)) { if (b !== id && !g.adj.get(id).includes(b)) { g.adj.get(id).push(b); g.adj.get(b).push(id); } continue; }
      const other = Object.keys(ids).find((l) => ids[l].has(b));
      if (other) g.out.get(id).push({ level: other, id: b, lift: null });
    }
  }
  return graph;
}
module.exports = { parseNavGraph };
