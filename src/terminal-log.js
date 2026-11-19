// Prints what the simulator is doing in the server terminal: every command with its reply, and the live events
// (robot state changes, movement, tasks, collisions, obstacles, map, simulation). Only the server uses this; the
// engines know nothing about it.
// mode (env LOG): 'off'; 'normal' (default: a moving robot's position about once a second); 'all' (every robot:update).
const MOVE_EVERY_MS = 1000;

function terminalLog(engine, mode = 'normal') {
  const on = mode !== 'off';
  const line = (...a) => { if (on) console.log(new Date().toTimeString().slice(0, 8), ...a); };
  const short = (o) => { const s = JSON.stringify(o); return !s || s === '{}' ? '' : s.length > 160 ? s.slice(0, 160) + '…' : s; };
  const where = (r) => `${r.level ? r.level.split('#')[0] : '-'} x ${r.position.x.toFixed(2)} y ${r.position.y.toFixed(2)} heading ${Math.round(r.orientation)}°`;
  const seen = new Map();   // robotID -> { state, activeTask, at (last time its movement was printed) }

  engine.on('robot:update', (r) => {
    const was = seen.get(r.robotID), now = Date.now();
    const doing = r.activeTask ? ` (${r.activeTask})` : r.activeMove ? ` (robot:move ${r.activeMove.command})` : '';
    if (!was) line(`robot  ${r.robotID} "${r.name ?? ''}" ${r.state} at ${where(r)}`);
    else if (was.state !== r.state || was.activeTask !== r.activeTask)
      line(`robot  ${r.robotID} ${was.state} -> ${r.state}${r.stopped && r.state !== 'Paused' ? ' (stopped)' : ''}${doing} at ${where(r)}`);
    else if (mode === 'all' || now - was.at >= MOVE_EVERY_MS) {
      if (r.state !== 'Moving' && mode !== 'all') return;
      const progress = r.activeTask ? ` route ${Math.round((r.routeProgress || 0) * 100)}%` : r.activeMove ? ` done ${r.activeMove.done.toFixed(2)}` : '';
      line(`move   ${r.robotID} ${r.state} ${where(r)}${progress}${doing}`);
    } else return;
    seen.set(r.robotID, { state: r.state, activeTask: r.activeTask, at: now });
  });
  engine.on('robots:snapshot', (list) => {
    for (const id of seen.keys()) if (!list.some((r) => r.robotID === id)) { seen.delete(id); line(`robot  ${id} removed`); }
    for (const r of list) seen.set(r.robotID, { state: r.state, activeTask: r.activeTask, at: Date.now() });
    line(`robots ${list.length ? list.map((r) => `${r.robotID} ${r.state}`).join(', ') : 'none'}`);
  });
  engine.on('task:update', (t) => line(`task   ${t.taskID} ${t.status}  ${t.robotID || 'unassigned'} ->${t.destination} (${t.level ? t.level.split('#')[0] : '-'})`
    + `${t.note ? '  | ' + t.note : ''}${t.outcome ? '  | ' + t.outcome : ''}${t.error ? `  | error ${t.error.failureCode}: ${t.error.failureMessage}` : ''}`));
  engine.on('task:removed', (d) => line(`task   ${d.taskID} removed (robot ${d.robotID} deleted)`));
  engine.on('collision:event', (c) => line(`COLLISION ${c.robotIDs.join(', ')} ${c.type} -> ${c.response}: ${c.message}`));
  engine.on('obstacles:snapshot', (l) => line(`obstacles ${l.length ? l.map((o) => `${o.obstacleID} (${o.level.split('#')[0]} ${o.x.toFixed(1)}, ${o.y.toFixed(1)} r ${o.radius})`).join(', ') : 'none'}`));
  engine.on('map:loaded', (m) => line(`map    loaded ${m.site || ''}: ${m.levels.map((l) => `${l.name} (${l.nodes} nodes${l.floorPlan ? ', floor plan' : ''})`).join(', ')}`));
  engine.on('sim:state', (s) => line(`sim    ${s}`));

  return {
    // a command and the reply it got
    command(ev, payload, reply) {
      const res = reply.success ? 'ok' : `FAILED ${reply.failureCode}: ${reply.failureMessage}`;
      line(`cmd    ${ev} ${short(payload)} -> ${res}`);
    },
    clients(n, joined) { line(`client ${joined ? 'connected' : 'disconnected'} (${n} open)`); },
    http(what) { line(what); },
  };
}
module.exports = { terminalLog };
