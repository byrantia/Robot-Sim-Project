// Connects the nav graph viewer to the Socket.IO API. Loaded after the viewer's own script, so it can reach
// the viewer's globals (S, side, draw, el, assignTask, stopRobot, addRobot, loadText ...) and swap in server calls.
(() => {//iife means auto start when run
  const socket = io(); //html alr call this lib so can use is in src = ...
  let net = 'connecting', mapMsg = '', simState = 'running';

  // send a command and wait for its reply (the Socket.IO acknowledgement)
  const call = (event, payload = {}) => new Promise((resolve) => {
    if (!socket.connected) return resolve({ success: false, failureCode: 0, failureMessage: 'Not connected to the server.' });
    socket.timeout(5000).emit(event, payload, (err, reply) =>
      resolve(err ? { success: false, failureCode: 0, failureMessage: 'No reply from the server.' } : reply));
  });
  window.robotApi = { socket, call };//global obj so everyone can access

  const badge = el('span', null, 'mute'); badge.id = 'net'; badge.setAttribute('role', 'status');
  $('header').insertBefore(badge, $('#status'));
  const note = (m) => { if (m !== undefined) mapMsg = m; badge.textContent = `Server: ${net}${simState === 'paused' ? ' · paused' : ''}${mapMsg ? ' · ' + mapMsg : ''}`; };
  const fail = (r, res) => { S.msg = `${r ? r.name + ': ' : ''}${res.failureMessage} [${res.failureCode}]`; side(); draw(); };

  // The server plans the route. Its steps name levels and node ids; the page draws with level indexes and its own nodes.
  const lvOf = (name) => S.levels.findIndex((L) => L.name === name);
  const toSteps = (steps) => {
    const out = (steps || []).map((s) => s.type === 'drive'
      ? { type: 'drive', lv: lvOf(s.level), path: s.path.map((id) => S.levels[lvOf(s.level)]?.byId.get(id)).filter(Boolean) }
      : { type: 'lift', lift: s.lift, at: s.at, from: lvOf(s.from), to: lvOf(s.to) });
    return out.some((s) => s.lv < 0 || s.from < 0 || s.to < 0) ? [] : out;   // a level the page does not have: draw nothing
  };
  const taskFrom = (d) => ({ taskID: d.taskID, robotID: d.robotID, destination: d.node, destinationName: d.destination,
    destinationLevel: d.level, lift: d.lift, status: d.status, createdAt: d.createdAt, outcome: d.outcome, error: d.error, note: d.note,
    steps: toSteps(d.steps) });
  const sync = (list) => {
    S.robots = list.map((r) => Object.assign(S.robots.find((x) => x.robotID === r.robotID) || {}, r));
    for (const r of S.robots) r.color = r.color || nextColor();
    if (S.robot && !S.robots.includes(S.robot)) S.robot = null;
    robotSeq = Math.max(robotSeq, ...S.robots.map((r) => +String(r.robotID).replace(/\D/g, '') || 0));
    side(); draw();
  };

  socket.on('connect', () => { net = 'connected'; note(); });
  socket.on('disconnect', () => { net = 'offline'; note(); });
  socket.on('robots:snapshot', sync);
  // The server owns state, position and route (live while robots move); the page keeps its own drawing details.
  // The side panel is rebuilt when the state, task or level changes; otherwise (route, settings, and about once a
  // second while robots move) only when you are not typing in it.
  let panelAt = 0;
  socket.on('robot:update', (d) => {
    const r = S.robots.find((x) => x.robotID === d.robotID), now = Date.now();
    const typing = document.activeElement && document.activeElement.closest('#side input, #side select');
    const panel = !r || r.state !== d.state || r.activeTask !== d.activeTask || r.level !== d.level ||
      (!typing && ('route' in d || now - panelAt > 1000));
    if (r) Object.assign(r, d); else S.robots.push({ ...d, color: nextColor() });
    if (panel) { panelAt = now; side(); }
    draw();
  });
  socket.on('task:update', (d) => {
    const t = S.tasks.find((x) => x.taskID === d.taskID);
    if (t) Object.assign(t, { robotID: d.robotID, status: d.status, outcome: d.outcome, error: d.error, note: d.note, steps: toSteps(d.steps) }); else S.tasks.push(taskFrom(d));
    side(); draw();
  });
  socket.on('task:removed', (d) => { S.tasks = S.tasks.filter((t) => t.taskID !== d.taskID); side(); draw(); });
  socket.on('sim:state', (s) => { if (s === 'reset') { S.tasks = []; side(); draw(); } else { simState = s; showSim(); } });

  // Assign task: task:assign creates the job (reserved for this robot, nothing moves yet), then robot:navigate has the
  // robot take it: the server plans the route and starts it moving (its updates arrive before this reply).
  // If the robot cannot start it (no route, busy ...), the job is cancelled so it is not left waiting.
  assignTask = async function (r, node, levelName, lift) {
    S.picking = null; S.msg = '';
    const job = await call('task:assign', { robotID: r.robotID, destination: node.poi || node.id, level: levelName, lift: lift || undefined });
    if (!job.success) return fail(r, job);
    const res = await call('robot:navigate', { robotID: r.robotID, taskID: job.data.taskID });
    if (!res.success) { call('task:cancel', { taskID: job.data.taskID }); return fail(r, res); }
    let t = S.tasks.find((x) => x.taskID === res.data.taskID);
    if (!t) { t = taskFrom(res.data); S.tasks.push(t); }
    side(); draw();
  };
  const _stop = stopRobot;
  stopRobot = function (r) { if (taskOf(r) || r.activeMove) call('robot:emergency-stop', { robotID: r.robotID }); _stop(r); };
  // Stop is a pause (robot:stop) until Resume (robot:start); Cancel task is task:cancel. The server holds the robot meanwhile.
  pauseRobot = function (r) { call('robot:stop', { robotID: r.robotID }).then((res) => { if (!res.success) fail(r, res); }); };
  resumeRobot = function (r) { call('robot:start', { robotID: r.robotID }).then((res) => { if (!res.success) fail(r, res); }); };
  cancelTask = function (r, t) { call('task:cancel', { taskID: t.taskID }).then((res) => { if (!res.success) fail(r, res); }); };

  // Adding a robot on the map also creates it on the server, which assigns the real ID.
  const _add = addRobot;
  addRobot = function (x, y) {
    const r = _add(x, y);
    call('robot:create', { name: r.name, speed: r.speed, sensorRange: r.sensorRange, maxDetour: r.maxDetour,
      startPosition: { level: r.level, x: r.position.x, y: r.position.y, z: r.position.z, orientation: r.orientation } }).then((res) => {
      if (res.success) {
        r.robotID = res.data.robotID; r.state = res.data.state; r.position = res.data.position;   // the server sets z from the level
        S.robots = S.robots.filter((o, i, a) => a.findIndex((p) => p.robotID === o.robotID) === i);
        side(); draw();
      } else { S.robots = S.robots.filter((o) => o !== r); if (S.robot === r) S.robot = null; fail(null, res); }
    });
    return r;
  };
  // "Delete robot" also removes it on the server (this runs before the button's own handler).
  $('#side').addEventListener('click', (e) => {
    if (e.target.textContent !== 'Delete robot' || !S.robot) return;
    const id = S.robot.robotID, busy = taskOf(S.robot);
    (busy ? call('robot:emergency-stop', { robotID: id }) : Promise.resolve()).then(() => call('robot:delete', { robotID: id }))
      .then((res) => { if (!res.success) call('robot:list').then((l) => l.success && sync(l.data)); });
  }, true);
  // Dragging a robot moves it on the server too (robot:localize). A robot on a task cannot be moved: it snaps back.
  robotMoved = function (r) {
    call('robot:localize', { robotID: r.robotID, position: { x: r.position.x, y: r.position.y } }).then((res) => {
      if (res.success) { Object.assign(r, res.data); side(); draw(); return; }
      fail(r, res);
      call('robot:get', { robotID: r.robotID }).then((g) => { if (g.success) { Object.assign(r, g.data); side(); draw(); } });
    });
  };
  // Low-level movement (robot:move): the server drives the robot over time and sends robot:update as it goes.
  moveRobot = function (r, command, amount) {
    const p = { robotID: r.robotID, command };
    if (command === 'rotate') p.angle = amount; else if (command !== 'stop') p.distance = amount;
    call('robot:move', p).then((res) => { if (!res.success) fail(r, res); });
  };
  // Settings changed in the panel go to the server (robot:configure).
  robotChanged = function (r, fields) {
    call('robot:configure', { robotID: r.robotID, fields }).then((res) => { if (!res.success) fail(r, res); });
  };
  // After an emergency stop the robot is in Error until it is started again.
  const _panel = taskPanel;
  taskPanel = function (rs, r) {
    _panel(rs, r);
    if (r.state !== 'Error') return;
    const b = el('button', 'Clear error (robot:start)');
    b.onclick = async () => { const res = await call('robot:start', { robotID: r.robotID }); if (!res.success) fail(r, res); };
    rs.append(b);
  };

  // Obstacles live on the server, so every dashboard sees the same ones.
  socket.on('obstacles:snapshot', (list) => { S.obstacles = list; side(); draw(); });
  addObstacle = function (x, y) {
    const L = lvl(), radius = +S.obsDraft.radius > 0 ? +S.obsDraft.radius : 0.5;
    S.placingObs = false; S.obsGhost = null;
    call('obstacle:add', { level: L.name, x, y, radius }).then((res) => { if (!res.success) fail(null, res); });
  };
  removeObstacle = function (o) { call('obstacle:remove', { obstacleID: o.obstacleID }).then((res) => { if (!res.success) fail(null, res); }); };

  // Simulation controls (draft 5.1, 8.4): pause or resume the clock that moves robots; reset robots and tasks.
  const pauseBtn = el('button', 'Pause'), resetBtn = el('button', 'Reset sim');
  pauseBtn.title = 'Pause or resume robot movement (sim:pause / sim:resume)';
  resetBtn.title = 'Put every robot back at its start point and clear all tasks (sim:reset)';
  pauseBtn.onclick = () => call(simState === 'running' ? 'sim:pause' : 'sim:resume').then((res) => { if (!res.success) fail(null, res); });
  resetBtn.onclick = () => call('sim:reset').then((res) => { if (!res.success) fail(null, res); });
  $('header').insertBefore(pauseBtn, $('#undo')); $('header').insertBefore(resetBtn, $('#undo'));
  function showSim() { pauseBtn.textContent = simState === 'running' ? 'Pause' : 'Resume'; note(); }

  // Reordering levels on the page reorders them on the server too, so z (the level number) matches on both.
  const _order = setOrder;
  setOrder = function (next) {
    _order(next);
    call('map:level-order', { levels: S.levels.map((L) => L.name) }).then((res) => { if (!res.success) fail(null, res); });
  };

  // The nav graph is also sent to the server (the one HTTP endpoint in the draft); the server resets robots and tasks.
  const sendMap = (text) => {
    note('sending nav graph…');
    fetch('/api/map/import', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: text })
      .then((r) => r.json())
      .then((res) => note(res.success ? 'nav graph loaded on the server' : `server rejected the nav graph (${res.failureCode}): ${res.failureMessage}`))
      .catch(() => note('cannot reach the server'));
  };
  const _load = loadText;
  loadText = function (text, src) { _load(text, src); if (!S.error) sendMap(text); };
  if (S.levels.length && S.json) sendMap(JSON.stringify(S.json));   // the page may have auto-loaded before this script ran
  note();
})();
