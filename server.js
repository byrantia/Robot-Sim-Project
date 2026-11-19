const path = require('node:path'), fs = require('node:fs'), http = require('node:http');
const express = require('express');
const { Server } = require('socket.io');
const { CODES, ok, fail } = require('./src/protocol');
const { Engine } = require('./src/engine');
const { parseNavGraph } = require('./src/navgraph');
const { Simulation } = require('./src/simulation');
const { terminalLog } = require('./src/terminal-log');

const PORT = process.env.PORT || 3000, HOST = process.env.HOST || '127.0.0.1';
const PUBLIC = path.join(__dirname, 'public'), DATA = path.join(__dirname, 'data');
fs.mkdirSync(DATA, { recursive: true });

const engine = new Engine();
const sim = new Simulation(engine);   // the clock that moves robots; runs from start-up, sim:pause stops it
// what happens is printed here: LOG=all prints every robot:update, LOG=off nothing (default: movement about once a second)
const log = terminalLog(engine, process.env.LOG || 'normal');
const app = express(), server = http.createServer(app), io = new Server(server);

// ---- HTTP: dashboard files + the single API endpoint from the draft (nav graph upload)
app.use(express.static(PUBLIC));
app.post('/api/map/import', express.json({ limit: '50mb' }), (req, res) => {
  try { const nav = parseNavGraph(req.body); engine.setNav(nav); res.json(ok(nav.summary())); }
  catch (e) { log.http(`map    import rejected: ${e.message}`); res.status(400).json(fail(CODES.INVALID, e.message)); }
});

// ---- helpers for the dashboard's own load and Save (kept so its Save button keeps working)
const NAME = /^[\w .()-]+\.json$/, EDITED = /^[\w .()-]+_edited\.json$/;
app.get('/api/default', (req, res) => {
  const f = fs.readdirSync(DATA).find((n) => NAME.test(n) && !EDITED.test(n));
  f ? res.json({ file: f }) : res.status(404).json(fail(CODES.INVALID, 'No .json file in data/.'));
});
app.get('/:name', (req, res, next) => {
  if (!NAME.test(req.params.name)) return next();
  res.sendFile(path.join(DATA, req.params.name), (err) => err && res.status(404).end());
});
app.put('/:name', express.text({ type: () => true, limit: '50mb' }), (req, res) => {
  if (!EDITED.test(req.params.name)) return res.status(400).json({ error: 'Only *_edited.json files can be saved.' });
  try { JSON.parse(req.body); } catch { return res.status(400).json({ error: 'Body is not valid JSON.' }); }
  fs.writeFileSync(path.join(DATA, req.params.name), req.body);
  res.json(ok());
});
app.use((err, req, res, next) => res.status(err.status || 500).json(fail(CODES.INVALID, err.message)));

// ---- Socket.IO: command events (each answered with a reply) and live events
const handlers = {
  'robot:create': (p) => engine.createRobot(p),
  'robot:list': () => engine.listRobots(),
  'robot:get': (p) => engine.getRobot(p),
  'robot:status': (p) => engine.robotStatus(p),
  'robot:configure': (p) => engine.configureRobot(p),
  'robot:delete': (p) => engine.deleteRobot(p),
  'robot:start': (p) => engine.startRobot(p),
  'robot:emergency-stop': (p) => engine.emergencyStop(p),
  'robot:position': (p) => engine.robotPosition(p),
  'robot:path': (p) => engine.robotPath(p),
  'robot:localize': (p) => engine.localize(p),
  'robot:move': (p) => engine.moveRobot(p),
  'robot:stop': (p) => engine.stopRobot(p),
  'task:cancel': (p) => engine.cancelTask(p),
  'robot:obstacles': (p) => engine.robotObstacles(p),
  'obstacle:add': (p) => engine.addObstacle(p),
  'obstacle:list': () => engine.listObstacles(),
  'obstacle:remove': (p) => engine.removeObstacle(p),
  'task:assign': (p) => engine.assignTask(p),        // creates the job (Queued); nothing moves yet
  'robot:navigate': (p) => engine.navigateRobot(p),  // { robotID, taskID }: the robot takes the job and starts driving
  'task:get': (p) => engine.getTask(p),
  'map:get': () => engine.getMap(),
  'map:level-order': (p) => engine.setLevelOrder(p),
  'sim:reset': () => engine.reset(),
  'sim:start': () => ok(sim.start()),
  'sim:resume': () => ok(sim.start()),
  'sim:pause': () => ok(sim.pause()),
};
// In the draft but not built yet: they answer cleanly instead of timing out.
for (const ev of ['robot:pause', 'robot:return-to-dock',
  'test:list', 'test:run', 'test:get'])
  handlers[ev] = () => fail(CODES.NOT_IMPLEMENTED, `${ev} is not implemented yet.`);

// Thin adapter: engine events -> Socket.IO broadcasts
for (const ev of ['robot:update', 'task:update', 'task:removed', 'robots:snapshot', 'obstacles:snapshot', 'collision:event', 'map:loaded', 'sim:state']) engine.on(ev, (d) => io.emit(ev, d));

io.on('connection', (socket) => {
  log.clients(io.engine.clientsCount, true);
  socket.on('disconnect', () => log.clients(io.engine.clientsCount, false));
  socket.emit('sim:state', sim.state);
  socket.emit('robots:snapshot', engine.snapshot());
  socket.emit('obstacles:snapshot', engine.obstacleList());
  for (const t of engine.taskList()) socket.emit('task:update', t);
  for (const [ev, fn] of Object.entries(handlers)) {
    socket.on(ev, (payload, ack) => {
      if (typeof payload === 'function') { ack = payload; payload = {}; }
      let reply;
      try { reply = fn(payload && typeof payload === 'object' ? payload : {}); }
      catch (e) { console.error(e); reply = fail(CODES.INTERNAL, 'Internal error.'); }
      log.command(ev, payload, reply);
      if (typeof ack === 'function') ack(reply);
    });
  }
});

sim.start();
server.listen(PORT, HOST, () => console.log(`Robot simulator API: http://${HOST === '127.0.0.1' ? 'localhost' : HOST}:${PORT}`));
