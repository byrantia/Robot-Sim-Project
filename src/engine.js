// Task Engine + virtual robot environment. No network code in here, so it can be tested on its own.
// Every command method returns a standard reply { success, failureCode?, failureMessage?, data? }.
const { EventEmitter } = require('node:events');
const { CODES: C, ok, fail } = require('./protocol');
const { planRoute, planDetour } = require('./planner');
const { advance, routeLength, firstBlock, blocks, edgeDist, distToPath, entry, norm, ROBOT_RADIUS, STOP_GAP, SLOW_FACTOR, TURN_RATE } = require('./simulation');
const { TrafficManager, SAFE_DIST, RESUME_GAP, polyline, nearest, pathGap, corridorMap } = require('./traffic');
const isNum = (v) => typeof v === 'number' && Number.isFinite(v);
const clone = (o) => structuredClone(o);
const DONE_SECONDS = 2;    // a robot stays Completed this long after reaching its destination, then goes Idle
const DECIDE_SECONDS = 1;  // after stopping for an obstacle, the robot waits this long before trying a reroute
const RETRY_SECONDS = 3;   // while waiting at an obstacle, it tries a reroute again this often
const STUCK_SECONDS = 10;  // still no reroute this long after the first failed try: the robot goes from Waiting to Stuck
const FAR_DETOUR = 25;     // m: how far off the planned path to look for any way round, to tell "too far" from "none"
const GIVE_WAY_SECONDS = 5;        // robots blocking each other this long: the lower-priority one gives way
const GIVE_WAY_STEP = 2;           // ... and if it cannot, the next one tries this much later (all before STUCK_SECONDS)
const MAKE_WAY_METRES = 20;        // a robot making way drives at most this far to a node off the other robots' paths

class Engine extends EventEmitter {
  constructor({ now = () => new Date().toISOString() } = {}) {
    super();
    this.now = now; this.nav = null;
    this.robots = new Map(); this.starts = new Map(); this.tasks = new Map();
    this.obstacles = new Map(); this.obstacleSeq = 0;   // obstacleID -> { obstacleID, level, x, y, z, radius }
    this.motion = new Map();   // robotID -> movement bookkeeping while it drives a route, or { idleIn } after it arrives
    this.robotSeq = 0; this.taskSeq = 0;
    this.traffic = new TrafficManager(); this.trafficSeq = 0;   // multi-robot path reservations (see traffic.js)
  }
  snapshot() { return [...this.robots.values()].map(clone); }
  taskList() { return [...this.tasks.values()].map(clone); }
  _pushRobot(r) { this.emit('robot:update', clone(r)); }
  _pushTask(t) { this.emit('task:update', clone(t)); }
  _find(p) {
    if (typeof p.robotID !== 'string') return [null, fail(C.INVALID, 'robotID is required.')];
    const r = this.robots.get(p.robotID);
    return r ? [r] : [null, fail(C.ROBOT_NOT_FOUND, `No robot ${p.robotID}.`)];
  }
  // level name -> z (its index in the nav graph), or null when there is no nav graph
  _z(level) { const i = this.nav ? this.nav.levelNames.indexOf(level) : -1; return i >= 0 ? i : null; }

  setNav(nav) {
    this.nav = nav; this.obstacles.clear(); this.obstacleSeq = 0;
    this.reset(true); this._pushObstacles(); this.emit('map:loaded', nav.summary());
  }
  getMap() { return this.nav ? ok(this.nav.summary()) : fail(C.NO_NAV_GRAPH, 'No nav graph imported yet.'); }
  // The dashboard reordered its levels: z (the level number) follows the new order for robots, start points and tasks.
  setLevelOrder(p) {
    if (!this.nav) return fail(C.NO_NAV_GRAPH, 'No nav graph imported yet.');
    const names = this.nav.levelNames, l = p.levels;
    if (!Array.isArray(l) || l.length !== names.length || !names.every((n) => l.includes(n))) return fail(C.INVALID, 'levels must list every level name once.');
    names.splice(0, names.length, ...l);
    for (const r of this.robots.values()) { r.position.z = this._z(r.level); for (const q of r.route) q.z = this._z(q.level); }
    for (const s of this.starts.values()) s.position.z = this._z(s.level);
    for (const t of this.tasks.values()) { t.z = this._z(t.level); this._pushTask(t); }
    for (const o of this.obstacles.values()) o.z = this._z(o.level);
    this._pushObstacles();
    this.emit('robots:snapshot', this.snapshot());
    return ok(this.nav.summary());
  }

  // reset(true) also forgets the robots (used when a new nav graph replaces the environment)
  reset(forgetRobots = false) {
    this.tasks.clear(); this.taskSeq = 0; this.motion.clear();
    if (forgetRobots) { this.robots.clear(); this.starts.clear(); this.robotSeq = 0; }
    for (const r of this.robots.values()) Object.assign(r, clone(this.starts.get(r.robotID)), { state: 'Idle', activeTask: null, route: [], routeIndex: 0, routeProgress: 0, blockedBy: [], activeMove: null, stopped: false });
    this.emit('sim:state', 'reset'); this.emit('robots:snapshot', this.snapshot());
    return ok();
  }

  createRobot(p) {
    if (p.name !== undefined && (typeof p.name !== 'string' || !p.name.trim())) return fail(C.INVALID, 'name must be non-empty text.');
    if (p.speed !== undefined && !(isNum(p.speed) && p.speed > 0)) return fail(C.INVALID, 'speed must be a number above 0.');
    const bad = this._badSensing(p); if (bad) return bad;
    const sp = p.startPosition === undefined ? {} : p.startPosition;
    if (!sp || typeof sp !== 'object') return fail(C.INVALID, 'startPosition must be an object.');
    for (const k of ['x', 'y', 'z', 'orientation']) if (sp[k] !== undefined && !isNum(sp[k])) return fail(C.INVALID, `startPosition.${k} must be a number.`);
    if (sp.level !== undefined && this.nav && !this.nav.levels[sp.level]) return fail(C.INVALID, 'startPosition.level is not on the nav graph.');
    if (sp.z !== undefined && !Number.isInteger(sp.z)) return fail(C.INVALID, 'startPosition.z must be a whole level number.');
    const id = 'robot_' + ++this.robotSeq;
    const level = sp.level ?? (this.nav ? this.nav.levelNames[0] : null);
    const r = {
      robotID: id, name: p.name ? p.name.trim() : 'Robot ' + this.robotSeq, state: 'Idle', level,
      // z is the level: its index in the nav graph (the node pose.z in the file is a heading, not a floor)
      position: { x: sp.x ?? 0, y: sp.y ?? 0, z: this._z(level) ?? sp.z ?? 0 }, orientation: sp.orientation ?? 0,
      speed: p.speed ?? 0.5, activeTask: null, route: [], routeIndex: 0, routeProgress: 0,
      // sensorRange: how far (m) it senses obstacles; maxDetour: how far (m) a reroute may stray from the planned path
      sensorRange: p.sensorRange ?? 2, maxDetour: p.maxDetour ?? 5, blockedBy: [],
      activeMove: null,   // the robot:move command it is carrying out: { command, distance | angle, done }
      stopped: false,     // paused by robot:stop (holds its task or move until robot:start)
    };
    this.robots.set(id, r);
    this.starts.set(id, { level: r.level, position: clone(r.position), orientation: r.orientation });
    this._pushRobot(r);
    return ok(clone(r));
  }
  listRobots() { return ok(this.snapshot()); }
  getRobot(p) { const [r, e] = this._find(p); return e || ok(clone(r)); }
  robotStatus(p) { const [r, e] = this._find(p); return e || ok({ robotID: r.robotID, state: r.state }); }
  configureRobot(p) {
    const [r, e] = this._find(p); if (e) return e;
    const f = p.fields;
    if (!f || typeof f !== 'object') return fail(C.INVALID, 'fields is required.');
    for (const k of Object.keys(f)) if (!['name', 'speed', 'sensorRange', 'maxDetour'].includes(k)) return fail(C.INVALID, `Cannot change "${k}".`);
    const bad = this._badSensing(f); if (bad) return bad;
    if (f.name !== undefined && (typeof f.name !== 'string' || !f.name.trim())) return fail(C.INVALID, 'name must be non-empty text.');
    if (f.speed !== undefined && !(isNum(f.speed) && f.speed > 0)) return fail(C.INVALID, 'speed must be a number above 0.');
    if (f.name !== undefined) r.name = f.name.trim();
    if (f.speed !== undefined) r.speed = f.speed;
    if (f.sensorRange !== undefined) r.sensorRange = f.sensorRange;
    if (f.maxDetour !== undefined) r.maxDetour = f.maxDetour;
    this._pushRobot(r);
    return ok(clone(r));
  }
  _badSensing(f) {
    if (f.sensorRange !== undefined && !(isNum(f.sensorRange) && f.sensorRange > 0)) return fail(C.INVALID, 'sensorRange must be a number above 0.');
    if (f.maxDetour !== undefined && !(isNum(f.maxDetour) && f.maxDetour >= 0)) return fail(C.INVALID, 'maxDetour must be a number, 0 or more.');
    return null;
  }
  deleteRobot(p) {
    const [r, e] = this._find(p); if (e) return e;
    if (r.activeTask) return fail(C.ROBOT_UNAVAILABLE, `${r.robotID} has a task. Stop it first (robot:emergency-stop).`);
    this.robots.delete(r.robotID); this.starts.delete(r.robotID); this.motion.delete(r.robotID);
    // its tasks go with it, so none are left pointing at a robot that no longer exists
    for (const t of [...this.tasks.values()]) if (t.robotID === r.robotID) {
      this.tasks.delete(t.taskID); this.emit('task:removed', { taskID: t.taskID, robotID: r.robotID });
    }
    this.emit('robots:snapshot', this.snapshot());
    return ok();
  }
  // Resumes a robot paused by robot:stop, carrying on with its task or move from where it is.
  // Otherwise clears an Error state (or a finished Completed one) so the robot can take a task again.
  startRobot(p) {
    const [r, e] = this._find(p); if (e) return e;
    const m = this.motion.get(r.robotID);
    if (r.stopped && m && m.stopped) {
      const t = this.tasks.get(r.activeTask), was = m.stopped;
      delete m.stopped;
      r.stopped = false; r.state = was.state;
      if (t) { t.status = was.status; t.note = 'Resumed (robot:start).'; this._pushTask(t); }
      this._pushRobot(r);
      return ok(clone(r));
    }
    if (r.state === 'Error' || r.state === 'Completed') { r.state = 'Idle'; this.motion.delete(r.robotID); this._pushRobot(r); }
    else if (r.state !== 'Idle') return fail(C.ROBOT_UNAVAILABLE, `${r.robotID} is ${r.state}.`);
    return ok(clone(r));
  }
  // Pause (draft 8.2 robot:stop): the robot holds where it is, keeping its task or move, until robot:start resumes it.
  // While stopped the robot is Paused (and stopped: true); its task stays active as Waiting. Waiting on the robot itself
  // means it is working out a way past an obstacle (or riding a lift). task:cancel cancels the task instead.
  stopRobot(p) {
    const [r, e] = this._find(p); if (e) return e;
    const m = this.motion.get(r.robotID);
    if (!m || !(r.activeTask || r.activeMove)) return fail(C.ROBOT_UNAVAILABLE, `${r.robotID} has no task or move to stop.`);
    if (r.stopped) return ok(clone(r));
    const t = this.tasks.get(r.activeTask);
    m.stopped = { state: r.state, status: t ? t.status : null };
    r.stopped = true; r.state = 'Paused';
    if (t) { t.status = 'Waiting'; t.note = 'Paused (robot:stop). robot:start resumes it, task:cancel cancels it.'; this._pushTask(t); }
    this._pushRobot(r);
    return ok(clone(r));
  }
  // Cancels a task that is still active (running, waiting or stopped). It ends as Failed with a "Cancelled" outcome,
  // and its robot goes Idle where it is.
  cancelTask(p) {
    const t = typeof p.taskID === 'string' && this.tasks.get(p.taskID);
    if (!t) return fail(C.TASK_NOT_FOUND, `No task ${p.taskID}.`);
    if (!['Queued', 'Running', 'Waiting'].includes(t.status)) return fail(C.INVALID, `${t.taskID} is ${t.status}: only an active task can be cancelled.`);
    Object.assign(t, { status: 'Failed', outcome: 'Cancelled before reaching the destination.',
      error: { failureCode: C.TASK_CANCELLED, failureMessage: 'Cancelled (task:cancel).' }, note: null });
    this._pushTask(t);
    const r = this.robots.get(t.robotID);
    if (r && r.activeTask === t.taskID) {
      Object.assign(r, { activeTask: null, state: 'Idle', route: [], routeIndex: 0, routeProgress: 0, blockedBy: [], stopped: false });
      this.motion.delete(r.robotID);
      this._pushRobot(r);
    }
    return ok(clone(t));
  }
  emergencyStop(p) {
    const [r, e] = this._find(p); if (e) return e;
    const t = r.activeTask && this.tasks.get(r.activeTask);
    if (t) {
      Object.assign(t, { status: 'Failed', outcome: 'Not reached. Emergency stop.', error: { failureCode: C.EMERGENCY_STOP, failureMessage: 'Emergency stop' } });
      this._pushTask(t);
    }
    r.activeTask = null; r.state = 'Error'; r.route = []; r.routeIndex = 0; r.routeProgress = 0; r.blockedBy = []; r.activeMove = null; r.stopped = false; this.motion.delete(r.robotID);
    this._pushRobot(r);
    return ok(clone(r));
  }
  // Position and planned path (draft 8.3).
  robotPosition(p) { const [r, e] = this._find(p); return e || ok({ robotID: r.robotID, level: r.level, position: clone(r.position), orientation: r.orientation }); }
  robotPath(p) { const [r, e] = this._find(p); return e || ok({ robotID: r.robotID, route: clone(r.route), routeIndex: r.routeIndex, routeProgress: r.routeProgress }); }
  // Set where the robot is (and where sim:reset puts it back).
  // A robot on a task can be moved too, like pushing it out of the way by hand, but only while it is not driving
  // (Waiting, Stuck or Paused, and not in a lift): it then plans its route again from where it is put down, and
  // sim:reset still puts it back at its start point.
  localize(p) {
    const [r, e] = this._find(p); if (e) return e;
    const pos = p.position;
    if (!pos || typeof pos !== 'object' || !isNum(pos.x) || !isNum(pos.y)) return fail(C.INVALID, 'position.x and position.y must be numbers.');
    if (p.orientation !== undefined && !isNum(p.orientation)) return fail(C.INVALID, 'orientation must be a number.');
    if (p.level !== undefined && this.nav && !this.nav.levels[p.level]) return fail(C.INVALID, 'level is not on the nav graph.');
    if (r.activeTask) return this._carry(r, p);
    if (r.activeMove) return fail(C.ROBOT_UNAVAILABLE, `${r.robotID} is carrying out robot:move. Stop it first.`);
    if (p.level !== undefined) r.level = p.level;
    r.position = { x: pos.x, y: pos.y, z: this._z(r.level) ?? r.position.z };
    if (p.orientation !== undefined) r.orientation = p.orientation;
    this.starts.set(r.robotID, { level: r.level, position: clone(r.position), orientation: r.orientation });
    this._pushRobot(r);
    return ok(clone(r));
  }
  // Moving a robot on a task by hand (see localize).
  _carry(r, p) {
    const m = this.motion.get(r.robotID), t = this.tasks.get(r.activeTask);
    if (!m || !t || m.idleIn !== undefined) return fail(C.ROBOT_UNAVAILABLE, `${r.robotID} is finishing ${r.activeTask}.`);
    if (!r.stopped && !['Waiting', 'Stuck'].includes(r.state)) return fail(C.ROBOT_UNAVAILABLE, `${r.robotID} is ${r.state} on ${r.activeTask}. Pause it (robot:stop) to move it by hand.`);
    if (m.lift != null) return fail(C.ROBOT_UNAVAILABLE, `${r.robotID} is in a lift.`);
    const level = p.level ?? r.level, at = { level, x: p.position.x, y: p.position.y };
    // its route on from there (it must have one, or it is not moved)
    const lift = t.lift && t.level !== level && this.nav.lifts.some((l) => l.id === t.lift && l.levels.includes(level)) ? t.lift : null;
    const plan = planRoute(this.nav, at, { level: t.level, node: t.node }, lift);
    if (plan.error) return fail(C.NO_ROUTE, `No route from there to ${t.destination}: ${plan.error}`);
    r.level = level;
    r.position = { x: at.x, y: at.y, z: this._z(level) ?? r.position.z };
    if (p.orientation !== undefined) r.orientation = p.orientation;
    Object.assign(r, { route: plan.route.map((q) => ({ ...q, z: this._z(q.level) })), routeIndex: 0, routeProgress: 0, blockedBy: [] });
    Object.assign(m, { travelled: 0, total: routeLength(r), avoid: null, traffic: null, claim: null, evade: null,
      planned: [{ level, x: at.x, y: at.y }, ...r.route] });
    const note = `Moved by hand to (${at.x.toFixed(2)}, ${at.y.toFixed(2)}). Route planned again from there.`;
    t.steps = plan.steps; t.note = note;
    if (r.stopped) Object.assign(m.stopped, { state: 'Moving', status: 'Running' });   // still paused: carries on when resumed
    else { r.state = 'Moving'; t.status = 'Running'; }
    this._pushTask(t); this._pushRobot(r);
    return ok(clone(r));
  }

  // task:assign creates a job (Queued) and nothing moves: robot:navigate gives it to a robot and starts it.
  // robotID is optional: it reserves the job for that robot, so only that robot can navigate it.
  // lift is optional: the lift to take if the route changes level (checked against the robot's level at navigate).
  assignTask(p) {
    if (typeof p.destination !== 'string' || !p.destination.trim()) return fail(C.INVALID, 'destination is required.');
    if (p.taskID !== undefined && (typeof p.taskID !== 'string' || !p.taskID.trim() || this.tasks.has(p.taskID))) return fail(C.INVALID, 'taskID must be unused text.');
    if (p.robotID !== undefined) { const [, e] = this._find(p); if (e) return e; }
    if (!this.nav) return fail(C.NO_NAV_GRAPH, 'Import a nav graph first.');
    const poi = this.nav.findDestination(p.destination, p.level);
    if (!poi) return fail(C.DEST_NOT_FOUND, `"${p.destination}" is not a point of interest or node on the nav graph.`);
    if (p.lift !== undefined && (typeof p.lift !== 'string' || !p.lift.trim())) return fail(C.INVALID, 'lift must be a lift id.');
    if (p.lift !== undefined) {
      const l = this.nav.lifts.find((x) => x.id === p.lift);
      if (!l) return fail(C.INVALID, `No lift "${p.lift}" on the nav graph.`);
      if (!l.levels.includes(poi.level)) return fail(C.INVALID, `Lift ${p.lift} does not go to ${poi.level}.`);
    }
    let id = p.taskID;
    if (!id) do id = 'task_' + ++this.taskSeq; while (this.tasks.has(id));
    const t = { taskID: id, robotID: p.robotID ?? null, destination: poi.name, level: poi.level, z: this._z(poi.level), node: poi.node,
      lift: p.lift ?? null, status: 'Queued', createdAt: this.now(), outcome: null, error: null, note: null, steps: [] };
    this.tasks.set(id, t);
    this._pushTask(t);
    return ok(clone(t));
  }
  // robot:navigate { robotID, taskID }: gives a Queued job to the robot, plans the route (BFS on the nav graph, via a
  // lift if needed) and starts it driving. One active job per robot: the robot must be Idle (or just Completed).
  // With no route the job stays Queued (another robot, or this one from somewhere else, may still reach it).
  navigateRobot(p) {
    const [r, e] = this._find(p); if (e) return e;
    if (typeof p.taskID !== 'string') return fail(C.INVALID, 'taskID is required.');
    const t = this.tasks.get(p.taskID);
    if (!t) return fail(C.TASK_NOT_FOUND, `No task ${p.taskID}.`);
    if (t.status !== 'Queued') return fail(C.INVALID, `${t.taskID} is ${t.status}: only a Queued task can be started.`);
    if (t.robotID && t.robotID !== r.robotID) return fail(C.INVALID, `${t.taskID} is reserved for ${t.robotID}.`);
    if (r.activeTask) return fail(C.ROBOT_UNAVAILABLE, `${r.robotID} already has ${r.activeTask}. One task at a time.`);
    if (r.state !== 'Idle' && r.state !== 'Completed') return fail(C.ROBOT_UNAVAILABLE, `${r.robotID} is ${r.state}. It must be Idle to start a task.`);
    if (!this.nav) return fail(C.NO_NAV_GRAPH, 'Import a nav graph first.');
    // the lift only matters when the route changes level; it must also serve the robot's level
    let lift = null;
    if (t.lift && t.level !== r.level) {
      const l = this.nav.lifts.find((x) => x.id === t.lift);
      if (!l || !l.levels.includes(r.level)) return fail(C.INVALID, `Lift ${t.lift} does not go between ${r.level} and ${t.level}.`);
      lift = l.id;
    }
    const plan = planRoute(this.nav, { level: r.level, x: r.position.x, y: r.position.y }, { level: t.level, node: t.node }, lift);
    if (plan.error) {
      t.note = `${r.robotID} has no route: ${plan.error}`; this._pushTask(t);
      return fail(C.NO_ROUTE, `No route for ${r.robotID} to ${t.destination}: ${plan.error}`);
    }
    Object.assign(t, { robotID: r.robotID, steps: plan.steps, status: 'Running', note: null, startedAt: this.now() });
    this._startRoute(r, t, plan.route);
    this._pushTask(t); this._pushRobot(r);
    return ok(clone(t));
  }
  // Puts the robot on a planned route for task t; tick() then drives it (turn, drive, lifts, obstacles).
  _startRoute(r, t, route) {
    r.route = route.map((q) => ({ ...q, z: this._z(q.level) })); r.routeIndex = 0; r.routeProgress = 0;
    r.activeTask = t.taskID; r.state = 'Moving'; r.blockedBy = [];
    // planned: the route as first planned (from where the robot stood); reroutes are measured against it
    // prio: traffic priority (lower goes first): jobs started earlier go first
    this.motion.set(r.robotID, { travelled: 0, total: routeLength(r), lift: null, avoid: null, traffic: null, claim: null,
      prio: ++this.trafficSeq, planned: [{ level: r.level, x: r.position.x, y: r.position.y }, ...r.route] });
  }
  // One simulation step (called by the Simulation clock): move every robot on a task, finish tasks that arrive.
  // The Traffic Manager first decides how far each robot on a task may drive (its reservation), then every robot moves.
  tick(dt) {
    const grants = this._trafficPlan();
    for (const [id, m] of this.motion) {
      if (m.stopped) continue;   // paused by robot:stop
      const r = this.robots.get(id);
      if (!r) { this.motion.delete(id); continue; }
      if (m.move) { this._stepMove(r, m, dt); continue; }
      if (m.idleIn !== undefined) {
        if ((m.idleIn -= dt) <= 0) { this.motion.delete(id); if (r.state === 'Completed') { r.state = 'Idle'; this._pushRobot(r); } }
        continue;
      }
      const t = this.tasks.get(r.activeTask), before = r.state;
      const g = grants.get(id) || { allow: Infinity, waitFor: [], len: 0 };
      if (m.evade) {   // making way for other robots
        const arrived = r.routeIndex >= r.route.length;
        this._holding(r, m, t, dt, arrived);
        if (arrived && m.evade) continue;   // waiting at the node it made way to
      }
      if (this._yield(r, m, t, g, dt)) continue;                                // waiting for other robots
      const obs = [...this.obstacles.values()].filter((o) => o.level === r.level);
      const block = firstBlock(r, obs);
      if (this._avoid(r, m, t, block, dt, g.allow)) continue;
      const near = obs.some((o) => edgeDist(r, o) <= r.sensorRange + 1e-6);   // something in sensor range: slow down
      if (advance(r, m, dt, Math.min(block ? block.at : Infinity, g.allow), near ? SLOW_FACTOR : 1, (p) => this._exitClear(r, p))) {
        if (m.evade) { this._live(r); continue; }   // reached the node it made way to, not its destination
        if (t) Object.assign(t, { status: 'Completed', outcome: `Reached ${t.destination} on ${t.level}.` });
        r.state = 'Completed'; r.activeTask = null; r.blockedBy = [];
        this.motion.set(id, { idleIn: DONE_SECONDS });
        if (t) this._pushTask(t);
        this._pushRobot(r);
        continue;
      }
      if (t && r.state !== before) { t.status = r.state === 'Waiting' ? 'Waiting' : 'Running'; this._pushTask(t); }
      this._live(r);
    }
  }
  // Hands every robot to the Traffic Manager: a robot driving a task route (not paused) with its route ahead on its
  // level, priority and current reservation; every other robot as a footprint the others keep clear of.
  // Remembers each new reservation as a distance along the route, so it carries over to the next tick.
  _trafficPlan() {
    const list = [];
    for (const r of this.robots.values()) {
      const m = this.motion.get(r.robotID), e = { id: r.robotID, level: r.level, x: r.position.x, y: r.position.y, prio: Infinity, speed: r.speed, path: null, held: 0 };
      const onTask = m && m.prio && r.activeTask && m.idleIn === undefined;
      const path = [{ x: r.position.x, y: r.position.y }];
      if (onTask) for (let i = r.routeIndex; i < r.route.length && r.route[i].level === r.level; i++) path.push(r.route[i]);
      if (onTask && !m.stopped) Object.assign(e, { prio: m.prio, path, held: m.claim && m.claim.route === r.route ? m.claim.end - m.travelled : 0 });
      else if (m) m.claim = null;   // paused: it gives up its reservation until it resumes ...
      if (onTask && m.stopped) Object.assign(e, { path, paused: true });   // ... but others keep out of its way
      if (onTask) e.corridors = this._corridors(r);   // a paused robot still holds the corridor it is in
      list.push(e);
    }
    const grants = this.traffic.plan(list);
    for (const [id, g] of grants) { const r = this.robots.get(id), m = this.motion.get(id); m.claim = { route: r.route, end: m.travelled + g.allow }; }
    return grants;
  }
  // The narrow corridors on the robot's route ahead on its level, as distances along it (from < 0: it is in one).
  _corridors(r) {
    const cm = corridorMap(this.nav, r.level), out = [];
    if (!cm.size) return out;
    const prev = r.routeIndex > 0 && r.route[r.routeIndex - 1].level === r.level ? r.route[r.routeIndex - 1] : null;
    let d = 0, at = r.position;
    for (let i = r.routeIndex; i < r.route.length && r.route[i].level === r.level; i++) {
      const p = r.route[i], a = i === r.routeIndex ? prev : r.route[i - 1], len = Math.hypot(p.x - at.x, p.y - at.y);
      const c = a && a.nodeID && p.nodeID && cm.get(`${a.nodeID}|${p.nodeID}`);
      // on the link it is driving now: how far it is past the node it came from (so from < 0 once it is in)
      if (c) out.push({ key: c.chain, dir: c.dir, from: i === r.routeIndex ? -Math.hypot(r.position.x - a.x, r.position.y - a.y) : d, to: d + len });
      d += len; at = p;
    }
    return out;
  }
  // A robot coming out of a lift waits in it while another robot is within SAFE_DIST of the lift door.
  _exitClear(r, p) {
    for (const o of this.robots.values()) if (o !== r && o.level === p.level && Math.hypot(o.position.x - p.x, o.position.y - p.y) < SAFE_DIST) return false;
    return true;
  }
  // Waiting for other robots (traffic.js). A robot that cannot reserve any of its route waits (Waiting, blockedBy
  // the robots in its way) and carries on by itself, on the same route, once it can reserve RESUME_GAP again (or the
  // rest of its route).
  // Give way: when robots on jobs block each other (a ring: it waits for one that waits ... for it) and none of them
  // has moved for GIVE_WAY_SECONDS, the lowest-priority one gives way: it goes round the others (the same detour /
  // nav graph reroute as for obstacles) or, if it cannot, makes way at a node off their path until they have passed
  // (_makeWay). If it cannot, the next one in the ring tries GIVE_WAY_STEP later, and so on, all before they are
  // Stuck: a Stuck robot never gives way. Nobody gives way to a robot that is not waiting for it: an idle robot in
  // the way, or one that is only passing, is just waited for. A paused robot is just waited for (never Stuck).
  // If the robots it waits for have stayed put for STUCK_SECONDS
  // it is Stuck: from then on it only waits there, it does not give way or turn round (you can drag one of them out
  // of the way), and it carries on once the way is clear.
  // Returns true while it waits.
  _yield(r, m, t, g, dt) {
    const tw = m.traffic;
    if (!tw) {
      if (m.avoid || g.len <= 1e-6 || g.allow > 1e-6) return false;   // free to drive (or handling an obstacle)
      m.traffic = { by: g.waitFor, stalled: 0, ring: 0, retry: 0, stuck: false, paused: '', why: '' };
      this._obstacleEvent(r, t, 'wait', null, `Waiting for ${this._names(g.waitFor)} to clear the way.`, g.waitFor);
      return true;
    }
    if (g.allow >= Math.min(RESUME_GAP, g.len) - 1e-9) {
      m.traffic = null;
      this._obstacleEvent(r, t, 'resume', null, 'The way is clear of other robots. Carrying on.', tw.by);
      return false;
    }
    if (g.waitFor.length && g.waitFor.join() !== tw.by.join()) {
      tw.by = g.waitFor;
      if (!tw.stuck) this._obstacleEvent(r, t, 'wait', null, `Waiting for ${this._names(tw.by)} to clear the way.`, tw.by);
    }
    const others = tw.by.map((id) => this.robots.get(id)).filter(Boolean);
    if (!others.length || others.some((o) => o.state === 'Moving')) { tw.stalled = 0; tw.ring = 0; return true; }   // they are on their way
    // a paused robot (robot:stop) carries on once it is resumed: just wait for it (never Stuck)
    const paused = others.filter((o) => o.state === 'Paused').map((o) => o.robotID);
    if (paused.length) {
      tw.stalled = 0; tw.ring = 0;
      if (tw.paused !== paused.join()) {
        tw.paused = paused.join();
        this._obstacleEvent(r, t, 'wait', null, `Waiting for ${this._names(tw.by)} to clear the way (${this._names(paused)} ${paused.length > 1 ? 'are' : 'is'} paused).`, tw.by);
      }
      return true;
    }
    tw.paused = '';
    tw.stalled += dt;
    const ring = this._ring(r);
    if (ring) tw.ring += dt; else { tw.ring = 0; tw.retry = 0; }
    // Give way, only before it is Stuck (once Stuck it stays put): the lowest priority in the ring tries first, at
    // GIVE_WAY_SECONDS, then the next one GIVE_WAY_STEP later if the ring is still there (it could not), and so on
    if (ring && !m.evade && !tw.stuck) {
      const rank = ring.members.filter((o) => this.motion.get(o.robotID).prio > m.prio).length;   // 0: lowest priority
      if (tw.ring >= GIVE_WAY_SECONDS + rank * GIVE_WAY_STEP - 1e-9 && (tw.retry -= dt) <= 1e-9) {
        tw.retry = RETRY_SECONDS;
        const mine = ring.blockers;   // the robots in the ring it waits for
        // go round the ones standing on its path ahead (as obstacles that keep SAFE_DIST between the two robots)
        const ahead = polyline([{ x: r.position.x, y: r.position.y }, ...r.route.slice(r.routeIndex).filter((p) => p.level === r.level)]);
        const inPath = mine.filter((o) => o.level === r.level && nearest(o.position, ahead).d < SAFE_DIST + 0.05);
        const rr = inPath.length ? this._reroute(r, m, t, inPath.map((o) => ({ obstacleID: o.robotID, level: o.level, x: o.position.x, y: o.position.y, radius: SAFE_DIST - ROBOT_RADIUS })))
          : { why: `${this._names(mine.map((o) => o.robotID))} ${mine.length > 1 ? 'are' : 'is'} not on its path, so there is nothing to go round` };
        if (rr.ok) {
          m.traffic = null;
          this._obstacleEvent(r, t, 'replan', null, `Giving way: going round ${this._names(inPath.map((o) => o.robotID))} (up to ${rr.dev.toFixed(1)} m off the planned path).`, tw.by);
          return true;   // drives the new route from the next tick, once the Traffic Manager has reserved it
        }
        const mw = this._makeWay(r, m, t, mine);
        if (mw.ok) {
          m.traffic = null;
          this._obstacleEvent(r, t, 'replan', null, `Giving way to ${this._names(mine.map((o) => o.robotID))}: making way at ${mw.node}.`, tw.by);
          return true;
        }
        tw.why = `${rr.why}, and ${mw.why}`;
      }
    }
    if (!tw.stuck && tw.stalled >= STUCK_SECONDS - 1e-9) {
      tw.stuck = true;
      this._obstacleEvent(r, t, 'stuck', null, `Stuck: ${this._names(tw.by)} still in the way after ${STUCK_SECONDS} s${tw.why ? ` (no way to give way: ${tw.why})` : ''}. Waiting here until the way is clear.`, tw.by);
    }
    return true;
  }
  // A deadlock ring through robot r: following who waits for whom (robots on jobs, waiting for traffic), it gets back
  // to r. Returns { members, blockers } (every robot in the ring, and the ones r itself waits for), or null.
  _ring(r) {
    const waits = (o) => { const om = this.motion.get(o.robotID); return om && om.prio && om.traffic ? om.traffic.by : null; };
    const blockers = [], members = new Map();
    for (const id of waits(r) || []) {
      const seen = new Set([id]), queue = [id];
      let back = false;
      for (let i = 0; i < queue.length && !back; i++) {
        const o = this.robots.get(queue[i]), by = o && waits(o);
        if (!by) continue;
        for (const n of by) { if (n === r.robotID) { back = true; break; } if (!seen.has(n)) { seen.add(n); queue.push(n); } }
      }
      if (back) { blockers.push(this.robots.get(id)); for (const n of seen) if (waits(this.robots.get(n) || {})) members.set(n, this.robots.get(n)); }
    }
    return blockers.length ? { blockers, members: [...members.values()] } : null;
  }
  _names(ids) { return ids.length ? ids.join(', ') : 'other robots'; }
  // Low-level movement (draft 8.3 robot:move): forward / backward by a distance (m) along the robot's heading,
  // rotate by an angle (degrees, + is anticlockwise / left, - is clockwise / right) on the spot, or stop.
  // It drives at the robot's speed and turns at TURN_RATE over the following ticks (state Moving), then goes Idle.
  // It is not a task: a robot on a task, or in Error, cannot take one. A drive stops STOP_GAP short of an obstacle,
  // and short of a wall on the floor plan; the command then ends early with a collision:event (response stop).
  moveRobot(p) {
    const [r, e] = this._find(p); if (e) return e;
    const cmd = p.command;
    if (!['forward', 'backward', 'rotate', 'stop'].includes(cmd)) return fail(C.INVALID, 'command must be forward, backward, rotate or stop.');
    if (r.activeTask) return fail(C.ROBOT_UNAVAILABLE, `${r.robotID} is on ${r.activeTask}. Use robot:emergency-stop to stop it.`);
    if (cmd === 'stop') {
      if (r.activeMove) this._endMove(r);
      return ok(clone(r));
    }
    if (r.state === 'Error') return fail(C.ROBOT_UNAVAILABLE, `${r.robotID} is in Error. Clear it with robot:start first.`);
    if (cmd === 'rotate' && !(isNum(p.angle) && p.angle !== 0)) return fail(C.INVALID, 'angle must be a number of degrees, not 0 (+ turns left, - turns right).');
    if (cmd !== 'rotate' && !(isNum(p.distance) && p.distance > 0)) return fail(C.INVALID, 'distance must be a number of metres above 0.');
    const move = cmd === 'rotate' ? { command: cmd, angle: p.angle, done: 0 } : { command: cmd, distance: p.distance, done: 0 };
    // how much clearance from walls to keep: the robot's radius, or less if it already starts closer than that
    const occ = this.nav && this.nav.occupancy[r.level], at = r.position;
    const wall = !occ ? null : occ.free(at.x, at.y, ROBOT_RADIUS) ? ROBOT_RADIUS : occ.free(at.x, at.y, 0) ? 0 : null;
    this.motion.set(r.robotID, { move, wall });
    Object.assign(r, { activeMove: move, state: 'Moving', blockedBy: [], stopped: false });
    this._pushRobot(r);
    return ok(clone(r));
  }
  _stepMove(r, m, dt) {
    const mv = m.move;
    if (mv.command === 'rotate') {
      const turn = Math.min(Math.abs(mv.angle) - mv.done, TURN_RATE * dt);
      r.orientation = norm(r.orientation + Math.sign(mv.angle) * turn); mv.done += turn;
      if (mv.done >= Math.abs(mv.angle) - 1e-9) return this._endMove(r);
      return this._live(r);
    }
    const a = r.orientation * Math.PI / 180, s = mv.command === 'backward' ? -1 : 1, ux = Math.cos(a) * s, uy = Math.sin(a) * s;
    const step = Math.min(r.speed * dt, mv.distance - mv.done), from = { x: r.position.x, y: r.position.y };
    const to = { x: from.x + ux * step, y: from.y + uy * step };
    let go = step, hit = null;
    // other robots count as obstacles too: it stops SAFE_DIST (centre to centre) short of them
    const robots = [...this.robots.values()].filter((o) => o !== r && o.level === r.level)
      .map((o) => ({ robotID: o.robotID, level: o.level, x: o.position.x, y: o.position.y, radius: SAFE_DIST - ROBOT_RADIUS - STOP_GAP }));
    for (const o of [...this.obstacles.values(), ...robots]) {
      if (o.level !== r.level || (o.x - from.x) * ux + (o.y - from.y) * uy <= 0) continue;   // behind it: moving away is fine
      const d = entry(from, to, o, o.radius + ROBOT_RADIUS + STOP_GAP) * step;
      if (d < go - 1e-9) { go = d; hit = o; }
    }
    const occ = this.nav && this.nav.occupancy[r.level];
    if (occ && m.wall != null) {
      for (let k = 1, n = Math.max(1, Math.ceil(go / 0.05)); k <= n; k++) {
        const d = (go * k) / n;
        if (!occ.free(from.x + ux * d, from.y + uy * d, m.wall)) { go = (go * (k - 1)) / n; hit = 'wall'; break; }
      }
    }
    r.position.x += ux * go; r.position.y += uy * go; mv.done += go;
    if (hit) {
      const what = hit === 'wall' ? 'a wall' : hit.robotID || hit.obstacleID, done = mv.done.toFixed(2);
      this.emit('collision:event', { robotIDs: [r.robotID, ...(hit.robotID ? [hit.robotID] : [])], type: hit === 'wall' ? 'wall' : hit.robotID ? 'robot' : 'obstacle',
        obstacleIDs: hit === 'wall' || hit.robotID ? [] : [hit.obstacleID],
        response: 'stop', message: `${mv.command} ${mv.distance} m stopped after ${done} m: ${what} is in the way.`, level: r.level, position: clone(r.position) });
      return this._endMove(r);
    }
    if (mv.done >= mv.distance - 1e-9) return this._endMove(r);
    this._live(r);
  }
  _endMove(r) {
    this.motion.delete(r.robotID);
    Object.assign(r, { activeMove: null, state: 'Idle', stopped: false });
    this._pushRobot(r);
  }
  _live(r) {
    this.emit('robot:update', { robotID: r.robotID, state: r.state, level: r.level, position: clone(r.position), orientation: r.orientation,
      activeTask: r.activeTask, routeIndex: r.routeIndex, routeProgress: r.routeProgress, activeMove: r.activeMove && clone(r.activeMove) });
  }
  // Obstacle handling (draft 10.3). When an obstacle in the robot's way comes within its sensor range, the robot stops
  // (Waiting) and after DECIDE_SECONDS tries a reroute: first a detour round the obstacle on free floor, then another
  // route on the nav graph. Either is taken only if no point of it is more than maxDetour from the planned path.
  // Otherwise it drives up slowly, stops STOP_GAP short of the obstacle and waits there (Waiting), trying again every
  // RETRY_SECONDS. If it still has no way past STUCK_SECONDS after the first failed try, it is Stuck. Waiting or
  // Stuck, it keeps trying and carries on by itself once a reroute works or the way is clear.
  // Returns true when it handled this tick (the robot is not driving its route normally).
  // cap: how far the Traffic Manager lets it drive this tick.
  _avoid(r, m, t, block, dt, cap = Infinity) {
    const a = m.avoid;
    if (!a) {
      if (!block || edgeDist(r, block.o) > Math.max(r.sensorRange, ROBOT_RADIUS + STOP_GAP) + 1e-6) return false;
      m.avoid = { phase: 'deciding', timer: DECIDE_SECONDS };
      this._obstacleEvent(r, t, 'stop', block.o, `Stopped: ${block.o.obstacleID} is in the way. Looking for a way around.`);
      return true;
    }
    if (!block) {
      m.avoid = null;
      this._obstacleEvent(r, t, 'resume', null, 'The way is clear again. Carrying on.');
      return false;
    }
    if (a.phase !== 'deciding') a.failedFor += dt;
    if (a.phase === 'approach') {
      if (block.at > 1e-6) { advance(r, m, dt, Math.min(block.at, cap), SLOW_FACTOR); this._live(r); return true; }
      Object.assign(a, { phase: 'waiting', timer: RETRY_SECONDS });
      this._obstacleEvent(r, t, 'wait', block.o, `Waiting at ${block.o.obstacleID}: ${a.why}.`);
      return true;
    }
    if (!a.stuck && a.failedFor >= STUCK_SECONDS - 1e-9) {
      a.stuck = true;
      this._obstacleEvent(r, t, 'stuck', block.o, `Stuck at ${block.o.obstacleID}: still no way past after ${STUCK_SECONDS} s (${a.why}).`);
      return true;
    }
    if ((a.timer -= dt) > 1e-9) return true;
    const rr = this._reroute(r, m, t);
    if (rr.ok) {
      m.avoid = null;
      this._obstacleEvent(r, t, 'replan', block.o, rr.how === 'detour'
        ? `Going round ${block.o.obstacleID} (up to ${rr.dev.toFixed(1)} m off the planned path).`
        : `Rerouted round ${block.o.obstacleID} on the nav graph (up to ${rr.dev.toFixed(1)} m off the planned path).`);
      return false;
    }
    if (a.phase === 'deciding') {
      Object.assign(a, { phase: 'approach', why: rr.why, failedFor: 0, stuck: false });
      this._obstacleEvent(r, t, 'approach', block.o, `No reroute: ${rr.why}. Moving up to ${block.o.obstacleID} to wait.`);
    } else Object.assign(a, { why: rr.why, timer: RETRY_SECONDS });
    return true;
  }
  // A way past the obstacle: a detour round it on free floor first, else another route on the nav graph.
  // When neither works, why says whether a way round is too far from the planned path or there is none at all.
  // extra: more things to keep clear of besides the obstacles (other robots, for a traffic deadlock).
  _reroute(r, m, t, extra = []) {
    if (!t) return { why: 'no task' };
    const d = this._detour(r, m, extra);
    if (d.ok) return d;
    const g = this._graphReroute(r, m, t, extra);
    return g.ok ? g : { why: `${d.why}, and ${g.why}` };
  }
  // Detour round the obstacle, off the nav graph (planDetour), rejoining the route once past it.
  _detour(r, m, extra = []) {
    const ahead = [];
    for (let i = r.routeIndex; i < r.route.length && r.route[i].level === r.level; i++) ahead.push(r.route[i]);
    const opts = { level: r.level, from: { x: r.position.x, y: r.position.y }, ahead,
      obstacles: [...this.obstacles.values(), ...extra].filter((o) => o.level === r.level), occupancy: this.nav.occupancy[r.level] || null,
      planned: m.planned, maxDetour: r.maxDetour, clearance: ROBOT_RADIUS };
    const d = planDetour(opts);
    if (d.why) {
      if (/destination/.test(d.why)) return d;
      // is there a way round further out? then it is only too far (wait), not impossible (stuck)
      const wider = planDetour({ ...opts, maxDetour: Math.max(FAR_DETOUR, r.maxDetour) });
      return wider.why ? { why: 'no space to go round it on the floor plan' }
        : { why: `the way round on the floor plan goes ${wider.dev.toFixed(2)} m from the planned path (limit ${r.maxDetour} m)` };
    }
    const z = this._z(r.level);
    r.route = [...d.points.map((p) => ({ level: r.level, nodeID: null, x: p.x, y: p.y, z, detour: true })), ...r.route.slice(r.routeIndex + d.rejoin)];
    r.routeIndex = 0;
    m.total = m.travelled + routeLength(r);
    return { ok: true, dev: d.dev, how: 'detour' };
  }
  // Another route on the nav graph to the task's destination that avoids every obstacle. It starts from a node the
  // robot can reach along its route: the next node if nothing is in the way, else the node it last passed. (A robot
  // not on the graph yet starts from the nearest node it can drive straight to.)
  _graphReroute(r, m, t, extra = []) {
    const here = { x: r.position.x, y: r.position.y }, blocked = this._blockedBy(extra), start = this._startNode(r, blocked);
    if (!start) return { why: 'no node it can reach' };
    const plan = planRoute(this.nav, { level: r.level, ...here }, { level: t.level, node: t.node }, t.lift, { start, blocked });
    if (plan.error) return { why: 'no other route on the nav graph' };
    const dev = Math.max(0, distToPath({ level: r.level, ...here }, m.planned), ...plan.route.map((p) => distToPath(p, m.planned)));
    if (dev > r.maxDetour) return { why: `the other route on the nav graph goes ${dev.toFixed(2)} m from the planned path (limit ${r.maxDetour} m)` };
    t.steps = plan.steps;
    this._setRoute(r, m, plan.route);
    return { ok: true, dev, how: 'graph' };
  }
  _setRoute(r, m, route) {
    r.route = route.map((q) => ({ ...q, z: this._z(q.level) })); r.routeIndex = 0;
    m.total = m.travelled + routeLength(r);
  }
  // link a-b on a level may not be used when it passes an obstacle (or one of extra)
  _blockedBy(extra = []) {
    const obs = [...this.obstacles.values(), ...extra];
    return (level, a, b) => obs.some((o) => o.level === level && blocks(o, a, b));
  }
  // The node a new route starts from: the next node on its route if nothing is in the way, else the node it last
  // passed; a robot not on the graph yet starts from the nearest node it can drive straight to.
  _startNode(r, blocked) {
    const here = { x: r.position.x, y: r.position.y }, g = this.nav.graph[r.level], onLevel = (p) => p.nodeID && p.level === r.level;
    const next = r.route.slice(r.routeIndex).find(onLevel), prev = r.route.slice(0, r.routeIndex).reverse().find(onLevel);
    const start = [next, prev].find((p) => p && !blocked(r.level, here, p));
    if (start) return start.nodeID;
    if (prev || !g) return null;
    const nodes = [...g.nodes].sort(([, a], [, b]) => Math.hypot(a.x - here.x, a.y - here.y) - Math.hypot(b.x - here.x, b.y - here.y));
    return (nodes.find(([, n]) => !blocked(r.level, here, n)) || [])[0] || null;
  }
  // Make way (a traffic deadlock with no way round, e.g. two robots that need each other's single-lane corridor):
  // drive to a nearby node (the shortest drive, within MAKE_WAY_METRES) that is off the paths of the robots it makes
  // way for and clear of every robot, wait there until they have passed, then plan on to its destination (_holding).
  _makeWay(r, m, t, others) {
    if (!t || !this.nav.graph[r.level]) return { why: 'nowhere to make way' };
    const here = { x: r.position.x, y: r.position.y }, mine = r.robotID;
    const robots = [...this.robots.values()].filter((o) => o.robotID !== mine && o.level === r.level);
    const paths = others.map((o) => this._pathAhead(o)).filter(Boolean);   // off the way of the robots it makes way for
    const blocked = this._blockedBy(robots.map((o) => ({ level: o.level, x: o.position.x, y: o.position.y, radius: SAFE_DIST - ROBOT_RADIUS })));
    const start = this._startNode(r, blocked);
    if (!start) return { why: 'nowhere to make way' };
    const spots = [...this.nav.graph[r.level].nodes].filter(([, n]) => paths.every((p) => nearest(n, p).d >= SAFE_DIST + 0.05)
      && robots.every((o) => Math.hypot(o.position.x - n.x, o.position.y - n.y) >= SAFE_DIST + 0.05))
      .filter(([, n]) => Math.hypot(n.x - here.x, n.y - here.y) <= MAKE_WAY_METRES)
      .sort(([, a], [, b]) => Math.hypot(a.x - here.x, a.y - here.y) - Math.hypot(b.x - here.x, b.y - here.y)).slice(0, 25);
    let best = null;
    for (const [node] of spots) {   // the one with the shortest drive there
      const plan = planRoute(this.nav, { level: r.level, ...here }, { level: r.level, node }, null, { start, blocked });
      if (plan.error) continue;
      const len = routeLength({ level: r.level, position: here, route: plan.route });
      if (len <= MAKE_WAY_METRES && (!best || len < best.len)) best = { node, plan, len };
    }
    if (!best) return { why: `nowhere within ${MAKE_WAY_METRES} m off the other robots' paths to make way` };
    this._setRoute(r, m, best.plan.route);
    m.evade = { node: best.node, for: others.map((o) => o.robotID), arrived: false, check: 1 };
    return { ok: true, node: best.node };
  }
  // the route a robot on a task still has to drive on its level, from where it is (null if none)
  _pathAhead(o) {
    const om = this.motion.get(o.robotID);
    if (!om || !om.prio || !o.activeTask) return null;
    const pts = [{ x: o.position.x, y: o.position.y }];
    for (let i = o.routeIndex; i < o.route.length && o.route[i].level === o.level; i++) pts.push(o.route[i]);
    return polyline(pts);
  }
  // Making way: once the robots it made way for no longer need to pass within SAFE_DIST of its way on to its
  // destination, it takes that way. It checks while it waits at the node it made way to (arrived), and about once a
  // second on its way there (they may have got past already).
  _holding(r, m, t, dt, arrived) {
    const ev = m.evade;
    if (arrived && !ev.arrived) {
      ev.arrived = true;
      this._obstacleEvent(r, t, 'wait', null, `Making way for ${this._names(ev.for)} at ${ev.node}: waiting for ${ev.for.length > 1 ? 'them' : 'it'} to pass.`, ev.for);
    }
    if (!t || (!arrived && (ev.check -= dt) > 1e-9)) return;
    ev.check = 1;
    const start = arrived ? ev.node : this._startNode(r, this._blockedBy());
    const plan = start && planRoute(this.nav, { level: r.level, x: r.position.x, y: r.position.y }, { level: t.level, node: t.node }, t.lift, { start });
    if (!plan || plan.error) return;
    const way = [{ x: r.position.x, y: r.position.y }];
    for (const p of plan.route) { if (p.level !== r.level) break; way.push(p); }
    const mine = polyline(way);
    const passed = ev.for.every((id) => { const o = this.robots.get(id), p = o && o.level === r.level && this._pathAhead(o); return !p || pathGap(p, mine) >= SAFE_DIST; });
    if (!passed) return;
    t.steps = plan.steps; this._setRoute(r, m, plan.route); m.evade = null; m.traffic = null;
    this._obstacleEvent(r, t, 'resume', null, `${this._names(ev.for)} ${ev.for.length > 1 ? 'have' : 'has'} passed. On the way to ${t.destination} again.`, ev.for);
  }
  // Applies an obstacle response to the robot and task and broadcasts it (collision:event, draft section 9).
  // response: stop | wait | replan (draft 10.3), stuck (still no way past after STUCK_SECONDS: robot Stuck, task Waiting),
  // resume (way clear again) or approach (moving up to the obstacle; not broadcast).
  // robots: for traffic (other robots in the way instead of obstacle o): type robot, and robotIDs lists them too.
  _obstacleEvent(r, t, response, o, note, robots = null) {
    const waiting = ['stop', 'wait', 'stuck'].includes(response);
    r.state = response === 'stuck' ? 'Stuck' : waiting ? 'Waiting' : 'Moving';
    r.blockedBy = ['replan', 'resume'].includes(response) ? [] : robots ? [...robots] : o ? [o.obstacleID] : [];
    if (t) { t.status = waiting ? 'Waiting' : 'Running'; t.note = note; this._pushTask(t); }
    this._pushRobot(r);
    if (response !== 'approach') this.emit('collision:event', { robotIDs: [r.robotID, ...(robots || [])], type: robots ? 'robot' : 'obstacle',
      obstacleIDs: !robots && o ? [o.obstacleID] : [], response, message: note, level: r.level, position: clone(r.position) });
  }
  // Obstacles (draft 10.3): circles on a level. For now robots only report them (robot:obstacles);
  // stop / wait / reroute when one is in the way comes later. They stay through sim:reset; a new nav graph clears them.
  obstacleList() { return [...this.obstacles.values()].map(clone); }
  _pushObstacles() { this.emit('obstacles:snapshot', this.obstacleList()); }
  addObstacle(p) {
    if (!this.nav) return fail(C.NO_NAV_GRAPH, 'Import a nav graph first.');
    if (typeof p.level !== 'string' || !this.nav.levels[p.level]) return fail(C.INVALID, 'level must be a level on the nav graph.');
    if (!isNum(p.x) || !isNum(p.y)) return fail(C.INVALID, 'x and y must be numbers.');
    if (p.radius !== undefined && !(isNum(p.radius) && p.radius > 0)) return fail(C.INVALID, 'radius must be a number above 0.');
    const id = 'obstacle_' + ++this.obstacleSeq;
    const o = { obstacleID: id, level: p.level, x: p.x, y: p.y, z: this._z(p.level), radius: p.radius ?? 0.5 };
    this.obstacles.set(id, o);
    this._pushObstacles();
    return ok(clone(o));
  }
  listObstacles() { return ok(this.obstacleList()); }
  removeObstacle(p) {
    if (typeof p.obstacleID !== 'string') return fail(C.INVALID, 'obstacleID is required.');
    if (!this.obstacles.delete(p.obstacleID)) return fail(C.OBSTACLE_NOT_FOUND, `No obstacle ${p.obstacleID}.`);
    this._pushObstacles();
    return ok();
  }
  // Obstacles on the part of the robot's route it has not driven yet (draft 8.3 robot:obstacles).
  robotObstacles(p) {
    const [r, e] = this._find(p); if (e) return e;
    const pts = [{ level: r.level, x: r.position.x, y: r.position.y }, ...r.route.slice(r.routeIndex)], hit = new Set();
    for (let i = 1; i < pts.length; i++) {
      const a = pts[i - 1], b = pts[i];
      if (a.level !== b.level) continue;
      for (const o of this.obstacles.values()) if (o.level === a.level && blocks(o, a, b)) hit.add(o.obstacleID);
    }
    return ok({ robotID: r.robotID, obstacles: [...hit].map((id) => clone(this.obstacles.get(id))) });
  }
  getTask(p) {
    const t = typeof p.taskID === 'string' && this.tasks.get(p.taskID);
    return t ? ok(clone(t)) : fail(C.TASK_NOT_FOUND, `No task ${p.taskID}.`);
  }
}
module.exports = { Engine };
