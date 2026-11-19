const test = require('node:test'), assert = require('node:assert/strict');
const { Engine } = require('../src/engine');
const { parseNavGraph } = require('../src/navgraph');
const { go } = require('./helpers');   // task:assign + robot:navigate

// L1: A(0,0) - B(4,0) - C(4,3) "Lab". L2: E(0,0).
const nav = () => parseNavGraph({ graphs: {
  L1: { A: { pose: { x: 0, y: 0 }, branches: ['B'] }, B: { pose: { x: 4, y: 0 }, branches: ['C'] }, C: { pose: { x: 4, y: 3 }, locationName: 'Lab', branches: [] } },
  L2: { E: { pose: { x: 0, y: 0 }, branches: [] } } } });
const setup = () => { const e = new Engine(); e.setNav(nav()); return e; };

test('obstacles can be added, listed and removed, and every change is broadcast', () => {
  const e = setup(), seen = [];
  e.on('obstacles:snapshot', (l) => seen.push(l.length));
  const o = e.addObstacle({ level: 'L2', x: 1, y: 2 }).data;
  assert.deepEqual([o.obstacleID, o.level, o.z, o.radius], ['obstacle_1', 'L2', 1, 0.5]);
  e.addObstacle({ level: 'L1', x: 3, y: 3, radius: 2 });
  assert.equal(e.listObstacles().data.length, 2);
  assert.equal(e.removeObstacle({ obstacleID: 'obstacle_1' }).success, true);
  assert.equal(e.removeObstacle({ obstacleID: 'obstacle_1' }).failureCode, 2005);
  assert.deepEqual(seen, [1, 2, 1]);
});

test('bad obstacles are rejected', () => {
  const e = setup();
  assert.equal(e.addObstacle({ level: 'L9', x: 0, y: 0 }).failureCode, 1001);
  assert.equal(e.addObstacle({ level: 'L1', x: 'a', y: 0 }).failureCode, 1001);
  assert.equal(e.addObstacle({ level: 'L1', x: 0, y: 0, radius: 0 }).failureCode, 1001);
  assert.equal(new Engine().addObstacle({ level: 'L1', x: 0, y: 0 }).failureCode, 2004);
});

test('obstacles stay through sim:reset, follow level order, and go when a new map is loaded', () => {
  const e = setup();
  e.addObstacle({ level: 'L1', x: 0, y: 0 });
  e.reset();
  assert.equal(e.obstacleList().length, 1);
  e.setLevelOrder({ levels: ['L2', 'L1'] });
  assert.equal(e.obstacleList()[0].z, 1);
  e.setNav(nav());
  assert.equal(e.obstacleList().length, 0);
});

test('robot:obstacles reports obstacles on the rest of its route', () => {
  const e = setup();
  e.nav.occupancy.L1 = { free: () => false };                 // walls everywhere off the nav graph: no way round
  const id = e.createRobot({ speed: 1, startPosition: { level: 'L1', x: 0, y: 0 } }).data.robotID;
  e.addObstacle({ level: 'L1', x: 4, y: 1.5, radius: 0.3 });   // on B-C
  e.addObstacle({ level: 'L1', x: 2, y: 2, radius: 0.3 });     // off the route
  e.addObstacle({ level: 'L2', x: 4, y: 1.5 });                // other level
  go(e, { robotID: id, destination: 'Lab' });
  assert.deepEqual(e.robotObstacles({ robotID: id }).data.obstacles.map((o) => o.obstacleID), ['obstacle_1']);
  for (let i = 0; i < 150; i++) e.tick(0.1);                    // no way around at all on this map: Stuck at the obstacle
  assert.deepEqual([e.getRobot({ robotID: id }).data.state, e.getRobot({ robotID: id }).data.blockedBy], ['Stuck', ['obstacle_1']]);
  e.removeObstacle({ obstacleID: 'obstacle_1' });
  for (let i = 0; i < 80; i++) e.tick(0.1);
  assert.equal(e.getRobot({ robotID: id }).data.state, 'Idle');
  assert.deepEqual(e.robotObstacles({ robotID: id }).data.obstacles, []);
});
