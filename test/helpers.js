// Gives a robot a job and starts it, as a client does: task:assign creates the job, robot:navigate starts it.
// Returns robot:navigate's reply, or task:assign's if that one failed.
function go(e, { robotID, ...job }) {
  const a = e.assignTask(job);
  return a.success ? e.navigateRobot({ robotID, taskID: a.data.taskID }) : a;
}
module.exports = { go };
