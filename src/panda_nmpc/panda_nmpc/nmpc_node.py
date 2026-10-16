"""Simulation-only nonlinear MPC. No hardware/Gazebo command publishers.

Seven independent nonlinear joints are a teaching plant, not Panda dynamics.
Run --demo for a standalone comparison, otherwise run as a ROS 2 node.
"""
import argparse
import time
from pathlib import Path

import casadi as ca
import numpy as np


class JointSpaceNMPC:
    def __init__(self, n=7, horizon=12, dt=0.05):
        if n < 1 or horizon < 2 or not np.isfinite(dt) or dt <= 0:
            raise ValueError("Invalid dimensions or timestep")
        self.n, self.horizon, self.dt = n, horizon, dt
        # Synthetic parameters: NOT identified Panda/FR3 inertias or limits.
        self.inertia = np.linspace(1.5, 0.5, n)
        self.gravity = np.linspace(6.0, 1.0, n)
        self.damping = 0.3
        self.friction = 0.1
        self.q_limit, self.v_limit, self.tau_limit = 2.5, 1.0, 12.0
        x, u = ca.SX.sym('x', 2*n), ca.SX.sym('tau', n)
        q, v = x[:n], x[n:]
        # I_i*qdd_i + b*qd_i + g_i*sin(q_i) + f*tanh(qd_i/.05) = tau_i
        a = (u - ca.DM(self.gravity)*ca.sin(q) - self.damping*v
             - self.friction*ca.tanh(v/0.05)) / ca.DM(self.inertia)
        rhs = ca.Function('rhs', [x, u], [ca.vertcat(v, a)])
        k1 = rhs(x, u)
        k2 = rhs(x + dt*k1/2, u)
        k3 = rhs(x + dt*k2/2, u)
        k4 = rhs(x + dt*k3, u)
        self.step = ca.Function('step', [x, u], [x + dt*(k1+2*k2+2*k3+k4)/6])
        opt = ca.Opti()
        self.X = opt.variable(2*n, horizon+1)
        self.U = opt.variable(n, horizon)
        self.x0 = opt.parameter(2*n)
        self.ref = opt.parameter(2*n, horizon+1)
        self.u_last = opt.parameter(n)
        opt.subject_to(self.X[:, 0] == self.x0)
        cost = 0
        for k in range(horizon):
            opt.subject_to(self.X[:, k+1] == self.step(self.X[:, k], self.U[:, k]))
            e = self.X[:, k+1] - self.ref[:, k+1]
            # Penalize effort relative to approximate equilibrium gravity torque.
            equilibrium = ca.DM(self.gravity)*ca.sin(self.ref[:n, k+1])
            du = self.U[:, k] - (self.u_last if k == 0 else self.U[:, k-1])
            cost += 100*ca.sumsqr(e[:n]) + 5*ca.sumsqr(e[n:])
            cost += .005*ca.sumsqr(self.U[:, k]-equilibrium) + .01*ca.sumsqr(du)
        cost += 200*ca.sumsqr(self.X[:n, -1]-self.ref[:n, -1])
        opt.subject_to(opt.bounded(-self.q_limit, self.X[:n, :], self.q_limit))
        opt.subject_to(opt.bounded(-self.v_limit, self.X[n:, :], self.v_limit))
        opt.subject_to(opt.bounded(-self.tau_limit, self.U, self.tau_limit))
        opt.minimize(cost)
        opt.solver('ipopt', {'print_time': False},
                   {'print_level': 0, 'sb': 'yes', 'max_iter': 100, 'tol': 1e-6})
        self.opt = opt
        self.reset()

    def reset(self):
        self.previous = None
        self.last_u = np.zeros(self.n)

    def solve(self, state, reference):
        state, reference = np.asarray(state), np.asarray(reference)
        if state.shape != (2*self.n,) or reference.shape != (2*self.n, self.horizon+1):
            raise ValueError('State or reference shape mismatch')
        if not np.isfinite(state).all() or not np.isfinite(reference).all():
            raise ValueError('Non-finite state/reference')
        self.opt.set_value(self.x0, state)
        self.opt.set_value(self.ref, reference)
        self.opt.set_value(self.u_last, self.last_u)
        if self.previous is None:
            self.opt.set_initial(self.X, np.tile(state[:, None], (1, self.horizon+1)))
            self.opt.set_initial(self.U, np.tile((self.gravity*np.sin(state[:self.n]))[:, None], (1, self.horizon)))
        else:
            xx, uu = self.previous
            self.opt.set_initial(self.X, np.column_stack((xx[:, 1:], xx[:, -1])))
            self.opt.set_initial(self.U, np.column_stack((uu[:, 1:], uu[:, -1])))
        started = time.perf_counter()
        result = self.opt.solve()  # Failure propagates; no unchecked debug iterate.
        ms = 1000*(time.perf_counter()-started)
        xx, uu = np.asarray(result.value(self.X)), np.asarray(result.value(self.U))
        if not np.isfinite(xx).all() or not np.isfinite(uu).all():
            raise RuntimeError('Non-finite optimizer result')
        if np.max(np.abs(xx[:self.n])) > self.q_limit+1e-5 or np.max(np.abs(xx[self.n:])) > self.v_limit+1e-5 or np.max(np.abs(uu)) > self.tau_limit+1e-5:
            raise RuntimeError('Optimizer constraint violation')
        self.previous = xx, uu
        self.last_u = uu[:, 0].copy()
        return self.last_u.copy(), ms


class TimedReference:
    """Continuous piecewise linear positions, with matching segment velocities."""
    def __init__(self, times, positions):
        self.t, self.q = np.asarray(times, dtype=float), np.asarray(positions, dtype=float)
        if self.t.ndim != 1 or len(self.t) < 2 or self.q.shape != (len(self.t), 7):
            raise ValueError('Expected >=2 samples with seven positions')
        if not np.isfinite(self.t).all() or not np.isfinite(self.q).all() or self.t[0] != 0 or np.any(np.diff(self.t) <= 0):
            raise ValueError('Finite samples with strictly increasing times starting at zero required')
        self.duration = self.t[-1]

    def sample(self, t):
        t = max(0.0, min(float(t), self.duration))
        i = min(int(np.searchsorted(self.t, t, side='right'))-1, len(self.t)-2)
        v = (self.q[i+1]-self.q[i])/(self.t[i+1]-self.t[i])
        q = self.q[i] + v*(t-self.t[i])
        if t >= self.duration:
            v = np.zeros(7)
        return np.r_[q, v]

    def horizon(self, t, controller):
        return np.column_stack([self.sample(t+k*controller.dt) for k in range(controller.horizon+1)])


def demo(out):
    """Closed-loop comparison on the SAME synthetic nonlinear plant."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    controller = JointSpaceNMPC()
    ts = np.linspace(0, 4, 401)
    s = ts/4
    blend = 10*s**3-15*s**4+6*s**5
    reference = TimedReference(ts, blend[:, None]*np.linspace(.4, -.3, 7))
    runs = {}
    for mode in ('PD + gravity', 'NMPC'):
        state, rows = np.zeros(14), []
        controller.reset()
        for t in np.arange(0, reference.duration, controller.dt):
            target = reference.sample(t)
            if mode == 'NMPC':
                u, ms = controller.solve(state, reference.horizon(t, controller))
            else:
                u = np.clip(40*(target[:7]-state[:7])+10*(target[7:]-state[7:])
                            +controller.gravity*np.sin(state[:7]), -controller.tau_limit, controller.tau_limit)
                ms = 0.0
            rows.append(np.r_[t, target[:7], state[:7], u, ms])
            state = np.asarray(controller.step(state, u)).reshape(-1)
        runs[mode] = np.asarray(rows)
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(3, 1, figsize=(9, 9), constrained_layout=True)
    first = next(iter(runs.values()))
    axes[0].plot(first[:, 0], first[:, 1], 'k--', label='Reference joint 1')
    for label, rows in runs.items():
        np.savetxt(out/('nmpc.csv' if label == 'NMPC' else 'baseline.csv'), rows, delimiter=',',
                   header=','.join(['time_s']+[f'q_ref_{i}' for i in range(1,8)]+[f'q_measured_{i}' for i in range(1,8)]+[f'tau_{i}' for i in range(1,8)]+['solve_ms']), comments='')
        errors = rows[:, 8:15]-rows[:, 1:8]
        axes[0].plot(rows[:, 0], rows[:, 8], label=label)
        axes[1].plot(rows[:, 0], np.linalg.norm(errors, axis=1), label=label)
        print(f'{label}: position RMSE={np.sqrt(np.mean(errors**2)):.6f} rad')
        if label == 'NMPC':
            axes[2].plot(rows[:, 0], rows[:, -1], label='NMPC solve time')
            print(f'Solve max={rows[:,-1].max():.1f} ms; overruns={sum(rows[:,-1]>controller.dt*1000)}')
    axes[2].axhline(controller.dt*1000, color='r', linestyle='--', label='Requested period')
    for ax, ylabel in zip(axes, ['Joint 1 [rad]', 'Joint error norm [rad]', 'Solve time [ms]']):
        ax.set(xlabel='Simulation time [s]', ylabel=ylabel)
        ax.legend()
        ax.grid(alpha=.3)
    fig.suptitle('Synthetic nonlinear simulation: PD + gravity versus NMPC\nNot Panda/FR3 hardware or Gazebo results')
    fig.savefig(out/'comparison.png', dpi=160)
    plt.close(fig)


def main(args=None):
    # Imports delayed so the offline comparison needs no ROS installation.
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64
    from trajectory_msgs.msg import JointTrajectory

    class SimulationNode(Node):
        def __init__(self):
            super().__init__('panda_nmpc')
            self.declare_parameter('prediction_dt', .05)
            self.declare_parameter('horizon', 12)
            self.declare_parameter('reference_topic', '/panda_nmpc/reference_trajectory')
            self.declare_parameter('joint_names', [f'panda_joint{i}' for i in range(1,8)])
            self.names = list(self.get_parameter('joint_names').value)
            if len(self.names) != 7 or len(set(self.names)) != 7:
                raise ValueError('Seven unique joint names required')
            self.mpc = JointSpaceNMPC(horizon=int(self.get_parameter('horizon').value), dt=float(self.get_parameter('prediction_dt').value))
            self.reference = None
            self.state = np.zeros(14)
            self.t = 0.0
            self.pub_state = self.create_publisher(JointState, '~/simulated_joint_states', 10)
            self.pub_ref = self.create_publisher(JointState, '~/reference_sample', 10)
            self.pub_error = self.create_publisher(JointState, '~/tracking_error', 10)
            self.pub_timing = self.create_publisher(Float64, '~/solve_time_ms', 10)
            self.sub = self.create_subscription(JointTrajectory, self.get_parameter('reference_topic').value, self.receive, 10)
            self.timer = self.create_timer(self.mpc.dt, self.tick)
            self.get_logger().info('INTERNAL SYNTHETIC SIMULATION ONLY; waiting for JointTrajectory. No Gazebo/hardware commands.')

        def receive(self, msg):
            try:
                if len(set(msg.joint_names)) != len(msg.joint_names):
                    raise ValueError('Duplicate joint names')
                indices = [list(msg.joint_names).index(name) for name in self.names]
                times = [p.time_from_start.sec + 1e-9*p.time_from_start.nanosec for p in msg.points]
                ref = TimedReference(times, [[p.positions[i] for i in indices] for p in msg.points])
                if np.max(np.abs(ref.q)) > self.mpc.q_limit:
                    raise ValueError('Reference exceeds synthetic position limits')
                if np.max(np.abs(np.diff(ref.q, axis=0)/np.diff(ref.t)[:,None])) > self.mpc.v_limit:
                    raise ValueError('Reference exceeds synthetic velocity limits')
                self.reference = ref
                # Each reference starts a new independent simulation at rest.
                self.state = np.r_[ref.q[0], np.zeros(7)]
                self.t = 0.0
                self.mpc.reset()
                self.get_logger().info('Reset internal plant; header start stamp ignored. Using internal elapsed simulation time.')
            except (ValueError, IndexError) as exc:
                self.get_logger().warning(f'Rejected reference: {exc}')

        def tick(self):
            if self.reference is None:
                return
            target = self.reference.sample(self.t)
            try:
                u, ms = self.mpc.solve(self.state, self.reference.horizon(self.t, self.mpc))
                next_state = np.asarray(self.mpc.step(self.state, u)).reshape(-1)
                if not np.isfinite(next_state).all():
                    raise RuntimeError('Non-finite simulated state')
            except (ValueError, RuntimeError) as exc:
                self.get_logger().error(f'Stopping internal simulation: {exc}')
                self.reference = None
                return
            stamp = self.get_clock().now().to_msg()
            for pub, q, v, effort in ((self.pub_state, self.state[:7], self.state[7:], u),
                                      (self.pub_ref, target[:7], target[7:], []),
                                      (self.pub_error, target[:7]-self.state[:7], [], [])):
                msg = JointState()
                msg.header.stamp, msg.name = stamp, self.names
                msg.position, msg.velocity, msg.effort = list(map(float,q)), list(map(float,v)), list(map(float,effort))
                pub.publish(msg)
            timing = Float64()
            timing.data = ms
            self.pub_timing.publish(timing)
            if ms > 1000*self.mpc.dt:
                self.get_logger().warning(f'Solve {ms:.1f} ms exceeded requested period')
            self.state = next_state
            self.t += self.mpc.dt
            if self.t >= self.reference.duration:
                self.reference = None
                self.get_logger().info('Internal simulation finished')

    rclpy.init(args=args)
    node = None
    try:
        node = SimulationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    if '--demo' in __import__('sys').argv:
        parser = argparse.ArgumentParser()
        parser.add_argument('--demo', action='store_true')
        parser.add_argument('--out', default='nmpc_comparison')
        demo(parser.parse_args().out)
    else:
        main()
