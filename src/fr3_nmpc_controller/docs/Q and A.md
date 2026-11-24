# Key Questions Before Integration

Before you start integrating the components, you should be able to answer these three questions.

## Q1: How can we prevent missed control deadlines when NMPC optimization is slow?

**Run NMPC optimization in the background while keeping `controller.update()` fast and nonblocking.**

For example, a 1 kHz controller has a 1 ms control period. If IPOPT takes 20 ms and runs directly inside `update()`, the callback cannot finish within that period.

In the proposed integration design:

- **SolverWorker** runs IPOPT in a background thread.
- **Coordinator** prepares requests and calls worker functions that may lock or allocate memory, outside the real-time control thread.
- **Two SPSC queues** provide bounded transfer of robot states to the coordinator and torque candidates back to the controller.
- **Controller** continues each cycle: read the robot state, publish a snapshot, check an available torque candidate, and apply an acceptable command or use the defined fallback control. It never waits for a solve to finish.

If a queue is full or empty, the controller follows its defined nonblocking policy instead of waiting. It checks a candidate's source-state age, reference version, validity, and torque limits before use. A previously accepted command may be reused only within its allowed validity window and control policy; otherwise, fallback control is required.

**Two timing requirements remain:** the controller must finish within its control period, and the solver must return useful results before they expire. Background optimization prevents solver execution from blocking `update()`; it does not by itself guarantee hard real-time behavior or make an outdated result usable.

## Q2: Why are two SPSC queues needed?

Robot states and torque commands travel in opposite directions. Each direction has its own producer and consumer, so this SPSC design uses a separate single-producer, single-consumer queue for each direction.

Think of them as **two one-way mailboxes**: one sends robot states to the solver; the other returns torque commands to the controller.

![Two SPSC queues: the controller sends robot states to the solver through the state queue, and the solver returns torque commands through the torque queue. Each queue has one producer and one consumer.](figures/two_spsc_queues.svg)

*Figure: The same two threads exchange different data in opposite directions. Here, q and dq are joint positions and velocities, and tau is the joint torque command.*

**Why not use one shared SPSC queue?** Both threads would write to and read from that queue, giving it two producers and two consumers. That breaks the SPSC contract. With two queues, each queue keeps exactly one writer and one reader.

## Q3: Why can't the controller simply execute every `TorqueCommand` in the queue?

An old command may have been computed from a past robot state. Before applying a solver result, the controller needs to check:

- Its timestamp.
- Its reference version.
- Its validity.
- Whether the torque satisfies the limits.

## Your Next Task

Run `test_realtime_exchange` first to verify that the data exchange module works correctly.

## Next Lesson

**Lesson 8 — Integrate RealtimeExchange into Fr3NmpcController**

After the test passes, the next lesson will cover these changes:

| Component | Changes |
| --- | --- |
| `controller.hpp` | Declare the exchange module and thread state. |
| `controller.cpp` | Implement state publishing, thread coordination, and result retrieval and validation. |
| `SolverWorker` | Keep its existing responsibility for background optimization. |
