# ADR 0002: Timestamp feedback and poll it outside the control loop

- Status: accepted
- Date: 2026-09-13

## Context

The upstream path requested one servo status packet from the 2 ms control task
only about every 100 ms. Because the five IDs were visited sequentially, a given
joint could be around 500 ms old by the time it was sampled again. That is too
stale for actuator characterization and can make a visible stick-slip event look
like a trajectory problem.

## Decision

Use a dedicated low-priority feedback poll task. It requests one ID every 20 ms,
so a complete five-joint cycle is approximately 100 ms (about 10 Hz per joint).
The receive path records a millisecond timestamp for every valid state packet.
Motion Lab telemetry reports both the per-joint sample timestamp and its age.
The round-robin advances only after the matching ID's valid response is parsed.
An unanswered request is retried once (two total attempts) with a 60 ms
timeout, then the joint is marked stale and the poller advances. This bounds a
single missing joint to approximately 140 ms including the scheduler wake-up,
so healthy IDs continue to receive samples inside the 250 ms freshness window.
An explicit selected-joint
high-rate mode (5--100 ms request period) deprioritizes the other IDs and
reports measured response rate and gaps.

## Consequences

- The 2 ms command path no longer schedules status reads or waits on their UART
  traffic.
- A characterization log can reject samples whose age is unexpectedly high.
- 10 Hz per joint is a conservative first rate that leaves margin on the shared
  half-duplex servo bus; increasing it is a measured follow-up, not an assumption.
- A missing response can lengthen one cycle, but bounded retries prevent it from
  blocking the rest of the bus indefinitely; `fb_stale[5]` makes the condition
  visible in logs.
- High-rate mode is deliberately opt-in. It changes only diagnostic poll/parser
  cadence, and its measured rate is reported rather than inferred from the
  configured request period.
- The existing raw speed/load encodings are preserved. This phase improves
  freshness and observability before interpreting or tuning them.
