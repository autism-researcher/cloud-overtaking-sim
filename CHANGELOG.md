# Changelog

## v1.1.0
**Collision-free vehicle insertion (correctness fix).**

- Fixed the vehicle-insertion logic in all simulators: new arrivals were
  previously placed at the origin (`s = 0`) regardless of occupancy, which under
  heavy demand injected a vehicle onto pavement already occupied by the previous
  one, producing non-physical same-direction overlaps (up to several meters).
- New behavior: an arrival enters at the origin or, once the queue reaches the
  origin, a minimum bumper-to-bumper gap `s0` behind the rear-most vehicle (the
  approach extends upstream as a physical feeder queue), at a speed no greater
  than that vehicle's. A hard non-overlap guard is retained as a redundant
  safeguard and is never triggered once safe insertion is in place.
- Verified: zero same-direction overlaps across all policies and demand levels;
  aggregate throughput and waiting reproduce the previously reported values to
  within seed variability (e.g., lambda=0.30: bridge 210/261, signal 844/140,
  cloud 562/162 veh/h / s).
- Repository completeness: added the experiment scripts that reproduce Tables
  3-5 (compare_baselines.py, comm_delay_experiment.py, comm_ext_experiment.py,
  comm_runner.py, regen_spacetime.py, reviewer_experiments.py) and removed a
  hardcoded absolute import path in comm_ext_experiment.py.

## v1.0.0
Initial release.
