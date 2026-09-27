# SwarmSearch

SwarmSearch is a software-only multi-UAV coordination and fault-recovery
system built using Python, MAVLink, and ArduPilot SITL.

Version 0.1 demonstrates:

- concurrent control of multiple simulated UAVs
- cooperative search-area partitioning
- lawnmower coverage-path generation
- altitude-based deconfliction
- telemetry-validated autonomous navigation
- deterministic UAV fault injection
- preservation and reassignment of unfinished coverage paths
- continued mission execution after vehicle loss

## Status

**v0.1 — active development**

This release establishes the simulation, coordination, mission-planning,
and fault-recovery foundation of SwarmSearch.

An interactive mission-control visualizer and expanded autonomous
coordination capabilities are under active development.