# Requirements: DC Motor Speed Control

Operating point: speed reference of 1 rad/s, actuator limit ±24 V.

| ID | Requirement | Verified by |
|---|---|---|
| REQ-01 | Settling time (2% band) < 2 s for a 1 rad/s speed step | `run_tests.m`, step response before t = 3 s |
| REQ-02 | Overshoot < 5% | `run_tests.m`, step response before t = 3 s |
| REQ-03 | Steady-state error < 1% | `run_tests.m`, value just before t = 3 s |
| REQ-04 | After a 0.01 N·m load torque step at t = 3 s, speed returns within 2% of the reference in < 1 s | `run_tests.m`, response after t = 3 s |
