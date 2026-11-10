# Replaced: see 2026-11-10-hold_stiffness.md

This file held the printed table of the 2026-11-09 run of `sim/hold_stiffness.py`. Its torque line read the
actuators' control range (unlimited), not the joint torque caps that MuJoCo applies (`actuatorfrcrange` in
`g1_12dof.xml`), so its torque columns are not kept. The 2026-11-10 run of the corrected script prints the caps and
the requested and applied torques, and its `caps` rows repeat this run's first-touch times and upright counts
exactly (the REPRO block at the end of [2026-11-10-hold_stiffness.md](2026-11-10-hold_stiffness.md)).
