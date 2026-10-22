# Tech debt

## process_node is unused and pending removal

`ugv1_controller/process_node.py` (OpenCV modes raw/gray/edges/overlay, publishes
`/video_processed`) is no longer used by anything. The web interface dropped the
processed feed when `/processed-feed` became `/drive`, and `start_ugv1.sh` has
never started it.

Left in the repo for now on purpose. To remove it:

- delete `ugv1_controller/process_node.py` and its `process_node` entry in `setup.py`
- remove the `process` node and its edge from the software diagram in
  `ugv1_controller/web/static/landing/app.js`
- remove its line from the README node list
- delete this entry

## Simulation is untested against a real Gazebo run

`sim/`, `launch/sim.launch.py`, `config/sim/` and `scripts/sim/` were written
without Gazebo or ROS available. Offline tests cover geometry, config
consistency, route logic and the scoring maths (`test/test_sim_assets.py`,
`test/test_vio_eval.py`); the SDF, bridge and launch file have only been
checked for well-formedness. See the checklist in `sim/README.md`.
