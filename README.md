# 버전 내역
- [v2.1 (2024.07)](https://github.com/MORAI-EDU/beginner_tutorials_answer/releases/tag/v2.1)
- [v2.0 (2024.02)](https://github.com/MORAI-EDU/beginner_tutorials_answer/releases/tag/v2.0_24.02)


---

*This code is just an example. If you want higher performance, please proceed with direct tuning and algorithm advancement.*

## Mini mission

The repository includes all three supplied competition data files:

- `ajou_mini_comp_init.json`
- `ajou_mini_comp_scenario.json`
- `path/ajou_mini_comp_global_path.txt`

Copy the init/scenario JSON files to the MORAI Launcher save directories,
load them in MORAI, and then run:

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch beginner_tutorials lattice_driving.launch
```

`lattice_driving.launch` uses `ajou_mini_comp_global_path.txt` by default.
Another path can be selected without changing the code:

```bash
roslaunch beginner_tutorials lattice_driving.launch path_file:=kcity.txt
```

Set every traffic-light ID and stop-line waypoint in
`config/mini_mission.yaml` after checking `/GetTrafficLightStatus` in MORAI.

Useful debug topics:

- `/mission_state`: `CRUISE`, `APPROACH_RED`, `STOP_RED`,
  `OBJECT_SLOW`, `EMERGENCY_STOP`, or `FINISH`
- `/mission_target_speed`: final mission speed limit in km/h
- `/lattice_path`: path selected for vehicle control
