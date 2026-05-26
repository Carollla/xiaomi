#!/usr/bin/env python3
"""
Generate a Gazebo SDF world for the 2026 Xiaomi Cup "Wild Treasure Hunt" track.

Dimensions follow the 2026 official problem statement:
- total field: 4 m wide x 16 m long
- main usable lane width: 1 m
- yellow border: 0.15 m, curved parts in the statement are 0.10 m
- flagstones: 0.30 m wide, 0.05 m high, 0.20 m gap
- balls: 0.20 m diameter
- red height-limit bars: 1.10 m x 0.10 m x 0.10 m, bottom 0.40 m above ground
- non-crossable obstacle: two 0.20 m cubes with 0.20 m gap
- bridge, bottle, soccer and goal are approximate simulation primitives
"""
from pathlib import Path


ROOT = Path(__file__).resolve().parent
WORLD = ROOT / "wild_treasure_2026.world"


def box_model(name, pose, size, color, static=True, collide=True):
    collision = f"""
      <collision name='{name}_collision'>
        <geometry><box><size>{size}</size></box></geometry>
      </collision>""" if collide else ""
    return f"""
    <model name='{name}'>
      <static>{str(static).lower()}</static>
      <pose>{pose}</pose>
      <link name='{name}_link'>{collision}
        <visual name='{name}_visual'>
          <geometry><box><size>{size}</size></box></geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
        </visual>
      </link>
    </model>"""


def sphere_model(name, pose, radius, color, mass=0.08):
    inertia = 0.4 * mass * radius * radius
    return f"""
    <model name='{name}'>
      <static>false</static>
      <pose>{pose}</pose>
      <link name='{name}_link'>
        <inertial>
          <mass>{mass}</mass>
          <inertia><ixx>{inertia}</ixx><iyy>{inertia}</iyy><izz>{inertia}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name='{name}_collision'>
          <geometry><sphere><radius>{radius}</radius></sphere></geometry>
          <surface><friction><ode><mu>0.4</mu><mu2>0.4</mu2></ode></friction></surface>
        </collision>
        <visual name='{name}_visual'>
          <geometry><sphere><radius>{radius}</radius></sphere></geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
        </visual>
      </link>
    </model>"""


def cylinder_model(name, pose, radius, length, color, mass=1.0, static=False):
    return f"""
    <model name='{name}'>
      <static>{str(static).lower()}</static>
      <pose>{pose}</pose>
      <link name='{name}_link'>
        <inertial><mass>{mass}</mass></inertial>
        <collision name='{name}_collision'><geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry></collision>
        <visual name='{name}_visual'>
          <geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
        </visual>
      </link>
    </model>"""


def main():
    models = []
    models.append(box_model("ground_2026", "0 8 0 0 0 0", "4.2 16.4 0.02", "0.20 0.24 0.22 1", collide=True))

    # Segment bands and yellow borders. Coordinates use y as the long race direction.
    segment_edges = [0.0, 2.4, 5.2, 8.0, 11.8, 14.2, 16.0]
    for i in range(6):
        y0, y1 = segment_edges[i], segment_edges[i + 1]
        cy = (y0 + y1) / 2.0
        length = y1 - y0
        models.append(box_model(f"seg{i+1}_lane", f"0 {cy} 0.011 0 0 0", f"1.0 {length} 0.004", "0.32 0.36 0.34 1", collide=False))
        models.append(box_model(f"seg{i+1}_left_yellow", f"-0.575 {cy} 0.025 0 0 0", f"0.15 {length} 0.02", "1 1 0 1", collide=False))
        models.append(box_model(f"seg{i+1}_right_yellow", f"0.575 {cy} 0.025 0 0 0", f"0.15 {length} 0.02", "1 1 0 1", collide=False))
        models.append(box_model(f"seg{i+1}_start_line", f"0 {y0} 0.03 0 0 0", "1.25 0.04 0.02", "1 1 1 1", collide=False))
    models.append(box_model("finish_circle_marker", "0 15.75 0.035 0 0 0", "0.9 0.9 0.012", "0.9 0.9 0.9 1", collide=False))

    # Segment 1: flagstones.
    y = 0.45
    idx = 0
    while y < 2.15:
        models.append(box_model(f"flagstone_{idx:02d}", f"0 {y} 0.035 0 0 0", "0.30 0.22 0.05", "0.55 0.55 0.50 1"))
        y += 0.50
        idx += 1

    # Segment 2: 4x4 ball grid, one orange per row/column; entrance fixed blue positions are kept blue.
    xs = [-0.36, -0.12, 0.12, 0.36]
    ys = [2.95, 3.45, 3.95, 4.45]
    orange_cols = [1, 3, 0, 2]
    fixed_blue = {(3, 3), (3, 2), (2, 3)}
    for r, by in enumerate(ys):
        for c, bx in enumerate(xs):
            orange = c == orange_cols[r] and (r, c) not in fixed_blue
            color = "1 0.45 0.05 1" if orange else "0.45 0.80 1.0 1"
            name = "orange_ball" if orange else "blue_ball"
            models.append(sphere_model(f"seg2_{name}_r{r+1}c{c+1}", f"{bx} {by} 0.30 0 0 0", 0.10, color, mass=0.06))

    # Segment 3: curve-straight-curve approximated by guide borders inside lane.
    for k, (x, y, yaw) in enumerate([(-0.22, 5.65, 0.45), (0.0, 6.45, 0.0), (0.22, 7.25, -0.45)]):
        models.append(box_model(f"curve_inner_yellow_{k}", f"{x} {y} 0.04 0 0 {yaw}", "0.10 1.05 0.025", "1 1 0 1", collide=False))

    # Segment 4: tunnel search with three vertical lanes, obstacles and targets.
    for x in [-0.36, 0.0, 0.36]:
        models.append(box_model(f"tunnel_lane_{int((x+0.5)*100)}", f"{x} 9.85 0.04 0 0 0", "0.04 3.2 0.02", "1 1 0 1", collide=False))
    models.append(box_model("height_bar_left_post", "-0.55 9.00 0.45 0 0 0", "0.06 0.06 0.90", "1 0 0 1"))
    models.append(box_model("height_bar_right_post", "0.55 9.00 0.45 0 0 0", "0.06 0.06 0.90", "1 0 0 1"))
    models.append(box_model("height_limit_bar", "0 9.00 0.45 0 0 0", "1.10 0.10 0.10", "1 0 0 1"))
    models.append(box_model("block_obstacle_a", "-0.22 10.10 0.10 0 0 0", "0.20 0.20 0.20", "0.15 0.15 0.15 1"))
    models.append(box_model("block_obstacle_b", "0.22 10.10 0.10 0 0 0", "0.20 0.20 0.20", "0.15 0.15 0.15 1"))
    models.append(cylinder_model("coke_bottle", "-0.34 10.90 0.30 0 0 0", 0.08, 0.60, "0.08 0.03 0.02 1", mass=0.6))
    models.append(sphere_model("seg4_orange_ball", "0.33 10.75 0.70 0 0 0", 0.10, "1 0.45 0.05 1", mass=0.06))
    models.append(sphere_model("seg4_soccer", "0.0 11.25 0.12 0 0 0", 0.10, "0.95 0.95 0.95 1", mass=0.12))
    models.append(box_model("seg4_goal", "0.0 11.55 0.15 0 0 0", "0.50 0.08 0.30", "0.1 0.1 0.1 0.35", collide=False))

    # Segment 5: narrow bridge.
    models.append(box_model("narrow_bridge", "0 13.0 0.14 0 0 0", "0.32 2.25 0.18", "0.45 0.32 0.18 1"))
    models.append(box_model("bridge_jump_line", "0 13.75 0.25 0 0 0", "0.42 0.04 0.02", "1 1 1 1", collide=False))

    # Segment 6: final soccer and endpoint.
    models.append(sphere_model("final_soccer", "0 14.55 0.12 0 0 0", 0.10, "0.95 0.95 0.95 1", mass=0.12))
    models.append(box_model("final_goal_exit", "0 15.05 0.16 0 0 0", "0.65 0.08 0.32", "0.1 0.1 0.1 0.35", collide=False))

    sdf = f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <world name='wild_treasure_2026'>
    <gravity>0 0 -9.8</gravity>
    <physics name='default_physics' type='ode'>
      <max_step_size>0.002</max_step_size>
      <real_time_update_rate>500</real_time_update_rate>
    </physics>
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
    {''.join(models)}
  </world>
</sdf>
"""
    WORLD.write_text(sdf, encoding="utf-8")
    print(WORLD)


if __name__ == "__main__":
    main()
