#!/usr/bin/env python3
"""Render per-run robot and Nav2 configs from locomotion calibration."""
import argparse,json
from pathlib import Path
import yaml

p=argparse.ArgumentParser(); p.add_argument("--config",type=Path,required=True); p.add_argument("--nav2-template",type=Path,required=True); p.add_argument("--calibration",type=Path,required=True); p.add_argument("--output-dir",type=Path,required=True); a=p.parse_args()
cal=json.loads(a.calibration.read_text()); calibrated_speed=float(cal["selected_max_forward_mps"])
config=yaml.safe_load(a.config.read_text()); limits=config.setdefault("nav2_navigation",{})
# Calibration establishes what the policy can do; the configured exploration
# limit establishes what it is allowed to do near walls.
speed=min(calibrated_speed,float(limits.get("max_forward_mps",.60)))
minimum_speed=min(float(limits.get("min_forward_mps",.40)),speed)
yaw_rate=float(limits.get("max_yaw_rate_radps",.40))
policy=str(limits.get("locomotion_policy","")).strip()
min_yaw_rate=float(limits.get("min_yaw_rate_radps",0.0))
if policy and policy != "unitree_rl_gym_native":
    raise ValueError(f"Production navigation supports only unitree_rl_gym_native, got {policy!r}.")
limits["max_forward_mps"]=speed
nav=yaml.safe_load(a.nav2_template.read_text()); follow=nav["controller_server"]["ros__parameters"]["FollowPath"]
# Keep DWB forward-only in normal exploration. The bridge still clamps any
# reverse commands defensively, but backing down narrow corridors is too risky
# for the native Unitree policy.
follow["min_vel_x"]=0.0; follow["min_speed_xy"]=0.0
follow["max_vel_x"]=speed; follow["max_speed_xy"]=speed; follow["max_vel_theta"]=yaw_rate
follow["min_speed_theta"]=min_yaw_rate
follow["acc_lim_x"]=float(limits.get("max_linear_accel_mps2",.20)); follow["acc_lim_theta"]=float(limits.get("max_yaw_accel_radps2",.40))
follow["decel_lim_x"]=-float(limits.get("max_linear_decel_mps2",1.20)); follow["decel_lim_theta"]=-float(limits.get("max_yaw_decel_radps2",1.20))
follow["BaseObstacle.scale"]=float(limits.get("obstacle_critic_scale",follow.get("BaseObstacle.scale",.20)))
follow["PathAlign.scale"]=float(limits.get("path_align_scale",follow.get("PathAlign.scale",18.0)))
follow["GoalAlign.scale"]=float(limits.get("goal_align_scale",follow.get("GoalAlign.scale",8.0)))
follow["PathDist.scale"]=float(limits.get("path_dist_scale",follow.get("PathDist.scale",24.0)))
follow["GoalDist.scale"]=float(limits.get("goal_dist_scale",follow.get("GoalDist.scale",10.0)))
costmap_resolution=float(limits.get("costmap_resolution_m",0.05))
robot_radius=float(limits.get("costmap_robot_radius_m",0.60))
inflation_radius=float(limits.get("inflation_radius_m",1.20))
inflation_scale=float(limits.get("inflation_cost_scaling",1.25))
for section in ("global_costmap","local_costmap"):
    params=nav[section][section]["ros__parameters"]
    params["resolution"]=costmap_resolution
    params["robot_radius"]=robot_radius
    inflation=params.get("inflation_layer",{})
    inflation["inflation_radius"]=inflation_radius
    inflation["cost_scaling_factor"]=inflation_scale
    params["inflation_layer"]=inflation
a.output_dir.mkdir(parents=True,exist_ok=True); (a.output_dir/"resolved_config.yaml").write_text(yaml.safe_dump(config,sort_keys=False)); (a.output_dir/"resolved_nav2_params.yaml").write_text(yaml.safe_dump(nav,sort_keys=False))
