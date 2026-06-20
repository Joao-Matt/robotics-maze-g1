#!/usr/bin/env python3
"""Write per-run locomotion limits for Nav2 navigation."""

from __future__ import annotations

import argparse, hashlib, json
from pathlib import Path

import yaml


def main():
    p=argparse.ArgumentParser(); p.add_argument("--output-dir",type=Path,required=True); p.add_argument("--unitree-rl-gym-repo",type=Path,default=Path("third_party/unitree_rl_gym")); p.add_argument("--config",type=Path,default=Path("configs/default.yaml")); p.add_argument("--cache-root",type=Path,default=Path("runs/calibration")); args=p.parse_args()
    config=yaml.safe_load(args.config.read_text()) if args.config.is_file() else {}
    limits=config.get("nav2_navigation",{})
    policy=str(limits.get("locomotion_policy","unitree_rl_gym_native"))
    args.output_dir.mkdir(parents=True,exist_ok=True)
    if policy!="unitree_rl_gym_native":
        raise ValueError(f"Production supports only unitree_rl_gym_native, got {policy!r}.")
    policy_path=args.unitree_rl_gym_repo/"deploy"/"pre_train"/"g1"/"motion.pt"
    model_path=args.unitree_rl_gym_repo/"resources"/"robots"/"g1_description"/"g1_12dof.xml"
    deploy_config=args.unitree_rl_gym_repo/"deploy"/"deploy_mujoco"/"configs"/"g1.yaml"
    missing=[path for path in (policy_path,model_path,deploy_config) if not path.exists()]
    if missing:
        raise FileNotFoundError("Missing Unitree RL Gym assets: "+", ".join(str(path) for path in missing)+". Run `make fetch-unitree-rl-gym-policy`.")
    policy_hash=hashlib.sha256(policy_path.read_bytes()).hexdigest()
    model_hash=hashlib.sha256(model_path.read_bytes()+deploy_config.read_bytes()).hexdigest()
    digest=hashlib.sha256((policy+policy_hash+model_hash).encode()).hexdigest()[:16]
    target=args.output_dir/"locomotion_calibration.json"
    selected=float(limits.get("max_forward_mps",1.0))
    values={
        "schema_version":1,
        "policy":policy,
        "policy_hash":policy_hash,
        "model_config_hash":model_hash,
        "cache_key":digest,
        "selected_max_forward_mps":selected,
        "command_limits":{
            "min_forward_mps":float(limits.get("min_forward_mps",0.1)),
            "max_forward_mps":selected,
            "max_reverse_mps":float(limits.get("max_reverse_mps",-0.4)),
            "min_yaw_rate_radps":float(limits.get("min_yaw_rate_radps",0.0)),
            "max_yaw_rate_radps":float(limits.get("max_yaw_rate_radps",1.0)),
        },
        "status":"passed",
        "source":"native_policy_static_limits",
        "criteria":{"native_policy_usable_yaw_rate_radps":1.0,"native_policy_forward_range_mps":[0.1,1.0],"native_policy_reverse_mps":-0.4},
    }
    target.write_text(json.dumps(values,indent=2,sort_keys=True)+"\n")
    manifest=args.output_dir/"run_manifest.json"
    if manifest.is_file():
        manifest_values=json.loads(manifest.read_text()); manifest_values["policy_sha256"]=policy_hash; manifest_values["model_config_sha256"]=model_hash; manifest_values["locomotion_policy"]=policy
        manifest.write_text(json.dumps(manifest_values,indent=2,sort_keys=True)+"\n")
    print(target); return 0


if __name__=="__main__":raise SystemExit(main())
