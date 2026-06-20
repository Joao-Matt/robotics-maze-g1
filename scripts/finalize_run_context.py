#!/usr/bin/env python3
"""Make an allocated run honest after normal failure or interruption."""
import argparse,json,sys
from datetime import datetime
from pathlib import Path

p=argparse.ArgumentParser(); p.add_argument("run_dir",type=Path); p.add_argument("--fallback-status",default="INTERRUPTED"); a=p.parse_args()
manifest=a.run_dir/"run_manifest.json"; summary=a.run_dir/"summary.json"
if not manifest.is_file():sys.exit(0)
m=json.loads(manifest.read_text())
if m.get("final_status")=="RUNNING":
    if summary.is_file():
        s=json.loads(summary.read_text()); status=s.get("final_status",a.fallback_status)
        if status=="RUNNING":status=a.fallback_status; s["final_status"]=status; s["status"]="failed"; summary.write_text(json.dumps(s,indent=2)+"\n")
    else:
        status=a.fallback_status; summary.write_text(json.dumps({"status":"failed","final_status":status,"run_directory":str(a.run_dir)},indent=2)+"\n")
    started=datetime.fromisoformat(m["started_at"]); m["final_status"]=status; m["ended_at"]=datetime.now().astimezone(started.tzinfo).isoformat(); m["summary"]=str(summary)
    manifest.write_text(json.dumps(m,indent=2,sort_keys=True)+"\n")
