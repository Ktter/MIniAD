#!/usr/bin/env python3
import json, os, argparse
def main():
    p=argparse.ArgumentParser();p.add_argument('--scenario',default='acc');p.add_argument('--output',default='results/latest');a=p.parse_args();os.makedirs(a.output,exist_ok=True);r={'scenario':a.scenario,'status':'baseline_ready','collision_count':0};open(os.path.join(a.output,'metrics.json'),'w').write(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
if __name__=='__main__':main()
