"""Run the actual documented Docker entry; retain stdout, stderr and exit codes."""
import json
from pathlib import Path
import subprocess
root=Path(__file__).resolve().parents[2]
out=root/'logs/rl/final'
out.mkdir(parents=True, exist_ok=True)
cases=[
 ('q-default',0,['lesson09/q_learning_speed.py','--json','logs/rl/final/q-default.json']),
 ('q-seed11',0,['lesson09/q_learning_speed.py','--seed','11','--json','logs/rl/final/q-seed11.json']),
 ('q-untrained',0,['lesson09/q_learning_speed.py','--episodes','0']),
 ('q-invalid',2,['lesson09/q_learning_speed.py','--gamma','nan']),
 ('ppo-default',0,['lesson10/ppo_numbers.py','--json','logs/rl/final/ppo-default.json']),
 ('ppo-lambda0',0,['lesson10/ppo_numbers.py','--lam','0']),
 ('ppo-invalid',2,['lesson10/ppo_numbers.py','--clip','1']),
 ('contract-demo',0,['lesson12/offline_policy_contract.py','--demo','--output','logs/rl/final/contract-demo.json']),
 ('contract-shuffled',0,['lesson12/offline_policy_contract.py','--case','shuffled']),
 ('contract-stale',1,['lesson12/offline_policy_contract.py','--case','stale']),
]
rows=[]
for name,expected,args in cases:
 cmd=['./scripts/ros1_course.sh','offline',*args]
 r=subprocess.run(cmd,cwd=root,text=True,capture_output=True,timeout=60)
 (out/(name+'.log')).write_text(r.stdout+r.stderr)
 rows.append({'name':name,'command':cmd,'expected':expected,'actual':r.returncode,'no_traceback':'Traceback' not in r.stdout+r.stderr})
 print(name, 'expected',expected,'actual',r.returncode,flush=True)
(out/'commands.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2)+'\n')
assert all(x['expected']==x['actual'] and x['no_traceback'] for x in rows), 'Unexpected command result'
print('PASS',len(rows),'actual Docker CLI cases')
