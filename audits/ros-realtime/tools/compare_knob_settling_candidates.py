"""Read-only recorded-candidate attribution; no ROS nodes or hardware I/O."""
import copy
import json
import pickle
from pathlib import Path
import numpy as np
import torch
from catheter_control.orchestration.runtime import load_runtime

base = Path('/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions')
evidence = base/'planner_evidence_knob_settling_20261008T210905505162549Z/worker_2355452'
meta = pickle.loads((evidence/'metadata.pkl').read_bytes())
torch.set_num_threads(2)
runtime = load_runtime(meta['manifest'], device='cpu', options=meta['options']).runtime
records = {i:pickle.loads((evidence/f'plan_{i:06d}.pkl').read_bytes()) for i in [548,549,550]}
weights = torch.tensor([1.,1.,1.,6.], dtype=torch.float32)

def score(record, rates, gap_from=None):
    snapshot = copy.deepcopy(record['input_root'])
    if gap_from is not None:
        source = records[gap_from]['input_root'].belief['mean']
        mean = snapshot.belief['mean'].copy()
        eq = float(runtime.mechanics.stationary_lambda(torch.as_tensor(mean[:24]).to(runtime.mechanics.mode)))
        source_eq = float(runtime.mechanics.stationary_lambda(torch.as_tensor(source[:24]).to(runtime.mechanics.mode)))
        mean[30] = eq+float(source[30])-source_eq
        snapshot.belief['mean'] = mean
    state = runtime.import_planning_state(snapshot)
    prediction = runtime.predict_control_sequence_coarse(state, rates[:,:,:3], .2)
    tips = prediction.tip_base_m.cpu()
    target = torch.as_tensor(record['target'], dtype=tips.dtype)
    costs = (((tips-target)*1000).square().sum(-1)*weights).sum(-1).numpy()
    return costs, tips.numpy()

print('RECONSTRUCTION')
for i,r in records.items():
    ledger = r['candidate_ledgers'][-1]
    costs,tips = score(r,np.asarray(ledger['motor_rates']))
    print(json.dumps(dict(root=i, maximum_tracking_cost_error=float(np.max(np.abs(costs-ledger['tracking_cost']))), maximum_tip_error_mm=float(np.max(np.linalg.norm(tips-ledger['predicted_tip_sequence_m'],axis=-1))*1000))))

for pair in [(549,550),(548,549)]:
    pools = [records[i]['candidate_ledgers'][-1] for i in pair]
    rates = np.concatenate([x['motor_rates'] for x in pools])
    directions = np.sign(rates[:,0,:3]).astype(int)
    eligible = np.concatenate([x['eligible'] for x in pools])
    # Freeze nontracking components to their recorded pool values. This is an
    # objective attribution, not a complete arbiter/projection counterfactual.
    other = np.concatenate([x['total_cost']-x['tracking_cost'] for x in pools])
    print('MATCHED_POOL',pair)
    for root,gap in [(pair[0],None),(pair[1],None),(pair[1],pair[0])]:
        costs,tips = score(records[root],rates,gap)
        result = dict(root=root, gap_from=gap, directions={})
        for dr in [-1,0,1]:
            ids = np.flatnonzero(eligible & (directions[:,0]==dr))
            k = int(ids[np.argmin((costs+other)[ids])])
            result['directions'][dr] = dict(candidate=int(k), tracking=float(costs[k]), fixed_nontracking=float(other[k]), total=float(costs[k]+other[k]), terminal_error_mm=float(np.linalg.norm(tips[k,-1]-records[root]['target'])*1000), first_motor_rad_s=rates[k,0,:3].tolist())
        print(json.dumps(result))
