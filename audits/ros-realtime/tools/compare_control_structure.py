"""Two final offline comparisons; no ROS nodes or commands."""
import argparse
import copy
import hashlib
import inspect
import json
import pickle
from pathlib import Path
import numpy as np
import torch
from cr_meta_lnn.deployment import load_runtime_bundle
from cr_meta_lnn.deployment.experimental.engagement_runtime import EngagementState
from cr_meta_lnn.deployment.experimental.joint_engagement_filter import JointEngagementFilter
from cr_meta_lnn.training.hybrid.profile_diagnostic import StaticMapInputPrior
from cr_meta_lnn.training.hybrid.control_residual_diagnostic import frozen_prefix_residual
from cr_meta_lnn.training.hybrid.control_response import task_response
from inspect_control_response import interpolate
from inspect_confirmation_profile import ENCODER_RADIANS_PER_COUNT


def score(rows, key):
    metrics = [task_response(r.get('error') or [0,0,0],r[key],r['measured'],r['weights']) for r in rows]
    resolved = [(r,m) for r,m in zip(rows,metrics) if m['measured_response_mm'] >= .3]
    referenced = [(r,m) for r,m in resolved if r.get('error') is not None]
    return dict(count=len(rows),resolved=len(resolved),
        wrong_halfspace=sum(m['response_cosine'] is not None and m['response_cosine'] <= 0 for _,m in resolved),
        median_cosine=float(np.median([m['response_cosine'] for _,m in resolved if m['response_cosine'] is not None])) if resolved else None,
        median_scale=float(np.median([m['response_scale_ratio'] for _,m in resolved])) if resolved else None,
        referenced=len(referenced),task_direction_disagreement=sum(
            (m['predicted_descent_projection_mm2'] > 0)!=(m['measured_descent_projection_mm2'] > 0) for _,m in referenced))


def run(args):
    torch.set_num_threads(2)
    saved=json.loads(Path(args.roots).read_text())
    if not saved['completed']: raise ValueError('completed full-root replay required')
    for path in (args.cache,args.manifest):
        item=dict(path=str(Path(path).resolve()),sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest())
        if item not in saved['identities']: raise ValueError('input identity changed: '+path)
    datasets=pickle.loads(Path(args.cache).read_bytes())
    runtime=load_runtime_bundle(args.manifest,device='cpu',dtype='float32').runtime
    report=dict(completed=False,scope='Achieved-input conditional forecasts and frozen past-prefix residual; no executed commands.',
        limitations=['Planner-only restoration: transmission recurrence is not reconstructed or used by engaged planner.',
            'Future achieved inputs are supplied to all forecasts; unexecuted candidate outcomes and closed-loop improvement are unknown.',
            'Residual is Cartesian response correction, not identified distal adaptation. No intercept; operator norm <=0.5.',
            'Earlier half training and later half frozen testing within each recording; overlapping windows are not independent trials.',
            'Training uses root ENGAGED and resolved completed labels; no future confirmation used to select held-out cases.'],sessions=[])
    paths=(args.roots,args.cache,args.manifest,__file__,inspect.getfile(StaticMapInputPrior),inspect.getfile(frozen_prefix_residual),inspect.getfile(type(runtime)))
    report['identities']={str(Path(p).resolve()):hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in paths}
    for source in saved['sessions']:
        data=next(d for d in datasets if d['name']==source['name'])
        manifest=Path(data['params']['model_manifest'])
        identity=dict(path=str(manifest.resolve()),sha256=hashlib.sha256(manifest.read_bytes()).hexdigest())
        if identity!=source['baseline_manifest']: raise ValueError('baseline manifest changed')
        report['identities'][identity['path']]=identity['sha256']
        base=load_runtime_bundle(data['params']['model_manifest'],device='cpu',dtype='float32',options={'marker_estimator':'ukf','adaptation_enabled':False}).runtime
        law=StaticMapInputPrior(base.history_model,base.motor_transmission,runtime.tendon_mm_per_shaft_rad,runtime.profile.input_knots)
        branch=copy.copy(runtime); branch.profile=law
        rows=[]
        for record in source['rows']:
            root=record['static_inference_root']; stamp=int(record['start_ns'])
            proximal=runtime.proximal.clone()
            proximal.jacobian.J_normalized=np.asarray(root['proximal_jacobian'])*proximal.jacobian.action_scale[None,:]/proximal.jacobian.state_scale[:,None]
            owner=JointEngagementFilter(runtime.mechanics,runtime.kinematics,root['strain'],root['interface_pose'],root['coordinate'],root['coordinate_std'],runtime.priors,config=runtime.config,profile=law,proximal=proximal)
            owner.mean=np.asarray(root['mean']).copy(); owner.covariance=np.asarray(root['covariance']).copy()
            state=EngagementState(runtime.runtime_key,owner)
            initial=runtime.endpoint_kinematics.endpoint(torch.as_tensor(root['interface_pose']).to(runtime.mechanics.mode),state.strain).detach().numpy()
            before=state.mean.copy()
            for case in record['cartesian']:
                endpoint=int(case['end_ns'])
                grid=np.array([stamp]+[int(t) for t in record['input_trace']['timestamps_ns'] if stamp<t<endpoint]+[endpoint],np.int64)
                counts=[interpolate(data['encoder'],int(t)) for t in grid]
                if any(c is None for c in counts): raise ValueError('unsupported encoder grid')
                raw=np.asarray(counts)[:,:3]*ENCODER_RADIANS_PER_COUNT
                dt=np.diff(grid)*1e-9; rates=np.diff(raw,axis=0)/dt[:,None]
                signs=[np.sign(rates[:,axis][np.abs(rates[:,axis])>1e-6]) for axis in range(3)]
                reversals=[int(np.count_nonzero(np.diff(s))) for s in signs]
                prediction=branch.predict_control_sequence(state,rates[None],dt).tip_base_m[0,-1].detach().numpy()
                passive=branch.predict_control_sequence(state,np.zeros_like(rates)[None],dt).tip_base_m[0,-1].detach().numpy()
                row=dict(start_ns=stamp,end_ns=endpoint,horizon_s=case['horizon_s'],phase=record['phase'],
                    reversal=any(reversals),shaft_reversal_counts=reversals,weights=case['weights'],error=case['target_error_mm'],measured=case['measured_delta_mm'],
                    mean=case['predictions']['consistent_static'],planner=(1000*(prediction-initial)).tolist(),
                    planner_passive=(1000*(passive-initial)).tolist(),
                    prior=case['predictions']['v171_engaged' if source['actual_baseline_compensation_enabled'] else 'v171_history'],
                    passive=case['passive']['v171_engaged' if source['actual_baseline_compensation_enabled'] else 'v171_history'])
                rows.append(row)
            np.testing.assert_array_equal(state.mean,before)
        session=dict(name=source['name'],planner_contract={},residual={})
        for horizon in (.2,.8):
            subset=[r for r in rows if r['horizon_s']==horizon]
            session['planner_contract'][str(horizon)]={key:score(subset,key) for key in ('mean','planner','prior')}
            session['planner_contract'][str(horizon)]['paired_difference_median_mm']=float(np.median([np.linalg.norm(np.array(r['planner'])-r['mean']) for r in subset]))
            for phase in sorted({r['phase'] for r in subset}):
                session['planner_contract'][str(horizon)][phase]={key:score([r for r in subset if r['phase']==phase],key) for key in ('mean','planner')}
            for reversal in (False,True):
                session['planner_contract'][str(horizon)]['with_reversal' if reversal else 'without_reversal']={key:score([r for r in subset if r['reversal']==reversal],key) for key in ('mean','planner')}
            fitted=frozen_prefix_residual(subset)
            fitted['summary']={key:score(fitted['rows'],key) for key in ('prior','corrected')}
            session['residual'][str(horizon)]=fitted
        session['rows']=rows; report['sessions'].append(session)
    report['completed']=True
    out=Path(args.output); out.mkdir(parents=True,exist_ok=False)
    (out/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print('Report:',out/'report.json')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('roots','cache','manifest','output'): p.add_argument('--'+key,required=True)
    run(p.parse_args())
