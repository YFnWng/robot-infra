"""G4 bundle preparation and recorded replay CLI; no hardware startup."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import numpy as np

from .engagement_qualification import (
    EngagementQualification, prepare_events, offline_replay, finite_json)


def parser():
    result=argparse.ArgumentParser(description=__doc__)
    modes=result.add_subparsers(dest='mode',required=True)
    prepare=modes.add_parser('prepare',help='Hash-bind artifacts and export native sensor events from a finalized session')
    prepare.add_argument('--session',required=True)
    prepare.add_argument('--prior',required=True)
    prepare.add_argument('--profile',required=True)
    prepare.add_argument('--interface-checkpoint',required=True)
    prepare.add_argument('--geometry-json',help='Reviewed geometry override; default is frozen checkpoint and baseline marker fractions')
    prepare.add_argument('--prefix',default='')
    prepare.add_argument('--output',required=True,help='New external G4 directory')
    replay=modes.add_parser('replay')
    add_runtime_arguments(replay)
    replay.add_argument('--events',required=True)
    replay.add_argument('--duration-s',type=float,default=60.)
    replay.add_argument('--progress-interval-s',type=float,default=5.,
        help='Progress logging interval in wall seconds (default: 5)')
    replay.add_argument('--outdir',required=True)
    compare=modes.add_parser('compare-anchors',help='Offline identical-candidate posterior/equilibrium input comparison')
    add_runtime_arguments(compare)
    compare.add_argument('--session',required=True)
    compare.add_argument('--trial',type=int,default=2)
    compare.add_argument('--offsets-s',type=float,nargs='+',default=[1.5,4.15,7.5,10.1,12.9,15.3])
    compare.add_argument('--outdir',required=True)
    banks=modes.add_parser('compare-banks',help='Rescore trusted saved anchor roots with sampled/constant/union proposals')
    add_runtime_arguments(banks)
    banks.add_argument('--anchor-report',required=True)
    banks.add_argument('--seeds',type=int,nargs='+',default=[17,29,43])
    banks.add_argument('--outdir',required=True)
    return result


def add_runtime_arguments(result):
    result.add_argument('--manifest',required=True)
    result.add_argument('--device',choices=('cpu','cuda'),default='cpu')
    result.add_argument('--planner-process',action='store_true',
        help='Use the production bounded planner worker; estimator stays on CPU')
    result.add_argument('--limits',default='/home/chen-lab/Yifan/robot-infra/src/control_interface/config/catheter_limits.yaml')
    result.add_argument('--catheter',default='imricor_test')
    result.add_argument('--samples',type=int,default=512)
    result.add_argument('--plain',action='store_true')
    result.add_argument('--marker-std-mm',type=float,default=.5)


def make_engine(args):
    if getattr(args,'planner_process',False):
        import torch
        torch.set_num_threads(4)
    return EngagementQualification(args.manifest,args.limits,catheter=args.catheter,
        device=args.device,samples=args.samples,grouped=not args.plain,
        marker_std_m=args.marker_std_mm*.001,
        planner_process=getattr(args,'planner_process',False))


def prepare(args):
    from cr_meta_lnn.deployment.experimental.engagement_bundle import make_manifest
    from cr_meta_lnn.deployment.experimental.unified_engagement_checkpoint import load_unified_engagement
    from cr_meta_lnn.deployment.v171_streaming_runtime import DEFAULT_MARKER_FRACTIONS
    from catheter_control.safety.hardware_contract import MOTOR_AXIS_UNITS_PER_ENCODER_COUNT
    out=Path(args.output)
    if out.exists():raise ValueError('use a new output directory; artifacts are never overwritten')
    model=load_unified_engagement(args.prior,device='cpu').mechanics
    lengths=model.kinematics.section_lengths.cpu().numpy()
    rigid=float(model.kinematics.rigid_tip_length)
    geometry=dict(section_lengths_m=lengths.tolist(),rigid_tip_length_m=rigid,
        marker_material_coordinates_m=((lengths.sum()+rigid)*np.asarray(DEFAULT_MARKER_FRACTIONS)).tolist())
    if args.geometry_json:geometry=json.loads(Path(args.geometry_json).read_text())
    # Same physical encoder zero and motor-axis bending coordinate used by
    # identification/phase2e training; no hardware calibration write exists.
    calibration=dict(bending_axis=2,mm_per_count=float(MOTOR_AXIS_UNITS_PER_ENCODER_COUNT[2]),offset_mm=0.)
    raw=make_manifest(args.prior,args.profile,geometry,calibration,
        interface_checkpoint=args.interface_checkpoint)
    out.mkdir(parents=True)
    (out/'manifest.json').write_text(json.dumps(raw,indent=2)+'\n')
    engine=EngagementQualification(out/'manifest.json',
        '/home/chen-lab/Yifan/robot-infra/src/control_interface/config/catheter_limits.yaml',samples=512)
    prepare_events(args.session,out/'sensor_events.json',args.prefix)
    (out/'preparation.json').write_text(json.dumps(dict(created_utc=datetime.now(timezone.utc).isoformat(),
        session=str(Path(args.session).resolve()),runtime_identity=engine.bundle.identity.manifest_sha256,
        geometry_source=args.geometry_json or 'frozen checkpoint + DEFAULT_MARKER_FRACTIONS',
        calibration_source='HardwareContract.MOTOR_AXIS_UNITS_PER_ENCODER_COUNT; read-only zero',
        marker_uncertainty='explicit trial std, not calibrated from confidence',command_capable=False),indent=2)+'\n')
    return out/'manifest.json'


def main(argv=None):
    args=parser().parse_args(argv)
    if args.mode=='prepare':
        print(json.dumps({'manifest':str(prepare(args))}));return
    if args.mode=='compare-anchors':
        print(json.dumps({'report':str(compare_anchors(args))}));return
    if args.mode=='compare-banks':
        print(json.dumps({'report':str(compare_banks(args))}));return
    if not np.isfinite(args.duration_s) or args.duration_s<=0:raise ValueError('positive replay duration required')
    if Path(args.outdir).exists():raise ValueError('use a new report directory')
    if not np.isfinite(args.progress_interval_s) or args.progress_interval_s<=0:
        raise ValueError('positive progress interval required')
    print(json.dumps(dict(stage='loading_runtime',device=args.device,samples=args.samples,
        requested_recorded_s=args.duration_s)),flush=True)
    engine=make_engine(args)
    try:
        plans=offline_replay(engine,args.events,maximum_seconds=args.duration_s,
            progress=lambda row:print(json.dumps(finite_json(row)),flush=True),
            progress_interval_s=args.progress_interval_s)
        print(json.dumps({'report':str(engine.report(args.outdir,scope='recorded_input_compute_replay_not_ROS_scheduling',plans=plans))}))
    finally:
        engine.close()


def compare_banks(args):
    """Trusted local roots only; recorded status never creates command entities."""
    import hashlib
    import bisect
    import torch
    from dataclasses import replace
    from catheter_control.planning.mppi import CatheterMppi, MppiConfig
    from catheter_control.transmission.backlash import BacklashConfig, BacklashFeedforwardCompensator
    from .compute_profile import read_events
    from .engagement_qualification import compare_input_anchors
    from .replay_takeup_startup import status_array, transmission_from_status
    source=Path(args.anchor_report); saved=json.loads(source.read_text());out=Path(args.outdir)
    if out.exists():raise ValueError('use a new report directory')
    if args.planner_process:raise ValueError('bank attribution requires local diagnostic solves')
    if not saved.get('completed') or not saved.get('rows'):raise ValueError('completed anchor report required')
    if not 1<=len(args.seeds)<=5 or len(set(args.seeds))!=len(args.seeds) or any(s<0 for s in args.seeds):
        raise ValueError('require 1..5 unique nonnegative seeds')
    torch.set_num_threads(4)
    engine=make_engine(args)
    try:
        if engine.bundle.identity.manifest_sha256!=saved['runtime_manifest_sha256']:
            raise ValueError('runtime manifest differs from saved roots')
        session=Path(saved['session']);record_path=session/'controller_manifest.json'
        if hashlib.sha256(record_path.read_bytes()).hexdigest()!=saved['controller_record_sha256']:
            raise ValueError('controller record changed')
        params=json.loads(record_path.read_text())['controller_parameters']
        engine.config=MppiConfig(**saved['configuration'])
        maximum=np.asarray(params['controller_velocity_max'],float)
        if np.any(maximum<0) or np.any(maximum>engine.contract.velocity_max):
            raise ValueError('recorded velocities exceed contract')
        engine.contract=replace(engine.contract,velocity_max=maximum,
            velocity_min=np.minimum(engine.contract.velocity_min,maximum))
        rates=BacklashFeedforwardCompensator(BacklashConfig(
            width_rad=tuple(params['backlash_width_rad']),
            takeup_velocity=tuple(params['backlash_takeup_velocity']))).motor_radians_per_second(engine.contract)
        status_events=[e for e in read_events(session,include_status=True,require_plan=False)
                       if e.kind=='status']
        times=[e.receipt_ns for e in status_events]
        bank_path=source.parent/'candidate_bank.npy'
        if hashlib.sha256(bank_path.read_bytes()).hexdigest()!=saved['candidate_bank_sha256']:
            raise ValueError('constant bank changed')
        constant=np.load(bank_path,allow_pickle=False)
        out.mkdir(parents=True);rows=[]
        for index,case in enumerate(saved['rows']):
            root_path=source.parent/case['root_file']
            if hashlib.sha256(root_path.read_bytes()).hexdigest()!=case['root_sha256']:
                raise ValueError('saved root changed')
            # Hash-bound locally generated snapshots; pickle is never safe for untrusted inputs.
            packed=torch.load(root_path,map_location='cpu',weights_only=False)
            state=engine.runtime.import_planning_state(packed['roots']['posterior'])
            k=bisect.bisect_right(times,case['requested_ns'])-1
            if k<0:raise ValueError('missing prior recorded arbitration status')
            event=status_events[k];age=(case['requested_ns']-event.receipt_ns)/1e9
            if age>.2:raise ValueError('recorded arbitration status older than 200ms')
            d=event.values
            inputs=dict(previous_logical_velocity=status_array(d,'command',(6,)),
                takeup_motor_radians_per_second=rates,
                blocked_motor_direction=status_array(d,'planner_blocked_motor_direction',signs=True),
                direction_lease=status_array(d,'reversal_lease_direction',signs=True),
                approved_reversal_direction=status_array(d,'reversal_approved_direction',signs=True))
            snapshot=(state,packed['position'],case['position_ns'],packed['target'],transmission_from_status(d))
            for seed in args.seeds:
                engine.config=replace(engine.config,seed=seed)
                sampled=CatheterMppi(engine.runtime,engine.contract,engine.config)._samples()
                for name,bank in [('sampled',sampled),('constant',constant),
                                  ('union',np.concatenate((sampled,constant[1:]),axis=0))]:
                    row,ledgers,_=compare_input_anchors(engine,snapshot,bank,planner_inputs=inputs)
                    path=out/f'case_{index}_{seed}_{name}.npz'
                    np.savez_compressed(path,**{f'{label}_{key}':value
                        for label,ledger in ledgers.items() for key,value in ledger.items()})
                    row.update(case=index,offset_s=case['requested_offset_s'],seed=seed,bank=name,
                        candidate_file=path.name,status_age_s=age,
                        candidate_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                        maximum_takeup_cost=float(np.max(ledgers['posterior']['takeup_cost'])),
                        recorded_planner_inputs=finite_json(inputs))
                    rows.append(row)
                print(json.dumps(dict(stage='bank_case',case=index,seed=seed)),flush=True)
        report=dict(completed=True,executed_commands=0,scope='saved_reconstructed_root_candidate_population_ablation',
            anchor_report_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),rows=rows,
            limitations=['Saved causal reconstructed roots, not original live roots.',
                'Fresh seeded proposals with zero warm start, not original live sampled population.',
                'Recorded prior status arbitration is combined with reconstructed roots; mixed-state diagnostic.',
                'Restored take-up rates; no arbiter/actuator execution or plant simulation.',
                'Intrusive diagnostic compute, not representative timing.'])
        path=out/'bank_comparison.json';path.write_text(json.dumps(finite_json(report),indent=2,allow_nan=False)+'\n')
        return path
    finally:engine.close()


def compare_anchors(args):
    """Feed native events once; paired diagnostic solves never alter the owner."""
    import hashlib
    import inspect
    import torch
    from dataclasses import asdict, replace
    import time
    from .compute_profile import read_events
    from .qualify_paced_compute import recorded_config
    from .engagement_qualification import compare_input_anchors
    from .replay_takeup_startup import constant_candidates
    session=Path(args.session); out=Path(args.outdir)
    torch.set_num_threads(4)
    if out.exists():raise ValueError('use a new report directory')
    if args.planner_process:raise ValueError('anchor attribution requires local intrusive offline solves')
    offsets=args.offsets_s
    if (not 1<=len(offsets)<=12 or offsets!=sorted(set(offsets))
            or not all(np.isfinite(t) and t>=0 for t in offsets)):
        raise ValueError('require 1..12 ascending unique finite nonnegative offsets')
    trials=[json.loads(line) for line in (session/'trials.jsonl').read_text().splitlines()]
    start=next(row for row in trials if row['event']=='trial_started' and row['trial']==args.trial)
    end=next(row for row in trials if row['event']=='trial_completed' and row['trial']==args.trial)
    stamps=[start['ros_time_ns']+round(t*1e9) for t in offsets]
    if stamps[-1]>end['ros_time_ns']:raise ValueError('offset falls outside the selected trial')
    record=json.loads((session/'controller_manifest.json').read_text())
    params=record['controller_parameters']
    engine=make_engine(args)
    try:
        if getattr(engine.runtime.profile,'kind',None)!='v171_static':
            raise ValueError('use a non-commanding v171_static manifest')
        engine.config=replace(recorded_config(params),samples=args.samples)
        maximum=np.asarray(params['controller_velocity_max'],float)
        if np.any(maximum<0) or np.any(maximum>engine.contract.velocity_max):
            raise ValueError('recorded velocities exceed hardware contract')
        engine.contract=replace(engine.contract,velocity_max=maximum,
            velocity_min=np.minimum(engine.contract.velocity_min,maximum))
        bank=constant_candidates(engine.contract,engine.config.horizon_steps)
        out.mkdir(parents=True)
        np.save(out/'candidate_bank.npy',bank)
        rows=[]; index=0; last_progress=time.monotonic()
        for event in read_events(session,'',require_plan=False,include_comparison=True):
            if time.monotonic()-last_progress>=5.:
                print(json.dumps(dict(stage='reconstructing_posterior',
                    receipt_ns=event.receipt_ns,cases_completed=index)),flush=True)
                last_progress=time.monotonic()
            # The first event at/after a requested time is NOT fed into that root.
            while index<len(stamps) and event.receipt_ns>=stamps[index]:
                snapshot=engine.snapshot()
                if snapshot is None:raise ValueError('missing prior replay snapshot')
                state,position,position_ns,_,belief=snapshot
                if not 0<=stamps[index]-state.timestamp_ns<=150_000_000:
                    raise ValueError('stale or future reconstructed root')
                snapshot=(state,position,position_ns,np.asarray(start['target_m']),belief)
                row,ledgers,roots=compare_input_anchors(engine,snapshot,bank)
                row.update(requested_offset_s=offsets[index],requested_ns=stamps[index])
                ledger_path=out/f'case_{index}_candidates.npz'
                np.savez_compressed(ledger_path,**{f'{label}_{key}':v
                    for label,ledger in ledgers.items() for key,v in ledger.items()})
                root_path=out/f'case_{index}_roots.pt'
                torch.save(dict(roots=roots,transmission_belief=belief,
                    position=position.copy(),target=np.asarray(start['target_m']),
                    configuration=asdict(engine.config)),root_path)
                row.update(candidate_file=ledger_path.name,root_file=root_path.name,
                    candidate_sha256=hashlib.sha256(ledger_path.read_bytes()).hexdigest(),
                    root_sha256=hashlib.sha256(root_path.read_bytes()).hexdigest())
                rows.append(row); index+=1
                print(json.dumps(finite_json(dict(stage='anchor_case',**row))),flush=True)
            if index==len(stamps):break
            engine.feed(event)
        if index!=len(stamps):raise ValueError('recording ended before requested snapshots')
        report=dict(scope='offline_causal_reconstructed_posterior_anchor_ablation',
            completed=True,executed_commands=0,session=str(session.resolve()),trial=args.trial,
            runtime_manifest_sha256=engine.bundle.identity.manifest_sha256,
            source_sha256={str(Path(path).resolve()):hashlib.sha256(Path(path).read_bytes()).hexdigest()
                for path in (__file__,inspect.getfile(compare_input_anchors),
                             inspect.getfile(type(engine.runtime)))},
            controller_record_sha256=hashlib.sha256((session/'controller_manifest.json').read_bytes()).hexdigest(),
            candidate_bank_sha256=hashlib.sha256((out/'candidate_bank.npy').read_bytes()).hexdigest(),
            configuration=asdict(engine.config),rows=rows,
            limitations=['Causal offline estimator reconstruction, not original live planner snapshots.',
                'Fixed constant proposal bank, not original random/warm-start population.',
                'Shared replay confirmation/marker uncertainty, not exact production scheduling or ownership.',
                'No simulated plant or observed outcomes for hypothetical commands; intrusive timing is not qualification.',
                'Packed .pt roots are trusted local diagnostics; do not load untrusted pickle artifacts.'])
        path=out/'anchor_comparison.json'
        path.write_text(json.dumps(finite_json(report),indent=2,allow_nan=False)+'\n')
        return path
    finally:engine.close()


if __name__=='__main__':main()
