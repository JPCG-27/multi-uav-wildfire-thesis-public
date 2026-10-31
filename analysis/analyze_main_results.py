#!/usr/bin/env python3
"""Recompute current-cohort means and recent descriptive cohort summaries."""
import csv, json, math
from collections import defaultdict
from pathlib import Path

ROOT=Path(__file__).resolve().parent
DATA=ROOT/'data'
T95_DF14=2.1447866879169273

def readcsv(path):
    with path.open(encoding='utf-8-sig',newline='') as f: return list(csv.DictReader(f))
def mean(xs): return sum(xs)/len(xs)
def ci95(xs):
    n=len(xs); m=mean(xs)
    sd=math.sqrt(sum((x-m)**2 for x in xs)/(n-1))
    half=T95_DF14*sd/math.sqrt(n)
    return {'mean':m,'ci95_low':m-half,'ci95_high':m+half,'seed_blocks':n}

rows=readcsv(DATA/'main_run_records.csv')
if len(rows)!=12960: raise ValueError(f'Expected 12960 main rows, got {len(rows)}')
byarm=defaultdict(list)
for r in rows: byarm[r['arm']].append(r)
expected={'mine_full_fov':3240,'mine_target_cell':3240,'random':3240,'oracle':3240}
if {k:len(v) for k,v in byarm.items()}!=expected: raise ValueError({k:len(v) for k,v in byarm.items()})

# Oracle's dynamic/static_uniform records are repeated outcomes. Verify mission
# outcome fields, then retain the static_uniform record once per scenario.
oracle_by_scenario=defaultdict(dict)
for r in byarm['oracle']:
    if r['regime'] in oracle_by_scenario[r['scenario_id']]: raise ValueError('duplicate Oracle scenario/regime')
    oracle_by_scenario[r['scenario_id']][r['regime']]=r
if len(oracle_by_scenario)!=1620: raise ValueError(f'Expected 1620 Oracle scenarios, got {len(oracle_by_scenario)}')
mission_fields=('total_persons','persons_detected','detection_fraction','mean_detection_latency_ticks','median_latency','recall_at_T','final_tick')
for sid,pair in oracle_by_scenario.items():
    if set(pair)!={'dynamic','static_uniform'}: raise ValueError(f'incomplete Oracle pair {sid}')
    for field in mission_fields:
        if pair['dynamic'][field]!=pair['static_uniform'][field]: raise ValueError(f'Oracle mission outcomes differ: {sid} {field}')
oracle_unique=[pair['static_uniform'] for pair in oracle_by_scenario.values()]
means={a:{'n':len(v),'equal_run_mean':mean([float(r['detection_fraction']) for r in v]),'percent':100*mean([float(r['detection_fraction']) for r in v])} for a,v in sorted(byarm.items()) if a!='oracle'}
means['oracle']={'n':len(oracle_unique),'equal_run_mean':mean([float(r['detection_fraction']) for r in oracle_unique]),'percent':100*mean([float(r['detection_fraction']) for r in oracle_unique]),'deduplication':'one static_uniform record per scenario after verifying paired mission outcomes'}

pairs={}
for arm in ('mine_full_fov','random'):
    for r in byarm[arm]:
        key=(r['scenario_id'],r['regime'])
        if key in pairs.setdefault(arm,{}): raise ValueError(f'duplicate pair key {key} for {arm}')
        pairs[arm][key]=(int(r['seed']),float(r['detection_fraction']))
if set(pairs['mine_full_fov'])!=set(pairs['random']): raise ValueError('unmatched main method scenarios')
seed_diffs=defaultdict(list)
for key,full in pairs['mine_full_fov'].items():
    rnd=pairs['random'][key]
    if full[0]!=rnd[0]: raise ValueError('seed mismatch')
    seed_diffs[full[0]].append(full[1]-rnd[1])
if set(seed_diffs)!=set(range(27,42)) or any(len(v)!=216 for v in seed_diffs.values()): raise ValueError('Unexpected seed-block structure')
paired=ci95([mean(seed_diffs[s]) for s in sorted(seed_diffs)])
paired['percent_points']={k:100*paired[k] for k in ('mean','ci95_low','ci95_high')}
paired['pairs']=len(pairs['mine_full_fov'])

fr=readcsv(DATA/'fully_random_oct22_run_summaries.csv')
if len(fr)!=3240 or any(int(r['final_tick'])!=8000 for r in fr): raise ValueError('Unexpected Oct 22 fully-random cohort')
fully_random={'n':len(fr),'equal_run_mean':mean([float(r['detection_fraction']) for r in fr]),'percent':100*mean([float(r['detection_fraction']) for r in fr]),'measured_date':'2026-10-22','interpretation':'descriptive; separately implemented cohort'}

camera={}
def cohort_cell_belief(name, aggregate_file, index_file, date_label):
    dat=readcsv(DATA/aggregate_file); index=readcsv(DATA/index_file)
    if len(dat)!=3240 or len(index)!=3240: raise ValueError(f'Unexpected run count for {name}')
    statuses=defaultdict(int)
    for r in index: statuses[r['status']]+=1
    if statuses!={'complete':3240}: raise ValueError(f'Incomplete {name} run index: {dict(statuses)}')
    idx={r['run_id']:r for r in index}
    vals=[]; bygrid=defaultdict(list)
    if len(idx)!=len(index) or {r['run_id'] for r in dat}!=set(idx): raise ValueError(f'Run-index mismatch for {name}')
    for r in dat:
        v=float(r['detection_fraction']); g=int(idx[r['run_id']]['grid']); vals.append(v); bygrid[g].append(v)
    return {'metric':'cell-belief coverage (aggregate detection_fraction; equal-run mean)','measured_date':date_label,'runs':len(vals),'run_index_statuses':dict(statuses),'equal_run_mean':mean(vals),'percent':100*mean(vals),'by_grid_percent':{str(g):100*mean(v) for g,v in sorted(bygrid.items())},'source_file':aggregate_file,'run_index_file':index_file,'comparison_scope':'descriptive cohort summary; no matched comparison with Oct 21 main cohort'}

camera['mixed_ideal_plus_camera']=cohort_cell_belief('mixed_ideal_plus_camera','camera_studies/mixed_ideal_plus_camera_aggregate.csv','camera_studies/mixed_ideal_plus_camera_run_index.csv','2026-10-09')
camera['four_camera']=cohort_cell_belief('four_camera','camera_studies/four_camera_aggregate.csv','camera_studies/four_camera_run_index.csv','2026-10-11')
three_rows=readcsv(DATA/'camera_studies/three_ideal_cell_belief_run_summaries.csv'); three_index=readcsv(DATA/'camera_studies/three_ideal_completed_attempt16_run_index.csv')
if len(three_rows)!=3240 or len(three_index)!=3240 or any(r['status']!='complete' for r in three_index): raise ValueError('Unexpected accepted three-ideal run records')
three_vals=[float(r['detection_fraction']) for r in three_rows]; three_bygrid=defaultdict(list)
for r in three_rows: three_bygrid[int(r['grid'])].append(float(r['detection_fraction']))
three_report=json.loads((DATA/'camera_studies/three_ideal_analysis.json').read_text(encoding='utf-8'))
three_sensor=three_report['metrics']['ideal']['three_ideal']
camera['three_ideal']={'metric':'cell-belief coverage (summary.csv detection_fraction; equal-run mean)','measured_date':'2026-10-18','runs':len(three_rows),'run_index_statuses':{'complete':len(three_index)},'equal_run_mean':mean(three_vals),'percent':100*mean(three_vals),'by_grid_percent':{str(g):100*mean(v) for g,v in sorted(three_bygrid.items())},'source_file':'camera_studies/three_ideal_cell_belief_run_summaries.csv','run_index_file':'camera_studies/three_ideal_completed_attempt16_run_index.csv','execution_verification':'provenance/three_ideal_attempt16_execution_summary.json','comparison_scope':'descriptive cohort summary; not a matched comparison with Oct 21 main cohort'}
camera['three_ideal_sensor_based']={'metric':'ideal-sensor witness coverage (person-run witnesses; not cell-belief coverage)','measured_date':'2026-10-18','runs':three_sensor['runs'],'equal_run_mean':three_sensor['equal_run_coverage'],'percent':100*three_sensor['equal_run_coverage'],'source_file':'camera_studies/three_ideal_analysis.json'}

result={'scope':'recent measured cohorts only; historical rows and comparisons excluded','main_cohort_date':'2026-10-21','main_rows':len(rows),'main_unique_scenarios':1620,'means_metric':'cell-belief coverage (summary.csv detection_fraction; equal-run mean)','means':means,'full_minus_random_paired_seed_block_ci':paired,'fully_random_oct22':fully_random,'camera_studies':camera,'limitations':'Main interval is Student-t over 15 paired seed means. Main Oracle is counted once per scenario after paired outcomes are checked. Camera/team-size cohorts are reported as separate cell-belief summaries; the three-ideal sensor-based witness metric is listed separately. These cohorts are not compared as matched controls to the Oct 21 main cohort.'}
(DATA/'portable_analysis_results.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(result,indent=2))

