"""Recompute complementary objectives from the bundled compact recorded evidence.

Run with Python 3.10+: python analysis/complementary_objectives/reproduce.py
Standard library only. No simulator, absolute source paths, or external data required.
"""
from pathlib import Path
from collections import Counter, defaultdict
import csv, gzip, hashlib, json, math, statistics

HERE=Path(__file__).resolve().parent
HORIZON=8000
SEEDS=tuple(range(27,42))
# Two-sided 95% Student-t critical value, 14 degrees of freedom.
T95_DF14=2.1447866879169273
METRICS=(
    'primary_cell_belief_coverage_pct','eventual_before_death_pct','early_before_death_pct',
    'penalized_first_credit_delay_ticks','timeliness_score_pct','recorded_belief_uptime_pct',
    'untracked_danger_ticks_per_person','first_fire_penalized_tick','fire_found_pct')

def read_gzip(name):
    with gzip.open(HERE/name,'rt',encoding='utf-8',newline='') as f:return list(csv.DictReader(f))
def write_csv(name,rows):
    with (HERE/name).open('w',encoding='utf-8',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
def interval(blocks):
    assert len(blocks)==15
    m=statistics.mean(blocks);h=T95_DF14*statistics.stdev(blocks)/math.sqrt(15)
    return m,m-h,m+h
def main():
    names=json.loads((HERE/'method_names.json').read_text(encoding='utf-8'))
    provenance=json.loads((HERE/'provenance.json').read_text(encoding='utf-8'))
    for name,expected in provenance['evidence_sha256'].items():
        assert hashlib.sha256((HERE/name).read_bytes()).hexdigest()==expected,(name,'SHA-256 mismatch')
    scenarios={};scenario_rosters=defaultdict(set)
    for r in read_gzip('scenario_person_states.csv.gz'):
        scenarios[(r['scenario_id'],int(r['person_id']))]=(int(r['appear_tick']),int(r['first_dead_tick']))
        scenario_rosters[r['scenario_id']].add(int(r['person_id']))
    people=defaultdict(list)
    for r in read_gzip('person_evidence.csv.gz'):people[int(r['run_number'])].append(r)
    rows=[];person_rows=0
    for run in read_gzip('run_evidence.csv.gz'):
        rid=int(run['run_number']);n=int(run['persons']);ps=people[rid];assert len(ps)==n
        assert len({int(p['person_id']) for p in ps})==n
        assert {int(p['person_id']) for p in ps}==scenario_rosters[run['scenario_id']]
        assert int(run['first_tick'])==1 and int(run['final_tick'])==HORIZON
        credited=alive=early=delay=tracked=gaps=danger=0
        for p in ps:
            pid=int(p['person_id']);ap=int(p['appear_tick']);first=int(p['first_detect_tick'])
            gap=int(p['total_tracking_gap_ticks']);dg=int(p['untracked_danger_ticks'])
            truth_ap,dead=scenarios[(run['scenario_id'],pid)]
            assert ap==truth_ap==0
            assert first==-1 or 1<=first<=HORIZON
            ok=first>=0 and (dead<0 or first<dead)
            credited+=first>=0;alive+=ok;early+=ok and first-ap<=500
            delay+=first-ap if ok else HORIZON+1-ap
            value=HORIZON+1-first-gap if first>=0 else 0
            assert 0<=value<=HORIZON and (first>=0 or gap==0)
            tracked+=value;gaps+=gap;danger+=dg
        assert credited==int(run['persons_detected'])
        assert gaps==int(run['total_tracking_gap_ticks'])
        assert danger==int(run['total_untracked_danger_ticks'])
        fire=int(run['fire_discovery_tick']);assert fire==-1 or 1<=fire<=HORIZON
        rows.append(dict(method=run['method'],method_label=names[run['method']],scenario_id=run['scenario_id'],
                         regime=run['regime'],grid=int(run['grid']),seed=int(run['seed']),persons=n,
                         primary_cell_belief_coverage_pct=100*credited/n,eventual_before_death_pct=100*alive/n,
                         early_before_death_pct=100*early/n,penalized_first_credit_delay_ticks=delay/n,
                         timeliness_score_pct=100*(HORIZON+1-delay/n)/HORIZON,
                         recorded_belief_uptime_pct=100*tracked/(HORIZON*n),
                         untracked_danger_ticks_per_person=danger/n,
                         first_fire_penalized_tick=fire if fire>=0 else HORIZON+1,fire_found_pct=100*int(fire>=0)))
        person_rows+=n
    assert len(rows)==provenance['runs']==22680
    assert person_rows==provenance['person_run_rows']
    assert Counter(r['method'] for r in rows)=={m:3240 for m in names}
    keys={(r['method'],r['scenario_id'],r['regime']) for r in rows}
    assert len(keys)==len(rows)
    expected={(s,g) for m,s,g in keys if m=='mine_full_fov'}
    assert all({(s,g) for m,s,g in keys if m==method}==expected for method in names)
    write_csv('per_run_scores.csv',rows)
    summaries=[];effects=[];seed_effects=[];allocation_effects=[]
    for grid in ['all',100,200,300]:
        for reg in ['all','static','dynamic']:
            group=[r for r in rows if (grid=='all' or r['grid']==grid) and (reg=='all' or r['regime']==reg)]
            lookup={(r['method'],r['scenario_id'],r['regime']):r for r in group}
            for method,label in names.items():
                g=[r for r in group if r['method']==method]
                summaries.append(dict(method=method,method_label=label,grid=grid,regime=reg,runs=len(g),
                                      **{m:statistics.mean(r[m] for r in g) for m in METRICS}))
                if method=='mine_full_fov':continue
                pairs=[(a,lookup[('mine_full_fov',a['scenario_id'],a['regime'])]) for a in g]
                for metric in METRICS:
                    blocks=[statistics.mean(a[metric]-b[metric] for a,b in pairs if a['seed']==seed) for seed in SEEDS]
                    for seed,delta in zip(SEEDS,blocks):
                        seed_effects.append(dict(method=method,method_label=label,reference='Priority planner',
                                                 grid=grid,regime=reg,metric=metric,seed=seed,paired_seed_mean_delta=delta))
                    mean,lo,hi=interval(blocks)
                    effects.append(dict(method=method,method_label=label,reference='Priority planner',
                                        difference='method minus Priority planner',grid=grid,regime=reg,metric=metric,
                                        pairs=len(pairs),seed_blocks=15,mean_seed_block_delta=mean,ci95_low=lo,ci95_high=hi))
        for method,label in names.items():
            g=[r for r in rows if r['method']==method and (grid=='all' or r['grid']==grid)]
            lookup={(r['scenario_id'],r['regime']):r for r in g}
            pairs=[(a,lookup[(a['scenario_id'],'static')]) for a in g if a['regime']=='dynamic']
            for metric in METRICS:
                blocks=[statistics.mean(a[metric]-b[metric] for a,b in pairs if a['seed']==seed) for seed in SEEDS]
                mean,lo,hi=interval(blocks)
                allocation_effects.append(dict(method=method,method_label=label,difference='dynamic minus static',grid=grid,
                                               metric=metric,pairs=len(pairs),seed_blocks=15,
                                               mean_seed_block_delta=mean,ci95_low=lo,ci95_high=hi))
    write_csv('method_summaries.csv',summaries)
    write_csv('paired_effects_vs_priority.csv',effects)
    write_csv('paired_seed_effects.csv',seed_effects)
    write_csv('allocation_effects.csv',allocation_effects)
    report=dict(
        scope=provenance['scope'],runs=len(rows),person_run_rows=person_rows,
        definitions={
            'primary_cell_belief_coverage_pct':'100 times fraction with any recorded first credit by tick 8000; retains the original primary outcome and includes post-death credit.',
            'eventual_before_death_pct':'100 times fraction with first credit strictly before earliest retained trajectory state 3; equal death and credit ticks fail.',
            'early_before_death_pct':'100 times fraction with first credit within 500 ticks of appearance and strictly before death.',
            'penalized_first_credit_delay_ticks':'Run mean of first-credit tick minus appearance for strict before-death credits; missing or post-death-only credits receive 8001 minus appearance.',
            'timeliness_score_pct':'100*(8001-penalized_first_credit_delay_ticks)/8000. Affine restatement of the existing delay outcome, not independent supporting evidence; equals normalized area under the cumulative strict before-death first-credit curve on ticks 1..8000.',
            'recorded_belief_uptime_pct':'100*sum(8001-first_detect_tick-total_tracking_gap_ticks)/(8000*persons), with never-credited persons contributing 0. Includes pre-first-credit absence, post-death states and recorder belief-proxy semantics; not continuous visual tracking or alive-only uptime.',
            'untracked_danger_ticks_per_person':'Recorded untracked active-fire ticks divided by total persons. Includes pre-first-credit absence and post-death states; not alive-only danger coverage. Danger threshold 0.5 acts on binary active fire.',
            'first_fire_penalized_tick':'First positive fire-observation tick or 8001 if absent. Measures first observed fire, not sustained fire monitoring.',
            'fire_found_pct':'100 if any positive fire observation is recorded in run, otherwise 0.'},
        aggregation='Equal-run means in each grid/regime stratum. All methods have identical scenario/regime keys. Each contrast pairs scenario and regime, averages differences within each seed, then uses a two-sided Student-t 95% interval across 15 seed means (seeds 27..41, df 14). Intervals are descriptive and unadjusted for multiple comparisons.',
        limitations=['Recorded belief positivity/freshness is a cell-based proxy and may borrow evidence from nearby or previously occupied cells.',
                     'Uptime and danger include post-death records; alive-only continuous observation/danger exposure cannot be recovered from aggregate gap totals.',
                     'All first-credit measures are event summaries and do not establish visual person identification, rescue, survival improvements or sustained fire monitoring.',
                     'Several cohorts use different retained builds. Fully random also changes allocation policy; these matched contrasts do not isolate planner causality.',
                     'Compact scenario-person death records and source hashes support numerical replay; full source trajectories and per-tick belief states are not bundled here.'],
        checks=provenance['checks'],settings=provenance['settings'],
        summaries=summaries,paired_effects=effects,allocation_effects=allocation_effects)
    (HERE/'results.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps([s for s in summaries if s['grid']=='all' and s['regime']=='all'],indent=2))
    print('Wrote 84 method strata, 648 paired contrasts and 252 allocation effects; all evidence hashes verified.')

if __name__=='__main__':main()
