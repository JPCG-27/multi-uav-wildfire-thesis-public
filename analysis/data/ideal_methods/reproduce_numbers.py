from pathlib import Path
import csv, math, statistics
from scipy.stats import t
src=Path(__file__).resolve().parent/'per_run_metrics.csv'
out=Path(__file__).resolve().parent
with src.open(encoding='utf-8-sig',newline='') as f: rows=list(csv.DictReader(f))
methods=list(dict.fromkeys(r['dataset'] for r in rows))
metrics=['legacy_fraction','alive_fraction','alive_recall_within_500','miss_penalized_latency','distance']
for r in rows:
 for k in ['seed','grid','persons'] : r[k]=int(r[k])
 for k in metrics:r[k]=float(r[k])
assert len(rows)==22680 and len(methods)==7
assert all(sum(r['dataset']==m for r in rows)==3240 for m in methods)
def avg(x):return statistics.mean(x)
def interval(x):
 m=avg(x); h=t.ppf(.975,len(x)-1)*statistics.stdev(x)/math.sqrt(len(x));return m,m-h,m+h
# Equal-run mean of per-run fractions (the thesis CELL-BELIEF COVERAGE definition).
summary=[]
for m in methods:
 for reg in ['all','static','dynamic']:
  for grid in ['all',100,200,300]:
   g=[r for r in rows if r['dataset']==m and (reg=='all' or r['regime']==reg) and (grid=='all' or r['grid']==grid)]
   if not g:continue
   z={'method':m,'regime':reg,'grid':grid,'runs':len(g),'persons':sum(r['persons'] for r in g)}
   for metric in metrics:z[metric]=avg([r[metric] for r in g])
   summary.append(z)
with (out/'numbers_methods.csv').open('w',newline='',encoding='utf-8') as f:
 w=csv.DictWriter(f,fieldnames=summary[0]);w.writeheader();w.writerows(summary)
lookup={(r['dataset'],r['scenario_id'],r['regime']):r for r in rows}
contrasts=[('lawnmower_v2','mine_full_fov'),('lawnmower_v2','lawnmower_v1_aligned')]
effects=[]
for a,b in contrasts:
 for reg in ['all','static','dynamic']:
  for grid in ['all',100,200,300]:
   pairs=[(x,y) for (m,s,rr),x in lookup.items() if m==a and (reg=='all' or rr==reg) and (grid=='all' or x['grid']==grid) for y in [lookup[(b,s,rr)]]]
   for metric in metrics:
    # Pair within scenario/regime, average paired deltas within each seed; t CI over 15 seed blocks.
    blocks=[]
    for seed in range(27,42):
     ds=[x[metric]-y[metric] for x,y in pairs if x['seed']==seed]
     if ds:blocks.append(avg(ds))
    m,lo,hi=interval(blocks)
    effects.append({'contrast':a+'_minus_'+b,'regime':reg,'grid':grid,'metric':metric,'pairs':len(pairs),'seed_blocks':len(blocks),'mean_seed_block_delta':m,'ci95_low':lo,'ci95_high':hi})
with (out/'numbers_key_contrasts.csv').open('w',newline='',encoding='utf-8') as f:
 w=csv.DictWriter(f,fieldnames=effects[0]);w.writeheader();w.writerows(effects)
print(f'wrote {len(summary)} strata and {len(effects)} paired effects')
