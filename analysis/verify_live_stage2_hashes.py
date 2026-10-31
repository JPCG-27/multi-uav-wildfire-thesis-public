from pathlib import Path
import csv
base=Path(__file__).resolve().parent/'data/live_render_stage2'
def hashes(p):
    with p.open(encoding='utf-8-sig',newline='') as f:
        return {(r['event'],r['capture_tick'],r['consume_tick'],r['logical_id']):r['pixel_hash'] for r in csv.DictReader(f) if r['pixel_hash']}
for team,n in [('mixed',799),('allcamera',3199)]:
    a=hashes(base/f'{team}_original.csv'); b=hashes(base/f'{team}_repeat.csv')
    assert len(a)==len(b)==n and a==b
    print(team,n,'matching consumed-image hashes')
