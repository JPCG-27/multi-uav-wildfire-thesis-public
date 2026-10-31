"""Verify retained October live-to-live sensor-image hashes (no simulator run)."""
import csv
import json
from pathlib import Path

root=Path(__file__).resolve().parent/'data/live_render_repeat'
fields=('event','dense_slot','logical_id','capture_tick','consume_tick','rendered',
        'capture_position_xyz','capture_quaternion_xyzw','current_position_xyz',
        'current_quaternion_xyzw','capture_fov_deg','current_fov_deg','capture_width',
        'capture_height','projection_state_json','pixel_hash_algorithm','pixel_hash')
records=[]
for name in ('baseline_camera_state.csv','repeat_camera_state.csv'):
    with (root/name).open(encoding='utf-8-sig',newline='') as f:
        rows=list(csv.DictReader(f))
    assert len(rows)==6399
    assert all(r['cache_mode']=='record' and r['rendered']=='true' for r in rows)
    captures=[r for r in rows if r['event']=='capture']
    pixels=[r for r in rows if r['pixel_hash']]
    assert len(captures)==3200 and len(pixels)==3199
    assert all(r['event']=='consume' and r['pixel_hash_algorithm']=='fnv1a64-bgra8' for r in pixels)
    assert all((r['readback_width'],r['readback_height'])==('640','640') for r in pixels)
    records.append(rows)
assert [[r[k] for k in fields] for r in records[0]]==[[r[k] for k in fields] for r in records[1]]
print(json.dumps({'passed':True,'live_runs':2,'scenarios':1,'paired_image_hashes':3199,
                  'hash_mismatches':0,'scope':'640x640 BGRA8 sensor image hashes over 8000 ticks'}))
