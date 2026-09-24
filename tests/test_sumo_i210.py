import pytest
from pathlib import Path

REPO=Path('data/raw/i210')
pytestmark=pytest.mark.skipif(not REPO.exists(),reason='I-210 benchmark not downloaded')


def test_layout_respects_cfl_single_inflow_and_all_meters():
    from trace_csp.sumo_i210 import extract_layout,corridor_from_layout
    L=extract_layout(str(REPO))
    assert all(c['length']>=L['min_cell_m'] for c in L['cells'])
    metered=[c['inflow']['name'] for c in L['cells'] if c['inflow'] and c['inflow']['kind']=='metered']
    assert sorted(metered,key=lambda x:int(x[6:]))==[f'Onramp{k}' for k in range(5,18)]
    edges=[e for c in L['cells'] for e in c['edges']]
    assert edges==[f'S{k}_M' for k in range(25,68)]
    c=corridor_from_layout(L)
    assert all(lo==hi for lo,hi,cell in zip(c.ramp_min,c.ramp_max,[x for x in L['cells'] if x['inflow']]) if cell['inflow']['kind']!='metered')


def test_calibrated_corridor_is_a_valid_model():
    from trace_csp.sumo_i210 import extract_layout,corridor_from_layout
    calibration=Path('results/pems-i210e-calibration.json')
    if not calibration.exists(): pytest.skip('calibration not generated')
    c=corridor_from_layout(extract_layout(str(REPO)),calibration=str(calibration),exit_permille=[100]*25)
    assert len(c.jam)==25 and min(c.capacity)>0 and c.wave_speed_kmh>0
