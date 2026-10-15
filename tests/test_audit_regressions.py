"""Regressions for the October 2026 audit: initial-queue verification, held actuation
periods, device release tables, receiving-capacity bounds and the shield fallback."""
import random
from dataclasses import replace
import pytest
from trace_csp.corridor import generate_corridor
from trace_csp.data import instance_from_dict
from trace_csp.domain import Action, Corridor, State, Scenario, Config, Instance
from trace_csp.traffic.ctm import step
from trace_csp.csp.solver import solve
from trace_csp.csp.verifier import verify, verify_plan
from trace_csp.csp.controller import Controller
from trace_csp.graph.physics import interval_core, interval_reach

ONE_CELL = Corridor((500,), (1,), (100,), (20,), (0,), (10,), (5,))
HIGH, LOW = Action((10,), (100,)), Action((0,), (100,))


def one_cell(queue, arrivals, horizon=1, block=1, previous=HIGH):
    cfg = Config(horizon=horizon, dt_s=10, vsl_values=(100,), max_dm=10, time_limit_s=5, control_block=block)
    return Instance(ONE_CELL, State((0,), (queue,), 0, previous),
                    (Scenario(1, (0,)*horizon, ((arrivals,),)*horizon, ((1000,),)*horizon),), cfg)


def test_initial_queue_above_storage_is_rejected_and_counted():
    i = one_cell(queue=6, arrivals=0)
    hard = verify(i, HIGH)
    assert not hard.safe and 's0/t0: queue bound' in hard.violations and hard.max_queue_excess == 1
    assert verify(i, HIGH, queue_slack=1).safe
    assert solve(i).status == 'INFEASIBLE' and interval_core(i) == 'queue/0/0/0'
    assert Controller(mode='hard').act(i).mode == 'no-certified-action'


def test_relaxation_reports_a_draining_initial_violation():
    i = one_cell(queue=6, arrivals=0)
    relaxed = solve(i, relax='lexicographic')
    assert relaxed.status == 'OPTIMAL' and relaxed.max_queue_excess == 1 == relaxed.values['excess/max']
    d = Controller(mode='degrade').act(i)
    assert d.mode == 'degraded' and d.audit['trigger'] == 'certificate' and d.audit['planned_max_queue_excess'] == 1
    assert d.audit['exceedance_proved_minimal'] is True


def test_receiving_capacity_above_vehicle_count_does_not_make_the_model_infeasible():
    i = one_cell(queue=0, arrivals=1)  # 1 vehicle in the system, receiving capacity 11
    r = solve(i)
    assert r.status == 'OPTIMAL' and verify(i, r.actions[0]).safe


def test_commands_are_held_and_verified_over_the_actuation_period():
    i = one_cell(queue=0, arrivals=5, horizon=3, block=3, previous=LOW)
    changing = verify_plan(i, (LOW, HIGH, HIGH))
    assert not changing.safe and any('within its actuation period' in v for v in changing.violations)
    held = verify(i, LOW)  # the period lasts three steps: queues 5, 10, 15
    assert not held.safe and [s.queues[0] for s in held.trajectories[0]] == [0, 5, 10, 15]
    r = solve(i)
    assert r.actions[0] == r.actions[1] == r.actions[2] and verify(i, r.actions[0]).safe


def metered_corridor(seed, block=3):
    """Six-cell corridor whose metered ramps follow a green-phase release table."""
    table = ((0, 0, 0), (1, 0, 0), (1, 1, 0), (1, 1, 1), (2, 1, 1), (2, 2, 1), (2, 2, 2), (3, 2, 2))
    i = generate_corridor(seed, cells=6, horizon=6, time_limit_s=5)
    c = i.corridor; r = len(c.ramp_segment)
    c = replace(c, ramp_min=(1,)*r, ramp_max=(7,)*r, release_table=(table,)*r)
    cfg = replace(i.config, control_block=block, max_dm=7)
    previous = replace(i.state.previous, ramp=(4,)*r)
    return replace(i, corridor=c, config=cfg, state=replace(i.state, previous=previous))


def test_release_table_is_shared_by_ctm_verifier_solver_and_certificate():
    for seed in range(6):
        i = metered_corridor(seed); c = i.corridor; rng = random.Random(seed)
        commands = [Action(tuple(rng.randint(1, 7) for _ in c.ramp_segment), i.state.previous.vsl) for _ in range(2)]
        plan = tuple(a for a in commands for _ in range(3))
        cert = verify_plan(i, plan, queue_slack=10**6)
        assert 'mass conservation' not in cert.violations
        bounds = interval_reach(i)
        for wi, w in enumerate(i.scenarios):
            s = i.state
            for t, a in enumerate(plan):
                tr = step(c, s, a, w, t, i.config); s = tr.state
                assert s == cert.trajectories[wi][t+1]
                assert all(x <= c.release_table[k][a.ramp[k]][t % 3] for k, x in enumerate(tr.ramp_release))
                for k, q in enumerate(s.queues):
                    lo, hi = bounds[f'q/{wi}/{t+1}/{k}']
                    assert lo <= q <= hi
        # The solver raises if its states differ from the verifier's replay.
        r = solve(i, relax='lexicographic')
        assert r.actions is not None and all(r.actions[t] == r.actions[t - t % 3] for t in range(6))


def test_instance_with_release_table_round_trips_through_json():
    from dataclasses import asdict
    import json
    i = metered_corridor(1)
    assert instance_from_dict(json.loads(json.dumps(asdict(i)))) == i


def test_shield_falls_back_to_the_verified_reference_when_the_solver_has_no_time():
    i = metered_corridor(0)
    i = replace(i, config=replace(i.config, time_limit_s=0))
    ctl = Controller(mode='degrade'); ctl.reference = [5]*len(i.corridor.ramp_segment)
    assert interval_core(i) is None and verify(i, Action(tuple(ctl.reference), i.state.previous.vsl)).safe
    d = ctl.act(i)
    assert d.mode == 'fallback-reference' and d.action.ramp == tuple(ctl.reference)


@pytest.mark.skipif(not __import__('pathlib').Path('data/raw/i210').exists(), reason='I-210 benchmark not downloaded')
def test_meter_release_table_matches_the_upstream_signal_plan():
    import importlib.util, json
    from pathlib import Path
    from trace_csp.sumo_i210 import BENCHMARK, meter_commands, meter_release_table, green_starts
    base = Path('data/raw/i210')/BENCHMARK
    spec = importlib.util.spec_from_file_location('freewayControl', base/'freewayControl.py')
    fc = importlib.util.module_from_spec(spec); spec.loader.exec_module(fc)
    class Env:
        def convertUnit(self, t): return round(t)
    for name, cfg in json.loads((base/'meterConfig.json').read_text()).items():
        lanes = json.loads((base/'onrampConfig.json').read_text())[name]['numRampLane']
        meter = fc.RampMeter(Env(), cfg['meterID'], [0, 1], lanes, cfg['numVehPerGreenPerLane'], cfg['greenPhaseLen'],
                             cfg['minRedPhaseLen'], cfg['minRedPhaseLen'], cfg['maxRedPhaseLen'])
        lo, hi, table = meter_release_table(cfg, lanes)
        for n, red in meter_commands(cfg).items():
            meter.generateMeteringPlan([meter.greenPhaseLen, red], 30)
            plan = [x == 0 for x in meter.meteringPlan]
            assert tuple(lanes*cfg['numVehPerGreenPerLane']*x for x in green_starts(plan, 10)) == table[n]
            assert sum(table[n]) == n*lanes*cfg['numVehPerGreenPerLane']
        # ALINEA's flow-to-red conversion only issues these red phases.
        for flow in range(int(meter.convertRedPhaseLenToMeteredFlow(meter.maxRedPhaseLen, 30)),
                          int(meter.convertRedPhaseLenToMeteredFlow(meter.minRedPhaseLen, 30))+1, 7):
            assert meter.convertMeteredFlowToRedPhaseLen(flow, 30) in meter_commands(cfg).values()


def test_pems_manifest_lists_81_dates_and_fitted_lanes_match_metadata():
    import json, csv
    from pathlib import Path
    dates = [l for l in Path('configs/data/pems_i210_dates.txt').read_text().splitlines() if l and not l.startswith('#')]
    assert len(dates) == len(set(dates)) == 81
    fits = {int(r['station']): int(r['lanes']) for r in csv.DictReader(open('results/pems-fd-i210-calibration.csv'))}
    meta = {int(r['ID']): int(r['Lanes']) for r in csv.DictReader(open('data/interim/pems/i210e_station_positions.csv'))}
    report = json.loads(Path('results/pems-i210e-calibration.json').read_text())
    used = {r['station'] for r in report['validation_by_station']} | {s for c in report['cells'] for s in c['stations']}
    assert used and all(fits[s] == meta[s] for s in used)


def _replay_one(tmp_path, mutate):
    """Replay one degraded decision of a stored run after a mutation; return the exit code."""
    import json, subprocess, sys
    from pathlib import Path
    report = json.loads(Path('results/i210-final/shield-halt-f0.8-s6.json').read_text())
    row = next(r for r in report['decisions'] if r['mode'] == 'degraded' and r['certified_core'].split('/')[2] == '0')
    report['decisions'] = [mutate(dict(row))]
    path = tmp_path/'run.json'; path.write_text(json.dumps(report))
    return subprocess.run([sys.executable, 'scripts/replay_i210_decisions.py', str(path)], capture_output=True, text=True,
                          env={'PYTHONPATH': 'src', 'PATH': '/usr/bin:/bin'}).returncode


def test_replay_accepts_a_stored_decision(tmp_path):
    assert _replay_one(tmp_path, lambda r: r) == 0


def test_replay_rejects_an_understated_exceedance(tmp_path):
    assert _replay_one(tmp_path, lambda r: {**r, 'planned_excess': 0}) == 1


def test_replay_rejects_an_executed_command_outside_the_device_domain(tmp_path):
    assert _replay_one(tmp_path, lambda r: {**r, 'executed': [0]+r['executed'][1:]}) == 1


def test_replay_rejects_a_record_without_its_declared_bound(tmp_path):
    assert _replay_one(tmp_path, lambda r: {k: v for k, v in r.items() if k != 'planned_excess'}) == 1
