from trace_csp.simulation.synthetic_env import run_episode, experiment_matrix
from trace_csp.csp.verifier import verify
from trace_csp.domain import Action
from trace_csp.data import generate_instance


def test_closed_loop_is_reproducible_and_accounts_for_vehicles():
    a=run_episode(seed=3,controller='cp_sat',periods=5)
    b=run_episode(seed=3,controller='cp_sat',periods=5)
    def deterministic(row):
        return ({k:v for k,v in row.items() if k not in ('feasible_at_deadline','halt_elapsed_s','records')},
                [{k:v for k,v in rec.items() if k not in ('control_elapsed_s','deadline_met')}
                 for rec in row['records']])
    assert deterministic(a)==deterministic(b)
    assert a['steps']==5 and a['hard_violations']==0
    assert a['total_time_spent']>=0
    assert a['exited_vehicles']>=0
    assert a['initial_vehicles']+a['arriving_vehicles']==a['final_vehicles']+a['exited_vehicles']
    assert a['demand_multiplier']==1.0 and a['incident_factor'] is None
    assert a['feasible_at_deadline']==sum(r['deadline_met'] for r in a['records'])


def test_severe_fault_halts_when_no_certified_action():
    a=run_episode(seed=6,controller='cp_sat',periods=2)
    assert a['steps']==0
    assert a['halt_reason']=='no-certified-action'


def test_incident_recovery_changes_realized_period_conditions():
    normal=run_episode(1,'fixed',periods=3,family='normal',incident_factor=800)
    mild=run_episode(1,'fixed',periods=3,family='mild_incident',incident_factor=800)
    severe=run_episode(1,'fixed',periods=3,family='severe_incident',incident_factor=800)
    assert [r['period_incident_factor'] for r in normal['records']]==[800,1000,1000]
    assert [r['period_incident_factor'] for r in mild['records']]==[800,900,1000]
    assert [r['period_incident_factor'] for r in severe['records']]==[800,800,800]


def test_matrix_uses_identical_seeds_across_controllers():
    rows=experiment_matrix(controllers=('fixed','alinea','cp_sat'),seeds=(1,2),periods=2)
    assert len(rows)==6
    assert set(x['seed'] for x in rows)=={1,2}
    assert set(x['controller'] for x in rows)=={'fixed','alinea','cp_sat'}


def test_closed_loop_cli_writes_requested_matrix(tmp_path):
    import subprocess,sys,gzip,json
    out=tmp_path/'episodes.jsonl.gz'
    run=subprocess.run([sys.executable,'scripts/run_closed_loop.py','--output',str(out),'--seeds','2',
                        '--periods','2','--controllers','fixed','cp_sat'],capture_output=True,text=True)
    assert run.returncode==0,run.stderr
    with gzip.open(out,'rt') as f: rows=[json.loads(line) for line in f]
    assert len(rows)==4
    assert set(x['seed'] for x in rows)=={0,1}


def test_closed_loop_cli_can_sample_multiple_families(tmp_path):
    import subprocess,sys,gzip,json
    out=tmp_path/'families.jsonl.gz'
    run=subprocess.run([sys.executable,'scripts/run_closed_loop.py','--output',str(out),'--seeds','1',
                        '--periods','1','--controllers','fixed','--families','normal','structural_ood'],
                       capture_output=True,text=True)
    assert run.returncode==0,run.stderr
    with gzip.open(out,'rt') as f: rows=[json.loads(line) for line in f]
    assert {x['family'] for x in rows}=={'normal','structural_ood'}
