from trace_csp.data import generate_instance
from trace_csp.pipeline import compare_controllers
from trace_csp.simulation.synthetic_env import run_episode
from trace_csp.evaluation.plots import create_report


def test_report_writes_plots_and_counts(tmp_path):
    runs=compare_controllers([generate_instance(0,horizon=1),generate_instance(1,horizon=1)],controllers=('cp_sat','warm_cp_sat'))
    episode=run_episode(1,'cp_sat',periods=2)
    report=create_report(runs,[episode],tmp_path)
    assert report['controllers']['cp_sat']['attempted']==2
    assert report['controllers']['warm_cp_sat']['attempted']==2
    assert (tmp_path/'runtime_ecdf.pdf').stat().st_size>0
    assert (tmp_path/'queue_trajectory.pdf').stat().st_size>0
    assert (tmp_path/'report.json').stat().st_size>0
