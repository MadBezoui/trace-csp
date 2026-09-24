import csv
from trace_csp.data_sources.pems import summarize_pems


def test_pems_summary_groups_by_date_and_flags_missing_values(tmp_path):
    path=tmp_path/'pems.csv'
    with path.open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['time','station','flow','speed','occupancy'])
        w.writerows([['2025-01-01 08:00','A','360','72','0.15'],
                     ['2025-01-01 08:05','A','720','60','0.25'],
                     ['2025-01-02 08:00','A','','',''],
                     ['2025-01-02 08:05','B','180','80','0.10']])
    columns={'timestamp':'time','station':'station','flow':'flow','speed':'speed','occupancy':'occupancy'}
    result=summarize_pems(path,columns,speed_unit='kmh',occupancy_unit='fraction')
    assert result['stations']['A']['valid_rows']==2
    assert result['stations']['A']['missing_rows']==1
    assert result['stations']['A']['median_flow_veh_per_hour']==540
    assert set(result['dates'])=={'2025-01-01','2025-01-02'}
    assert result['status']=='descriptive calibration only'
    mph=summarize_pems(path,columns,speed_unit='mph',occupancy_unit='fraction')
    assert mph['stations']['A']['median_speed_kmh']==66*1.609344
    with path.open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['time','station','flow','speed','occupancy'])
        w.writerow(['2025-01-01 08:00','A','360','60','15'])
    percent=summarize_pems(path,columns,speed_unit='mph',occupancy_unit='percent')
    assert percent['stations']['A']['median_occupancy_fraction']==.15
