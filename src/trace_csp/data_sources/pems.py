"""Strict descriptive preprocessing of a user-supplied PeMS CSV export.

No implicit station-to-I-210-segment mapping or CTM calibration is inferred.
"""
import csv
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from statistics import median

REQUIRED=('timestamp','station','flow','speed','occupancy')


def summarize_pems(path,columns,flow_unit='veh_per_hour',speed_unit=None,occupancy_unit=None):
    if set(columns)!=set(REQUIRED): raise ValueError('all five explicit column names required')
    if flow_unit not in ('veh_per_hour','vehicles_per_interval'): raise ValueError('flow unit must be explicit')
    if speed_unit not in ('mph','kmh'): raise ValueError('speed unit must be mph or kmh')
    if occupancy_unit not in ('fraction','percent'): raise ValueError('occupancy unit must be fraction or percent')
    stations=defaultdict(lambda:{'flow':[],'speed':[],'occupancy':[],'missing_rows':0,'valid_rows':0})
    dates=set(); total=0
    with Path(path).open(newline='',encoding='utf-8-sig') as f:
        reader=csv.DictReader(f)
        if not reader.fieldnames or any(name not in reader.fieldnames for name in columns.values()):
            raise ValueError('missing required PeMS columns')
        for row in reader:
            total+=1
            station=row[columns['station']].strip()
            if not station: raise ValueError(f'row {total}: missing station ID')
            try: date=datetime.fromisoformat(row[columns['timestamp']].strip()).date().isoformat()
            except ValueError as e: raise ValueError(f'row {total}: invalid timestamp') from e
            dates.add(date); rec=stations[station]
            try:
                flow=float(row[columns['flow']]); speed=float(row[columns['speed']]); occupancy=float(row[columns['occupancy']])
                import math
                upper=100 if occupancy_unit=='percent' else 1
                valid=all(math.isfinite(v) for v in (flow,speed,occupancy)) and flow>=0 and speed>0 and 0<=occupancy<=upper
            except (ValueError,TypeError): valid=False
            if valid:
                rec['valid_rows']+=1; rec['flow'].append(flow)
                rec['speed'].append(speed*1.609344 if speed_unit=='mph' else speed)
                rec['occupancy'].append(occupancy/100 if occupancy_unit=='percent' else occupancy)
            else: rec['missing_rows']+=1
    if not total: raise ValueError('empty PeMS export')
    out={}
    for station,values in sorted(stations.items()):
        out[station]={'valid_rows':values['valid_rows'],'missing_rows':values['missing_rows'],
                      'median_flow_veh_per_hour':median(values['flow']) if values['flow'] and flow_unit=='veh_per_hour' else None,
                      'median_flow_vehicles_per_interval':median(values['flow']) if values['flow'] and flow_unit=='vehicles_per_interval' else None,
                      'median_speed_kmh':median(values['speed']) if values['speed'] else None,
                      'median_occupancy_fraction':median(values['occupancy']) if values['occupancy'] else None}
    return {'status':'descriptive calibration only','source_rows':total,'dates':sorted(dates),
            'flow_unit':flow_unit,'source_speed_unit':speed_unit,
            'source_occupancy_unit':occupancy_unit,'stations':out}
