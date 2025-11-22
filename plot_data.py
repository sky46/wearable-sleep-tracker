from datetime import datetime, timedelta
import csv
from matplotlib import pyplot as plt, dates as m_dates
import pandas as pd

period_end = datetime.fromisoformat('2025-11-20T16:36:07')
period_start = period_end - timedelta(minutes=30)
leg_dts, arm_dts = [], []

cur_time = (period_start -
    timedelta(
        seconds=period_start.second,
        microseconds=period_start.microsecond
    )
)
while cur_time <= period_end:
    leg_dts.append(cur_time)
    arm_dts.append(cur_time)
    cur_time += timedelta(minutes=1)

with open('periodic_log.csv', 'r') as log_file:
    csv_reader = csv.reader(log_file)
    # Skip first line
    next(csv_reader)
    for line in csv_reader:
        line_dt = datetime.fromisoformat(line[0])
        if line_dt >= period_start:
            # Floor to most recent minute
            line_dt -= timedelta(seconds=line_dt.second, microseconds=line_dt.microsecond)
            if line[1] == 'ARM':
                arm_dts.append(line_dt)
            else:
                leg_dts.append(line_dt)

axes = plt.gca()
x_axis_formatter = m_dates.DateFormatter('%H:%M')
axes.xaxis.set_major_formatter(x_axis_formatter)

arm_df = pd.DataFrame(arm_dts)
leg_df = pd.DataFrame(leg_dts)
arm_counts = arm_df[0].value_counts().sort_index()
print(arm_counts)
leg_counts = leg_df[0].value_counts().sort_index()
arm_counts -= 1
leg_counts -= 1
print(arm_counts)
plt.plot(arm_counts.index, arm_counts.values, color='blue')
plt.plot(leg_counts.index, leg_counts.values, color='red')
plt.show()
