import bagpy
from bagpy import bagreader 
import matplotlib.pyplot as plt 
import numpy as np 
import seaborn as sea 
import pandas as pd
bag = bagreader('/home/hmruthvik/catkin_ws/src/gnss_ros_driver_/data/obs_stationary.bag')
bag.topic_table
def data_csv(bag):
    csvfiles = []
    for t in bag.topics:
        data = bag.message_by_topic(t)
        csvfiles.append(data)
        
    print(csvfiles[0])
    data = pd.read_csv(csvfiles[0])
    return data
csv_data = data_csv(bag)
figure = pd.read_csv("/home/hmruthvik/catkin_ws/src/gnss_ros_driver_/data/obs_stationary/gnss.csv")
figure['UTM_easting_median'] = figure['UTM_easting'] - figure['UTM_easting'].median(axis=0)
figure['UTM_northing_median'] = figure['UTM_northing'] - figure['UTM_northing'].median(axis=0)
figure
figure['quality']. value_counts()
x=()
fig, a = bagpy.create_fig(1)
scatter = a[0].scatter(x='UTM_easting_median', y = 'UTM_northing_median', data=figure, c='quality')
m = figure['UTM_easting_median'].median()
n = figure['UTM_northing_median'].median()
sigx=figure['UTM_easting_median'].std()
sigy=figure['UTM_northing_median'].std()
CEP=0.62*sigx + 0.56*sigy
DRMS=2*np.sqrt((sigx*sigx)+(sigy*sigy))
circle = plt.Circle((m,n),DRMS,color='blue',alpha=0.1)
circle1 = plt.Circle((m,n),CEP,color='forestgreen',alpha=0.1)
plt.title("UTM Northing vs UTM Easting")
plt.xlabel("UTM_easting(meter)")
plt.ylabel("UTM_northing(meter)")
legend1 =a[0].legend(*scatter.legend_elements(), loc="upper right", title="Quality")
a[0].axis("equal")
a[0].add_artist(legend1)
a[0].add_patch(circle)
a[0].add_patch(circle1)
plt.show()