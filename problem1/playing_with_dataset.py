# -*- coding: utf-8 -*-
"""
Created on Sun Sep 17 14:55:59 2017

@author: PulkitMaloo
"""

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
plt.style.use('ggplot')

df_city = pd.read_table('city-gps.txt', sep=' ', names=['city', 'lat', 'lon'])
df_city.plot(x='lat', y='lon', kind='scatter')
#print(df_city[df_city.duplicated(keep=False)])
df_city.drop_duplicates(inplace=True)
#df_city[df_city.notnull()]         # No null values

df_road = pd.read_table('road-segments.txt', sep=' ', names=['from_city', 'to_city', 'dist', 'speed', 'highway_name'])
df_road['time'] = df_road.dist/df_road.speed
df_road.drop_duplicates(inplace=True)
#print(df_road[df_road.duplicated(keep=False)])
#df_road[df_road.notnull()]         # No null values