# -*- coding: utf-8 -*-
"""
Created on Thu May  7 18:11:06 2020

@author: TiagoFilipe
"""

import pandas, folium

route_default_datafile = pandas.read_excel('./Recursos/MarineTraffic/route_default_HK-Man.xlsx')

areamap = folium.Map(location=[22.333332, 114.1166662])

route_default_points = route_default_datafile.values.tolist()
folium.PolyLine(route_default_points).add_to(areamap)

areamap.save('route_default_HK-Man.html')