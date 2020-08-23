# -*- coding: utf-8 -*-
"""
Created on Tue May 26 10:31:48 2020

@author: TiagoFilipe
"""

import pandas, folium, json, math, pyproj
from datetime import datetime, timedelta
from shapely.geometry import Point, LineString, Polygon

col_list = ['timestamp', 'mmsi', 'latitude', 'longitude', 'speed', 'course', 'heading']
datafile = pandas.read_csv('pos_messages.csv', usecols=col_list, low_memory = False)
datafile.index = datafile['mmsi']
datafile = datafile.loc[datafile['speed'] < 30]                             #Filtrar posibles errores

datafile_ellipse = pandas.read_csv('./Recursos/Limitación_geográfica_Elipse/plot_ellipse.csv')  
ellipse_points = datafile_ellipse.values.tolist()
ellipse = Polygon(ellipse_points)                                #Limitación geográfica: Área de actuación

vessel_in_or_out=[]                                             #Filtrado de datos geográficamente
for i,row in datafile.iterrows():
    v = Point(row.latitude, row.longitude)
    check = v.within(ellipse)
    vessel_in_or_out.append(check)

datafile = datafile.loc[vessel_in_or_out]

datafile = datafile.sort_values(by='timestamp')
datafile["timestamp"] = datafile["timestamp"].astype('datetime64[ns]')
datafile["mmsi"] = datafile["mmsi"].apply(str)


clock_start = datetime(2020, 1, 8, 00, 00, 00, 000)            #Cuando empieza la ruta
clock_lim = datetime(2020, 1, 8, 23, 59, 59, 000)              #Límite de tiempo para hacer la ruta
clock = clock_start


#Crear el mapa
with open('./Recursos/Limitación_geográfica_Elipse/map_ellipse.geojson') as f:                          
    area = json.load(f)
    
areamap = folium.Map(location=[22.333332, 114.1166662])          #Centro del mapa
folium.GeoJson(area).add_to(areamap)


#Ruta a priori
route_default_datafile = pandas.read_excel('./Recursos/MarineTraffic/route_default_HK-Man.xlsx')

route_default_points = route_default_datafile.values.tolist()
route_default = LineString(route_default_points)

route_default_lat, route_default_long = [], []
for i in route_default_points:
    route_default_lat.append(i[0])
    route_default_long.append(i[1])

p1 = pyproj.Proj(proj='latlong', datum='WGS84')
p2 = pyproj.Proj(proj='utm', zone=50, datum='WGS84')    #He puesto todas en la zona 50 para simplificar, error pequeño
e1, n1 = pyproj.transform(p1, p2, route_default_long, route_default_lat) 

route_transformed_points = []
for x1, y1, x2, y2 in zip(route_default_long, route_default_lat, e1, n1):
    route_transformed_points.append([y2,x2])

route_transformed = LineString(route_transformed_points)


def Interpolate_points(geom, distance):
    if geom.geom_type == 'LineString':
        num_vert = int(round(geom.length / distance))
        if num_vert == 0:
            num_vert = 1
        return LineString(
            [geom.interpolate(float(n) / num_vert, normalized=True)
             for n in range(num_vert + 1)])
    elif geom.geom_type == 'MultiLineString':
        parts = [Interpolate_points(part, distance)
                 for part in geom]
        return type(geom)([p for p in parts if not p.is_empty])
    else:
        raise ValueError('unhandled geometry %s', (geom.geom_type,))

step = 100
routee = Interpolate_points(route_transformed, step)     #Obtener un punto de la ruta a cada step metros

routee_y, routee_x = [], []
for i in routee.coords[:]:
    routee_y.append(i[0])
    routee_x.append(i[1])

e2, n2 = pyproj.transform(p2, p1, routee_x, routee_y)  

route_points = []
for x1, y1, x2, y2 in zip(routee_x, routee_y, e2, n2):
    route_points.append([y2,x2])

route = LineString(route_points)


def Create_Circle(C_lat,C_long,circle_r):
    circle_points = []
    circle_n = 360          #Núm. puntos del círculo
    for k in range(circle_n):
        angle = math.pi*2*k/circle_n
        dx = circle_r*math.cos(angle)
        dy = circle_r*math.sin(angle)
        lat = C_lat + (180/math.pi)*(dy/6378137)
        long = C_long + (180/math.pi)*(dx/6378137)/math.cos(C_lat*math.pi/180)
        circle_point = [lat,long]
        circle_points.append(circle_point)
    circle_points.append(circle_points[0])
    circle = Polygon(circle_points)
    return circle

class Vessel:
    def __init__(self, timestamp, mmsi, latitude, longitude, speed, course, heading, point):
        self.timestamp = timestamp
        self.mmsi = mmsi
        self.latitude = latitude
        self.longitude = longitude
        self.speed = speed
        self.course = course
        self.heading = heading
        self.point = point
        

class Ekranoplan:
    def __init__(self, timestamp, latitude, longitude, speed, course, heading, circle):
        self.timestamp = timestamp
        self.latitude = latitude
        self.longitude = longitude
        self.speed = speed
        self.course = course
        self.heading = heading
        self.circle = circle

waveflyer_speed = 250   #km/h
waveflyer_speed_m = waveflyer_speed*1000/3600
height = 4
b = 26.98   #span
def Radius_of_turn():
    bankangle = math.asin(height/(b/2))
    n = 1/math.cos(bankangle)
    r = (waveflyer_speed_m**2)/(9.81*math.sqrt((n**2)-1))
    return r
radius_of_turn = Radius_of_turn()   #Depende de v y z, las cuales se considera que no cambian durante el vuelo
   
waveflyer = Ekranoplan(clock_start, route_points[0][0], route_points[0][1], waveflyer_speed_m, 0, 0, 
                       Create_Circle(route_points[0][0], route_points[0][1], radius_of_turn))


get_clock_end_lim = 1400/waveflyer_speed
clock_end_lim = clock_start + timedelta(hours=get_clock_end_lim)

datafile = datafile.loc[datafile['timestamp'].between(clock_start,clock_end_lim)]

print(datafile)

step_time = step/waveflyer.speed
clock_prev = datetime(2020, 1, 7, 00, 00, 00, 000) 
vessels_register = []
vessels = []
warning, danger = [], []
#for h in range(0,20):
 #   clock_start = clock_start = datetime(2020, 1, 8, h, 00, 00, 000)
  #  print("Departure at", clock_start)
   # clock = clock_start
print('Inicio del vuelo. Salida a las', clock_start)
for i in range(0,len(route_points)):
        waveflyer.latitude, waveflyer.longitude = route_points[i][0], route_points[i][1]
        waveflyer.circle = Create_Circle(route_points[i][0], route_points[i][1], radius_of_turn)
        waveflyer.timestamp = clock
        datafile_slice = datafile.loc[(datafile['timestamp'] > clock_prev) & (datafile['timestamp'] <= clock)]
        
        for j,row in datafile_slice.iterrows():
            if row.mmsi in vessels_register:
                idx = vessels_register.index(row.mmsi)
                vessels[idx].timestamp = row.timestamp
                vessels[idx].latitude = row.latitude
                vessels[idx].longitude = row.longitude
                vessels[idx].speed = row.speed
                vessels[idx].course = row.course
                vessels[idx].heading = row.heading
                vessels[idx].point = Point(row.latitude, row.longitude)
            else:
                vessels.append(Vessel(row.timestamp,row.mmsi,row.latitude,row.longitude,
                                        row.speed,row.course,row.heading,Point(row.latitude,row.longitude)))
                vessels_register.append(row.mmsi)
                idx = vessels_register.index(row.mmsi)
            
            #Situaciones de aviso    
            if vessels[idx].point.within(waveflyer.circle):
                warning.append(print('warning',row.mmsi,'is around','(',waveflyer.latitude,',',
                                    waveflyer.longitude,') at', waveflyer.timestamp))
                folium.CircleMarker((row.latitude,row.longitude), radius=3, weight=2, 
                        tooltip=(row.timestamp,row.mmsi,row.speed,row.course,row.heading)).add_to(areamap)
                folium.CircleMarker((waveflyer.latitude, waveflyer.longitude), radius=3, weight=2,
                    tooltip=(waveflyer.timestamp,waveflyer.speed,waveflyer.course,waveflyer.heading), 
                    color='green').add_to(areamap)
                folium.PolyLine(waveflyer.circle.exterior.coords[:], color='red').add_to(areamap)
                
                #Cálculo del rumbo del ekranoplano
                latB, longB = math.radians(route_points[i-1][0]), math.radians(route_points[i-1][1])
                latA, longA = math.radians(route_points[i][0]), math.radians(route_points[i][1])
                dLong = longA-longB
                y = math.sin(dLong) * math.cos(latA);
                x = math.cos(latB) * math.sin(latA) - math.sin(latB)* math.cos(latA) * math.cos(dLong)
                waveflyer.course = math.atan2(y, x)
                waveflyer.course = math.degrees(waveflyer.course)
                waveflyer.course = (waveflyer.course + 360) % 360
                
                #Maniobra parte 1
                AB = LineString([route_points[i-1], route_points[i]])
                left = AB.parallel_offset(radius_of_turn/100000, 'left')
                right = AB.parallel_offset(radius_of_turn/100000, 'right')
                C = left.boundary[1]        #el 1 es para coger un sentido
                D = right.boundary[0]       #y el 0 el otro sentido
                CD = LineString([C, D])     #vector perpendicular a AB
                coords_aux_CD_ = list(CD.intersection(waveflyer.circle).coords[:])
                coords_aux_CD = [list(elem) for elem in coords_aux_CD_]
                if vessels[idx].course >= (waveflyer.course+180) or vessels[idx].course <= waveflyer.course:   #Elegir si el ekranoplano se va hacia der. o izq. según la dirección del barco
                    idx_CD = 0          #derecha del ekranoplano
                else:
                    idx_CD = 1          #izquierda del ekranoplano
                lat1 = coords_aux_CD[idx_CD][0]
                long1 = coords_aux_CD[idx_CD][1]
                m1_ = Create_Circle(lat1, long1, radius_of_turn)
                
                #Maniobra parte 2
                waveflyer.course = math.radians(waveflyer.course)
                lat1 = math.radians(lat1)
                long1 = math.radians(long1)
                lat2 = math.asin(math.sin(lat1)*math.cos(2*radius_of_turn/6378137) + 
                                 math.cos(lat1)*math.sin(2*radius_of_turn/6378137)*math.cos(waveflyer.course))
                long2 = long1 + math.atan2(math.sin(waveflyer.course)*math.sin(2*radius_of_turn/6378137)*math.cos(lat1),
                                math.cos(2*radius_of_turn/6378137)-math.sin(lat1)*math.sin(lat2))
                lat2 = math.degrees(lat2)
                long2 = math.degrees(long2)
                m2_ = Create_Circle(lat2, long2, radius_of_turn)
                
                #Maniobra parte 3
                lat2 = math.radians(lat2)
                long2 = math.radians(long2)
                lat3 = math.asin(math.sin(lat2)*math.cos(2*radius_of_turn/6378137) + 
                                 math.cos(lat2)*math.sin(2*radius_of_turn/6378137)*math.cos(waveflyer.course))
                long3 = long2 + math.atan2(math.sin(waveflyer.course)*math.sin(2*radius_of_turn/6378137)*math.cos(lat2),
                                math.cos(2*radius_of_turn/6378137)-math.sin(lat2)*math.sin(lat3))
                lat3 = math.degrees(lat3)
                long3 = math.degrees(long3)
                m3_ = Create_Circle(lat3, long3, radius_of_turn)
                
                #Intersecciones
                inters1__ = m1_.exterior.intersection(route)
                inters1_ = [[p.x, p.y] for p in inters1__]
                inters1 = inters1_[0]
                
                inters2lat = math.asin(math.sin(lat1)*math.cos(radius_of_turn/6378137) + 
                                 math.cos(lat1)*math.sin(radius_of_turn/6378137)*math.cos(waveflyer.course))
                inters2long = long1 + math.atan2(math.sin(waveflyer.course)*math.sin(radius_of_turn/6378137)*math.cos(lat1),
                                math.cos(radius_of_turn/6378137)-math.sin(lat1)*math.sin(inters2lat))
                inters2lat = math.degrees(inters2lat)
                inters2long = math.degrees(inters2long)
                inters2 = [inters2lat,inters2long]
                
                inters3lat = math.asin(math.sin(lat2)*math.cos(radius_of_turn/6378137) + 
                                 math.cos(lat2)*math.sin(radius_of_turn/6378137)*math.cos(waveflyer.course))
                inters3long = long2 + math.atan2(math.sin(waveflyer.course)*math.sin(radius_of_turn/6378137)*math.cos(lat2),
                                math.cos(radius_of_turn/6378137)-math.sin(lat2)*math.sin(inters3lat))
                inters3lat = math.degrees(inters3lat)
                inters3long = math.degrees(inters3long)
                inters3 = [inters3lat,inters3long]
                
                inters4__ = m3_.exterior.intersection(route)
                inters4_ = [[p.x, p.y] for p in inters4__]
                inters4 = inters4_[0]
                
                lat1, long1 = math.degrees(lat1), math.degrees(long1)
                lat2, long2 = math.degrees(lat2), math.degrees(long2)
                m1, m2_1, m2_2, m3 = [], [], [], []
                for k in m1_.exterior.coords[:]:
                    if idx_CD == 1:
                        if (k[0] < lat1) and (inters1[1] < k[1] < inters2[1]):
                            m1.append(k)
                    else:
                        if (inters1[0] > k[0] > inters2[0]) and (k[1] > long1):
                            m1.append(k)

                for k in m2_.exterior.coords[:]:
                    if (k[0] > lat2) and (inters2[1] < k[1] < inters3[1]):
                        m2_1.append(k)
                    if (inters2[0] > k[0] > inters3[0]) and (k[1] > long2):
                        m2_2.append(k)
                m2_2.sort()

                for k in m3_.exterior.coords[:]:
                    if idx_CD == 1:
                        if (inters3[0] > k[0] > inters4[0]) and (k[1] < long3):
                            m3.append(k)
                    else:
                        if (k[0] > lat3) and (inters3[1] < k[1] < inters4[1]):
                            m3.append(k)

                #Eliminar los puntos de la ruta que ya no forman parte de ella
                route_points = [k for k in route_points if (k[0]>inters1[0] or k[0]<inters4[0])]
                
                #Incluir los nuevos puntos de la ruta
                m1 = [list(elem) for elem in m1]
                m2_1 = [list(elem) for elem in m2_1]
                m2_2 = [list(elem) for elem in m2_2]
                m3 = [list(elem) for elem in m3]
                counter = 1
                for k in m1:
                    route_points.insert(i + counter, k)
                    counter += 1
                for k in reversed(m2_1):
                    route_points.insert(i + counter, k)
                    counter += 1
                for k in reversed(m2_2):
                    route_points.insert(i + counter, k)
                    counter += 1
                for k in m3:
                    route_points.insert(i + counter, k)
                    counter += 1

                #Situaciones de peligro. Se requiere maniobra para evitar la colisión
                dangerangle = 45        #ángulo del triángulo en frente del ekranoplano que define la zona de peligro
                if (vessels[idx].course <= waveflyer.course+180+dangerangle) and (vessels[idx].course >= waveflyer.course+180-dangerangle):
                    danger.append(print('danger, possible collision',row.mmsi,'is in the way of','(',waveflyer.latitude,
                                              ',',waveflyer.longitude,') at',waveflyer.timestamp))
                    
        clock_prev = clock
        clock += timedelta(seconds=step_time)
        if clock > clock_lim:
            print("You could not reach your destination in time")
            break

route = LineString(route_points)
folium.PolyLine(route_points, color='green').add_to(areamap)
areamap.save('route_collision_avoidance_copia.html')

route_lat, route_long = [], []
for i in route.coords[:]:
    route_lat.append(i[0])
    route_long.append(i[1])
e3, n3 = pyproj.transform(p1, p2, route_long, route_lat) 

route_xy_points = []
for x1, y1, x2, y2 in zip(route_long, route_lat, e3, n3):
    route_xy_points.append([y2,x2])

route_xy = LineString(route_xy_points)

clock_end = clock_start + timedelta(seconds=route_xy.length/waveflyer.speed)
print('Se ha llegado al destino a las', clock_end)
print('Se ha recorrido una distancia de', route_xy.length, 'm')