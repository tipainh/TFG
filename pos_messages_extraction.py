# -*- coding: utf-8 -*-
"""
Created on Thu Apr 23 17:51:08 2020

@author: TiagoFilipe
"""

import pandas

datafile = pandas.read_csv('spire_south_china_sea_8jan2020.csv', index_col = 'created_at')


pos_messages_auxA = datafile[datafile.msg_type != 5]                         #Elimina mensajes estáticos A
pos_messages_auxB = pos_messages_auxA[pos_messages_auxA.msg_type != 24]      #Elimina mensajes estáticos B
pos_messages = pos_messages_auxB[pos_messages_auxB.msg_type != 4]            #Elimina mensajes estación base

print(datafile)
print("Number of AIS position messages in the area of interest: {0}".format(len(pos_messages)))

pos_messages.to_csv('pos_messages.csv')