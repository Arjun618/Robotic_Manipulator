import csv
from math import sin,cos
F1=open("ogpoint.csv","r")
F2=open("position.csv","r")
csvobject1=csv.reader(F1)
csvobject2=csv.reader(F2)
records=[]
for i,j in zip(csvobject1,csvobject2):
   Zx=((float(j[0])-float(i[0]))/(float(i[0])))*100
   Zy=((float(j[1])-float(i[1]))/(float(i[1])))*100
   Zz=((float(j[2])-float(i[2]))/(float(i[2])))*100
   L=[round(Zx,4),round(Zy,4),round(Zz,4)]
   F3=open("error.csv","w",newline='')
   csvobject3=csv.writer(F3)
   records.append(L)
csvobject3.writerows(records)
