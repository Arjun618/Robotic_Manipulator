import csv
from math import sin,cos
F=open("myfile.csv","r")
csvobject=csv.reader(F)
print("P1 | P2 | P3")
records=[]
for i in csvobject:
    Q1=float(i[0])
    Q2=float(i[1])
    Q3=float(i[2])
    Q4=float(i[3])
    Px = cos(Q1)*(0.126*cos(Q2+Q3+Q4) + 0.124*cos(Q2+Q3) + 0.13*cos(Q2))
    Py= sin(Q1)*(0.126*cos(Q2+Q3+Q4) + 0.124*cos(Q2+Q3) + 0.13*cos(Q2))
    Pz = 0.077 + 0.126*sin(Q2+Q3+Q4) + 0.124*sin(Q2+Q3) + 0.13*sin(Q2)
    F=open('position.csv','w',newline='')
    csv_writer = csv.writer(F)
    L=[round(Px,4),round(Py,4),round(Pz,4)]
    records.append(L)
csv_writer.writerows(records)
