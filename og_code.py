import csv
from math import sin,cos
F=open("ogpoint.csv","w",newline='')
csvobject=csv.writer(F)
records = [
    [0.2, 0.1000, 0.1100],
    [0.2, 0.0914, 0.1507],
    [0.2, 0.0669, 0.1843],
    [0.2, 0.0309, 0.2051],
    [0.2, -0.0105, 0.2095],
    [0.2, -0.0500, 0.1966],
    [0.2, -0.0809, 0.1688],
    [0.2, -0.0978, 0.1308],
    [0.2, -0.0978, 0.0892],
    [0.2, -0.0809, 0.0512],
    [0.2, -0.0500, 0.0234],
    [0.2, -0.0105, 0.0105],
    [0.2, 0.0309, 0.0149],
    [0.2, 0.0669, 0.0357],
    [0.2, 0.0914, 0.0693],
    [0.2, 0.1000, 0.1100]
]

csvobject.writerows(records)

    