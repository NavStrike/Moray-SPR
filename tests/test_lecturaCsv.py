from functions.adminData import accessCsv
from functions.process import analizeData

# creacion de un punto de acceso a la informacion de los archivos csv
p = accessCsv()

# Numeracion de los archivos csv
n1, n2 = p.numberCsv()
print(f'Cantidad de archivos csv almacenados: {n1} y {n2}')

# Ver los archivos csv que se encontro en dicha posicion
a1, a2 = p.viewCsv(2)
print(f'Archivos seleccionados: {a1} y {a2}')

# acceso a la informacion del archivo
x, y1, y2 = p.viewData(2, filter = 'median', kernel=5)

# Analisis de la informacion (calidad de las curvas)
m = analizeData(x[1], y1[1]).quality()
n = analizeData(x[1], y2[1]).quality()

# Impresion de la imagen
p.viewCycles()

