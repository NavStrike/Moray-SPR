# Analisis del efecto de la distancia del laser sobre los resultados
# MorayTech - 11/02/2026
# Archivo analizado: 20260211_142305_datageneral.csv

from functions.adminData import accessCsv

# creacion de un punto de acceso a la informacion de los archivos csv
p = accessCsv()

# Numeracion de los archivos csv
n1, n2 = p.numberCsv()
print(f'Cantidad de archivos csv almacenados: {n1} y {n2}')

### VER DATOS SIN PBS ###
# Ver los archivos csv que se encontro en dicha posicion
a1, a2 = p.viewCsv(5)
print(f'Archivos seleccionados: {a1} y {a2}')

# acceso a la informacion del archivo
p.viewData(5, filter = 'median', kernel=5)

# Impresion de la imagen
p.viewCycles(40, 100)