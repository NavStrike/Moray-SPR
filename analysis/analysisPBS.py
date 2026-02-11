from functions.adminData import accessCsv

# creacion de un punto de acceso a la informacion de los archivos csv
p = accessCsv()

# Numeracion de los archivos csv
n1, n2 = p.numberCsv()
print(f'Cantidad de archivos csv almacenados: {n1} y {n2}')

### VER DATOS SIN PBS ###
print("****************")
print("DATOS SIN PBS ")
print("****************")
# Ver los archivos csv que se encontro en dicha posicion
a1, a2 = p.viewCsv(2)
print(f'Archivos seleccionados: {a1} y {a2}')

# acceso a la informacion del archivo
x, y1, y2 = p.viewData(2, filter = 'median', kernel=5)

# Impresion de la imagen
p.viewCycles()

print("****************")

### VER DATOS CON PBS ###
print("****************")
print("DATOS CON PBS ")
print("****************")

# creacion de un punto de acceso a la informacion de los archivos csv
p2 = accessCsv()

# Ver los archivos csv que se encontro en dicha posicion
a1, a2 = p2.viewCsv(4)
print(f'Archivos seleccionados: {a1} y {a2}')

# acceso a la informacion del archivo
x, y1, y2 = p2.viewData(4, filter = 'median', kernel=5)

# Impresion de la imagen
p2.viewCycles()