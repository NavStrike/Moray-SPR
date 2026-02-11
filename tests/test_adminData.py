from functions.adminData import accessData

# Creacion de un punto de acceso
pointAccess = accessData("config.json")
# Ingreso de los nuevos angulos
pointAccess.changeAngles([70,90])
# Volcado del JSON en una variable
infoJson = pointAccess.data
# Impresion del JSON
print(infoJson)