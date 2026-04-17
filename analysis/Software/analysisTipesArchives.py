# Evaluación del tamaño ocupado por los diferentes formatos de archivos:
# Fecha: 15/04/2026
# Autor: Gustavo Rafael Perez Garcia

from matplotlib import pyplot as plt
import pandas as pd

sizeFiles = {
    "level": ["nulo", "bajo", "alto"],
    "csv": [76.678, 31.804, 31.784],
    "h5": [31.636, 30.849, 27.977],
    "parquet": [16.463, 15.601, 15.557]
}

table = pd.DataFrame(sizeFiles)

# Gráfica de los resultados
graph = plt.figure(figsize=(12,6))
# Ploteo de las gráficas
plt.plot(table["level"], table["csv"], label="csv")
plt.plot(table["level"], table["h5"], label = "h5")
plt.plot(table["level"], table["parquet"], label = "parquet")
plt.scatter(table["level"], table["csv"])
plt.scatter(table["level"], table["h5"])
plt.scatter(table["level"], table["parquet"])
# Nombres de los ejes y leyenda
plt.xlabel('Nivel de compresión')
plt.ylabel('Tamaño (MB)')
plt.title("Tipos de archivos")
plt.legend()
plt.show()
