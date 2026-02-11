import configparser
import os

# Definir la ruta (Carpeta oculta en el home del usuario)
ruta_config = os.path.expanduser('mi_aplicacion.conf')
config = configparser.ConfigParser()

# 1. GUARDAR AJUSTES
config['USER'] = {'nombre': 'Gustavo', 'tema': 'oscuro', 'volumen': '80'}
with open(ruta_config, 'w') as f:
    config.write(f)

# 2. LEER AJUSTES
config.read(ruta_config)
tema_actual = config['USER']['tema']