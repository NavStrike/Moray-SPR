import platform
import os
import subprocess


# Dirección de  partida
dir = os.getcwd()
dir = dir.replace('\\', '/')

# Ejecución de comandos necesarios funcionamiento
OS = platform.system()
print(OS)

if OS == 'Windows':
    command = "dir && .\\entornoGUI\\Scripts\\activate.bat && python main.py"
    subprocess.run(command, shell=True)

elif OS == 'Linux':
    comando = "cd /home/pi/spr && ls && source ./venv/bin/activate && python ./main.py"
    subprocess.run(comando, shell=True, executable="/bin/bash")

print("Lanzador finalizado")