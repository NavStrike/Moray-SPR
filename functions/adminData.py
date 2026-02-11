import json
import os
from pathlib import Path
import pandas as pd
import numpy as np
from scipy.signal import find_peaks, peak_widths
from scipy.ndimage import median_filter, uniform_filter

import matplotlib.pyplot as plt

from functions.process import analizeData, equalizeMin

class accessData():
    def __init__(self,  fileName='config.json'):
        # Establece el directorio de los archivos .json
        self.root = Path(__file__).resolve().parent.parent
        self.dir = Path(f'{self.root}/data/{fileName}').expanduser()
        os.makedirs(os.path.dirname(self.dir), exist_ok=True)
        self.dir.parent.mkdir(parents=True, exist_ok=True)

        self.data = None
        self.charge()

    def charge(self):
        #Carga los datos desde el archivo si existe
        if self.dir.exists():
            with open(self.dir, 'r') as f:
                self.data = json.load(f)
                print(f"Información cargada desde {self.dir}")

    def update(self):
        #Guarda el estado actual del diccionario en el archivo JSON
        with open(self.dir, 'w') as f:
            json.dump(self.data, f, indent=4)
        print("Datos guardados correctamente.")

    def changeAngles(self, new_angles):
        #Actualiza solo los ángulos
        self.data["angMin"] = new_angles[0]
        self.data["angMax"] = new_angles[1]
        self.update()

class accessCsv():
    def __init__(self):
        self.root = Path(__file__).resolve().parent.parent
        self.dirGeneral = Path(f'{self.root}/exports/datageneral').expanduser()
        self.dirCycles = Path(f'{self.root}/exports/angulosvsciclo').expanduser()
        self.f_dataGeneral = []
        self.f_angVsCiclos = []
        self.addFiles()

    def addFiles(self):
        dir1 = self.dirGeneral
        dir2 = self.dirCycles
        if dir1.exists() and dir1.is_dir():
            self.f_dataGeneral = [f.name for f in dir1.iterdir() if f.is_file()]
        if dir2.exists() and dir2.is_dir():
            self.f_angVsCiclos = [f.name for f in dir2.iterdir() if f.is_file()]
    
    def viewCsv(self, n = None):
        if n != None and type(n) == int:
            return self.f_dataGeneral[n-1], self.f_angVsCiclos[n-1]
        else:
            return self.f_dataGeneral, self.f_angVsCiclos

    def numberCsv(self):
        # print(f'Hay {len(self.f_angVsCiclos)} archivos:')
        return len(self.f_dataGeneral), len(self.f_angVsCiclos)
    
    def viewData(self, n, filter = 'None', kernel = 5):
        self.n = n
        self.arch = Path(f'{self.root}/exports/datageneral/{self.f_dataGeneral[n-1]}').expanduser()
        self.df = np.genfromtxt(self.arch, delimiter=",", skip_header=1)

        cycles = []
        bkpoints = np.where(np.diff(self.df[:,0]) < 0)[0] + 1

        if len(bkpoints) > 0:
            bkpoints = np.insert(bkpoints, 0, 0)
            bkpoints = np.insert(bkpoints, len(bkpoints), len(bkpoints))
            for i in range(len(bkpoints)-1):
                cycle = self.df[bkpoints[i]:bkpoints[i+1]]
                if len(cycle) > 5:
                    cycles.append(cycle)
        else:
            cycles.append(self.df)

        self.nCycles = len(cycles)
        print(f'Hay un total de {self.nCycles} ciclos')

        self.x = [a[:,0] for a in cycles]
        self.y1 = [a[:,1] for a in cycles]
        self.y2 = [a[:,2] for a in cycles]

        # Aplicacion de un filtro
        if filter == 'median' or filter == None:
            for i in range(self.nCycles):
                self.y1[i] = median_filter(self.y1[i], size=kernel, mode="reflect")
                self.y2[i] = median_filter(self.y2[i], size=kernel, mode="reflect")
        elif filter == 'average':
            for i in range(self.nCycles):
                self.y1[i] = uniform_filter(self.y1[i], size=kernel, mode="reflect")
                self.y2[i] = uniform_filter(self.y2[i], size=kernel, mode="reflect")

        return self.x, self.y1, self.y2
    
    def viewCycle(self, n_signal=None, listAd=None):
        x = self.x
        y1 = self.y1
        y2 = self.y2
        n = n_signal

        plt.figure(figsize=(6, 4))
        plt.xlabel("Angulo (deg)")
        plt.ylabel("Resistencia (KOhms)")
        
        an1 = analizeData(x[n-1], y1[n-1])
        an2 = analizeData(x[n-1], y2[n-1])

        peaks1 = an1.PointPeaks()
        peaks2 = an2.PointPeaks()

        peak_widths1 = an1.WidthMaxPeaks()
        peak_widths2 = an2.WidthMaxPeaks()

        q1 = an1.quality()
        q2 = an2.quality()

        # Ploteo de las curvas
        plt.plot(x[n-1], y1[n-1], marker='.', linestyle='-', color='blue', label=f'Canal 1: q = {q1}')
        plt.plot(x[n-1], y2[n-1], marker='.', linestyle='-', color='green', label=f'Canal 2: q = {q2}')

        # Ploteo de los picos
        plt.plot(peaks1[0], peaks1[1], marker='*', linestyle='-', color='red')
        plt.plot(peaks2[0], peaks2[1], marker='*', linestyle='-', color='red')

        # ploteo del ancho del pico
        plt.plot(peak_widths1[0], peak_widths1[1], marker='*', linestyle='-', color='red')
        plt.plot(peak_widths2[0], peak_widths2[1], marker='*', linestyle='-', color='red')

        plt.legend(
            loc="upper right",      # Posición
            fontsize="small",       # Tamaño de letra
            shadow=True,            # Sombra decorativa
            title=f"Curva SPR"      # Título de la cajita de leyenda
        )
        
        plt.title(f"Grafica {self.f_dataGeneral[self.n-1]}")
        plt.grid(True)

        plt.show()
    
    def viewCycles(self, cut_start=None, cut_end=None, listAd=None):  
        x = self.x
        y1 = self.y1
        y2 = self.y2

        if cut_start == None or type(cut_start) != int: cut_start = 0
        if cut_end == None or type(cut_end) != int: cut_end = self.nCycles

        ran = [cut_start, cut_end]

        plt.figure(figsize=(6, 4))
        plt.xlabel("Angulo (deg)")
        plt.ylabel("Resistencia (KOhms)")

        cmap1 = plt.cm.plasma
        cmap2 = plt.cm.viridis
        # cmapPeaks = plt.cm.gray

        listPeaks1 = []
        listPeaks2 = []

        q1s = []
        q2s = []

        plt.annotate('Ch 1', xy=(x[-1][1], y1[-1][1]), xytext=(x[-1][1], y1[-1][1]), color = 'black')
        plt.annotate('Ch 2', xy=(x[-1][1], y2[-1][1]), xytext=(x[-1][1], y2[-1][1]), color = 'blue')

        for i in range(ran[0], ran[1]):
            color1 = cmap1(i / self.nCycles)
            color2 = cmap2(i / self.nCycles)

            try:
                y1_i = equalizeMin(y1[i], y2[i]).newCurve()

                an1 = analizeData(x[i], y1_i)
                an2 = analizeData(x[i], y2[i])

                q1s.append(an1.quality())
                q2s.append(an2.quality())

                listPeaks1.append(an1.PointPeaks())
                listPeaks2.append(an2.PointPeaks())

                # Ploteo de las curvas
                plt.plot(x[i], y1_i, marker='.', linestyle='-', color=color1)
                plt.plot(x[i], y2[i], marker='.', linestyle='-', color=color2)
            except:
                print(f'El ciclo {i+1} no pudo graficarse')
        
        for n, p1, p2 in zip(range(len(listPeaks2)), listPeaks1, listPeaks2):   
            #colorPeaks = cmapPeaks(n / self.nCycles)
            colorPeaks = 'red'
            # Ploteo de los picos
            plt.plot(p1[0], p1[1], marker='*', linestyle='-', color=colorPeaks)
            plt.plot(p2[0], p2[1], marker='*', linestyle='-', color=colorPeaks)

        tit=f"Curvas SPR: \nq1 = [{min(q1s)} - {max(q1s)}] \nq2 = [{min(q2s)} - {max(q2s)}]"

        plt.legend(
            loc="upper right",      # Posición
            fontsize="small",       # Tamaño de letra
            shadow=True,            # Sombra decorativa
            title=tit               # Título de la leyenda
        )
        
        if listAd != None:
            for _list in listAd:
                plt.plot(_list[0], _list[1], marker='.', linestyle='-', color='black')

        plt.show()