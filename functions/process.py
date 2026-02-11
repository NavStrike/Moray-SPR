import numpy as np
from scipy.signal import find_peaks, peak_widths

class analizeData():
    def __init__(self, x, y):
        self.x = x.tolist()
        self.y = y.tolist()
        # Calculo de picos
        # Esto devuelve: los indices de ocurrencia y los valores de los picos
        self.posPeaks, self.valuePeaks = find_peaks(y, height=min(y), distance=len(x))
        self.posPeaks = self.posPeaks.tolist()
        self.heightPeaks = self.valuePeaks['peak_heights'].tolist()
        # Calculo del ancho de los picos
        # Esto devuelve: [anchos, alturas_donde_se_midió, izquierda, derecha]
        self.infoPeaks = peak_widths(y, self.posPeaks, rel_height=0.5)
        # Se busca el mayor pico
        self.maxHeightPeak = max(self.heightPeaks)
        # Se encuentra el punto de ocurrencia del mayor pico
        self.indMaxPeak = self.heightPeaks.index(self.maxHeightPeak)
        self.posMaxPeak = self.posPeaks[self.indMaxPeak]
        # Informacion de los picos maximos
        self.withPeaks = self.infoPeaks[0].tolist()
        self.halfPeaks = self.infoPeaks[1].tolist()
        self.infPeaks = self.infoPeaks[2].tolist()
        self.supPeaks = self.infoPeaks[3].tolist()

        # print('***')
        # print(f'Los picos se encontraron en las posiciones: {self.posPeaks}')
        # print(f'La altura de los picos son: {self.heightPeaks}')
        # print('***')
        # print(f'La informacion obtenida de los picos es:')
        # print(f'Los anchos de los picos son: {self.withPeaks}')
        # print(f'Los valores medios de los picos (alturas medias) son: {self.halfPeaks}')
        # print(f'Los extremos inferiores de los picos son: {self.infPeaks}')
        # print(f'LLos extremos superiores de los picos son: {self.supPeaks}')
        # print('***')
        
        # Se escalan los anchos de los picos a un valor angular
        self.truWidthPeaks = []
        self.infAngPeaks = []
        self.supAngPeaks = []
        
        for a, b in zip(self.infPeaks, self.supPeaks):
            anglesWidthPeaks = np.interp([a, b], np.arange(1, len(self.x)+1), self.x).tolist()
            truWidthPeak = anglesWidthPeaks[1] - anglesWidthPeaks[0]
            self.truWidthPeaks.append(truWidthPeak)
            self.infAngPeaks.append(anglesWidthPeaks[0])
            self.supAngPeaks.append(anglesWidthPeaks[1])

        # print('***')
        # print(f'Los anchos reales de los picos son: {self.truWidthPeaks}')
        # print(f'Los extremos angulares inferiores son: {self.infAngPeaks}')
        # print(f'Los extremos angulares superiores son: {self.supAngPeaks}')
        # print('***')

        self.truWidthMaxPeak = self.truWidthPeaks[self.indMaxPeak]

        self._quality = (self.maxHeightPeak - min(self.y[0:self.posMaxPeak]))/self.truWidthMaxPeak

        #print(f'Indice calidad: {self.quality}')

    def PointPeaks(self):
        self.xPeaks = [self.x[i] for i in self.posPeaks]
        self.yPeaks = [self.y[i] for i in self.posPeaks]
        return [self.xPeaks, self.yPeaks]
    
    def WidthMaxPeaks(self):
        self.limPeaks = self.infAngPeaks + self.supAngPeaks
        self._2XhalfPeaks = [x for x in self.halfPeaks for i in range(2)]
        return [self.limPeaks, self._2XhalfPeaks]

    def quality(self):
        return round(self._quality, 2)
    
class equalizeMin():
    def __init__(self, y1 = None, y2 = None):
        self.y1, self.y2 = y1, y2

        self.posPeak1, _ = find_peaks(self.y1, height=min(self.y1), distance=len(self.y1))
        self.posPeak2, _ = find_peaks(self.y2, height=min(self.y2), distance=len(self.y2))

        # minimos izquierda:
        ha1, ha2 = min(self.y1[ : self.posPeak1[0]]), min(self.y2[ : self.posPeak2[0]])
        hc1, hc2 = min(self.y1[self.posPeak1[0]+1: ]), min(self.y2[self.posPeak2[0]+1: ])

        # minimos derecha:

        # Relacion Lineal entre las curvas (se considera la curva 1: x y la curva 2 : y)
        # y = ax + b
        # Valores sabidos -> x = ha1 // y = ha2 (minimos izquierda)
        #                    x = hc1 // y = hc2 (minimos derecha)
        # Resultado => a = (hc2 - ha2)/(hc1 - ha1)
        #              b = ha2 - a*ha1

        a = (hc2 - ha2)/(hc1 - ha1)
        b = ha2 - a*ha1

        # Escalamiento de la curva 1 (x) para coincidir con la curva 2 (y):
        if type(self.y1) == list: self.y1 = np.array(self.y1)
        if type(self.y2) == list: self.y2 = np.array(self.y2)

        self.y1 = a*self.y1 + b

    def newCurve(self):
        return self.y1