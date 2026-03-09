// ===== IMPORTS =====
import QtQuick 6.0
import QtQuick.Controls 6.0
import QtQuick.Layouts 6.0
import QtQuick.Window 6.0
import QtQuick.Dialogs 6.0

Page {

    // ===== Processing =====
    property var data_p: []
    property int nCiclos_p: 0
    property var angle_p: []
    property var ch1_p: []
    property var ch2_p: []

    // Ángulos de resonancia
    property var resonance_angles_ch1: []
    property var resonance_angles_ch2: []
    property var resonance_angles_ch1_gaussian: []
    property var resonance_angles_ch2_gaussian: []
    property var resonance_angles_ch1_fit: []
    property var resonance_angles_ch2_fit: []

    // Parámetros de ajuste
    property real rango_angulos: 1.75
    property int grado_polinomio: 4
    property int puntos_minimos: grado_polinomio + 1

    // Resultados
    property var desviacion_estandar: []
    property real sensibility: 80
    property var detection_limit: []

    // Datos procesados por ciclo
    property var unique_angles: []
    property var avg_ch1_mean: []
    property var avg_ch2_mean: []
    property var avg_ch1_median: []
    property var avg_ch2_median: []
    property var avg_ch1_Filtro_Gaussiano_s2: []
    property var avg_ch2_Filtro_Gaussiano_s2: []
    property var avg_ch1_Media_Movil5: []
    property var avg_ch2_Media_Movil5: []

    id: processingWin
    visible: win.viewPage === "process"
    title: "Procesamiento"
    Layout.fillWidth: true
    Layout.fillHeight: true

    // ===== FUNCIONES DE UTILIDAD =====

    function unique(arr, options = {}) {
        const { stable = true } = options;
        if (stable) {
            return [...new Set(arr)];
        } else {
            return Array.from(new Set(arr)).sort((a, b) => a - b);
        }
    }

    function findIndices(arr, condition) {
        let indices = [];
        for (let i = 0; i < arr.length; i++) {
            if (condition(arr[i], i, arr)) {
                indices.push(i);
            }
        }
        return indices;
    }

    function mean(arr) {
        if (arr.length === 0) return 0;
        const sum = arr.reduce((a, b) => a + b, 0);
        return sum / arr.length;
    }

    function median(arr) {
        if (arr.length === 0) return 0;
        const sorted = [...arr].sort((a, b) => a - b);
        const mid = Math.floor(sorted.length / 2);
        return sorted.length % 2 === 0 ? (sorted[mid - 1] + sorted[mid]) / 2 : sorted[mid];
    }

    function movmean(arr, windowSize) {
        const result = [];
        const half = Math.floor(windowSize / 2);
        
        for (let i = 0; i < arr.length; i++) {
            let start = Math.max(0, i - half);
            let end = Math.min(arr.length - 1, i + half);
            let window = arr.slice(start, end + 1);
            result.push(mean(window));
        }
        return result;
    }

    function movmedian(arr, windowSize) {
        const result = [];
        const half = Math.floor(windowSize / 2);
        
        for (let i = 0; i < arr.length; i++) {
            let start = Math.max(0, i - half);
            let end = Math.min(arr.length - 1, i + half);
            let window = arr.slice(start, end + 1);
            result.push(median(window));
        }
        return result;
    }

    function gausswin(length, sigma) {
        const window = [];
        const center = (length - 1) / 2;
        
        for (let i = 0; i < length; i++) {
            const x = (i - center) / sigma;
            window.push(Math.exp(-0.5 * x * x));
        }
        return window;
    }

    function conv(signal, kernel, mode = 'same') {
        const result = [];
        const kLen = kernel.length;
        const half = Math.floor(kLen / 2);
        
        for (let i = 0; i < signal.length; i++) {
            let sum = 0;
            for (let j = 0; j < kLen; j++) {
                const idx = i - half + j;
                if (idx >= 0 && idx < signal.length) {
                    sum += signal[idx] * kernel[j];
                }
            }
            result.push(sum);
        }
        return result;
    }

    // Función para ajuste polinomial (mínimos cuadrados)
    function polyfit(x, y, degree) {
        const n = x.length;
        const m = degree + 1;
        
        // Construir matriz de Vandermonde
        const A = [];
        for (let i = 0; i < n; i++) {
            const row = [];
            for (let j = 0; j < m; j++) {
                row.push(Math.pow(x[i], j));
            }
            A.push(row);
        }
        
        // Resolver usando pseudoinversa: (A^T * A) * p = A^T * y
        const AtA = [];
        const AtY = [];
        
        for (let i = 0; i < m; i++) {
            AtA[i] = [];
            for (let j = 0; j < m; j++) {
                let sum = 0;
                for (let k = 0; k < n; k++) {
                    sum += A[k][i] * A[k][j];
                }
                AtA[i][j] = sum;
            }
        }
        
        for (let i = 0; i < m; i++) {
            let sum = 0;
            for (let k = 0; k < n; k++) {
                sum += A[k][i] * y[k];
            }
            AtY[i] = sum;
        }
        
        // Resolver sistema lineal (eliminación gaussiana simplificada)
        const coeffs = solveLinearSystem(AtA, AtY);
        return coeffs;
    }

    function solveLinearSystem(A, b) {
        const n = b.length;
        // Crear matriz aumentada
        const augmented = [];
        for (let i = 0; i < n; i++) {
            augmented[i] = [...A[i], b[i]];
        }
        
        // Eliminación gaussiana
        for (let i = 0; i < n; i++) {
            // Pivoteo
            let maxRow = i;
            for (let k = i + 1; k < n; k++) {
                if (Math.abs(augmented[k][i]) > Math.abs(augmented[maxRow][i])) {
                    maxRow = k;
                }
            }
            [augmented[i], augmented[maxRow]] = [augmented[maxRow], augmented[i]];
            
            // Hacer ceros debajo del pivote
            for (let k = i + 1; k < n; k++) {
                const factor = augmented[k][i] / augmented[i][i];
                for (let j = i; j <= n; j++) {
                    augmented[k][j] -= factor * augmented[i][j];
                }
            }
        }
        
        // Sustitución hacia atrás
        const x = new Array(n).fill(0);
        for (let i = n - 1; i >= 0; i--) {
            x[i] = augmented[i][n];
            for (let j = i + 1; j < n; j++) {
                x[i] -= augmented[i][j] * x[j];
            }
            x[i] /= augmented[i][i];
        }
        return x;
    }

    function polyval(coeffs, x) {
        // coeffs: [a0, a1, a2, a3, a4] para grado 4
        let result = 0;
        for (let i = 0; i < coeffs.length; i++) {
            result += coeffs[i] * Math.pow(x, i);
        }
        return result;
    }

    function linspace(start, end, n) {
        const step = (end - start) / (n - 1);
        return Array.from({ length: n }, (_, i) => start + i * step);
    }

    function findMinIndex(arr, excludeBorders = true, borderSize = 10) {
        let minVal = Infinity;
        let minIndices = [];
        
        let startIdx = excludeBorders ? borderSize : 0;
        let endIdx = excludeBorders ? arr.length - borderSize : arr.length;
        
        for (let i = startIdx; i < endIdx; i++) {
            if (arr[i] < minVal) {
                minVal = arr[i];
                minIndices = [i];
            } else if (Math.abs(arr[i] - minVal) < 1e-10) {
                minIndices.push(i);
            }
        }
        return minIndices;
    }

    // ===== FUNCIONES DE PROCESAMIENTO PRINCIPAL =====

    function dividirSecuencias(angle_deg) {
        // Simula la función dividirSecuenciasCrecientes de MATLAB
        const sequences = [];
        let currentSeq = [];
        let indices = [];
        let startIdx = 0;
        
        for (let i = 1; i < angle_deg.length; i++) {
            if (angle_deg[i] < angle_deg[i-1]) {
                // Cambio de dirección - nueva secuencia
                sequences.push(angle_deg.slice(startIdx, i));
                indices.push([startIdx, i-1]);
                startIdx = i;
            }
        }
        // Última secuencia
        sequences.push(angle_deg.slice(startIdx));
        indices.push([startIdx, angle_deg.length - 1]);
        
        return { sequences, indices };
    }

    function dividirDatosPorIndices(data, indices) {
        const result = [];
        for (let idx of indices) {
            const [start, end] = idx;
            result.push(data.slice(start, end + 1));
        }
        return result;
    }

    function processing() {
        console.log("Iniciando procesamiento...");
        
        // ===== PASO 1: Cargar y dividir datos =====
        // Asumimos que win.data contiene los datos crudos con estructura:
        // { cycle: n, angle: valor, ch1: valor, ch2: valor }
        
        // Agrupar por ciclo
        const dataByCycle = {};
        win.data.forEach(item => {
            if (!dataByCycle[item.cycle]) {
                dataByCycle[item.cycle] = [];
            }
            dataByCycle[item.cycle].push(item);
        });
        
        processingWin.nCiclos_p = Object.keys(dataByCycle).length;
        
        // Extraer ángulos y canales por ciclo
        const angle_deg_all = [];
        const ch1_all = [];
        const ch2_all = [];
        
        for (let cycle = 1; cycle <= processingWin.nCiclos_p; cycle++) {
            const cycleData = dataByCycle[cycle] || [];
            // Ordenar por ángulo
            cycleData.sort((a, b) => a.angle - b.angle);
            
            cycleData.forEach(item => {
                angle_deg_all.push(item.angle);
                ch1_all.push(item.ch1);
                ch2_all.push(item.ch2);
            });
        }
        
        // ===== PASO 2: Dividir en secuencias =====
        const { sequences: angle_deg_arrays, indices: indices_secuencias } = 
            dividirSecuencias(angle_deg_all);
        
        const ch1_arrays = dividirDatosPorIndices(ch1_all, indices_secuencias);
        const ch2_arrays = dividirDatosPorIndices(ch2_all, indices_secuencias);
        
        // ===== PASO 3: Inicializar arrays de resultados =====
        const numSecuencias = angle_deg_arrays.length;
        
        processingWin.unique_angles = new Array(numSecuencias);
        processingWin.avg_ch1_mean = new Array(numSecuencias);
        processingWin.avg_ch2_mean = new Array(numSecuencias);
        processingWin.avg_ch1_median = new Array(numSecuencias);
        processingWin.avg_ch2_median = new Array(numSecuencias);
        processingWin.avg_ch1_Filtro_Gaussiano_s2 = new Array(numSecuencias);
        processingWin.avg_ch2_Filtro_Gaussiano_s2 = new Array(numSecuencias);
        processingWin.avg_ch1_Media_Movil5 = new Array(numSecuencias);
        processingWin.avg_ch2_Media_Movil5 = new Array(numSecuencias);
        
        // ===== PASO 4: Procesar cada secuencia =====
        for (let i = 0; i < numSecuencias; i++) {
            // Encontrar ángulos únicos preservando orden
            const uniqueVals = [];
            const seen = new Set();
            for (let ang of angle_deg_arrays[i]) {
                if (!seen.has(ang)) {
                    seen.add(ang);
                    uniqueVals.push(ang);
                }
            }
            processingWin.unique_angles[i] = uniqueVals;
            
            // Inicializar arrays para esta secuencia
            const nUnique = uniqueVals.length;
            processingWin.avg_ch1_mean[i] = new Array(nUnique).fill(0);
            processingWin.avg_ch2_mean[i] = new Array(nUnique).fill(0);
            processingWin.avg_ch1_median[i] = new Array(nUnique).fill(0);
            processingWin.avg_ch2_median[i] = new Array(nUnique).fill(0);
            
            // Calcular promedios y medianas para cada ángulo único
            for (let j = 0; j < nUnique; j++) {
                const currentAngle = uniqueVals[j];
                const indices = findIndices(angle_deg_arrays[i], val => 
                    Math.abs(val - currentAngle) < 1e-10
                );
                
                const ch1Vals = indices.map(idx => ch1_arrays[i][idx]);
                const ch2Vals = indices.map(idx => ch2_arrays[i][idx]);
                
                processingWin.avg_ch1_mean[i][j] = mean(ch1Vals);
                processingWin.avg_ch2_mean[i][j] = mean(ch2Vals);
                processingWin.avg_ch1_median[i][j] = median(ch1Vals);
                processingWin.avg_ch2_median[i][j] = median(ch2Vals);
            }
            
            // Aplicar filtros
            
            // Filtro de media móvil
            processingWin.avg_ch1_Media_Movil5[i] = movmean(processingWin.avg_ch1_median[i], 5);
            processingWin.avg_ch2_Media_Movil5[i] = movmean(processingWin.avg_ch2_median[i], 5);
            
            // Filtro Gaussiano
            const sigma = 2;
            const gaussFilter = gausswin(11, sigma);
            const sumFilter = gaussFilter.reduce((a, b) => a + b, 0);
            const normalizedFilter = gaussFilter.map(v => v / sumFilter);
            
            processingWin.avg_ch1_Filtro_Gaussiano_s2[i] = conv(
                processingWin.avg_ch1_median[i], normalizedFilter, 'same'
            );
            processingWin.avg_ch2_Filtro_Gaussiano_s2[i] = conv(
                processingWin.avg_ch2_median[i], normalizedFilter, 'same'
            );
        }
        
        // ===== PASO 5: Calcular ángulos de resonancia =====
        processingWin.resonance_angles_ch1 = new Array(numSecuencias).fill(0);
        processingWin.resonance_angles_ch2 = new Array(numSecuencias).fill(0);
        processingWin.resonance_angles_ch1_gaussian = new Array(numSecuencias).fill(0);
        processingWin.resonance_angles_ch2_gaussian = new Array(numSecuencias).fill(0);
        processingWin.resonance_angles_ch1_fit = new Array(numSecuencias).fill(0);
        processingWin.resonance_angles_ch2_fit = new Array(numSecuencias).fill(0);
        
        processingWin.desviacion_estandar = new Array(numSecuencias).fill(0);
        processingWin.detection_limit = new Array(numSecuencias).fill(0);
        
        // Configuración de parámetros
        const rango_angulos = processingWin.rango_angulos;
        const grado_polinomio = processingWin.grado_polinomio;
        const puntos_minimos = processingWin.puntos_minimos;
        
        console.log(`=== AJUSTE POLINOMIAL DE GRADO ${grado_polinomio} ===`);
        console.log(`Rango de búsqueda: ±${rango_angulos} grados`);
        console.log(`Puntos mínimos requeridos: ${puntos_minimos}`);
        
        for (let i = 0; i < numSecuencias; i++) {
            // Mínimo directo (sin filtro)
            const minIndices_ch1 = findMinIndex(processingWin.avg_ch1_mean[i], true, 10);
            const minAngles_ch1 = minIndices_ch1.map(idx => processingWin.unique_angles[i][idx]);
            processingWin.resonance_angles_ch1[i] = mean(minAngles_ch1);
            
            const minIndices_ch2 = findMinIndex(processingWin.avg_ch2_mean[i], true, 10);
            const minAngles_ch2 = minIndices_ch2.map(idx => processingWin.unique_angles[i][idx]);
            processingWin.resonance_angles_ch2[i] = mean(minAngles_ch2);
            
            // Mínimo con filtro Gaussiano
            const minIndices_ch1_g = findMinIndex(processingWin.avg_ch1_Filtro_Gaussiano_s2[i], true, 10);
            const minAngles_ch1_g = minIndices_ch1_g.map(idx => processingWin.unique_angles[i][idx]);
            processingWin.resonance_angles_ch1_gaussian[i] = mean(minAngles_ch1_g);
            
            const minIndices_ch2_g = findMinIndex(processingWin.avg_ch2_Filtro_Gaussiano_s2[i], true, 10);
            const minAngles_ch2_g = minIndices_ch2_g.map(idx => processingWin.unique_angles[i][idx]);
            processingWin.resonance_angles_ch2_gaussian[i] = mean(minAngles_ch2_g);
            
            // Ajuste polinomial - CH1
            const angulo_ref_ch1 = processingWin.resonance_angles_ch1[i];
            const idx_ch1 = findIndices(processingWin.unique_angles[i], ang => 
                Math.abs(ang - angulo_ref_ch1) <= rango_angulos
            );
            
            if (idx_ch1.length >= puntos_minimos) {
                const angulos_rango = idx_ch1.map(idx => processingWin.unique_angles[i][idx]);
                const valores_rango = idx_ch1.map(idx => processingWin.avg_ch1_mean[i][idx]);
                
                const p_ch1 = polyfit(angulos_rango, valores_rango, grado_polinomio);
                
                const angulos_fino = linspace(
                    Math.min(...angulos_rango), 
                    Math.max(...angulos_rango), 
                    500
                );
                const valores_ajuste = angulos_fino.map(x => polyval(p_ch1, x));
                
                const minVal = Math.min(...valores_ajuste);
                const minIdx = valores_ajuste.indexOf(minVal);
                processingWin.resonance_angles_ch1_fit[i] = angulos_fino[minIdx];
            } else {
                processingWin.resonance_angles_ch1_fit[i] = processingWin.resonance_angles_ch1[i];
                console.log(`⚠️ Ciclo ${i+1} CH1: Solo ${idx_ch1.length} puntos en rango, usando valor directo`);
            }
            
            // Ajuste polinomial - CH2
            const angulo_ref_ch2 = processingWin.resonance_angles_ch2[i];
            const idx_ch2 = findIndices(processingWin.unique_angles[i], ang => 
                Math.abs(ang - angulo_ref_ch2) <= rango_angulos
            );
            
            if (idx_ch2.length >= puntos_minimos) {
                const angulos_rango = idx_ch2.map(idx => processingWin.unique_angles[i][idx]);
                const valores_rango = idx_ch2.map(idx => processingWin.avg_ch2_mean[i][idx]);
                
                const p_ch2 = polyfit(angulos_rango, valores_rango, grado_polinomio);
                
                const angulos_fino = linspace(
                    Math.min(...angulos_rango), 
                    Math.max(...angulos_rango), 
                    500
                );
                const valores_ajuste = angulos_fino.map(x => polyval(p_ch2, x));
                
                const minVal = Math.min(...valores_ajuste);
                const minIdx = valores_ajuste.indexOf(minVal);
                processingWin.resonance_angles_ch2_fit[i] = angulos_fino[minIdx];
            } else {
                processingWin.resonance_angles_ch2_fit[i] = processingWin.resonance_angles_ch2[i];
                console.log(`⚠️ Ciclo ${i+1} CH2: Solo ${idx_ch2.length} puntos en rango, usando valor directo`);
            }
            
            // Calcular desviación estándar y límite de detección
            const diff_ch1 = processingWin.resonance_angles_ch1_fit[i] - processingWin.resonance_angles_ch1_fit[Math.max(0, i-1)];
            const diff_ch2 = processingWin.resonance_angles_ch2_fit[i] - processingWin.resonance_angles_ch2_fit[Math.max(0, i-1)];
            
            processingWin.desviacion_estandar[i] = Math.abs(diff_ch1 - diff_ch2) / 2;
            processingWin.detection_limit[i] = 3 * processingWin.desviacion_estandar[i] / processingWin.sensibility;
        }
        
        // Actualizar datos para las gráficas
        updateCycleData();
        
        console.log("Procesamiento completado");
    }

    function updateCycleData() {
        // Actualizar datos para la gráfica de ciclos
        win.cyclePeakCh1Times = [];
        win.cyclePeakCh1Angles = [];
        win.cyclePeakCh2Times = [];
        win.cyclePeakCh2Angles = [];
        
        for (let i = 0; i < processingWin.resonance_angles_ch1_fit.length; i++) {
            win.cyclePeakCh1Times.push(i + 1);
            win.cyclePeakCh1Angles.push(processingWin.resonance_angles_ch1_fit[i]);
            win.cyclePeakCh2Times.push(i + 1);
            win.cyclePeakCh2Angles.push(processingWin.resonance_angles_ch2_fit[i]);
        }
        
        // Forzar actualización de canvas
        plotCycles.requestPaint();
    }

    // ===== INTERFAZ DE USUARIO =====

    Rectangle {
        anchors.fill: parent
        color: "#111827"

        // ===== Layout raíz =====
        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 16
            spacing: 16

            // ===== Barra superior =====
            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: 60
                radius: 12
                color: "#111827"
                border.color: "#1f2937"; border.width: 1

                RowLayout {
                    anchors.fill: parent; anchors.margins: 12; spacing: 16

                    // Título
                    Label { text: "Procesamiento"; color: "white"; font.bold: true; font.pixelSize: 20 }

                    Rectangle { Layout.fillWidth: true; color: "transparent" }

                    Button {
                        text: "Procesar ciclos actuales"
                        onClicked: {
                            processing()
                        }
                        Layout.preferredHeight: 36; Layout.preferredWidth: 120; font.pixelSize: 14
                        ToolTip.visible: hovered
                        ToolTip.text: "Procesamiento de la data actual"
                    }
                }
            }

            // ===== Panel de lecturas =====
            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: 100
                radius: 12
                color: "#111827"
                border.color: "#1f2937"; border.width: 1

                RowLayout {
                    anchors.fill: parent; anchors.margins: 16; spacing: 32

                    ColumnLayout {
                        Label { text:"N ciclos";
                        color: '#fa5d35'; font.pixelSize: 14; font.bold: true }
                        Label { text: processingWin.nCiclos_p ? processingWin.nCiclos_p : "—";
                        color: "white";
                        font.pixelSize: 32;
                        font.bold: true
                        }
                    }

                    ColumnLayout {
                        Label { 
                            text: "Min desv (°)";
                            color: "#22c55e";
                            font.pixelSize: 14;
                            font.bold: true
                        }
                        Label {
                            text: processingWin.desviacion_estandar.length > 0 ? 
                                Math.min(...processingWin.desviacion_estandar).toFixed(3) : "—";
                            color: "white";
                            font.pixelSize: 32;
                            font.bold: true 
                        }
                    }

                    ColumnLayout {
                        Label {
                           text: "Max desv (°)";
                            color: "#60a5fa";
                            font.pixelSize: 14;
                            font.bold: true
                        }
                        Label { 
                            text: processingWin.desviacion_estandar.length > 0 ? 
                                Math.max(...processingWin.desviacion_estandar).toFixed(3) : "—";
                            color: "white";
                            font.pixelSize: 32;
                            font.bold: true
                        }
                    }

                    ColumnLayout {
                        Label {
                           text: "Sensibilidad (°/RIU)";
                            color: "#60a5fa";
                            font.pixelSize: 14;
                            font.bold: true
                        }
                        Label { 
                            text: processingWin.sensibility ? processingWin.sensibility : "—";
                            color: "white";
                            font.pixelSize: 32;
                            font.bold: true
                        }
                    }

                    ColumnLayout {
                        Label { 
                            text: "Min Limite detección (RIU)";
                            color: "#22c55e";
                            font.pixelSize: 14;
                            font.bold: true
                        }
                        Label {
                            text: processingWin.detection_limit.length > 0 ? 
                                Math.min(...processingWin.detection_limit).toFixed(3) : "—";
                            color: "white";
                            font.pixelSize: 32;
                            font.bold: true 
                        }
                    }

                    ColumnLayout {
                        Label {
                           text: "Max Limite detección (RIU)";
                            color: "#60a5fa";
                            font.pixelSize: 14;
                            font.bold: true
                        }
                        Label { 
                            text: processingWin.detection_limit.length > 0 ? 
                                Math.max(...processingWin.detection_limit).toFixed(3) : "—";
                            color: "white";
                            font.pixelSize: 32;
                            font.bold: true
                        }
                    }
                }
            }

            // ===== Gráficas =====
            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                radius: 12
                color: "#111827"
                border.color: "#1f2937"; border.width: 1

                Canvas {
                    id: plot
                    visible: win.viewMode === "curve"
                    anchors.fill: parent
                    anchors.margins: 20
                    antialiasing: true
                    property real xTickStep: 1.0

                    onPaint: {
                        const ctx = getContext("2d");
                        const W=width, H=height;

                        ctx.clearRect(0,0,W,H);
                        ctx.fillStyle="#111827"; ctx.fillRect(0,0,W,H);

                        const mLeft=100, mRight=10, mTop=10, mBottom=35;
                        const pw = W-mLeft-mRight, ph = H-mTop-mBottom;
                        
                        if (pw<=0 || ph<=0) {
                            if (win.debugLogs) console.log(`El área útil es negativa: pw=${pw} ph=${ph}`);
                            return;
                        }
                        
                        ctx.strokeStyle="#1f2937"; ctx.strokeRect(mLeft,mTop,pw,ph);
                        
                        ctx.fillStyle="#9ca3af"; ctx.font="12px sans-serif"; ctx.textAlign="center";
                        ctx.fillText("Ángulo (°)", mLeft+pw/2, H);

                        ctx.save(); ctx.translate(15, mTop+ph/2+30); ctx.rotate(-Math.PI/2);

                        if (win.deviceUnites === "resistance"){ctx.fillText("Resistencia (kΩ)", 0, 0);}
                        else if (win.deviceUnites === "current"){ctx.fillText("Corriente (mA)", 0, 0);}
                        ctx.restore();

                        if (win.dataCycleFilter.length === 0){
                            ctx.fillStyle="#9ca3af";
                            ctx.textAlign="left";
                            ctx.fillText("Sin datos agregados aún…", mLeft+10, mTop+20);
                            return;
                        }

                        const xmin = win.xMinDeg, xmax = win.xMaxDeg;
                        const xspan = Math.max(1e-9,xmax-xmin);
                        const xMap = (vx)=> mLeft + ((vx-xmin)/xspan)*pw;

                        let ymin = +Infinity, ymax = -Infinity;

                        for (let i=0; i<win.dataCycleFilter.length; i++){
                            const d=win.dataCycleFilter[i];
                            if (isFinite(d.ch1) && win.viewCh1){ymin=Math.min(ymin,d.ch1); ymax=Math.max(ymax,d.ch1);}
                            if (isFinite(d.ch2) && win.viewCh2){ymin=Math.min(ymin,d.ch2); ymax=Math.max(ymax,d.ch2);}
                        }

                        if (!(ymin<Infinity && ymax>-Infinity)){ymin = 0; ymax = 1;}
                        if (Math.abs(ymax-ymin)<1e-9) {ymax = ymin+0.5; ymin -= 0.5;}

                        const pad = (ymax-ymin)*0.05; const yMin=ymin-pad, yMax = ymax+pad;
                        const yMap = (vy)=> mTop + ph*(1-(vy-yMin)/(yMax-yMin));

                        ctx.strokeStyle="#253041"; ctx.lineWidth=1; ctx.beginPath();
                        for (let gy=0; gy<=6; gy++){ const y=mTop+ph*(gy/6); ctx.moveTo(mLeft,y); ctx.lineTo(mLeft+pw,y); }
                        ctx.stroke();

                        const step=Math.max(0.01, plot.xTickStep);
                        const firstTick=Math.ceil(xmin/step)*step;
                        const lastTick=Math.floor(xmax/step)*step;
                        ctx.beginPath();
                        for (let v=firstTick; v<=lastTick+1e-9; v+=step){
                            const x=xMap(v); ctx.moveTo(x,mTop); ctx.lineTo(x,mTop+ph);
                        }
                        ctx.strokeStyle="#253041"; ctx.stroke();

                        ctx.fillStyle="#9ca3af"; ctx.font="12px sans-serif"; ctx.textAlign="center";
                        for (let v=firstTick; v<=lastTick+1e-9; v+=step){
                            const x=xMap(v); ctx.fillText(v.toFixed(step<1?2:0), x, mTop+ph+18);
                        }
                        ctx.textAlign="right";
                        for (let gy=0; gy<=6; gy++){
                            const vy=yMin+(yMax-yMin)*(gy/6); const y=mTop+ph*(1-gy/6);
                            ctx.fillText(vy.toFixed(3), mLeft-6, y+4);
                        }

                        function drawPolyline(color, acc){
                            ctx.strokeStyle=color; ctx.lineWidth=2; ctx.beginPath();
                            let started=false;
                            for (let i=0;i<win.dataCycleFilter.length;i++){
                                const d=win.dataCycleFilter[i]; const v=acc(d); if (!isFinite(v)) continue;
                                const x=xMap(d.angle), y=yMap(v);
                                if(!started){ctx.moveTo(x,y); started=true;} else ctx.lineTo(x,y);
                            }
                            ctx.stroke();
                            ctx.fillStyle=color; const r=2.0;
                            for (let i=0;i<win.dataCycleFilter.length;i++){
                                const d=win.dataCycleFilter[i]; const v=acc(d); if (!isFinite(v)) continue;
                                const x=xMap(d.angle), y=yMap(v);
                                ctx.beginPath(); ctx.arc(x,y,r,0,Math.PI*2); ctx.fill();
                            }
                        }
                        
                        if (win.viewCh1) {drawPolyline("#22c55e", d=>d.ch1);}
                        if (win.viewCh2) {drawPolyline("#60a5fa", d=>d.ch2);}
                    }
                }

                Canvas {
                    id: plotCycles
                    visible: win.viewMode === "cycles"
                    anchors.fill: parent
                    anchors.margins: 20
                    antialiasing: true

                    onPaint: {
                        const ctx=getContext("2d");
                        const W=width, H=height;

                        ctx.clearRect(0,0,W,H);
                        ctx.fillStyle="#111827"; ctx.fillRect(0,0,W,H);

                        const ml=80,mr=10,mt=10,mb=30;
                        const pw=W-ml-mr, ph=H-mt-mb;
                        
                        if (pw<=0||ph<=0) {console.log("Los margenes son"); return;}

                        ctx.strokeStyle="#1f2937"; ctx.strokeRect(ml,mt,pw,ph);

                        ctx.fillStyle="#9ca3af"; ctx.font="12px sans-serif"; ctx.textAlign="center";
                        ctx.fillText("Ciclo", ml+pw/2, H);
                        ctx.save(); ctx.translate(16, mt+ph/2); ctx.rotate(-Math.PI/2);
                        ctx.fillText("Ángulo de resonancia (°)", 0, 0); ctx.restore();

                        const n1=win.cyclePeakCh1Angles.length, n2=win.cyclePeakCh2Angles.length;
                        const N=Math.max(n1,n2);
                        
                        if (N===0){
                            ctx.fillStyle="#9ca3af"; ctx.textAlign="left";
                            ctx.fillText("Sin ciclos completados aún…", ml+10, mt+20); return;
                        }

                        // Autoescalado eje X
                        let xmin=+Infinity,xmax=-Infinity;

                        for (let i=0;i<n1;i++){
                            const v=win.cyclePeakCh1Times[i];
                            if (isFinite(v) && win.viewCh1){xmin=Math.min(xmin,v); xmax=Math.max(xmax,v);}
                        }

                        for (let i=0;i<n2;i++){
                            const v=win.cyclePeakCh2Times[i];
                            if (isFinite(v) && win.viewCh2){xmin=Math.min(xmin,v); xmax=Math.max(xmax,v);}
                        }
                        
                        if (Math.abs(xmax-xmin)<1e-6) xmax=xmin+1.0;
                        const padx=(xmax-xmin)*0.08; xmin-=padx; xmax+=padx;
                        const xMap = (vx) => ml + ((vx-xmin)/(xmax - xmin))*pw;

                        // Autoescalado eje Y
                        let ymin=+Infinity,ymax=-Infinity;

                        for (let i=0;i<n1;i++){
                            const v=win.cyclePeakCh1Angles[i];
                            if (isFinite(v) && win.viewCh1){ymin=Math.min(ymin,v); ymax=Math.max(ymax,v);}
                        }
                        for (let i=0;i<n2;i++){
                            const v=win.cyclePeakCh2Angles[i];
                            if (isFinite(v) && win.viewCh2){ymin=Math.min(ymin,v); ymax=Math.max(ymax,v);}
                        }

                        if (!(ymin<Infinity && ymax>-Infinity)) { ymin=win.xMinDeg; ymax=win.xMaxDeg;}
                        if (Math.abs(ymax-ymin)<1e-6) ymax=ymin+1.0;
                        const pady=(ymax-ymin)*0.08; ymin-=pady; ymax+=pady;
                        const yMap=(vy)=> mt + ph*(1-(vy-ymin)/(ymax-ymin));

                        // grilla
                        ctx.strokeStyle="#253041"; ctx.beginPath();
                        for (let gy=0; gy<=6; gy++){ const y=mt+ph*(gy/6); ctx.moveTo(ml,y); ctx.lineTo(ml+pw,y); }
                        ctx.stroke();
                        ctx.textAlign="right"; ctx.fillStyle="#9ca3af";

                        for (let gy=0; gy<=6; gy++){
                            const vy=ymin+(ymax-ymin)*(gy/6); const y=mt+ph*(1-gy/6);
                            ctx.fillText(vy.toFixed(2), ml-6, y+4);
                        }

                        ctx.textAlign="center";

                        const maxTicks=10, step=Math.max(1, Math.ceil(xmax/maxTicks));

                        for (let c=0; c<=xmax; c+=step){ const x=xMap(c); ctx.fillText(c.toString(), x, mt+ph+18); }

                        function drawSeries(color, arr1, arr2){
                            ctx.strokeStyle=color; ctx.lineWidth=2; ctx.beginPath();
                            let started=false;
                            for (let i=0;i<arr1.length;i++){
                                const u=arr1[i], v=arr2[i];
                                if (!isFinite(v)) continue;
                                const x=xMap(u), y=yMap(v);
                                if(!started){ctx.moveTo(x,y); started=true;} else ctx.lineTo(x,y);
                            }
                            ctx.stroke(); ctx.fillStyle=color;
                            for (let i=0;i<arr1.length;i++){
                                const u=arr1[i], v=arr2[i];
                                if (!isFinite(v)) continue;
                                const x=xMap(u), y=yMap(v);
                                ctx.beginPath(); ctx.arc(x,y,3,0,Math.PI*2); ctx.fill();
                            }
                        }

                        if (win.viewCh1) {
                            drawSeries("#22c55e", win.cyclePeakCh1Times, win.cyclePeakCh1Angles);
                        }
                        if (win.viewCh2) {
                            drawSeries("#60a5fa", win.cyclePeakCh2Times, win.cyclePeakCh2Angles);
                        }
                        
                        // Opcional: Dibujar también las versiones con filtro gaussiano
                        if (false) { // Activar si se desea ver
                            drawSeries("#ec706c", win.cyclePeakCh1Times, processingWin.resonance_angles_ch1_gaussian);
                            drawSeries("#dac511", win.cyclePeakCh2Times, processingWin.resonance_angles_ch2_gaussian);
                        }
                    }
                }
            }
        }
    }
}