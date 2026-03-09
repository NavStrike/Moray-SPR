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
    property var resonance_angles_ch1_fit: []
    property var resonance_angles_ch2_fit: []
    property var diff_resonance_angles_fit: []
    property var resonance_angles_fit_dif_medianmov: []

    // Parametros de vista
    property bool viewFit: true
    property bool viewMean: true

    // Parámetros de ajuste
    property real rango_angulos: 1.75
    property real grado_polinomio: 4
    property real puntos_minimos: grado_polinomio + 1

    // Resultados
    property var standard_deviation: []
    property real sensibility: 80
    property var detection_limit: []

    id: processingWin
    visible: win.viewPage === "process"
    title: "Procesamiento"
    Layout.fillWidth: true
    Layout.fillHeight: true

    // Funciones de MATLAB

    function unique(arr, options = {}) {
        const { stable = true, sorted = false } = options;
        
        if (sorted) {
            // Versión ordenada (más eficiente para arrays grandes)
            const sortedArr = [...arr].sort();
            return sortedArr.filter((item, index) => 
                index === 0 || item !== sortedArr[index - 1]
            );
        } else if (stable) {
            // Versión que preserva orden (como unique de MATLAB por defecto)
            return [...new Set(arr)];
        } else {
            // Versión sin orden específico
            return Array.from(new Set(arr));
        }
    }

    function polyfit(x, y, grado) {
        // Verificar que los arrays tengan la misma longitud
        if (x.length !== y.length) {
            throw new Error('Los arrays x e y deben tener la misma longitud');
        }
        
        const n = x.length;
        const m = grado + 1;
        
        // Construir la matriz de Vandermonde
        let A = [];
        for (let i = 0; i < n; i++) {
            let fila = [];
            for (let j = 0; j < m; j++) {
                fila.push(Math.pow(x[i], j));
            }
            A.push(fila);
        }
        
        // Resolver el sistema de ecuaciones usando mínimos cuadrados
        // A * p = y, donde p son los coeficientes del polinomio
        
        // Calcular A^T * A
        let ATA = [];
        for (let i = 0; i < m; i++) {
            ATA[i] = [];
            for (let j = 0; j < m; j++) {
                let suma = 0;
                for (let k = 0; k < n; k++) {
                    suma += A[k][i] * A[k][j];
                }
                ATA[i][j] = suma;
            }
        }
        
        // Calcular A^T * y
        let ATy = [];
        for (let i = 0; i < m; i++) {
            let suma = 0;
            for (let k = 0; k < n; k++) {
                suma += A[k][i] * y[k];
            }
            ATy[i] = suma;
        }
        
        // Resolver el sistema lineal usando eliminación gaussiana
        return resolverSistemaLineal(ATA, ATy);
    }

    function resolverSistemaLineal(A, b) {
        const n = A.length;
        
        // Crear una copia de la matriz aumentada
        let Aug = [];
        for (let i = 0; i < n; i++) {
            Aug[i] = [...A[i], b[i]];
        }
        
        // Eliminación hacia adelante
        for (let i = 0; i < n; i++) {
            // Encontrar el pivote máximo
            let maxRow = i;
            for (let k = i + 1; k < n; k++) {
                if (Math.abs(Aug[k][i]) > Math.abs(Aug[maxRow][i])) {
                    maxRow = k;
                }
            }
            
            // Intercambiar filas
            [Aug[i], Aug[maxRow]] = [Aug[maxRow], Aug[i]];
            
            // Verificar si la matriz es singular
            if (Math.abs(Aug[i][i]) < 1e-10) {
                throw new Error('La matriz es singular o mal condicionada');
            }
            
            // Hacer ceros debajo del pivote
            for (let k = i + 1; k < n; k++) {
                let factor = Aug[k][i] / Aug[i][i];
                for (let j = i; j <= n; j++) {
                    Aug[k][j] -= factor * Aug[i][j];
                }
            }
        }
        
        // Sustitución hacia atrás
        let x = new Array(n).fill(0);
        for (let i = n - 1; i >= 0; i--) {
            x[i] = Aug[i][n];
            for (let j = i + 1; j < n; j++) {
                x[i] -= Aug[i][j] * x[j];
            }
            x[i] /= Aug[i][i];
        }
        
        return x;
    }

    // Función para evaluar el polinomio en un punto x
    function polyval(coeficientes, x) {
        let resultado = 0;
        for (let i = 0; i < coeficientes.length; i++) {
            resultado += coeficientes[i] * Math.pow(x, i);
        }
        return resultado;
    }

    // Función para evaluar el polinomio en múltiples puntos
    function polyvalArray(coeficientes, xArray) {
        return xArray.map(x => polyval(coeficientes, x));
    }

    function linspace(inicio, fin, n = 100) {
        // Si n es 1, retornar solo el valor final (como MATLAB)
        if (n === 1) return [fin];
        
        let paso = (fin - inicio) / (n - 1);
        let resultado = [];
        
        for (let i = 0; i < n; i++) {
            resultado.push(inicio + i * paso);
        }

        return resultado;
    }

    function movmedian(datos, ventana) {
        let resultado = [];
        let n = datos.length;
        
        // Asegurar que la ventana sea impar para ventana centrada
        let mediaVentana = Math.floor(ventana / 2);
        
        for (let i = 0; i < n; i++) {
            // Calcular índices de inicio y fin de la ventana
            let inicio = Math.max(0, i - mediaVentana);
            let fin = Math.min(n - 1, i + mediaVentana);
            
            // Extraer la ventana
            let ventanaDatos = datos.slice(inicio, fin + 1);
            
            // Calcular mediana
            let mediana = calcularMediana(ventanaDatos);
            resultado.push(mediana);
        }
        
        return resultado;
    }

    function calcularMediana(array) {
        if (array.length === 0) return NaN;
        
        // Ordenar el array
        let ordenado = [...array].sort((a, b) => a - b);
        let medio = Math.floor(ordenado.length / 2);
        
        if (ordenado.length % 2 === 0) {
            // Si es par, promediar los dos valores centrales
            return (ordenado[medio - 1] + ordenado[medio]) / 2;
        } else {
            // Si es impar, tomar el valor central
            return ordenado[medio];
        }
    }

    function restaVectores(vec1, vec2) {
        if (vec1.length !== vec2.length) {
            throw new Error('Los vectores deben tener la misma longitud');
        }
        
        let resultado = [];
        for (let i = 0; i < vec1.length; i++) {
            resultado.push(vec1[i] - vec2[i]);
        }
        return resultado;
    }

    // Funciones de procesamiento

    function divData(){

        let cycles = win.data.map(d => d.cycle)
        processingWin.nCiclos_p = Math.max(cycles)
        for (let i = 0; i < processingWin.nCiclos_p; i++) {
            win.data.filter(dat => dat.cycle == i);
            processingWin.data_p.push(item);
        };

    }

    function assigData(){
        for (let i = 0; i < processingWin.data_p; i++) {

            let angles = processingWin.data_p[i].map(d => d.angle);
            let ch1s = processingWin.data_p[i].map(d => d.ch1);
            let ch2s = processingWin.data_p[i].map(d => d.ch2);

            let unique_angles = unique(angles)
            processingWin.angle_p.push(unique_angles)

            for (let j = 0; j < unique_angles.length; j++) {
                // Crear máscara booleana para los índices donde el ángulo coincide
                let idx = angles.map(angle => angle === unique_angles[i][j]);
                
                // Filtrar los valores de ch1 y ch2 usando la máscara
                let ch1_filtered = ch1s.filter((_, index) => idx[index]);
                let ch2_filtered = ch2s.filter((_, index) => idx[index]);
                
                let avg_ch1_mean = ch1_filtered.reduce((a, b) => a + b, 0) / ch1_filtered.length;
                let avg_ch2_mean = ch2_filtered.reduce((a, b) => a + b, 0) / ch2_filtered.length;

                processingWin.ch1_p.push(unique_angles)
                processingWin.ch2_p.push(unique_angles)
            }
        }
    }

    function assigResonanceAngles(){
        for (let i = 0; i < processingWin.nCiclos_p; i++) {
            // Encontrar los índices donde avg_ch1_mean[i] es igual al mínimo en el rango 10:end-10
            let minValue_ch1 = Math.min(...processingWin.ch1_p[i].slice(9, -9));
            let minValue_ch2 = Math.min(...processingWin.ch2_p[i].slice(9, -9));
            let indices_ch1 = [];
            let indices_ch2 = [];

            // Encontrar todos los índices que coinciden con el valor mínimo
            for (let idx = 0; idx < processingWin.ch1_p[i].length; idx++) {
                if (processingWin.ch1_p[i][idx] === minValue_ch1) {
                    indices_ch1.push(idx);
                }
            }
            for (let idx = 0; idx < processingWin.ch1_p[i].length; idx++) {
                if (processingWin.ch1_p[i][idx] === minValue_ch1) {
                    indices_ch2.push(idx);
                }
            }

            // Calcular el promedio de unique_angles[i] en esos índices
            let sum_ch1 = indices_ch1.reduce((acc, val) => acc + unique_angles[i][val], 0);
            processingWin.resonance_angles_ch1.push(indices_ch1.length > 0 ? sum / indices_ch1.length : 0)

            let sum_ch2 = indices_ch2.reduce((acc, val) => acc + unique_angles[i][val], 0);
            processingWin.resonance_angles_ch2.push(indices_ch2.length > 0 ? sum / indices_ch2.length : 0)
        }
    }

    function curveAdjustment(){
        for (let i = 0; i < processingWin.nCiclos_p; i++) {

            // Canal 1
            let angulo_ref_ch1 = processingWin.resonance_angles_ch1[i];
            
            let idx_ch1 = processingWin.ch1_p[i]
            .map((angulo, index) => Math.abs(angulo - angulo_ref_ch1) <= rango_angulos ? index : null)
            .filter(index => index !== null);

            let angulos_rango = 0; let valores_rango = 0;

            if (idx_ch1.length >= processingWin.puntos_minimos){
                angulos_rango = processingWin.angle_p[i][idx_ch1];
                valores_rango = processingWin.ch1_p[i][idx_ch1];
            }
            let p_ch1 = polyfit(angulos_rango, valores_rango, processingWin.grado_polinomio);

            let angulos_fino = linspace(Math.min(angulos_rango), Math.max(angulos_rango), 500);

            let valores_ajuste = polyval(p_ch1, angulos_fino);

            // Encontrar el índice del valor mínimo
            let idx_min = valores_ajuste.indexOf(Math.min(...valores_ajuste));
            let resonance_angle_ch1_fit = angulos_fino[idx_min];

            processingWin.resonance_angles_ch1_fit.push(resonance_angle_ch1_fit)

            // Canal 2
            let angulo_ref_ch2 = processingWin.resonance_angles_ch2[i];
            
            let idx_ch2 = processingWin.ch2_p[i]
            .map((angulo, index) => Math.abs(angulo - angulo_ref_ch2) <= rango_angulos ? index : null)
            .filter(index => index !== null);

            if (idx_ch2.length >= processingWin.puntos_minimos){
                angulos_rango = processingWin.angle_p[i][idx_ch2];
                valores_rango = processingWin.ch2_p[i][idx_ch2];
            }
            let p_ch2 = polyfit(angulos_rango, valores_rango, processingWin.grado_polinomio);

            angulos_fino = linspace(Math.min(angulos_rango), Math.max(angulos_rango), 500);

            valores_ajuste = polyval(p_ch2, angulos_fino);

            // Encontrar el índice del valor mínimo
            idx_min = valores_ajuste.indexOf(Math.min(...valores_ajuste));
            let resonance_angle_ch2_fit = angulos_fino[idx_min];

            processingWin.resonance_angles_ch2_fit.push(resonance_angle_ch2_fit)

            // Diferencia
             
            let diff_resonance_angle_fit = resonance_angle_ch1_fit - resonance_angles_ch2_fit

            processingWin.diff_resonance_angles_fit.append(diff_resonance_angle_fit)
        }
    }

    function processing(){
        divData()
        assigData()
        assigResonanceAngles()
        curveAdjustment()

        let res = movmedian(processingWin.diff_resonance_angles_fit, 11)

        for (let i = 0; i < processingWin.nCiclos_p; i++) {
            processingWin.resonance_angles_fit_dif_medianmov.push(res[i])
        }

        plotResults.requestPaint()
    }

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


            // ===== Panel de lecturas (kΩ | mA) =====
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
                        Label { text: isFinite(processingWin.nCiclos_p) ? processingWin.nCiclos_p : "—";
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
                            text: isFinite(Math.min(processingWin.desviacion_estandar)) ? (Math.min(processingWin.desviacion_estandar).toFixed(3)) : "—";
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
                            text: isFinite(Math.max(processingWin.desviacion_estandar)) ? (Math.max(processingWin.desviacion_estandar).toFixed(3)) : "—";
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
                            text: isFinite(processingWin.sensibility) ? processingWin.sensibility : "—";
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
                            text: isFinite(Math.min(processingWin.detection_limit)) ? (Math.min(processingWin.detection_limit).toFixed(3)) : "—";
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
                            text: isFinite(Math.max(processingWin.detection_limit)) ? (Math.max(processingWin.detection_limit).toFixed(3)) : "—";
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
                    id: plotResults
                    visible: win.viewMode === "cycles"
                    anchors.fill: parent
                    anchors.margins: 20
                    antialiasing: true

                    onPaint: {
                        const ctx=getContext("2d");
                        const W=width, H=height;

                        ctx.clearRect(0,0,W,H); // Limpieza de la pantalla
                        ctx.fillStyle="#111827"; ctx.fillRect(0,0,W,H); //Creación de un rectangulo

                        const ml=80,mr=10,mt=10,mb=30;
                        const pw=W-ml-mr, ph=H-mt-mb;
                        
                        if (pw<=0||ph<=0) {console.log("Los margenes son"); return;}

                        // marco
                        ctx.strokeStyle="#1f2937"; ctx.strokeRect(ml,mt,pw,ph);

                        ctx.fillStyle="#9ca3af"; ctx.font="12px sans-serif"; ctx.textAlign="center";
                        ctx.fillText("Tiempo", ml+pw/2, H);
                        ctx.save(); ctx.translate(16, mt+ph/2); ctx.rotate(-Math.PI/2);
                        ctx.fillText("Ángulo del máximo (°)", 0, 0); ctx.restore();

                        const n1=win.cyclePeakCh1Angles.length, n2=win.cyclePeakCh2Angles.length;
                        const N=Math.max(n1,n2);
                        
                        if (N===0){
                            ctx.fillStyle="#9ca3af"; ctx.textAlign="left";
                            ctx.fillText("Sin ciclos completados aún…", ml+10, mt+20); return;
                        }

                        // Autoescalado eje X
                        let xmin=+Infinity,xmax=-Infinity;

                        for (let i=0;i<n1;i++){
                            const v=[...Array(processingWin.nCiclos_p).keys()].map(i => i + 1);
                            if (isFinite(v) && processingWin.viewFit){xmin=Math.min(xmin,v); xmax=Math.max(xmax,v);}
                        }

                        if (Math.abs(xmax-xmin)<1e-6) xmax=xmin+1.0;
                        const padx=(xmax-xmin)*0.08; xmin-=padx; xmax+=padx;
                        const xMap = (vx) => ml + ((vx-xmin)/(xmax - xmin))*pw;

                        // Autoescalado eje Y
                        let ymin=+Infinity, ymax=-Infinity;

                        for (let i=0;i<n1;i++){
                            const v=processingWin.diff_resonance_angles_fit[i];
                            if (isFinite(v) && processingWin.viewFit){ymin=Math.min(ymin,v); ymax=Math.max(ymax,v);}
                        }
                        for (let i=0;i<n2;i++){
                            const v=processingWin.resonance_angles_fit_dif_medianmov[i];
                            if (isFinite(v) && processingWin.viewMean){ymin=Math.min(ymin,v); ymax=Math.max(ymax,v);}
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

                        const maxTicks=10, step=Math.max(1, Math.floor(xmax/maxTicks));

                        for (let c=0; c<=xmax; c+=step){ const x=xMap(c); ctx.fillText(c.toString(), x, mt+ph+18); }

                        function drawSeries(color, arr1){
                            ctx.strokeStyle=color; ctx.lineWidth=2; ctx.beginPath();
                            let started=false;
                            for (let i=1;i<=arr1.length;i++){
                                const u=arr1[i-1], v=i;
                                if (!isFinite(v)) continue;
                                const x=xMap(u), y=yMap(v);
                                if(!started){ctx.moveTo(x,y); started=true;} else ctx.lineTo(x,y);
                            }
                            ctx.stroke(); ctx.fillStyle=color;
                            for (let i=1;i<=arr1.length;i++){
                                const u=arr1[i-1], v=i;
                                if (!isFinite(v)) continue;
                                const x=xMap(u), y=yMap(v);
                                ctx.beginPath(); ctx.arc(x,y,3,0,Math.PI*2); ctx.fill();
                            }
                        }

                        if (processingWin.viewFit) {
                            drawSeries("#60a5fa", processingWin.diff_resonance_angles_fit);
                        }
                        if (processingWin.viewMean) {
                            drawSeries("#22c55e", processingWin.resonance_angles_fit_dif_medianmov);
                        }
                    }
                }
            }
        }
    }

}