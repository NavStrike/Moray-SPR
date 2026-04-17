// ===== IMPORTS =====
import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Window
import QtQuick.Dialogs

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

    // Funciones de procesamiento

    function divData(){
        win.data.forEach(item => {
            if (!processingWin.data_p[item.cycle]) {processingWin.data_p[item.cycle] = [];}
            processingWin.data_p[item.cycle].push(item);
        });

        processingWin.nCiclos_p = len(processingWin.data_p)
    }

    function assigData(){
        processingWin.data_p.forEach(item => {
            let angles = item.map(d => d.angle);
            let ch1s = item.map(d => d.ch1);
            let ch2s = item.map(d => d.ch2);
            processingWinarr.angle_p.push(unique(angles));
            processingWinarr.ch1_p.push(unique(ch1s));
            processingWinarr.ch2_p.push(unique(ch2s));
        });
    }

    function assigResonanceAngles(){
        processingWin.resonance_angles_ch1 = min
    }

    function processing(){
        
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
                    id: plot
                    visible: win.viewMode === "curve"
                    anchors.fill: parent
                    anchors.margins: 20
                    antialiasing: true
                    property real xTickStep: 1.0

                    onPaint: {
                        const ctx = getContext("2d");
                        const W=width, H=height;

                        // Limpieza de la pantalla
                        ctx.clearRect(0,0,W,H);
                        //Creación de un rectangulo
                        ctx.fillStyle="#111827"; ctx.fillRect(0,0,W,H);

                        const mLeft=100, mRight=10, mTop=10, mBottom=35;
                        const pw = W-mLeft-mRight, ph = H-mTop-mBottom;
                        
                        if (pw<=0 || ph<=0) {
                            if (win.debugLogs) console.log(`El área útil es negativa: pw=${pw} ph=${ph}`);
                            return;
                        }
                        
                        // Creación del marco
                        ctx.strokeStyle="#1f2937"; ctx.strokeRect(mLeft,mTop,pw,ph);
                        
                        // Creación del label eje x
                        ctx.fillStyle="#9ca3af"; ctx.font="12px sans-serif"; ctx.textAlign="center";
                        ctx.fillText("Ángulo (°)", mLeft+pw/2, H);

                        ctx.save(); ctx.translate(15, mTop+ph/2+30); ctx.rotate(-Math.PI/2);

                        // Creación del label eje y
                        if (win.deviceUnites === "resistance"){ctx.fillText("Resistencia (kΩ)", 0, 0);}
                        else if (win.deviceUnites === "current"){ctx.fillText("Corriente (mA)", 0, 0);}
                        ctx.restore();

                        // Si no hay datos
                        if (win.dataCycleFilter.length === 0){
                            ctx.fillStyle="#9ca3af";
                            ctx.textAlign="left";
                            ctx.fillText("Sin datos agregados aún…", mLeft+10, mTop+20);
                            return;
                        }

                        // Mapeo de coordenadas max / min
                        const xmin = win.xMinDeg, xmax = win.xMaxDeg;
                        const xspan = Math.max(1e-9,xmax-xmin);
                        const xMap = (vx)=> mLeft + ((vx-xmin)/xspan)*pw;

                        // eje Y auto
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

                        // grilla H
                        ctx.strokeStyle="#253041"; ctx.lineWidth=1; ctx.beginPath();
                        for (let gy=0; gy<=6; gy++){ const y=mTop+ph*(gy/6); ctx.moveTo(mLeft,y); ctx.lineTo(mLeft+pw,y); }
                        ctx.stroke();

                        // ticks X exactos
                        const step=Math.max(0.01, plot.xTickStep);
                        const firstTick=Math.ceil(xmin/step)*step;
                        const lastTick=Math.floor(xmax/step)*step;
                        ctx.beginPath();
                        for (let v=firstTick; v<=lastTick+1e-9; v+=step){
                            const x=xMap(v); ctx.moveTo(x,mTop); ctx.lineTo(x,mTop+ph);
                        }
                        ctx.strokeStyle="#253041"; ctx.stroke();

                        // etiquetas
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

                        const maxTicks=10, step=Math.max(1, Math.floor(xmax/maxTicks));

                        for (let c=0; c<=xmax; c+=step){ const x=xMap(c); ctx.fillText(c.toString(), x, mt+ph+18); }

                        function drawSeries(color, arr1, arr2){
                            ctx.strokeStyle=color; ctx.lineWidth=2; ctx.beginPath();
                            let started=false;
                            for (let i=1;i<=arr1.length;i++){
                                const u=arr1[i-1], v=arr2[i-1];
                                if (!isFinite(v)) continue;
                                const x=xMap(u), y=yMap(v);
                                if(!started){ctx.moveTo(x,y); started=true;} else ctx.lineTo(x,y);
                            }
                            ctx.stroke(); ctx.fillStyle=color;
                            for (let i=1;i<=arr1.length;i++){
                                const u=arr1[i-1], v=arr2[i-1];
                                if (!isFinite(v)) continue;
                                const x=xMap(u), y=yMap(v);
                                ctx.beginPath(); ctx.arc(x,y,3,0,Math.PI*2); ctx.fill();
                            }
                        }

                        if (win.viewCh1) {
                            drawSeries("#22c55e", win.cyclePeakCh1Times, win.cyclePeakCh1Angles);
                            // drawSeries('#ec706c', win.cyclePeakCh1CentroidTimes, win.cyclePeakCh1CentroidAngles);
                        }
                        if (win.viewCh2) {
                            drawSeries("#60a5fa", win.cyclePeakCh2Times, win.cyclePeakCh2Angles);
                            // drawSeries('#dac511', win.cyclePeakCh2CentroidTimes, win.cyclePeakCh2CentroidAngles);
                        }
                    }
                }
            }
        }
    }


}