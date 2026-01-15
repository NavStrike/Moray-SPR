// ===== IMPORTS =====
import QtQuick 6.0
import QtQuick.Controls 6.0
import QtQuick.Layouts 6.0
import QtQuick.Window 6.0
import QtQuick.Dialogs 6.0

// ===== LDR Monitor =====
ApplicationWindow {
    id: win
    visible: true
    // visibility: Window.FullScreen
    visibility: Window.Maximized
    width: 800
    height: 480
    title: "LDR Monitor"
    color: "#0f172a"

    // ===== Import others files =====
    AnglesDialog {
        id: winAngles
    }

    // ===== Estado / datos =====
    property bool  active: false
    property bool  debugLogs: true
    property real  ch1: 0.0                 // A (desde backend)
    property real  ch2: 0.0                 // A (desde backend)
    property var   data: []                 // crudo -> CSV {angle:°, ch1:A, ch2:A}
    property real  angleAbs: NaN
    property real  angleRel: NaN

    // Vista
    property string viewMode: "curve"       // "curve" | "cycles"

    // Rango X (solo tramo de interés)
    property real xMinDeg: 70.8
    property real xMaxDeg: 78.2

    // Detección de ida 39→50 (base)
    property real forwardStartDeg: 0.0
    property real forwardEndDeg:   0.0
    property real epsAngle: 0.003
    property real prevAngle: NaN
    property bool collectingForward: false
    property bool pendingCycleSnapshot: false

    // Acumulación del ciclo actual (muestras crudas por ángulo)
    // key -> {angle, ch1Vals:[], ch2Vals:[]}
    property var cycleBuckets: ({})

    // Historial GLOBAL por ángulo: últimas N medianas de ciclos
    // key -> {angle, ch1Meds:[], ch2Meds:[]}
    property var globalAngleHistory: ({})

    // Ventana de ciclos para la mediana-de-medianas
    property int  medWindowCycles: 1      //5, quiero pasar de 5 para 1  
    property int  minCycleSamples: 5      // mínimo de muestras válidas en un ciclo
    property int  angleKeyDecimals: 2

    // Serie para "ángulo máximo vs nº de ciclo"
    property var cycleMaxCh1Angles: []
    property var cycleMaxCh2Angles: []
    property int cycleIndex: 0

    // Labels de máximos (de la curva agregada actual)
    property real maxCh1Angle: NaN
    property real maxCh1Value_mA: NaN
    property real maxCh2Angle: NaN
    property real maxCh2Value_mA: NaN

    // ======== Robustez de ida/vuelta ========
    property real hysteresisDeg: 0.02   // ° extra sobre epsAngle para ignorar jitter
    property int  incNeeded: 3          // subidas consecutivas para arrancar ida
    property int  decNeeded: 3          // bajadas consecutivas para cerrar/abortar
    property int  incStreak: 0
    property int  decStreak: 0

    property real startGateDelta: 0.20  // debe rebasar 39 + delta
    property real endGateDelta:   0.20  // debe alcanzar 50 - delta
    property real coverageMinSpan: 4.0  // ° recorridas mínimas durante la ida

    property bool sawStartGate: false   // cruzó 39+delta
    property bool sawEndGate:   false   // cruzó 50-delta
    property real minA:  9999.0         // mínimo ángulo dentro del ciclo
    property real maxA: -9999.0         // máximo ángulo dentro del ciclo
    // ========================================

    // === helpers estadísticos ===
    function _median(arr) {
        if (!arr || arr.length === 0) return NaN;
        const c = arr.slice().sort((x,y)=>x-y);
        const m = Math.floor(c.length/2);
        return (c.length % 2) ? c[m] : 0.5*(c[m-1] + c[m]);
    }

    // === agrega al historial con ventana deslizante de N ciclos ===
    function _pushHist(histArr, v, N) {
        if (!isFinite(v)) return;
        histArr.push(v);
        if (histArr.length > N) histArr.splice(0, histArr.length - N);
    }

    // === devuelve la MEDIANA-DE-MEDIANAS (mA) para un ángulo y canal ===
    function getMoM_mA(key, channel) {
        const g = win.globalAngleHistory[key]; // {angle, ch1Meds:[], ch2Meds:[]}
        if (!g) return NaN;
        const src = (channel === 1) ? g.ch1Meds : g.ch2Meds;
        if (!src || src.length === 0) return NaN;
        if (win.debugLogs) {
            const arrStr = src.map(v => (v*1000).toFixed(5));
            console.log(`[MoM] key=${key}° ch${channel} base(mA)[${src.length}]=[${arrStr.join(", ")}]`);
        }
        return _median(src) * 1000.0; // a mA
    }

    // === recomputa labels de MÁXIMOS desde la curva agregada actual ===
    function updateMaxLabelsFromMoM() {
        let best1 = {angle: NaN, val: -Infinity};
        let best2 = {angle: NaN, val: -Infinity};

        const keys = Object.keys(win.globalAngleHistory).sort((a,b)=>parseFloat(a)-parseFloat(b));
        for (let i=0;i<keys.length;i++){
            const k = keys[i];
            const a = parseFloat(k);
            if (a < win.xMinDeg || a > win.xMaxDeg) continue;

            const m1 = getMoM_mA(k, 1);
            const m2 = getMoM_mA(k, 2);
            if (isFinite(m1) && m1 > best1.val) { best1.val = m1; best1.angle = a; }
            if (isFinite(m2) && m2 > best2.val) { best2.val = m2; best2.angle = a; }
        }
        win.maxCh1Angle    = best1.angle;
        win.maxCh1Value_mA = (isFinite(best1.val) ? best1.val : NaN);
        win.maxCh2Angle    = best2.angle;
        win.maxCh2Value_mA = (isFinite(best2.val) ? best2.val : NaN);
    }

    // === cierra un ciclo con validaciones de ida (ahora busca MÁXIMOS) ===
    function finalizeCycleAndAppendPoint() {
        // Validaciones de fin real de ida
        const span = win.maxA - win.minA;
        if (!win.sawEndGate || !(isFinite(span) && span >= win.coverageMinSpan)) {
            if (win.debugLogs) console.warn("[cycle] ABORT finalize: endGate=", win.sawEndGate, " span=", span);
            // limpiar y salir sin graficar
            win.cycleBuckets = ({})
            win.incStreak = 0; win.decStreak = 0
            win.sawStartGate = false; win.sawEndGate = false
            win.minA = 9999.0; win.maxA = -9999.0
            return;
        }

        // 1) validar muestras del ciclo
        let sampleCount = 0;
        for (let k in win.cycleBuckets) {
            const b = win.cycleBuckets[k];
            sampleCount += (b?.ch1Vals?.length || 0) + (b?.ch2Vals?.length || 0);
        }
        if (sampleCount < win.minCycleSamples) {
            if (win.debugLogs) console.warn("[cycle] descartado por pocas muestras:", sampleCount);
            win.cycleBuckets = ({})
            win.incStreak = 0; win.decStreak = 0
            win.sawStartGate = false; win.sawEndGate = false
            win.minA = 9999.0; win.maxA = -9999.0
            return;
        }

        // 2) mediana del ciclo por ángulo (A) y alimentar historial
        const keys = Object.keys(win.cycleBuckets).sort((a,b)=>parseFloat(a)-parseFloat(b));
        let firstCycle = (win.cycleIndex === 0);

        let cycleMax1 = {angle: NaN, valA: -Infinity};
        let cycleMax2 = {angle: NaN, valA: -Infinity};

        for (let i=0;i<keys.length;i++){
            const k = keys[i];
            const b = win.cycleBuckets[k];
            const med1 = _median(b.ch1Vals);
            const med2 = _median(b.ch2Vals);

            if (isFinite(med1) && med1 > cycleMax1.valA) { cycleMax1.valA = med1; cycleMax1.angle = b.angle; }
            if (isFinite(med2) && med2 > cycleMax2.valA) { cycleMax2.valA = med2; cycleMax2.angle = b.angle; }

            let g = win.globalAngleHistory[k];
            if (!g) { g = { angle: b.angle, ch1Meds: [], ch2Meds: [] }; win.globalAngleHistory[k] = g; }
            if (isFinite(med1)) _pushHist(g.ch1Meds, med1, win.medWindowCycles);
            if (isFinite(med2)) _pushHist(g.ch2Meds, med2, win.medWindowCycles);
        }

        // 3) decidir el punto del gráfico "ángulo vs ciclo"
        let plotMax1 = {angle: NaN, val_mA: -Infinity};
        let plotMax2 = {angle: NaN, val_mA: -Infinity};

        if (firstCycle) {
            if (isFinite(cycleMax1.angle)) { plotMax1.angle = cycleMax1.angle; plotMax1.val_mA = cycleMax1.valA*1000.0; }
            if (isFinite(cycleMax2.angle)) { plotMax2.angle = cycleMax2.angle; plotMax2.val_mA = cycleMax2.valA*1000.0; }
        } else {
            const gkeys = Object.keys(win.globalAngleHistory).sort((a,b)=>parseFloat(a)-parseFloat(b));
            for (let i=0;i<gkeys.length;i++){
                const k = gkeys[i];
                const a = parseFloat(k);
                if (a < win.xMinDeg || a > win.xMaxDeg) continue;

                const m1 = getMoM_mA(k, 1);
                const m2 = getMoM_mA(k, 2);
                if (isFinite(m1) && m1 > plotMax1.val_mA) { plotMax1.val_mA = m1; plotMax1.angle = a; }
                if (isFinite(m2) && m2 > plotMax2.val_mA) { plotMax2.val_mA = m2; plotMax2.angle = a; }
            }
        }

        // 4) agregar punto a series
        if (isFinite(plotMax1.angle)) win.cycleMaxCh1Angles.push(plotMax1.angle);
        if (isFinite(plotMax2.angle)) win.cycleMaxCh2Angles.push(plotMax2.angle);
        win.cycleIndex = Math.max(win.cycleMaxCh1Angles.length, win.cycleMaxCh2Angles.length);

        if (win.debugLogs) {
            console.log(`[cycle-plot] idx=${win.cycleIndex} -> `
                        + `CH1=${isFinite(plotMax1.angle)?plotMax1.angle.toFixed(3):"NaN"}°, `
                        + `CH2=${isFinite(plotMax2.angle)?plotMax2.angle.toFixed(3):"NaN"}°`);
        }

        // 5) actualizar labels desde la curva agregada (MoM)
        updateMaxLabelsFromMoM();

        // 6) repintar
        plot.requestPaint();
        plotCycles.requestPaint();

        // 7) limpiar estado de ciclo
        win.cycleBuckets = ({})
        win.incStreak = 0; win.decStreak = 0
        win.sawStartGate = false; win.sawEndGate = false
        win.minA = 9999.0; win.maxA = -9999.0
    }

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

                // ⬅️ Logo a la izquierda
                Image {
                    id: logo
                    source: "assets/moray_logo.png"   // ruta relativa al LDRMonitor.qml
                    Layout.preferredWidth: 36
                    Layout.preferredHeight: 36
                    fillMode: Image.PreserveAspectFit
                    smooth: true
                    mipmap: true
                    antialiasing: true
                    Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter

                }

                // Título
                Label { text: "Sensor SPR"; color: "white"; font.bold: true; font.pixelSize: 20 }

                Rectangle { Layout.fillWidth: true; color: "transparent" }

                Button {
                    text: "Definir angulos" 
                    Layout.preferredHeight: 36; Layout.preferredWidth: 120; font.pixelSize: 14
                    ToolTip.visible: hovered
                    ToolTip.text: "Establece los ángulos máximo y mínimo de barrido"
                    onClicked: {
                        backend.viewAngles()   // solicita los ángulos actuales al backend
                        winAngles.open()
                    }
                }

                Button {
                    text: win.viewMode === "curve" ? "Ver Ciclos" : "Ver Curva"
                    onClicked: {
                        win.viewMode = (win.viewMode === "curve") ? "cycles" : "curve"
                        plot.requestPaint()
                        plotCycles.requestPaint()
                    }
                    Layout.preferredHeight: 36; Layout.preferredWidth: 120; font.pixelSize: 14
                    ToolTip.visible: hovered
                    ToolTip.text: "Alterna entre Curva (Ángulo vs Corriente) y Ciclos (máximos)"
                }

                Switch { checked: win.active; onToggled: backend.setActive(checked) }
                Label { text: win.active ? "Activo" : "Inactivo"; color: "white"; font.pixelSize: 14 }

                Button {
                    text: "Cero relativo"
                    onClicked: backend.setRelativeZero()
                    Layout.preferredHeight: 36
                    Layout.preferredWidth: 130
                    font.pixelSize: 14
                    ToolTip.visible: hovered
                    ToolTip.text: "Captura el ángulo absoluto actual como cero (ZC)"
                }

                //Button { text: "Salir"; onClicked: Qt.quit(); Layout.preferredHeight: 36; Layout.preferredWidth: 80; font.pixelSize: 14 }
                Button {
                    text: "Salir"
                    onClicked: {
                        if (win.active) backend.setActive(false)   // <-- envía "s\n" al ESP32
                        Qt.quit()
                    }
                    Layout.preferredHeight: 36
                    Layout.preferredWidth: 80
                    font.pixelSize: 14
                }
            }
        }


        // ===== Panel de lecturas (mA) =====
        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 100
            radius: 12
            color: "#111827"
            border.color: "#1f2937"; border.width: 1

            RowLayout {
                anchors.fill: parent; anchors.margins: 16; spacing: 32
                // CH1 (corriente es * 1000 y resistencia es entre 1000)
                ColumnLayout {
                    Label { text: "CH1 Resistencia (kΩ)"; color: "#22c55e"; font.pixelSize: 14; font.bold: true }
                    Label { text: isFinite(ch1) ? (ch1/1000).toFixed(2) : "—"; color: "white"; font.pixelSize: 32; font.bold: true }
                }
                // CH2 (corriente es * 1000 y resistencia es entre 1000)
                ColumnLayout {
                    Label { text: "CH2 Resistencia (kΩ)"; color: "#60a5fa"; font.pixelSize: 14; font.bold: true }
                    Label { text: isFinite(ch2) ? (ch2/1000).toFixed(2) : "—"; color: "white"; font.pixelSize: 32; font.bold: true }
                }
                ColumnLayout {
                    Label { text: "Ángulo abs (°)"; color: "#fbbf24"; font.pixelSize: 14; font.bold: true }
                    Label { text: isFinite(win.angleAbs) ? win.angleAbs.toFixed(2) : "—"; color: "white"; font.pixelSize: 28; font.bold: true }
                }
                ColumnLayout {
                    Label { text: "Ángulo rel (°)"; color: "#f472b6"; font.pixelSize: 14; font.bold: true }
                    Label { text: isFinite(win.angleRel) ? win.angleRel.toFixed(2) : "—"; color: "white"; font.pixelSize: 28; font.bold: true }
                }

                Rectangle { Layout.fillWidth: true; color: "transparent" }

                RowLayout {
                    Button {
                        text: "Iniciar"
                        enabled: !win.active
                        onClicked: {
                            win.prevAngle = NaN
                            win.collectingForward = false
                            win.pendingCycleSnapshot = true
                            win.cycleBuckets = ({})

                            // reset robustez
                            win.incStreak = 0; win.decStreak = 0
                            win.sawStartGate = false; win.sawEndGate = false
                            win.minA = 9999.0; win.maxA = -9999.0

                            backend.setActive(true)
                            if (win.debugLogs) console.log("[cycle] START (ida 39→50)")
                        }
                        Layout.preferredHeight: 45; Layout.preferredWidth: 120; font.pixelSize: 16
                    }
                    Button {
                        text: "Detener"
                        enabled: win.active
                        onClicked: {
                            backend.setActive(false)
                            // El guardado ocurre en onActiveChanged(false)
                        }
                        Layout.preferredHeight: 45; Layout.preferredWidth: 120; font.pixelSize: 16
                    }
                    Button {
                        text: "Limpiar todo"
                        onClicked: {
                            win.data = []
                            win.cycleBuckets = ({})
                            win.globalAngleHistory = ({})
                            win.cycleMaxCh1Angles = []
                            win.cycleMaxCh2Angles = []
                            win.cycleIndex = 0
                            win.maxCh1Angle = NaN; win.maxCh1Value_mA = NaN
                            win.maxCh2Angle = NaN; win.maxCh2Value_mA = NaN

                            win.incStreak = 0; win.decStreak = 0
                            win.sawStartGate = false; win.sawEndGate = false
                            win.minA = 9999.0; win.maxA = -9999.0

                            plot.requestPaint(); plotCycles.requestPaint()
                        }
                        Layout.preferredHeight: 45; Layout.preferredWidth: 120; font.pixelSize: 16
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

            // ---- Corriente (mA) vs Ángulo (°) ----
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

                    const ml=100, mr=10, mt=10, mb=30;
                    const pw=W-ml-mr, ph=H-mt-mb; if (pw<=0||ph<=0) return;

                    // marco
                    ctx.strokeStyle="#1f2937"; ctx.strokeRect(ml,mt,pw,ph);

                    const xmin=win.xMinDeg, xmax=win.xMaxDeg;
                    const xspan=Math.max(1e-9,xmax-xmin);
                    const xMap=(vx)=> ml + ((vx-xmin)/xspan)*pw;

                    // construir data desde la mediana-de-medianas
                    const keys = Object.keys(win.globalAngleHistory).sort((a,b)=>parseFloat(a)-parseFloat(b));
                    const dataArr=[];
                    for (let i=0;i<keys.length;i++){
                        const k=keys[i]; const a=parseFloat(k);
                        if (a<xmin||a>xmax) continue;
                        const ch1_mA = getMoM_mA(k,1);
                        const ch2_mA = getMoM_mA(k,2);
                        if (!isFinite(ch1_mA) && !isFinite(ch2_mA)) continue;
                        dataArr.push({angle:a, ch1_mA:ch1_mA, ch2_mA:ch2_mA});
                    }
                    if (dataArr.length===0){
                        ctx.fillStyle="#9ca3af"; ctx.fillText("Sin datos agregados aún…", ml+10, mt+20);
                        return;
                    }

                    // eje Y auto
                    let ymin=+Infinity,ymax=-Infinity;
                    for (let i=0;i<dataArr.length;i++){
                        const d=dataArr[i];
                        if (isFinite(d.ch1_mA)){ymin=Math.min(ymin,d.ch1_mA); ymax=Math.max(ymax,d.ch1_mA);}
                        if (isFinite(d.ch2_mA)){ymin=Math.min(ymin,d.ch2_mA); ymax=Math.max(ymax,d.ch2_mA);}
                    }
                    if (!(ymin<Infinity&&ymax>-Infinity)){ymin=0;ymax=1;}
                    if (Math.abs(ymax-ymin)<1e-9) ymax=ymin+1.0;
                    const pad=(ymax-ymin)*0.05; const yMin=ymin-pad, yMax=ymax+pad;
                    const yMap=(vy)=> mt + ph*(1-(vy-yMin)/(yMax-yMin));

                    // grilla H
                    ctx.strokeStyle="#253041"; ctx.lineWidth=1; ctx.beginPath();
                    for (let gy=0; gy<=6; gy++){ const y=mt+ph*(gy/6); ctx.moveTo(ml,y); ctx.lineTo(ml+pw,y); }
                    ctx.stroke();

                    // ticks X exactos
                    const step=Math.max(0.01, plot.xTickStep);
                    const firstTick=Math.ceil(xmin/step)*step;
                    const lastTick=Math.floor(xmax/step)*step;
                    ctx.beginPath();
                    for (let v=firstTick; v<=lastTick+1e-9; v+=step){
                        const x=xMap(v); ctx.moveTo(x,mt); ctx.lineTo(x,mt+ph);
                    }
                    ctx.strokeStyle="#253041"; ctx.stroke();

                    // etiquetas
                    ctx.fillStyle="#9ca3af"; ctx.font="12px sans-serif"; ctx.textAlign="center";
                    for (let v=firstTick; v<=lastTick+1e-9; v+=step){
                        const x=xMap(v); ctx.fillText(v.toFixed(step<1?2:0), x, mt+ph+18);
                    }
                    ctx.textAlign="right";
                    for (let gy=0; gy<=6; gy++){
                        const vy=yMin+(yMax-yMin)*(gy/6); const y=mt+ph*(1-gy/6);
                        ctx.fillText(vy.toFixed(3), ml-6, y+4);
                    }
                    ctx.textAlign="center";
                    ctx.fillText("Ángulo (°)", ml+pw/2, H-6);
                    ctx.save(); ctx.translate(12, mt+ph/2+30); ctx.rotate(-Math.PI/2);
                    ctx.fillText("Corriente (mA)", 0, 0); ctx.restore();

                    function drawPolyline(color, acc){
                        ctx.strokeStyle=color; ctx.lineWidth=2; ctx.beginPath();
                        let started=false;
                        for (let i=0;i<dataArr.length;i++){
                            const d=dataArr[i]; const v=acc(d); if (!isFinite(v)) continue;
                            const x=xMap(d.angle), y=yMap(v);
                            if(!started){ctx.moveTo(x,y); started=true;} else ctx.lineTo(x,y);
                        }
                        ctx.stroke();
                        ctx.fillStyle=color; const r=2.0;
                        for (let i=0;i<dataArr.length;i++){
                            const d=dataArr[i]; const v=acc(d); if (!isFinite(v)) continue;
                            const x=xMap(d.angle), y=yMap(v);
                            ctx.beginPath(); ctx.arc(x,y,r,0,Math.PI*2); ctx.fill();
                        }
                    }
                    drawPolyline("#22c55e", d=>d.ch1_mA);
                    drawPolyline("#60a5fa", d=>d.ch2_mA);
                }
            }

            // ---- Ángulo del máximo vs Nº de ciclo ----
            Canvas {
                id: plotCycles
                visible: win.viewMode === "cycles"
                anchors.fill: parent
                anchors.margins: 20
                antialiasing: true

                onPaint: {
                    const ctx=getContext("2d"); const W=width,H=height;
                    ctx.clearRect(0,0,W,H); ctx.fillStyle="#111827"; ctx.fillRect(0,0,W,H);

                    const ml=80,mr=10,mt=10,mb=30; const pw=W-ml-mr, ph=H-mt-mb;
                    if (pw<=0||ph<=0) return;
                    ctx.strokeStyle="#1f2937"; ctx.strokeRect(ml,mt,pw,ph);

                    ctx.fillStyle="#9ca3af"; ctx.font="12px sans-serif"; ctx.textAlign="center";
                    ctx.fillText("Número de ciclo", ml+pw/2, H-10);
                    ctx.save(); ctx.translate(16, mt+ph/2); ctx.rotate(-Math.PI/2);
                    ctx.fillText("Ángulo del máximo (°)", 0, 0); ctx.restore();

                    const n1=win.cycleMaxCh1Angles.length, n2=win.cycleMaxCh2Angles.length;
                    const N=Math.max(n1,n2); if (N===0){
                        ctx.fillStyle="#9ca3af"; ctx.textAlign="left";
                        ctx.fillText("Sin ciclos completados aún…", ml+10, mt+20); return;
                    }
                    const xmin=1,xmax=Math.max(2,N), xspan=xmax-xmin;
                    const xMap=(vx)=> ml + ((vx-xmin)/xspan)*pw;

                    let ymin=+Infinity,ymax=-Infinity;
                    for (let i=0;i<n1;i++){ const v=win.cycleMaxCh1Angles[i]; if (isFinite(v)){ymin=Math.min(ymin,v); ymax=Math.max(ymax,v);} }
                    for (let i=0;i<n2;i++){ const v=win.cycleMaxCh2Angles[i]; if (isFinite(v)){ymin=Math.min(ymin,v); ymax=Math.max(ymax,v);} }
                    if (!(ymin<Infinity&&ymax>-Infinity)) { ymin=win.xMinDeg; ymax=win.xMaxDeg; }
                    if (Math.abs(ymax-ymin)<1e-6) ymax=ymin+1.0;
                    const pad=(ymax-ymin)*0.08; ymin-=pad; ymax+=pad;
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
                    const maxTicks=10, step=Math.max(1, Math.floor(N/maxTicks));
                    for (let c=1; c<=N; c+=step){ const x=xMap(c); ctx.fillText(c.toString(), x, mt+ph+18); }

                    function drawSeries(color, arr){
                        ctx.strokeStyle=color; ctx.lineWidth=2; ctx.beginPath();
                        let started=false;
                        for (let i=1;i<=arr.length;i++){
                            const v=arr[i-1]; if (!isFinite(v)) continue;
                            const x=xMap(i), y=yMap(v);
                            if(!started){ctx.moveTo(x,y); started=true;} else ctx.lineTo(x,y);
                        }
                        ctx.stroke(); ctx.fillStyle=color;
                        for (let i=1;i<=arr.length;i++){
                            const v=arr[i-1]; if (!isFinite(v)) continue;
                            const x=xMap(i), y=yMap(v);
                            ctx.beginPath(); ctx.arc(x,y,3,0,Math.PI*2); ctx.fill();
                        }
                    }
                    drawSeries("#22c55e", win.cycleMaxCh1Angles);
                    drawSeries("#60a5fa", win.cycleMaxCh2Angles);
                }
            }
        }

        // ===== Labels de máximos de la curva agregada =====
        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 70
            radius: 12
            color: "#111827"
            border.color: "#1f2937"; border.width: 1

            RowLayout {
                anchors.fill: parent; anchors.margins: 16; spacing: 32

                ColumnLayout {
                    Label { text: "Máximo CH1 (curva agregada)"; color: "#22c55e"; font.pixelSize: 14; font.bold: true }
                    Label {
                        text: (isFinite(maxCh1Angle) && isFinite(maxCh1Value_mA))
                              ? `Ángulo: ${maxCh1Angle.toFixed(2)}° — Corriente: ${maxCh1Value_mA.toFixed(4)} mA`
                              : "—"
                        color: "white"; font.pixelSize: 18; font.bold: true
                    }
                }
                ColumnLayout {
                    Label { text: "Máximo CH2 (curva agregada)"; color: "#60a5fa"; font.pixelSize: 14; font.bold: true }
                    Label {
                        text: (isFinite(maxCh2Angle) && isFinite(maxCh2Value_mA))
                              ? `Ángulo: ${maxCh2Angle.toFixed(2)}° — Corriente: ${maxCh2Value_mA.toFixed(4)} mA`
                              : "—"
                        color: "white"; font.pixelSize: 18; font.bold: true
                    }
                }
            }
        }
    }

    // ===== Conexiones con Backend =====
    Connections {
        target: backend

        function onNewLDRSampleWithAngle(tt, v1, v2, absDeg, relDeg) {
            win.ch1 = v1; win.ch2 = v2;
            win.angleAbs = absDeg; win.angleRel = relDeg;

            const a = relDeg;
            const prev = win.prevAngle;
            const eps  = Math.max(win.epsAngle, win.hysteresisDeg);
            const inside = isFinite(a) && a >= win.forwardStartDeg && a <= win.forwardEndDeg;

            let inc = false, dec = false;
            if (isFinite(prev)) {
                const d = a - prev;
                if (d >  eps) inc = true;
                if (d < -eps) dec = true;
            }

            // rachas
            if (inc) { win.incStreak++; win.decStreak = 0; }
            else if (dec) { win.decStreak++; win.incStreak = 0; }

            // INICIO de ciclo (ida 39→50)
            if (!win.collectingForward) {
                if (inside && win.incStreak >= win.incNeeded) {
                    win.collectingForward = true;
                    win.pendingCycleSnapshot = true;
                    win.cycleBuckets = ({});

                    win.sawStartGate = (a >= win.forwardStartDeg + win.startGateDelta);
                    win.sawEndGate   = false;
                    win.minA = a; win.maxA = a;

                    if (win.debugLogs) console.log(`[cycle] ENTER a≈${a.toFixed(3)}° (incStreak=${win.incStreak})`);
                }
            } else {
                // DURANTE el ciclo: acumular si está dentro
                if (inside) {
                    if (a < win.minA) win.minA = a;
                    if (a > win.maxA) win.maxA = a;
                    if (a >= (win.forwardStartDeg + win.startGateDelta)) win.sawStartGate = true;
                    if (a >= (win.forwardEndDeg   - win.endGateDelta))   win.sawEndGate   = true;

                    win.data.push({ angle: a, ch1: v1, ch2: v2 });
                    const key = a.toFixed(win.angleKeyDecimals);
                    var bc = win.cycleBuckets[key];
                    if (!bc) { bc = { angle: parseFloat(key), ch1Vals: [], ch2Vals: [] }; win.cycleBuckets[key] = bc; }
                    if (isFinite(v1)) bc.ch1Vals.push(v1);
                    if (isFinite(v2)) bc.ch2Vals.push(v2);
                }

                // FIN válido: bajada sostenida después de haber alcanzado fin, o salir por arriba
                const reachedEnd = win.sawEndGate;
                const endByDec   = (reachedEnd && win.decStreak >= win.decNeeded);
                const endByExit  = (!inside && isFinite(a) && a > win.forwardEndDeg);
                const ending     = endByDec || endByExit;

                // Abort: bajada sostenida antes de alcanzar el fin
                const abortByEarlyDec = (!reachedEnd && win.decStreak >= win.decNeeded);

                if (ending && win.pendingCycleSnapshot) {
                    finalizeCycleAndAppendPoint();
                    win.pendingCycleSnapshot = false;
                    win.collectingForward = false;
                    win.cycleBuckets = ({})
                    win.cycleIndex = Math.max(win.cycleMaxCh1Angles.length, win.cycleMaxCh2Angles.length);
                    if (win.debugLogs) console.log("[cycle] EXIT (finalized)");
                } else if (abortByEarlyDec && win.pendingCycleSnapshot) {
                    if (win.debugLogs) console.warn("[cycle] EXIT ABORT (vuelta antes de fin)");
                    win.pendingCycleSnapshot = false;
                    win.collectingForward = false;
                    win.cycleBuckets = ({})
                    win.incStreak = 0; win.decStreak = 0
                    win.sawStartGate = false; win.sawEndGate = false
                    win.minA = 9999.0; win.maxA = -9999.0
                }
            }

            win.prevAngle = a;
        }

        function onActiveChanged(a) {
            win.active = a;
            if (!a) {
                // Guarda crudo
                if (win.data.length > 0) {
                    backend.saveCsv(win.data);
                    win.data = [];
                }
                // Guarda Ángulo vs Ciclo (usando las dos series)
                if (win.cycleMaxCh1Angles.length > 0 || win.cycleMaxCh2Angles.length > 0) {
                    backend.saveAngleVsCycleCsv(win.cycleMaxCh1Angles, win.cycleMaxCh2Angles);
                }
            }
        }

        function onAngleUpdate(absDeg, relDeg) {
            win.angleAbs = absDeg;
            win.angleRel = relDeg;
        }

        function onCsvSaved(path) { console.log("CSV guardado en: " + path); }
        
        function onCsvError(msg)  { console.log("Error CSV: " + msg); }
        
        function onAngleMaxMin(angMin, angMax) {
            win.forwardStartDeg = angMin;
            win.forwardEndDeg   = angMax;
        }
    }

    // Compatibilidad con pantallas antiguas
    Connections { target: backend; function onNewSample(tt, v0, v1) { win.ch1 = v0; win.ch2 = v1; } }
}

