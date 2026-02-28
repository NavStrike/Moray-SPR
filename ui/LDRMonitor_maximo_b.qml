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
    visibility: Window.FullScreen
    //visibility: Window.Maximized
    width: 800
    height: 480
    title: "LDR Monitor"
    color: "#0f172a"
    palette.buttonText: "black"
    
    // ===== Otros archivos importados =====
    AnglesDialog {id: winAngles}
    SpeedsDialog {id: winSpeeds}
    ToolsDialog {id: winTools}

    // ===== Estado / datos =====
    property bool active: false
    property bool debugLogs: true
    property string realTime: NaN                 // Tiempo real
    property real measurementTime: 0.0      // Tiempo de medida
    property real ch1: 0.0                  // Lectura de canal 1
    property real ch2: 0.0                  // Lectura de canal 2
    property real angleAbs: NaN             // Lectura de angulo absoluto
    property real angleRel: NaN             // Lectura de angulo relativo
    property var  data: []                  // crudo -> CSV {angle:°, ch1:<value>, ch2:<value>}

    // Vista
    property string viewMode: "curve"           // "curve" | "cycles"

    // Dispositivo usado para realizar la lectura del laser
    property string device: "unknown"           // "ldr" | "photodetector" | "unknown"
    property string deviceUnites: "current"    // "current" | "resistance"

    // Rango X (solo tramo de interés)
    property real xMinDeg: 0.0  
    property real xMaxDeg: 0.0

    // Limites fijos mecanismo
    // Solo cambiar al modificar el hardware
    // Limite movimiento
    property real upLimit: 85
    property real downLimit: 15

    // Limite de velocidad
    property real upLimitVel: 40
    property real downLimitVel: 0.1

    // Velocidades minima y maxima
    property real velMinCycle: 0
    property real velMaxCycle: 0

    // Sustancia
    property  var substances: []
    property  var anglesSubstances: []
    property string substance: ""

    // Fitros
    property string filterType: "ema"   // "median" | "ema"
    property real emaAlpha: 0.15        // factor de suavizado para EMA (0-1)
    property int  kernelMed: 5          // tamaño del kernel para mediana

    // Detección de ida forwardStartDeg → forwardEndDeg
    property real forwardStartDeg: 0
    property real forwardEndDeg:   0
    property real epsAngle: 0.003
    property real prevAngle: NaN
    property bool collectingForward: false
    property bool pendingCycleSnapshot: false

    // Acumulación del ciclo actual
    // [{angle:°, ch1:<value>, ch2:<value>}, ...]
    property var dataCycle: []

    // Acumulación del ciclo actual con filtro (Data para graficar)
    // {angle:[], ch1:[], ch2:[]}
    property var dataCycleFilter: []

    // Parametros de ciclo
    property int  minCycleSamples: 5      // mínimo de muestras válidas en un ciclo
    property int  angleKeyDecimals: 2

    // Serie para "ángulo máximo o mínimo vs nº de ciclo"
    property var cyclePeakCh1Angles: []
    property var cyclePeakCh1Times: []
    property var cyclePeakCh2Angles: []
    property var cyclePeakCh2Times: []
    property int cycleIndex: 0

    // Labels de los picos (peaks) de la curva agregada actual
    // Format -> [Angle, Value, Time]
    property real peakCh1Angle: NaN
    property real peakCh1Value: NaN
    property real peakCh1Time: NaN
    property real peakCh2Angle: NaN
    property real peakCh2Value: NaN
    property real peakCh2Time: NaN

    // ======== Robustez de ida/vuelta ========
    property real hysteresisDeg: 0.01   // ° extra sobre epsAngle para ignorar jitter
    property int  incNeeded: 3          // subidas consecutivas para arrancar ida
    property int  decNeeded: 3          // bajadas consecutivas para cerrar/abortar
    property int  incStreak: 0          // Numero de subidas consecutivas realizadas
    property int  decStreak: 0          // Numero de bajadas consecutivas realizadas

    property real startGateDelta: 0.50  // debe rebasar forwardStartDeg + delta
    property real endGateDelta:   0.50  // debe alcanzar forwardEndDeg - delta
    property real coverageMinSpan: 3.0  // ° recorridas mínimas durante la ida

    property bool sawStartGate: false   // cruzó xMinDeg+delta
    property bool sawEndGate:   false   // cruzó 50-delta
    property real minA:  9999.0         // mínimo ángulo dentro del ciclo
    property real maxA: -9999.0         // máximo ángulo dentro del ciclo
    // ========================================

    // ===== Variables de prueba =====

    // === helpers estadísticos ===
    // Filtro de mediana
    function _median(arr, kernelSize) {
        if (!arr || arr.length === 0) return NaN;
        const half = Math.floor(kernelSize/2);
        const result = [];

        for (let i = 0; i < arr.length; i++) {
            const window = [];
            for (let j = -half; j <= half; j++) {
                let index = i + j;
                if (index < 0) {index = 0;}
                if (index >= arr.length) {index = arr.length - 1;}
                window.push(arr[index]);
            }
            window.sort((a, b) => a - b);
            const median = window[Math.floor(window.length / 2)];
            result.push(median);
        }
        return result;
    }
    
    // Filtro EMA (Media Móvil Exponencial)
    function _ema(arr, alpha) {
        if (!arr || arr.length === 0) return NaN;
        let ema = arr[0];
        let result = [ema];
        for (let i = 1; i < arr.length; i++) {
            ema = alpha * arr[i] + (1 - alpha) * ema;
            result.push(ema);
        }
        return result;
    }

    // Funcion unificada de filtros
    function applyFilter(arr) {
        if (!arr || arr.length === 0) return NaN;

        win.dataCycleFilter = [];

        let arrTime = arr.map(d => d.time);
        let arrAngle = arr.map(d => d.angle);
        let arrCh1 = arr.map(d => d.ch1);
        let arrCh2 = arr.map(d => d.ch2);
        

        let arrCh1Filter = []; let arrCh2Filter = [];

        if (win.filterType === "median") {
            arrCh1Filter = _median(arrCh1, win.windowMed);
            arrCh2Filter = _median(arrCh2, win.windowMed);
        } else if (win.filterType === "ema") {
            arrCh1Filter = _ema(arrCh1, win.emaAlpha);
            arrCh2Filter = _ema(arrCh2, win.emaAlpha);
        }
        for (let i = 0; i < arr.length; i++) {
            dataCycleFilter.push({time: arrTime[i], angle: arrAngle[i], ch1: arrCh1Filter[i], ch2: arrCh2Filter[i]});
        }
    }

    // === adquiere labels de MÁXIMOS desde la curva actual ===
    function updatePeakLabelsFromDataFilter() {
        let valCh1 = dataCycleFilter.map(d => d.ch1); let valCh2 = dataCycleFilter.map(d => d.ch2);

        let best1 = {time: NaN, angle: NaN, val: -Infinity};
        let best2 = {time: NaN, angle: NaN, val: -Infinity};

        if (win.deviceUnites == "current"){
            best1.val = Math.min(...valCh1);
            best2.val = Math.min(...valCh2);
        } else {
            best1.val = Math.max(...valCh1);
            best2.val = Math.max(...valCh2);
        }

        let posBest1 = valCh1.indexOf(best1.val);
        best1.time = win.dataCycleFilter[posBest1].time;
        best1.angle = win.dataCycleFilter[posBest1].angle;

        let posBest2 = valCh2.indexOf(best2.val);
        best2.time = win.dataCycleFilter[posBest2].time;
        best2.angle = win.dataCycleFilter[posBest2].angle;

        win.peakCh1Time = (isFinite(best1.time) ? best1.time : NaN);
        win.peakCh1Angle = (isFinite(best1.angle) ? best1.angle : NaN);
        win.peakCh1Value = (isFinite(best1.val) ? best1.val : NaN);
        win.cyclePeakCh1Times.push(best1.time);
        win.cyclePeakCh1Angles.push(best1.angle);

        win.peakCh2Time = (isFinite(best2.time) ? best2.time : NaN);
        win.peakCh2Angle = (isFinite(best2.angle) ? best2.angle : NaN);
        win.peakCh2Value = (isFinite(best2.val) ? best2.val : NaN);
        win.cyclePeakCh2Times.push(best2.time);
        win.cyclePeakCh2Angles.push(best2.angle);

        console.log(win.peakCh2Time, win.peakCh2Angle, win.peakCh2Value);
    }

    function resetValues(){
        win.dataCycle = [];
        win.incStreak = 0; win.decStreak = 0
        win.sawStartGate = false; win.sawEndGate = false
        win.minA = 9999.0; win.maxA = -9999.0
    }

    // === cierra un ciclo con validaciones de ida (ahora busca MÁXIMOS) ===
    function finalizeCycleAndAppendPoint() {
        // Validaciones de fin real de ida
        const span = win.maxA - win.minA;
        if (!win.sawEndGate || !(isFinite(span) && span >= win.coverageMinSpan)) {
            if (win.debugLogs) console.warn("[cycle] ABORT finalize: endGate=", win.sawEndGate, " span=", span);
            // limpiar y salir sin graficar
            resetValues()
            return;
        }

        // 1) validar muestras del ciclo
        let sampleCount = win.dataCycle.length;
        if (sampleCount < win.minCycleSamples) {
            if (win.debugLogs) console.warn("[cycle] descartado por pocas muestras:", sampleCount);
            resetValues()
            return;
        }

        // 2) aplicar filtro
        applyFilter(win.dataCycle);

        // 3) actualizar labels desde la curva filtrada
        updatePeakLabelsFromDataFilter();

        // 4) Calculamos el número de ciclo actual (el índice del próximo punto a agregar)
        win.cycleIndex = Math.max(win.cyclePeakCh1Angles.length, win.cyclePeakCh2Angles.length);

        // 6) repintar
        plot.requestPaint();
        plotCycles.requestPaint();

        // 7) limpiar estado de ciclo
        resetValues()
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
                Label { text: "Sensor SPR B"; color: "white"; font.bold: true; font.pixelSize: 20 }

                Rectangle { Layout.fillWidth: true; color: "transparent" }

                Button {
                    text: "Terminal"
                    Layout.preferredHeight: 36; Layout.preferredWidth: 120; font.pixelSize: 14
                    ToolTip.visible: hovered;
                    ToolTip.text: "Permite la visualización de la terminal"
                }

                Button {
                    text: "Herramientas"
                    enabled: !win.active
                    Layout.preferredHeight: 36; Layout.preferredWidth: 120; font.pixelSize: 14
                    ToolTip.visible: hovered
                    ToolTip.text: "Opciones adicionales"
                    onClicked: {
                        winTools.open()
                    }
                }

                Button {
                    text: "Definir angulos"
                    enabled: !win.active
                    Layout.preferredHeight: 36; Layout.preferredWidth: 120; font.pixelSize: 14
                    ToolTip.visible: hovered
                    ToolTip.text: "Establece los ángulos máximo y mínimo de barrido"
                    onClicked: {
                        backend.viewAngles()   // solicita los ángulos actuales al backend
                        backend.viewSubstance() // Solicita el nombre de la sustancia actual
                        winAngles.open()
                    }
                }

                Button {
                    text: "Definir velocidades"
                    enabled: !win.active
                    Layout.preferredHeight: 36; Layout.preferredWidth: 130; font.pixelSize: 14
                    ToolTip.visible: hovered
                    ToolTip.text: "Establece las velocidades máximo y mínimo de barrido"
                    onClicked: {
                        backend.viewSpeeds()   // solicita las velocidades actuales al backend
                        winSpeeds.open()
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
                    ToolTip.text: "Alterna entre Curva (Ángulo vs Resistencia) y Ciclos (Picos vs tiempo)"
                }

                Switch { checked: win.active; onToggled: backend.setActive(checked) }
                Label { text: win.active ? "Activo" : "Inactivo"; color: "white"; font.pixelSize: 14 }

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
                    Label { 
                        text: deviceUnites === "resistance" ? "CH1 Resistencia (kΩ)": "CH1 Corriente (mA)";
                        color: "#22c55e";
                        font.pixelSize: 14;
                        font.bold: true
                    }
                    Label {
                        text: isFinite(ch1) ? (deviceUnites === "resistance" ? (ch1).toFixed(3) : ch1.toFixed(3)) : "—";
                        color: "white";
                        font.pixelSize: 32;
                        font.bold: true 
                    }
                }

                ColumnLayout {
                    Label {
                        text: deviceUnites === "resistance" ? "CH2 Resistencia (kΩ)": "CH2 Corriente (mA)";
                        color: "#60a5fa";
                        font.pixelSize: 14;
                        font.bold: true
                    }
                    Label { text: isFinite(ch2) ? (deviceUnites === "resistance" ? (ch2).toFixed(3) : ch2.toFixed(3)) : "—"; color: "white"; font.pixelSize: 32; font.bold: true }
                }
                ColumnLayout {
                    Label { text: "Ángulo (°)"; color: "#f472b6"; font.pixelSize: 14; font.bold: true }
                    Label { text: isFinite(win.angleRel) ? win.angleRel.toFixed(2) : "—"; color: "white"; font.pixelSize: 32; font.bold: true }
                }

                ColumnLayout {
                    Label { text:"Tiempo (s)";
                    color: "#fbbf24"; font.pixelSize: 14; font.bold: true }
                    Label { text: isFinite(win.measurementTime) ? win.measurementTime.toFixed(2) : "—"; color: "white"; font.pixelSize: 32; font.bold: true }
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
                            win.dataCycle = []

                            // reset robustez
                            win.incStreak = 0; win.decStreak = 0
                            win.sawStartGate = false; win.sawEndGate = false
                            win.minA = 9999.0; win.maxA = -9999.0

                            backend.setActive(true)
                            if (win.debugLogs) console.log("[cycle] START IDA")
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
                            win.dataCycle = []
                            win.dataCycleFilter = []
                            win.cyclePeakCh1Angles = []
                            win.cyclePeakCh1Times = []
                            win.cyclePeakCh2Angles = []
                            win.cyclePeakCh2Times = []
                            win.cycleIndex = 0
                            win.peakCh1Time = NaN; win.peakCh1Angle = NaN; win.peakCh1Value = NaN
                            win.peakCh2Time = NaN; win.peakCh2Angle = NaN; win.peakCh2alue = NaN

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

            // ---- Resistencia (kΩ) | Corrriente (mA) vs Ángulo (°) ----
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

                    ctx.fillStyle="#9ca3af"; ctx.font="12px sans-serif"; ctx.textAlign="center";
                    ctx.textAlign="center";
                    ctx.fillText("Ángulo (°)", ml+pw/2, H);
                    ctx.save(); ctx.translate(10, mt+ph/2+30); ctx.rotate(-Math.PI/2);

                    if (win.deviceUnites === "resistance"){
                        ctx.fillText("Resistencia (kΩ)", 0, 0); ctx.restore();
                    } else if (win.deviceUnites === "current"){
                        ctx.fillText("Corriente (mA)", 0, 0); ctx.restore();
                    }
                    
                    const xmin=win.xMinDeg, xmax=win.xMaxDeg;
                    const xspan=Math.max(1e-9,xmax-xmin);
                    const xMap=(vx)=> ml + ((vx-xmin)/xspan)*pw;

                    if (win.dataCycleFilter.length === 0){
                        ctx.fillStyle="#9ca3af"
                        ctx.textAlign="left";
                        ctx.fillText("Sin datos agregados aún…", ml+10, mt+20);
                        return;
                    }

                    // eje Y auto
                    let ymin=+Infinity,ymax=-Infinity;
                    for (let i=0;i<win.dataCycleFilter.length;i++){
                        const d=win.dataCycleFilter[i];
                        if (isFinite(d.ch1)){ymin=Math.min(ymin,d.ch1); ymax=Math.max(ymax,d.ch1);}
                        if (isFinite(d.ch2)){ymin=Math.min(ymin,d.ch2); ymax=Math.max(ymax,d.ch2);}
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
                    drawPolyline("#22c55e", d=>d.ch1);
                    drawPolyline("#60a5fa", d=>d.ch2);
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
                    const ctx=getContext("2d"); const W=width, H=height;
                    ctx.clearRect(0,0,W,H); ctx.fillStyle="#111827"; ctx.fillRect(0,0,W,H);

                    const ml=80,mr=10,mt=10,mb=30; const pw=W-ml-mr, ph=H-mt-mb;
                    if (pw<=0||ph<=0) return;

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
                        if (isFinite(v)){xmin=Math.min(xmin,v); xmax=Math.max(xmax,v);}
                    }
                    for (let i=0;i<n2;i++){
                        const v=win.cyclePeakCh2Times[i];
                        if (isFinite(v)){xmin=Math.min(xmin,v); xmax=Math.max(xmax,v);}
                    }
                    
                    if (Math.abs(xmax-xmin)<1e-6) xmax=xmin+1.0;
                    const padx=(xmax-xmin)*0.08; xmin-=padx; xmax+=padx;
                    const xMap = (vx) => ml + ((vx-xmin)/(xmax - xmin))*pw;

                    // Autoescalado eje Y
                    let ymin=+Infinity,ymax=-Infinity;

                    for (let i=0;i<n1;i++){
                        const v=win.cyclePeakCh1Angles[i];
                        if (isFinite(v)){ymin=Math.min(ymin,v); ymax=Math.max(ymax,v);}
                    }
                    for (let i=0;i<n2;i++){
                        const v=win.cyclePeakCh2Angles[i];
                        if (isFinite(v)){ymin=Math.min(ymin,v); ymax=Math.max(ymax,v);}
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
                    drawSeries("#22c55e", win.cyclePeakCh1Times, win.cyclePeakCh1Angles);
                    drawSeries("#60a5fa", win.cyclePeakCh2Times, win.cyclePeakCh2Angles);
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
                    Label {
                        text: (win.deviceUnites === "resistance") ? "Máximo CH1 (curva agregada)" : "Mínimo CH1 (curva agregada)";
                        color: "#22c55e";
                        font.pixelSize: 14;
                        font.bold: true 
                    }
                    Label {
                        text: (isFinite(win.peakCh1Time) && isFinite(win.peakCh1Angle) && isFinite(win.peakCh1Value)) ? `T: ${win.peakCh1Time.toFixed(1)} s — Áng: ${win.peakCh1Angle.toFixed(2)}° — Val: ${win.peakCh1Value.toFixed(4)} un` : "—"
                        color: "white";
                        font.pixelSize: 18;
                        font.bold: true
                    }
                }
                ColumnLayout {
                    Label { 
                        text: (deviceUnites === "resistance") ? "Máximo CH2 (curva agregada)" : "Mínimo CH2 (curva agregada)";
                        color: "#60a5fa";
                        font.pixelSize: 14;
                        font.bold: true 
                    }
                    Label {
                        text: (isFinite(win.peakCh2Time) && isFinite(win.peakCh2Angle) && isFinite(win.peakCh2Value)) ? `T: ${win.peakCh2Time.toFixed(1)} s — Áng: ${win.peakCh2Angle.toFixed(2)}° — Val: ${win.peakCh2Value.toFixed(4)} un` : "—"
                        color: "white";
                        font.pixelSize: 18;
                        font.bold: true
                    }
                }
                ColumnLayout {
                    Label {
                        text: `${win.realTime}`;
                        color: "white";
                        font.pixelSize: 30;
                        font.bold: true
                    }
                }
            }
        }
    }

    // ===== Conexiones con Backend =====
    Connections {
        target: backend

        function onNewLDRSampleWithAngle(tt, v1, v2, absDeg, relDeg) {
            
            win.measurementTime = tt;
            win.ch1 = v1; win.ch2 = v2;
            win.angleAbs = absDeg; win.angleRel  = relDeg;

            const a = relDeg;
            const prev = win.prevAngle;

            // Se asigna el paso minimo aceptado (resolucion de movimiento)
            const eps  = Math.max(win.epsAngle, win.hysteresisDeg);

            // Valida que el angulo se encuentre en el rango establecido
            const inside = isFinite(a) && a >= win.forwardStartDeg && a <= win.forwardEndDeg;
            let inc = false, dec = false;
            
            // Se determina si esta ocurriendo un movimiento ascendente o descendente
            if (isFinite(prev)) {
                const d = a - prev;
                if (d >  eps) inc = true;
                if (d < -eps) dec = true;
            }

            // Se incrementa la cantidad de subidas o bajadas
            if (inc) { win.incStreak++; win.decStreak = 0; }
            else if (dec) { win.decStreak++; win.incStreak = 0; }
            
            // Inicio de ciclo de ida
            if (!win.collectingForward) {
                if (inside && win.incStreak >= win.incNeeded) {
                    win.collectingForward = true;
                    win.pendingCycleSnapshot = true;
                    win.dataCycle = [];

                    // El angulo debe pasar el valor del angulo definido mas un valor delta
                    win.sawStartGate = (a >= (win.forwardStartDeg + win.startGateDelta));
                    win.sawEndGate   = false;
                    win.minA = a; win.maxA = a;

                    // Debuggg ***
                    // console.log("Inicio de la obtencion de data")

                    if (win.debugLogs) console.log(`[cycle] ENTER a=${a.toFixed(3)} deg (incStreak=${win.incStreak})`);
                }
            } else {
                // DURANTE el ciclo: acumular si está dentro
                if (inside) {
                    // Se asigan nuevos minimos y maximos
                    if (a < win.minA) win.minA = a;
                    if (a > win.maxA) win.maxA = a;
                    // Se comprueba si se esta dentro del rango de angulos considerando los delta
                    if (a >= (win.forwardStartDeg + win.startGateDelta)) win.sawStartGate = true;
                    if (a >= (win.forwardEndDeg   - win.endGateDelta))   win.sawEndGate   = true;
                    // console.log("esta dentro del rango")

                    // Se agrega la nueva lectura en la lista data
                    win.data.push({ time: tt, angle: a, ch1: v1, ch2: v2 });
                    // Se agrega la lectura a dataCycle
                    win.dataCycle.push({ time: tt, angle: a, ch1: v1, ch2: v2});
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

                    console.log("fin de ciclo")
                    win.pendingCycleSnapshot = false;
                    win.collectingForward = false;
                    win.dataCycle = []
                    win.cycleIndex = Math.max(win.cyclePeakCh1Angles.length, win.cyclePeakCh2Angles.length);
                    if (win.debugLogs) console.log("[cycle] EXIT (finalized)");
                } else if (abortByEarlyDec && win.pendingCycleSnapshot) {
                    if (win.debugLogs) console.warn("[cycle] EXIT ABORT (vuelta antes de fin)");
                    win.pendingCycleSnapshot = false;
                    win.collectingForward = false;
                    resetValues()
                }
            }

            win.prevAngle = a;
        }

        function onActiveChanged(a) {
            win.active = a;
            if (!a) {
                // Guarda crudo
                if (win.data.length > 0) {
                    backend.saveRawDataCsv(win.data);
                    win.data = [];
                }
                // Guarda Ángulo vs Ciclo (usando las dos series)
                if (win.cyclePeakCh1Times.length > 0 || win.cyclePeakCh2Times.length > 0) {
                    backend.saveAngleVsTimeCsv(win.cyclePeakCh1Times, win.cyclePeakCh2Times, win.cyclePeakCh1Angles, win.cyclePeakCh2Angles);
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
            win.xMinDeg = win.forwardStartDeg - 0.5;
            win.xMaxDeg = win.forwardEndDeg + 0.5;
        }

        function onSpeedMaxMin(vMin, vMax) {
            win.velMinCycle = vMin;
            win.velMaxCycle = vMax;
        }

        function onSubstanceAct(list_subs, list_angles, subs){
            win.substances = list_subs;
            win.anglesSubstances = list_angles;
            win.substance = subs;
        }

        function onAdqDeviceChanged(device, unites) {
            win.device = device;
            win.deviceUnites = unites;
        }

        function onTimeUpdate(time){
            win.realTime = time;
        }
    }
}
