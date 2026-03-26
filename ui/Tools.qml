// ===== IMPORTS =====
import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Window 2.15
import QtQuick.Dialogs 6.0
import QtQuick.VirtualKeyboard 2.15

Page {
    //-- textFiel que está seleccionado
    property TextField activeInput: null
    property bool activeMayus: false

    id: winTools
    visible: win.viewPage === "tools" 
    title: "Herramientas"
    Layout.fillWidth: true
    Layout.fillHeight: true

    // ===== Otras ventanas importadas =====
    AnglesDialog {id: winAngles}
    SpeedsDialog {id: winSpeeds}

    Rectangle {
        anchors.fill: parent
        color: "#111827"

        // ===== Layout raíz =====
        ColumnLayout{
            anchors.fill: parent
            anchors.margins: 16
            spacing: 16

            // ===== Barra superior =====
            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: 70
                radius: 12
                color: "#111827"
                border.color: "#1f2937"; border.width: 1

                RowLayout {
                    anchors.fill: parent; anchors.margins: 12; spacing: 16

                    // Título
                    Label { text: "Herramientas"; color: "white"; font.bold: true; font.pixelSize: 20 }

                    Rectangle { Layout.fillWidth: true; color: "transparent" }
                }
            }

            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                radius: 12
                color: "#111827"
                border.color: "#1f2937"; border.width: 1

                Flickable {
                    id: flickable
                    anchors {
                        fill: parent
                        bottomMargin: inputPanel.visible ? inputPanel.height : 0
                    }
                    contentHeight: contentOp.height
                    clip: true

                    ColumnLayout{
                        id: contentOp
                        anchors.fill: parent
                        anchors.margins: 20
                        spacing: 30

                        RowLayout{
                            Layout.fillWidth: true
                            spacing: 5

                            Label{
                                text: "Captura el ángulo absoluto actual como cero (ZC): "
                                color: "white" 
                                font.bold: true; font.pixelSize: 20
                            }
                                
                            Button {
                                text: "Fijar nuevo cero"
                                enabled: !win.active
                                Layout.preferredHeight: 36; Layout.preferredWidth: 130; font.pixelSize: 14
                                onClicked: backend.setNewAbsoluteZero()
                            }
                        }

                        RowLayout{
                            Layout.fillWidth: true
                            spacing: 5

                            Label{
                                text: "Establece los ángulos máximo y mínimo de barrido: "
                                color: "white" 
                                font.bold: true; font.pixelSize: 20
                            }

                            Button {
                                text: "Definir angulos"
                                enabled: !win.active
                                Layout.preferredHeight: 36; Layout.preferredWidth: 120; font.pixelSize: 14
                                onClicked: {
                                    backend.viewAngles()   // solicita los ángulos actuales al backend
                                    backend.viewSubstance() // Solicita el nombre de la sustancia actual
                                    winAngles.open()
                                }
                            }
                        }

                        RowLayout{
                            Layout.fillWidth: true
                            spacing: 5

                            Label{
                                text: "Establece las velocidades máximo y mínimo de barrido: "
                                color: "white" 
                                font.bold: true; font.pixelSize: 20
                            }

                            Button {
                                text: "Definir velocidades"
                                enabled: !win.active
                                Layout.preferredHeight: 36; Layout.preferredWidth: 130; font.pixelSize: 14
                                onClicked: {
                                    backend.viewSpeeds()   // solicita las velocidades actuales al backend
                                    winSpeeds.open()
                                }
                            }
                        }

                        RowLayout{
                            Layout.fillWidth: true
                            spacing: 5

                            Label{
                                text: "Cambiar el dispositivo de captura: "
                                color: "white" 
                                font.bold: true; font.pixelSize: 20
                            }
                                
                            Button {
                                text: win.device === "ldr" ? "Cambiar a fotodetector" : "Cambiar a LDR" 
                                enabled: !win.active
                                Layout.preferredHeight: 36; Layout.preferredWidth: 160; font.pixelSize: 14
                                onClicked: {
                                    win.device = win.device === "ldr" ? "photodetector" : "ldr";
                                    win.deviceUnites = win.device === "ldr" ? "resistance" : "current";
                                    win.update();
                                    backend.setAdqDevice(win.device);
                                }
                            }
                        }

                        RowLayout{
                            Layout.fillWidth: true
                            spacing: 5

                            Label{
                                text: "Guardar datos adquiridos: "
                                color: "white" 
                                font.bold: true; font.pixelSize: 20
                            }

                            TextField {
                                id: nameFile
                                text: ""
                                font.pixelSize: 20
                                Layout.preferredWidth: 150
                                Layout.preferredHeight: 36
                                horizontalAlignment: TextInput.AlignHCenter
                                verticalAlignment: TextInput.AlignVCenter
                                onActiveFocusChanged: if(activeFocus) activeInput = nameFile
                            }

                            Button {
                                text: "Guardar datos" 
                                enabled: !win.active
                                onClicked: {
                                    backend.setNameFile(nameFile.text)
                                    backend.saveRawDataCsv(win.data); 
                                    backend.saveAngleVsTimeCsv(win.cyclePeakCh1Times, win.cyclePeakCh2Times, win.cyclePeakCh1Angles, win.cyclePeakCh2Angles);
                                }
                                Layout.preferredHeight: 36
                                Layout.preferredWidth: 160
                                font.pixelSize: 14
                            }

                        }

                        RowLayout{
                            Layout.fillWidth: true
                            spacing: 5

                            Label{
                                text: "Modificar la corriente del motor:"
                                color: "white" 
                                font.bold: true; font.pixelSize: 20
                                Layout.preferredHeight: 36
                            }

                            TextField {
                                id: textMotorCurrent
                                text: win.motorCurrent
                                font.pixelSize: 15
                                Layout.preferredWidth: 150
                                Layout.preferredHeight: 36
                                horizontalAlignment: TextInput.AlignHCenter
                                verticalAlignment: TextInput.AlignVCenter
                                onActiveFocusChanged: if(activeFocus) activeInput = textMotorCurrent
                            }

                            Button {
                                text: "Establecer";
                                enabled: !win.active;
                                onClicked: {backend.setCurrent(textMotorCurrent.text);}
                                Layout.preferredHeight: 36
                                Layout.preferredWidth: 160
                                font.pixelSize: 14
                            }
                        }

                        RowLayout{
                            Layout.fillWidth: true
                            spacing: 5

                            Label{
                                text: "Regresar a los valores por defecto: "
                                color: "white" 
                                font.bold: true; font.pixelSize: 20
                            }
                                
                            Button {
                                text: "Reestablecer valores"
                                enabled: !win.active
                                Layout.preferredHeight: 36; Layout.preferredWidth: 200; font.pixelSize: 14
                                onClicked: backend.resetVariables()
                            }
                        }
                        Rectangle { Layout.fillHeight: true; color: "transparent" }

                    }
                }

                InputPanel {
                    id: inputPanel
                    anchors.left: parent.left
                    anchors.right: parent.right
                    anchors.bottom: parent.bottom
                    visible: Qt.inputMethod.visible
                }
            }
        }
    }

    function addCharacter (letter){
        if (! activeMayus){letter = letter.toLowerCase();}
        let newValue = activeInput.text + letter;
        activeInput.text = newValue;
    }

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
                    win.data.push({ cycle: win.cycleIndex + 1, time: tt, angle: a, ch1: v1, ch2: v2 });
                    // Se agrega la lectura a dataCycle
                    win.dataCycle.push({ cycle: win.cycleIndex + 1, time: tt, angle: a, ch1: v1, ch2: v2});
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
                    win.cycleIndex += 1
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

        function onCurrentChanged(current){
            win.motorCurrent = current;
        }

        function onTimeUpdate(time){
            win.realTime = time;
        }
    }
}