// ===== IMPORTS =====
import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Window 2.15
import QtQuick.Dialogs 6.0

ApplicationWindow {

    // ===== Pages =====
    property string viewPage: "monitor"

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
    property string viewMode: "curve"           // "curve" || "cycles"
    property bool viewCh1: true                 // true || false
    property bool viewCh2: true                 // true || false

    // Dispositivo usado para realizar la lectura del laser
    property string device: "unknown"           // "ldr" | "photodetector" | "unknown"
    property string deviceUnites: "current"     // "current" | "resistance"

    // Rango X (solo tramo de interés)
    property real xMinDeg: 0.0  
    property real xMaxDeg: 0.0

    // Limites fijos mecanismo
    // Solo cambiar al modificar el hardware
    // Limite movimiento
    property real upLimit: 85;
    property real downLimit: 15;

    // Limite de velocidad
    property real upLimitVel: 40;
    property real downLimitVel: 0.1;

    // Velocidades minima y maxima
    property real velMinCycle: 0;
    property real velMaxCycle: 0;

    // Corriente
    property real motorCurrent: 0;

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

    // Número de ciclo actual
    property int cycleIndex: 0

    // Ciclos borrados
    property var cyclesDelete: []

    // Acumulación del ciclo actual
    // [{time:s, angle:°, ch1:<value>, ch2:<value>}, ...]
    property var dataCycle: []

    // Acumulación del ciclo actual con filtro (Data para graficar)
    // {time:[], angle:[], ch1:[], ch2:[]}
    property var dataCycleFilter: []

    // Parametros de ciclo
    property int  minCycleSamples: 5      // mínimo de muestras válidas en un ciclo
    property int  angleKeyDecimals: 2

    // Serie para "ángulo máximo o mínimo vs nº de ciclo"
    property var cyclePeakCh1Angles: []
    property var cyclePeakCh1Times: []
    property var cyclePeakCh2Angles: []
    property var cyclePeakCh2Times: []

    property var cyclePeakCh1CentroidAngles: []
    property var cyclePeakCh1CentroidTimes: []
    property var cyclePeakCh2CentroidAngles: []
    property var cyclePeakCh2CentroidTimes: []

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

    id: win
    visible: true
    // visibility: Window.Maximized
    visibility: Window.FullScreen
    width: 800
    height: 480
    title: "Contenedor principal"
    color: "#0f172a"

    RowLayout{
        anchors.fill: parent
        spacing: 0

        ColumnLayout {
            id: tabBar_id
            Layout.fillHeight: true
            spacing: 20

            ButtonPage {
                id: buttonMonitor
                icon.source: "assets/monitor_w.png"
                onClicked: { viewPage = "monitor" }

                Rectangle {
                    id: separator1
                    width: 5
                    height: parent.height
                    color: viewPage == "monitor" ? 'white': "transparent"
                }
            }
            
            ButtonPage {
                icon.source: "assets/wrench_w.png"
                onClicked: { viewPage = "tools" }
                Rectangle {
                    id: separator2
                    width: 5
                    height: parent.height
                    color: viewPage == "tools" ? 'white': "transparent"
                }
            }

            ButtonPage {
                icon.source: "assets/gears_w.png"
                onClicked: { viewPage = "process" }
                Rectangle {
                    id: separator3
                    width: 5
                    height: parent.height
                    color: viewPage == "process" ? 'white': "transparent"
                }
            }
        }

        Rectangle {
            id: separator_id
            width: 1
            Layout.fillHeight: true
            color: '#1f2937'
        }

        Monitor{} // Page 1
        Tools{} // Page 2
        Processtemporal2{} // Page 3
    }
}