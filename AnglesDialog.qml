import QtQuick 6.0
import QtQuick.Controls 6.0
import QtQuick.Layouts 6.0
import QtQuick.Window 6.0
import QtQuick.Dialogs 6.0

Dialog {
    id: winAngles
    title: "Definir ángulos de barrido"
    modal: true
    width: 400
    height: 300

    // Evita visivilidad del encabezado
    header: null

    // Evita el cierre automático al hacer clic fuera del cuadro
    closePolicy: Popup.NoAutoClose

    // Posición centrada
    x: (parent.width - width) / 2
    y: (parent.height - height) / 2

    background: Rectangle {
        anchors.fill: parent
        color: "#111827"
        border.color: "#1f2937"
        border.width: 2
        radius: 15

        ColumnLayout{
            anchors.fill: parent
            anchors.margins: 20
            spacing: 8

            Label {
                text: winAngles.title
                font.pixelSize: 20
                color: "white"
                horizontalAlignment: Text.AlignHCenter
                Layout.alignment: Qt.AlignHCenter
            }

            RowLayout{
                Layout.fillWidth: true

                Label{
                    text: "Ángulo mínimo (°): "
                    color: "white" 
                    font.bold: true; font.pixelSize: 14
                }
                
                TextField {
                    id: anguloMin
                    //placeholderText: "Ángulo mínimo (°)"
                    text: win.forwardStartDeg
                    font.pixelSize: 15
                    Layout.fillWidth: true
                }
            }

            RowLayout{
                Layout.fillWidth: true
                spacing: 5

                Label{
                    text: "Ángulo máximo (°): "
                    color: "white" 
                    font.bold: true; font.pixelSize: 14
                }
                
                TextField {
                    id: anguloMax
                    //placeholderText: "Ángulo máximo (°)"
                    text: win.forwardEndDeg
                    font.pixelSize: 15
                    Layout.fillWidth: true
                }
            }
            
            RowLayout {
                Layout.fillWidth: true
                spacing: 10

                Button {
                    text: "Cancelar"
                    font.pixelSize: 14
                    Layout.fillWidth: true
                    Layout.preferredHeight: 36
                    onClicked: winAngles.reject()
                }

                Button {
                    text: "Aceptar"
                    font.pixelSize: 14
                    Layout.fillWidth: true
                    Layout.preferredHeight: 36
                    onClicked: {
                        let a = parseFloat(anguloMin.text)
                        let b = parseFloat(anguloMax.text)
                        if(a < 0 || b < 0 || a > 90 || b > 90){
                            console.log("Error: Los ángulos deben estar entre 0 y 90 grados.");
                            return
                        } else if (a>b) {
                            console.log("Error: El ángulo mínimo debe ser menor que el ángulo máximo.");
                            return
                        } else if(a == null || b == null || isNaN(a) || isNaN(b)){
                            console.log("Error: Los ángulos deben ser números válidos.");
                            return
                        } else { winAngles.accept() }
                        
                    }
                }
            }
        }
    }
    
    onAccepted: {
        backend.setMaxMinAngles(anguloMin.text, anguloMax.text)
        win.forwardStartDeg = anguloMin.text
        win.forwardEndDeg = anguloMax.text
        console.log("Angle min:", win.forwardStartDeg)
        console.log("Angle max:", win.forwardEndDeg)
    }

    onRejected: {
        anguloMin.text = win.forwardStartDeg
        anguloMax.text = win.forwardEndDeg
    }

}
