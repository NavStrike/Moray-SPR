import QtQuick 6.0
import QtQuick.Controls 6.0
import QtQuick.Layouts 6.0
import QtQuick.Window 6.0
import QtQuick.Dialogs 6.0

Dialog {
    //-- textFiel que está seleccionado
    property TextField activeInput: null

    id: winSpeeds
    title: "Definir velpocidades de barrido"
    modal: true
    width: 800
    height: 400
    padding: 20

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
    }

    RowLayout{
        anchors.fill: parent
        spacing: 8
        layoutDirection: Qt.LeftToRight
        //uniformCellSizes: true

        ColumnLayout{
            Layout.fillWidth: true
            Layout.fillHeight: true
            anchors.margins: 0 
            spacing: 30

            Label {
                Layout.fillWidth: true
                horizontalAlignment: Text.AlignHCenter
                Layout.alignment: Qt.AlignHCenter
                text: winSpeeds.title
                font.bold: true
                font.pixelSize: 20
                color: "white"
            }

            RowLayout{
                Layout.fillWidth: true

                Label{
                    text: "Velocidad mínima (°/s): "
                    color: "white" 
                    font.bold: true; font.pixelSize: 20
                }
                    
                TextField {
                    id: velocityMin
                    text: win.forwardStartDeg
                    font.pixelSize: 20
                    Layout.fillWidth: true
                    onActiveFocusChanged: if(activeFocus) activeInput = velocityMin
                }
            }

            RowLayout{
                Layout.fillWidth: true
                spacing: 5

                Label{
                    text: "Velocidad máxima (°/s): "
                    color: "white" 
                    font.bold: true; font.pixelSize: 20
                }
                    
                TextField {
                    id: velocityMax
                    text: win.forwardEndDeg
                    font.pixelSize: 20
                    Layout.fillWidth: true
                    onActiveFocusChanged: if(activeFocus) activeInput = velocityMax
                }
            }
                
            RowLayout {
                Layout.fillWidth: true
                spacing: 10

                Button1 {
                    text: "Cancelar"
                    //font.pixelSize: 14
                    Layout.fillWidth: true
                    Layout.preferredHeight: 50
                    onClicked: winSpeeds.reject()
                }

                Button1 {
                    text: "Aceptar"
                    //font.pixelSize: 14
                    Layout.fillWidth: true
                    Layout.preferredHeight: 50
                    onClicked: {
                        let a = parseFloat(velocityMin.text)
                        let b = parseFloat(velocityMax.text)
                        if (a == null || b == null || isNaN(a) || isNaN(b)){
                            velocityMin.color = "red"; velocityMax.color = "red"
                            console.log("Error: Las velocidades ser números válidos.");
                            return
                        } else if (a>b) {
                            velocityMin.color = '#a17400'; velocityMax.color = '#a17400'
                            console.log("Error: La velocidad mínima debe ser menor que la velocidad máxima.");
                            return
                        } else if (a < downLimitVel || b < downLimitVel || a > upLimitVel || b > upLimitVel){
                            velocityMin.color = "red"; velocityMax.color = "red"
                            console.log(`Error: Las velocidades deben estar entre ${downLimitVel} y ${upLimitVel} grados/s.`);
                            return
                        } else { winSpeeds.accept()} 
                    }
                }
            }

        }
        ColumnLayout{
            Layout.fillWidth: true
            anchors.margins: 20
            spacing: 8
            GridLayout{
                columns: 3
                rows: 3
                
                Button1 {text: "0"; onClicked: {addNumber(0)}}
                Button1 {text: "1"; onClicked: {addNumber(1)}}
                Button1 {text: "2"; onClicked: {addNumber(2)}}
                Button1 {text: "3"; onClicked: {addNumber(3)}}
                Button1 {text: "4"; onClicked: {addNumber(4)}}
                Button1 {text: "5"; onClicked: {addNumber(5)}}
                Button1 {text: "6"; onClicked: {addNumber(6)}}
                Button1 {text: "7"; onClicked: {addNumber(7)}}
                Button1 {text: "8"; onClicked: {addNumber(8)}}
                Button1 {text: "9"; onClicked: {addNumber(9)}}
                Button1 {text: "Borrar"; onClicked: {activeInput.text = ""}
                    Layout.columnSpan: 2
                    Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                    Layout.preferredWidth: 168
                }
            }     
        }
    }
    
    
    onAccepted: {
        backend.setMaxMinVel(velocityMin.text, velocityMax.text)
        win.velMinCycle = velocityMin.text
        win.velMaxCycle = velocityMax.text
        console.log("Velocity min:", win.velMinCycle)
        console.log("Velocity max:", win.velMaxCycle)
    }

    onRejected: {
        velocityMin.text = win.velMinCycle
        velocityMax.text = win.velMaxCycle
    }

    function addNumber (num){
        let newValue = activeInput.text + num
        if (Number(newValue) <= 90){activeInput.text = newValue}

        if (Number(newValue) <= downLimitVel || Number(newValue) >= upLimitVel){activeInput.color = 'red'}
        else {activeInput.color = "black"}
    }
}
