import QtQuick 6.0
import QtQuick.Controls 6.0
import QtQuick.Layouts 6.0
import QtQuick.Window 6.0
import QtQuick.Dialogs 6.0

Dialog {
    //-- textFiel que está seleccionado
    property TextField activeInput: null

    id: winSpeeds
    title: "Definir velocidades de barrido"
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
                    text: win.velMinCycle
                    font.pixelSize: 20
                    Layout.fillWidth: true
                    onActiveFocusChanged: if(activeFocus) {activeInput = velocityMin; Qt.inputMethod.hide()}
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
                    text: win.velMaxCycle
                    font.pixelSize: 20
                    Layout.fillWidth: true
                    onActiveFocusChanged: if(activeFocus) {activeInput = velocityMax; Qt.inputMethod.hide()}
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
                rows: 4
                
                Button1 {text: "7"; onClicked: {addNumber(7)}}
                Button1 {text: "8"; onClicked: {addNumber(8)}}
                Button1 {text: "9"; onClicked: {addNumber(9)}}
                Button1 {text: "4"; onClicked: {addNumber(4)}}
                Button1 {text: "5"; onClicked: {addNumber(5)}}
                Button1 {text: "6"; onClicked: {addNumber(6)}}
                Button1 {text: "1"; onClicked: {addNumber(1)}}
                Button1 {text: "2"; onClicked: {addNumber(2)}}
                Button1 {text: "3"; onClicked: {addNumber(3)}}
                Button1 {text: "0"; onClicked: {addNumber(0)}}
                Button1 {text: "."; onClicked: {addNumber(".")}}
                Button1 {text: "Borrar"; onClicked: {activeInput.text = ""; activeInput.color = "black"} }
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
        // No se agregan puntos si ya hay uno en el número o no hay otro numero antes del punto
        if (num === "." && activeInput.text.includes(".") || num === "." && activeInput.text === "") return
        // Se agrega el nuevo caracter al texto actual
        let newValue = activeInput.text + num;
        // Se actualiza el texto del campo solo si el nuevo valor es menor o igual a 90
        if (Number(newValue) <= 90 || num === "."){activeInput.text = newValue}
        // Se cambia el color del texto (rojo) si el nuevo valor es menor que el límite inferior o mayor que el límite superior
        if (Number(newValue) <= downLimitVel || Number(newValue) >= upLimitVel){activeInput.color = 'red'}
        else {activeInput.color = "black"}
    }

}
