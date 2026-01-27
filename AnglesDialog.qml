import QtQuick 6.0
import QtQuick.Controls 6.0
import QtQuick.Layouts 6.0
import QtQuick.Window 6.0
import QtQuick.Dialogs 6.0

Dialog {
    //-- textFiel que está seleccionado
    property TextField activeInput: null

    id: winAngles
    title: "Definir ángulos de barrido"
    modal: true
    width: 500
    height: 250
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
            spacing: 8

            Label {
                Layout.fillWidth: true
                horizontalAlignment: Text.AlignHCenter
                Layout.alignment: Qt.AlignHCenter
                text: winAngles.title
                font.pixelSize: 20
                color: "white"
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
                        onActiveFocusChanged: if(activeFocus) activeInput = anguloMin
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
                    onActiveFocusChanged: if(activeFocus) activeInput = anguloMax
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
        ColumnLayout{
            Layout.fillWidth: true
            anchors.margins: 20
            spacing: 8
            GridLayout{
                columns: 3
                rows: 3
                Button {
                    text: "0" 
                    Layout.preferredHeight: 50; Layout.preferredWidth: 50; font.pixelSize: 14
                    onClicked: {
                        let newValue = activeInput.text + "0";
                        if (Number(newValue) <= 90){activeInput.text = newValue}
                    }
                }
                Button {
                    text: "1" 
                    Layout.preferredHeight: 50; Layout.preferredWidth: 50; font.pixelSize: 14
                    onClicked: {
                        let newValue = activeInput.text + "1";
                        if (Number(newValue) <= 90){activeInput.text = newValue}
                    }
                }
                Button {
                    text: "2" 
                    Layout.preferredHeight: 50; Layout.preferredWidth: 50; font.pixelSize: 14
                    onClicked: {
                        let newValue = activeInput.text + "2";
                        if (Number(newValue) <= 90){activeInput.text = newValue}
                    }
                }
                Button {
                    text: "3" 
                    Layout.preferredHeight: 50; Layout.preferredWidth: 50; font.pixelSize: 14
                     onClicked: {
                        let newValue = activeInput.text + "3";
                        if (Number(newValue) <= 90){activeInput.text = newValue}
                    }
                    }
                Button {
                    text: "4" 
                    Layout.preferredHeight: 50; Layout.preferredWidth: 50; font.pixelSize: 14
                    onClicked: {
                        let newValue = activeInput.text + "4";
                        if (Number(newValue) <= 90){activeInput.text = newValue}
                    }
                }
                Button {
                    text: "5" 
                    Layout.preferredHeight: 50; Layout.preferredWidth: 50; font.pixelSize: 14
                    onClicked: {
                        let newValue = activeInput.text + "5";
                        if (Number(newValue) <= 90){activeInput.text = newValue}
                    }
                }
                Button {
                    text: "6" 
                    Layout.preferredHeight: 50; Layout.preferredWidth: 50; font.pixelSize: 14
                    onClicked: {
                        let newValue = activeInput.text + "6";
                        if (Number(newValue) <= 90){activeInput.text = newValue}
                    }
                }
                Button {
                    text: "7" 
                    Layout.preferredHeight: 50; Layout.preferredWidth: 50; font.pixelSize: 14
                    onClicked: {
                        let newValue = activeInput.text + "7";
                        if (Number(newValue) <= 90){activeInput.text = newValue}
                    }
                }
                Button {
                    text: "8" 
                    Layout.preferredHeight: 50; Layout.preferredWidth: 50; font.pixelSize: 14
                    onClicked: {
                        let newValue = activeInput.text + "8";
                        if (Number(newValue) <= 90){activeInput.text = newValue}
                    }
                }
                Button {
                    text: "9" 
                    Layout.preferredHeight: 50; Layout.preferredWidth: 50; font.pixelSize: 14
                    onClicked: {
                        let newValue = activeInput.text + "9";
                        if (Number(newValue) <= 90){activeInput.text = newValue}
                    }
                }
                Button {
                    text: "Borrar"
                    Layout.columnSpan: 2
                    Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                    Layout.preferredHeight: 50; Layout.preferredWidth: 108; font.pixelSize: 14
                    onClicked: {
                        activeInput.text = ""
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
