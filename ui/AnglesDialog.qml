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
                text: winAngles.title
                font.bold: true
                font.pixelSize: 20
                color: "white"
            }

            RowLayout{
                Layout.fillWidth: true

                Label{
                    text: "Sustancia: "
                    color: "white" 
                    font.bold: true; font.pixelSize: 20
                }
                    
                ComboBox {
                    id: comboSubstance
                    model: win.substances
                    currentIndex: 0
                    onActivated: {selectSubstance(currentText)}
                    font.pixelSize: 20
                    Layout.fillWidth: true
                    displayText: currentIndex === 0 ? "Especifique los valores" : currentText
                }
            }

            RowLayout{
                Layout.fillWidth: true

                Label{
                    text: "Ángulo mínimo (°): "
                    color: "white" 
                    font.bold: true; font.pixelSize: 20
                }
                    
                TextField {
                    id: anguloMin
                    //placeholderText: "Ángulo mínimo (°)"
                    text: win.forwardStartDeg
                    font.pixelSize: 20
                    Layout.fillWidth: true
                    onActiveFocusChanged: if(activeFocus) activeInput = anguloMin
                    //layout.preferredHeight: 60
                }
            }

            RowLayout{
                Layout.fillWidth: true
                spacing: 5

                Label{
                    text: "Ángulo máximo (°): "
                    color: "white" 
                    font.bold: true; font.pixelSize: 20
                }
                    
                TextField {
                    id: anguloMax
                    //placeholderText: "Ángulo máximo (°)"
                    text: win.forwardEndDeg
                    font.pixelSize: 20
                    Layout.fillWidth: true
                    onActiveFocusChanged: if(activeFocus) activeInput = anguloMax
                    //layout.preferredHeight: 60
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
                    onClicked: winAngles.reject()
                }

                Button1 {
                    text: "Aceptar"
                    //font.pixelSize: 14
                    Layout.fillWidth: true
                    Layout.preferredHeight: 50
                    onClicked: {
                        let a = parseFloat(anguloMin.text)
                        let b = parseFloat(anguloMax.text)
                        if (a == null || b == null || isNaN(a) || isNaN(b)){
                            anguloMin.color = "red"; anguloMax.color = "red"
                            console.log("Error: Los ángulos deben ser números válidos.");
                            return
                        } else if (a>b) {
                            anguloMin.color = '#a17400'; anguloMax.color = '#a17400'
                            console.log("Error: El ángulo mínimo debe ser menor que el ángulo máximo.");
                            return
                        } else if (a < downLimit || b < downLimit || a > upLimit || b > upLimit){
                            anguloMin.color = "red"; anguloMax.color = "red"
                            console.log(`Error: Los ángulos deben estar entre ${downLimit} y ${upLimit} grados.`);
                            return
                        } else { winAngles.accept()} 
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
                Button1 {text: "Borrar"; onClicked: {activeInput.text = ""; activeInput.color = "black"} 
                    Layout.columnSpan: 2
                    Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                    Layout.preferredWidth: 168
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

    function addNumber (num){
        let newValue = activeInput.text + num
        if (Number(newValue) <= 90){activeInput.text = newValue}

        if (Number(newValue) <= downLimit || Number(newValue) >= upLimit){activeInput.color = 'red'}
        else {activeInput.color = "black"}
    }

    function selectSubstance(){
        if (comboSubstance.currentIndex !== 0){
            anguloMin.enabled = false; anguloMax.enabled = false;
            anguloMin.text = win.anglesSubstances[comboSubstance.currentIndex][0];
            anguloMax.text = win.anglesSubstances[comboSubstance.currentIndex][1];
        } else{
            anguloMax.enabled = true; anguloMin.enabled = true;
        }
    }
}
