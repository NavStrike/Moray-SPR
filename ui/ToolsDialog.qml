// ===== IMPORTS =====
import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Window 2.15
import QtQuick.Dialogs 6.0

Page {
    //-- textFiel que está seleccionado
    property TextField activeInput: null
    property bool activeMayus: false

    id: winTools
    title: "Herramientas adicionales"
    // modal: true
    width: parent.width
    height: parent.height
    padding: 20

    // Evita visivilidad del encabezado
    header: null

    // Evita el cierre automático al hacer clic fuera del cuadro
    // closePolicy: Popup.NoAutoClose

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

    ColumnLayout{
        anchors.fill: parent
        spacing: 8
        layoutDirection: Qt.LeftToRight

        ColumnLayout{
            Layout.fillWidth: true
            Layout.fillHeight: true
            anchors.margins: 0 
            spacing: 30

            Label {
                Layout.fillWidth: true
                horizontalAlignment: Text.AlignHCenter
                Layout.alignment: Qt.AlignHCenter
                text: winTools.title
                font.bold: true
                font.pixelSize: 20
                color: "white"
            }

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
                    onClicked: backend.setRelativeZero()
                    Layout.preferredHeight: 36
                    Layout.preferredWidth: 130
                    font.pixelSize: 14
                    //ToolTip.visible: hovered
                    //ToolTip.text: "Captura el ángulo absoluto actual como cero (ZC)"
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
                    onClicked: {
                        win.device = win.device === "ldr" ? "photodetector" : "ldr";
                        win.deviceUnites = win.device === "ldr" ? "resistance" : "current";
                        win.update();
                        backend.setAdqDevice(win.device);
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
                    text: "Guardar datos adquiridos: "
                    color: "white" 
                    font.bold: true; font.pixelSize: 20
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
                    text: "Nombre de la carpeta:"
                    color: "white" 
                    font.bold: true; font.pixelSize: 20
                    Layout.preferredHeight: 36
                }

                TextField {
                    id: nameFile
                    text: ""
                    font.pixelSize: 20
                    Layout.preferredWidth: 150
                    Layout.preferredHeight: 36
                    onActiveFocusChanged: if(activeFocus) activeInput = nameFile

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
                
            RowLayout {
                Layout.fillWidth: true
                spacing: 10

                Button1 {
                    text: "Salir"
                    //font.pixelSize: 14
                    Layout.fillWidth: true
                    Layout.preferredHeight: 40
                    onClicked: winTools.reject()
                }

                Button1 {
                    text: "Aceptar"
                    //font.pixelSize: 14
                    Layout.fillWidth: true
                    Layout.preferredHeight: 40
                    onClicked: winTools.accept()
                }
            }
        }

        RowLayout{
            Layout.fillWidth: true
            anchors.margins: 10
            spacing: 8
            Layout.alignment: Qt.AlignHCenter

            GridLayout{
                columns: 16
                rows: 4
                
                Button1 {text: "Q"; onClicked: {addCharacter("Q")}}
                Button1 {text: "W"; onClicked: {addCharacter("W")}}
                Button1 {text: "E"; onClicked: {addCharacter("E")}}
                Button1 {text: "R"; onClicked: {addCharacter("R")}}
                Button1 {text: "S"; onClicked: {addCharacter("S")}}
                Button1 {text: "T"; onClicked: {addCharacter("T")}}
                Button1 {text: "Y"; onClicked: {addCharacter("Y")}}
                Button1 {text: "U"; onClicked: {addCharacter("U")}}
                Button1 {text: "I"; onClicked: {addCharacter("I")}}
                Button1 {text: "O"; onClicked: {addCharacter("O")}}
                Button1 {text: "P"; onClicked: {addCharacter("P")}}
                Button1 {text: "A"; onClicked: {addCharacter("A")}}

                Button1 {text: "6"; onClicked: {addCharacter("6")}}
                Button1 {text: "7"; onClicked: {addCharacter("7")}}
                Button1 {text: "8"; onClicked: {addCharacter("8")}}
                Button1 {text: "9"; onClicked: {addCharacter("9")}}

                Button1 {text: "S"; onClicked: {addCharacter("S")}}
                Button1 {text: "D"; onClicked: {addCharacter("D")}}
                Button1 {text: "F"; onClicked: {addCharacter("F")}}
                Button1 {text: "G"; onClicked: {addCharacter("G")}}
                Button1 {text: "H"; onClicked: {addCharacter("H")}}
                Button1 {text: "J"; onClicked: {addCharacter("J")}}
                Button1 {text: "K"; onClicked: {addCharacter("K")}}
                Button1 {text: "L"; onClicked: {addCharacter("L")}}
                Button1 {text: "Ñ"; onClicked: {addCharacter("Ñ")}}
                Button1 {text: "Z"; onClicked: {addCharacter("Z")}}
                Button1 {text: "X"; onClicked: {addCharacter("X")}}
                Button1 {text: "C"; onClicked: {addCharacter("C")}}

                Button1 {text: "2"; onClicked: {addCharacter("2")}}
                Button1 {text: "3"; onClicked: {addCharacter("3")}}
                Button1 {text: "4"; onClicked: {addCharacter("4")}}
                Button1 {text: "5"; onClicked: {addCharacter("5")}}

                Button1 {text: "V"; onClicked: {addCharacter("V")}}
                Button1 {text: "B"; onClicked: {addCharacter("B")}}
                Button1 {text: "N"; onClicked: {addCharacter("N")}}
                Button1 {text: "M"; onClicked: {addCharacter("M")}}
                Button1 {text: "K"; onClicked: {addCharacter("K")}}
                Button1 {text: "L"; onClicked: {addCharacter("L")}}
                Button1 {text: "Ñ"; onClicked: {addCharacter("Ñ")}}
                Button1 {text: "Z"; onClicked: {addCharacter("Z")}}
                Button1 {text: "_"; onClicked: {addCharacter("_")}}
                Button1 {text: "-"; onClicked: {addCharacter("-")}}
                Button1 {text: "Borrar"; onClicked: {activeInput.text = ""; activeInput.color = "black"}}
                Button1 {text: activeMayus ? "May" : "Min"; onClicked: {activeMayus = ! activeMayus;}}

                Button1 {text: "1"; onClicked: {addCharacter("1")}}
                Button1 {text: "0"; onClicked: {addCharacter("0")}}
                Button1 {text: "."; onClicked: {addCharacter(".")}}
                Button1 {text: "Borrar"; onClicked: {activeInput.text = ""; activeInput.color = "black"}}
            }
        }
    }

    function addCharacter (letter){
        if (! activeMayus){letter = letter.toLowerCase();}
        let newValue = activeInput.text + letter;
        activeInput.text = newValue;
    }
}