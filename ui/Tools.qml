// ===== IMPORTS =====
import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Window
import QtQuick.Dialogs
import QtQuick.VirtualKeyboard

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
}