import QtQuick 6.0
import QtQuick.Controls 6.0
import QtQuick.Layouts 6.0
import QtQuick.Window 6.0
import QtQuick.Dialogs 6.0

Dialog {
    id: winTools
    title: "Herramientas adicionales"
    modal: true
    width: parent.width
    height: parent.height
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
                        backend.saveRawDataCsv();
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
                    text: "Nombre:"
                    color: "white" 
                    font.bold: true; font.pixelSize: 20
                     Layout.preferredHeight: 50
                }

                TextField {
                    id: nameFile
                    text: ""
                    font.pixelSize: 20
                    Layout.preferredWidth: 150
                    Layout.preferredHeight: 50

                }

                Label{
                    text: ".csv:"
                    color: "white"
                    font.bold: true; font.pixelSize: 20
                    Layout.preferredHeight: 50
                }
            }
                
            RowLayout {
                Layout.fillWidth: true
                spacing: 10

                Button1 {
                    text: "Salir"
                    //font.pixelSize: 14
                    Layout.fillWidth: true
                    Layout.preferredHeight: 50
                    onClicked: winTools.reject()
                }

                Button1 {
                    text: "Aceptar"
                    //font.pixelSize: 14
                    Layout.fillWidth: true
                    Layout.preferredHeight: 50
                    onClicked: winTools.accept()
                }
            }
        }
    }
}