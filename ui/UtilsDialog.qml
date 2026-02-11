import QtQuick 6.0
import QtQuick.Controls 6.0
import QtQuick.Layouts 6.0
import QtQuick.Window 6.0
import QtQuick.Dialogs 6.0

Dialog {
    id: winUtils
    title: "Herramientas adicionales"
    modal: true
    width: 800
    height: 300
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
                text: winUtils.title
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
                
            RowLayout {
                Layout.fillWidth: true
                spacing: 10

                Button1 {
                    text: "Cancelar"
                    //font.pixelSize: 14
                    Layout.fillWidth: true
                    Layout.preferredHeight: 50
                    onClicked: winUtils.reject()
                }

                Button1 {
                    text: "Aceptar"
                    //font.pixelSize: 14
                    Layout.fillWidth: true
                    Layout.preferredHeight: 50
                    onClicked: winUtils.accept()
                }
            }

        }
    }
}