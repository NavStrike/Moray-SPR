// ===== IMPORTS =====
import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Window 2.15
import QtQuick.Dialogs 6.0

ApplicationWindow {

    id: id_mainWindow
    visible: true
    visibility: Window.FullScreen
    width: 800
    height: 480
    title: "Contenedor principal"
    color: "#0f172a"
    palette.buttonText: "black"

    RowLayout {
        anchors.fill: parent
        spacing: 0

        // ------------ Barra de Pestañas Vertical (a la izquierda) ------------

        TabBar {
            id: id_tabBar   // Nombre de la barra
            width: 200      // Ancho fijo
            height: parent.height    // Alto total
            

            contentItem: ListView {
                model: id_tabBar.contentModel
                currentIndex: id_tabBar.currentIndex
                orientation: ListView.Vertical
                spacing: 1
            }

            TabButton { 
                text: "Principal" 
                
                background: Rectangle {
                    color: parent.checked ? "#1976D2" : (parent.hovered ? "#E3F2FD" : "white")
                }
                
                contentItem: Text {
                    text: parent.text
                    color: parent.checked ? "#1976D2" : (parent.hovered ? "#E3F2FD" : "white")
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
            }
            
            TabButton { 
                text: "Herramientas"
                
                background: Rectangle {
                    color: parent.checked ? "#1976D2" : (parent.hovered ? "#E3F2FD" : "white")
                }
                
                contentItem: Text {
                    text: parent.text
                    color: parent.checked ? "#E3F2FD" : (parent.hovered ? "#1976D2" : "white")
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
            }
            
            TabButton { 
                text: "Procesamiento"
                
                background: Rectangle {
                    color: parent.checked ? "#1976D2" : (parent.hovered ? "#E3F2FD" : "white")
                }
                
                contentItem: Text {
                    text: parent.text
                    color: parent.checked ? "#1976D2" : (parent.hovered ? "#E3F2FD" : "white")
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
            }
        }
        
        // ------------ Separador visual ------------

        Rectangle {
            id: id_separator
            width: 1
            color: "gray"
            Layout.fillHeight: true
        }

        // ------------ Área de Contenido ------------

        StackLayout {
            id: id_content
            Layout.fillWidth: true
            Layout.fillHeight: true
            currentIndex: id_tabBar.currentIndex

            // Page 1
            LDRMonitor_maximo_b{}

            // Page 2
            ToolsDialog{}

            // Page 3
            LDRMonitor_maximo_b{}
        }
    }
}