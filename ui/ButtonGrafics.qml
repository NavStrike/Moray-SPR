// ===== IMPORTS =====
import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import QtQuick.Window
import QtQuick.Dialogs

Button {
    property real selectB: 100
    Layout.alignment: Qt.AlignCenter
    icon.width: 40
    icon.height: 40
    background: Rectangle {
        color: parent.icon.color == '#ffffff'  ? "transparent": '#63ffffff';
        radius: 12;
        border.color: "#1f2937";
        border.width: 2;
    }
}