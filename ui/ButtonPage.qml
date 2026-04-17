// ===== IMPORTS =====
import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import QtQuick.Window
import QtQuick.Dialogs

Button {
    Layout.alignment: Qt.AlignCenter
    icon.width: 70
    icon.height: 70
    icon.color: "white"
    background: Rectangle {
        color: parent.pressed ? '#63ffffff' : "transparent"
    }
}