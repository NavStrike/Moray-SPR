// ===== IMPORTS =====
import QtQuick 2.15
import QtQuick.Controls.Basic 2.15
import QtQuick.Layouts 1.15
import QtQuick.Window 2.15
import QtQuick.Dialogs 6.0

Button {
    Layout.alignment: Qt.AlignCenter
    icon.width: 70
    icon.height: 70
    icon.color: "transparent"
    background: Rectangle {
        color: parent.pressed ? '#63ffffff' : "transparent"

    }
}