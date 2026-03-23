// ===== IMPORTS =====
import QtQuick 2.15
import QtQuick.Controls.Basic 2.15
import QtQuick.Layouts 1.15
import QtQuick.Window 2.15
import QtQuick.Dialogs 6.0

Button {
    checkable: true
    Layout.alignment: Qt.AlignCenter
    icon.width: 40
    icon.height: 40
    icon.color: checked  ? 'transparent' : "white";
    background: Rectangle {
        color: checked  ? '#63ffffff' : "transparent";
        radius: 12;
        border.color: "#1f2937";
        border.width: 2;
    }
    
}