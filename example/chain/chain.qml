import QtQuick 2.0
import GoExtensions 1.0

Rectangle {
	width: 800
	height: 600
	color: "black"
	Sim {
		id: sim
		anchors.fill: parent
	}
	Timer {
		interval: 10; running: true; repeat: true
		onTriggered: {
			sim.iter()
			sim.update()
		}
	}
}
