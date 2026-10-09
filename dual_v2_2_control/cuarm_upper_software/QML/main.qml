import QtQuick 2.1
import QtQuick.Scene3D 2.0
import Qt3D.Core 2.0
import Qt3D.Render 2.9
import Qt3D.Input 2.0
import Qt3D.Extras 2.9

Item {
    Rectangle {
        id: scene
        anchors.fill: parent
        color: "#F3F0EC"
        
        Scene3D {
            id: scene3d
            anchors.fill: parent
            anchors.margins: 0
            focus: true
            aspects: ["render", "input", "logic"]
            cameraAspectRatioMode: Scene3D.AutomaticAspectRatio

            RenderEntity {
                id: render
                // RobotEntity{
                //     id: robot
                // }
            }
        }
    }

    property var meshInstances : []
    property var arm_size: 0

    Connections {
        target: cpp_program
        function onPoseChanged() {
            for (let arm_i=0; arm_i<arm_size; arm_i++){
                for (let i=0; i<meshInstances[arm_i].length; i++){
                    meshInstances[arm_i][i].origin = cpp_program.qmlGetSimuJointPose(arm_i, i)
                }
            }
        }
    }

    Rectangle {
        anchors.fill: parent
        anchors.margins: 1
        color: "transparent"
        border.color: "#DED8D1"
        border.width: 1
        radius: 7
        enabled: false
        z: 10
    }

    Rectangle {
        anchors.left: parent.left
        anchors.top: parent.top
        anchors.margins: 14
        width: hudLabel.width + 34
        height: 31
        color: "#FDFCFB"
        border.color: "#E5DED7"
        border.width: 1
        radius: 7
        enabled: false
        z: 11

        Rectangle {
            anchors.left: parent.left
            anchors.leftMargin: 9
            anchors.verticalCenter: parent.verticalCenter
            width: 4
            height: 13
            radius: 2
            color: "#E8752A"
        }

        Text {
            id: hudLabel
            anchors.centerIn: parent
            text: "LIVE KINEMATICS  ·  3D VIEW"
            color: "#5C554E"
            font.family: "Noto Sans CJK SC"
            font.pixelSize: 10
            font.weight: Font.Medium
            font.letterSpacing: 0.8
        }
    }

    Text {
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.margins: 14
        text: "拖动旋转  ·  滚轮缩放"
        color: "#958C84"
        font.family: "Noto Sans CJK SC"
        font.pixelSize: 10
        z: 11
    }

    function initRobot(){
        let cam_pos = cpp_program.qmlGetCameraPosition();
        let cam_cen = cpp_program.qmlGetCameraViewCenter();
        render.cameraFieldOfView = cpp_program.qmlGetCameraFieldOfView();
        render.cameraPosition = Qt.vector3d(cam_pos[0], cam_pos[1], cam_pos[2]);
        render.cameraViewCenter = Qt.vector3d(cam_cen[0], cam_cen[1], cam_cen[2]);

        //Add arm
        arm_size = cpp_program.qmlGetArmSize()
        let RobotOBJComponent = Qt.createComponent("RobotEntity.qml")
        let MeshOBJComponent = Qt.createComponent("MeshEntityOBJ.qml")
        let MeshSTLComponent = Qt.createComponent("MeshEntitySTL.qml")
        for (let arm_i=0; arm_i<arm_size; arm_i++){
            let robot = RobotOBJComponent.createObject(render, {})
                    
            let mesh_cnt = cpp_program.qmlGetMeshCnt(arm_i)
            let sub_meshInstances = []
            for (let i=0; i<mesh_cnt; i++){
                let mesh_source = cpp_program.qmlGetMeshSource(arm_i, i);
                let mesh_origin = cpp_program.qmlGetSimuJointPose(arm_i, i);
                let mesh_scale = cpp_program.qmlGetMeshScale(arm_i, i);
                let inst;
                if (mesh_source.substr(mesh_source.length - 3).toLowerCase() == 'stl' ){
                    inst = MeshSTLComponent.createObject(robot, {
                        source: mesh_source,
                        origin: mesh_origin,
                        meshScale: Qt.vector3d(mesh_scale[0], mesh_scale[1], mesh_scale[2])})
                }else{
                    inst = MeshOBJComponent.createObject(robot, {
                        source: mesh_source,
                        origin: mesh_origin,
                        meshScale: Qt.vector3d(mesh_scale[0], mesh_scale[1], mesh_scale[2])})
                }
                sub_meshInstances.push(inst)
            }
            meshInstances.push(sub_meshInstances)
        }
    }
    
    Component.onCompleted: {initRobot()}
}


// //! [import]
// import QtQuick
// import QtQuick3D
// import QtQuick3D.Helpers


// View3D {
//     anchors.fill: parent

//     PerspectiveCamera {
//         id: camera
//         position: Qt.vector3d(0, 200, 300)
//         eulerRotation.x: -30
//     }

//     DirectionalLight {
//         eulerRotation.x: -30
//         eulerRotation.y: -70
//     }
    
//     AxisHelper {
//         id: axisHelper
//     }

//     WasdController {
//         id: wasdController
//         controlledObject: camera
//     }

//     Model {
//         position: Qt.vector3d(0, 0, 0)
//         source: "#Rectangle"
//         scale: Qt.vector3d(5, 5, 1)
//         materials: [ DefaultMaterial {
//                 diffuseColor: "red"
//             }
//         ]
//     }
// }