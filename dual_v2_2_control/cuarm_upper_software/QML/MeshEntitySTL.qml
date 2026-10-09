import Qt3D.Core 2.0
import Qt3D.Render 2.0

Entity{
    id: root
    property url source: ""
    property var origin: [0, 0, 0, 1, 0, 0, 0] // [x,y,z,qw,qx,qy,qz]
    property vector3d meshScale: Qt.vector3d(1, 1, 1)
    onOriginChanged:{
        transform.translation = Qt.vector3d(origin[0], origin[1], origin[2])
        transform.rotation = Qt.quaternion(origin[3], origin[4], origin[5], origin[6])
    }
    components:[
        SceneLoader {
            source: root.source
        },
        Transform {
            id: transform
            translation: Qt.vector3d(origin[0], origin[1], origin[2])
            rotation: Qt.quaternion(origin[3], origin[4], origin[5], origin[6])
            scale3D: root.meshScale
        }
    ]
}