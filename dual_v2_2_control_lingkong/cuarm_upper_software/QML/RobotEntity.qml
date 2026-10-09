import Qt3D.Core 2.0
import Qt3D.Render 2.0 
import Qt3D.Input 2.0
import Qt3D.Extras 2.9

Entity {
    id: root
    property alias rotationX: transformation.rotationX
    property alias rotationY: transformation.rotationY
    property alias rotationZ: transformation.rotationZ
    property alias translation: transformation.translation
    components : [
        Transform {
            id: transformation
        }
    ]

    // PhongMaterial {
    //     id: material1
    //     ambient : Qt.rgba( 0.96, 0.96, 0.96, 1.0 )
    //     specular: Qt.rgba( 0, 0, 0, 1.0 )
    // }
}
