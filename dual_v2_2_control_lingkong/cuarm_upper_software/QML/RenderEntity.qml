import QtQuick 2.1
import Qt3D.Core 2.0
import Qt3D.Render 2.9
import Qt3D.Input 2.0
import Qt3D.Extras 2.9

Entity {
    id: root
    property alias cameraPosition: camera.position
    property alias cameraViewCenter: camera.viewCenter
    property alias cameraFieldOfView: camera.fieldOfView
     
    Camera {
        id: camera
        projectionType: CameraLens.PerspectiveProjection
        fieldOfView: 2.8
        nearPlane : 0.1
        farPlane : 1000.0
        position: Qt.vector3d( 30.0, 30.0, 8.0 )
        upVector: Qt.vector3d( 0.0, 0.0, 1.0 )
        viewCenter: Qt.vector3d( 0.0, 0.0, 0.75)
    }

    SOrbitCameraController { camera: camera }

    components: [
        RenderSettings {
            activeFrameGraph: ForwardRenderer {
                camera: camera
                clearColor: Qt.rgba(0.953, 0.941, 0.925, 1.0)
            }
        },
        InputSettings { }
    ]

    Entity {
        id: ground

        components: [
            PlaneMesh {
                id: groundMesh
                width: 5
                height: width
                meshResolution: Qt.size(5,5)
            },
            Transform {
                id: groundTransform
                rotationX:90
                translation: Qt.vector3d(-0.6, 0, 0)
            },
            PhongMaterial {
                id: groundMaterial
                diffuse: Qt.rgba(0.902, 0.875, 0.843, 1.0)
                ambient: Qt.rgba(0.961, 0.945, 0.925, 1.0)
                specular: Qt.rgba(0.910, 0.459, 0.165, 1.0)
                shininess: 12.0
            }
        ]
    }

    Entity {
        id: light
        components: [
            PointLight {
                color: "#fffaf4"
                intensity: 0.65
                constantAttenuation: 1.0
                linearAttenuation: 0.0
                quadraticAttenuation: 0.0025


            },
            Transform {
                translation: Qt.vector3d(2, 2, 3.0)
            }
        ]
    }

    Entity {
        components: [
            DirectionalLight {
                worldDirection: Qt.vector3d(0, 0, -1.0);
                color: "#fff4e8"
                intensity: 0.35
            }
        ]
    }
}
