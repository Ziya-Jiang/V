"""Publish shared RGB-D and temporally checked bar proposals; never actuators."""
import json,time
from pathlib import Path
import cv2,numpy as np,yaml
from .geometry import Rejected
from .camera_model import confirmed_upper_camera
from .handle_pair import detect_handle_pair

def packet_valid(data,now,boot_id):
    if str(data['boot_id'])!=boot_id:raise Rejected('RGB-D packet belongs to previous boot')
    age=now-float(data['monotonic_s'])
    if not np.isfinite(age) or not 0<=age<=1.:raise Rejected('Shared RGB-D packet stale/future')
    if abs(float(data['color_hardware_ms'])-float(data['depth_hardware_ms']))>40:raise Rejected('Color/depth timing skew')
    if str(data['serial'])!='CV2TC5100060' or not bool(data['aligned_depth']):raise Rejected('Wrong camera/alignment packet')
    if data['bgr'].shape!=(530,848,3) or data['depth'].shape!=(530,848):raise Rejected('Calibration image mode mismatch')
    return age

def blue_points(data):
    b=data['bgr'];dep=data['depth'];k=data['k'];d=data['d'];t=data['upper_T_camera'];mask=cv2.inRange(cv2.cvtColor(b,cv2.COLOR_BGR2HSV),np.array([95,60,20]),np.array([135,255,255]));v,u=np.nonzero((mask>0)&(dep>.1)&(dep<2.5));r=cv2.undistortPoints(np.column_stack([u,v]).astype(float).reshape(-1,1,2),k,d).reshape(-1,2)
    return np.column_stack([r[:,0]*dep[v,u],r[:,1]*dep[v,u],dep[v,u]])@t[:3,:3].T+t[:3,3]

def main(args=None):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image,CameraInfo
    from geometry_msgs.msg import PoseArray,Pose,TransformStamped
    from std_msgs.msg import String
    from tf2_ros import StaticTransformBroadcaster
    from scipy.spatial.transform import Rotation
    from cv_bridge import CvBridge
    root=Path(__file__).resolve().parents[1];cv2.setNumThreads(1)
    if not (root/'config/camera_user_20260921.yaml').exists():
        from ament_index_python.packages import get_package_share_directory
        root=Path(get_package_share_directory('seldon_cart_grasp'))
    spool_default=Path.home()/'Seldon/cart_grasp/runtime/rgbd/latest.npz'
    class Bridge(Node):
        def __init__(self):
            super().__init__('seldon_shared_rgbd_bridge');self.declare_parameter('spool',str(spool_default));self.declare_parameter('camera_config',str(root/'config/camera_user_20260921.yaml'));self.spool=Path(self.get_parameter('spool').value);camera=yaml.safe_load(Path(self.get_parameter('camera_config').value).read_text());self.upper=confirmed_upper_camera(camera);self.k=np.array(camera['K']);self.d=np.array(camera['D']);self.boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip();self.last_sequence=-1;self.previous=None;self.stable=0;self.bridge=CvBridge()
            self.rgb=self.create_publisher(Image,'/cart_grasp/camera/color/image_raw',2);self.depth=self.create_publisher(Image,'/cart_grasp/camera/depth_registered/image_raw',2);self.info=self.create_publisher(CameraInfo,'/cart_grasp/camera/color/camera_info',2);self.status=self.create_publisher(String,'/cart_grasp/rgbd_status',2);self.proposals=self.create_publisher(String,'/cart_grasp/handle_proposals',2);self.markers=self.create_publisher(PoseArray,'/cart_grasp/upper_handle_candidates',2)
            self.tf=StaticTransformBroadcaster(self);tf=TransformStamped();tf.header.stamp=self.get_clock().now().to_msg();tf.header.frame_id='SJ4Z';tf.child_frame_id='seldon_head_color_optical';tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=map(float,self.upper[:3,3]);q=Rotation.from_matrix(self.upper[:3,:3]).as_quat();tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=map(float,q);self.tf.sendTransform(tf);self.create_timer(.25,self.tick)
        def emit(self,value):
            msg=String();msg.data=json.dumps(value,allow_nan=False);self.status.publish(msg)
        def clear(self):
            self.previous=None;self.stable=0;p=PoseArray();p.header.frame_id='SJ4Z';p.header.stamp=self.get_clock().now().to_msg();self.markers.publish(p)
        def tick(self):
            try:
                with np.load(self.spool,allow_pickle=False) as archive:data={k:archive[k] for k in archive.files}
                age=packet_valid(data,time.monotonic(),self.boot)
                if not np.allclose(data['upper_T_camera'],self.upper) or not np.allclose(data['k'],self.k) or not np.allclose(data['d'],self.d):raise Rejected('RGB-D calibration changed')
                sequence=int(data['sequence'])
                if sequence==self.last_sequence:return
                if sequence<self.last_sequence:self.clear()
                self.last_sequence=sequence;stamp=self.get_clock().now().to_msg();timestamp=float(data['stamp_s']);stamp.sec=int(timestamp);stamp.nanosec=round((timestamp-int(timestamp))*1e9)
                if stamp.nanosec>=1000000000:stamp.sec+=1;stamp.nanosec=0
                color=self.bridge.cv2_to_imgmsg(data['bgr'],'bgr8');depth=self.bridge.cv2_to_imgmsg(data['depth'].astype(np.float32),'32FC1');info=CameraInfo();info.width=848;info.height=530;info.k=self.k.ravel().tolist();info.d=self.d.tolist();info.distortion_model='plumb_bob'
                for m,pub in [(color,self.rgb),(depth,self.depth),(info,self.info)]:m.header.stamp=stamp;m.header.frame_id='seldon_head_color_optical';pub.publish(m)
                proposal=detect_handle_pair(blue_points(data),timestamp);points=np.array(proposal['handles'])
                stable_previous=self.previous is not None and 0<timestamp-self.previous[0]<1.0 and np.max(np.linalg.norm(points-self.previous[1],axis=1))<.015
                self.stable=self.stable+1 if stable_previous else 1;self.previous=(timestamp,points);proposal.update(temporal_stable=self.stable>=5,stable_frames=self.stable,age_at_processing_s=age,alignment_physical_validation=False);msg=String();msg.data=json.dumps(proposal);self.proposals.publish(msg);poses=PoseArray();poses.header.frame_id='SJ4Z';poses.header.stamp=stamp
                for point in points:
                    p=Pose();p.position.x,p.position.y,p.position.z=map(float,point);p.orientation.w=1.;poses.poses.append(p)
                self.markers.publish(poses);self.emit({'state':'STABLE_PROPOSALS' if self.stable>=5 else 'TRACKING_PROPOSALS','sequence':sequence,'stable_frames':self.stable,'hardware_output':False,'alignment_physical_validation':False})
                latest=self.spool.parent/'live_proposals.json';tmp=latest.with_suffix('.tmp');tmp.write_text(json.dumps(proposal,indent=2));tmp.replace(latest)
            except Exception as error:self.clear();self.emit({'state':'REJECTED','reason':str(error),'hardware_output':False})
    rclpy.init(args=args);node=Bridge()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()
