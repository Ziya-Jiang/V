"""ROS 2 observer. Only /cart_grasp outputs; no motor publishers/services."""
import json,time
import numpy as np
import yaml
from .geometry import Rejected,pose,apply
from .perception import CartDetector,StableTracker,cloud_from_rgbd

def main(args=None):
    import cv2
    cv2.setNumThreads(1)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from rclpy.duration import Duration
    from sensor_msgs.msg import Image,CameraInfo
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseArray,Pose
    from cv_bridge import CvBridge
    from message_filters import Subscriber,ApproximateTimeSynchronizer
    from tf2_ros import Buffer,TransformListener
    from scipy.spatial.transform import Rotation
    from ament_index_python.packages import get_package_share_directory
    from pathlib import Path
    class Observer(Node):
        def __init__(self):
            super().__init__('seldon_cart_observer');default=Path(__file__).resolve().parents[1]/'config/cart_grasp.yaml';default=default if default.exists() else Path(get_package_share_directory('seldon_cart_grasp'))/'config/cart_grasp.yaml';self.declare_parameter('config_file',str(default));self.cfg=yaml.safe_load(Path(self.get_parameter('config_file').value).read_text());self.c=self.cfg['ros'];self.detector=CartDetector(self.cfg);self.tracker=StableTracker(self.cfg['perception']);self.bridge=CvBridge();self.buffer=Buffer();self.listener=TransformListener(self.buffer,self);self.last_run=0.;self.last_stamp=0.;self.last_valid=0.;self.info=None
            self.status=self.create_publisher(String,'/cart_grasp/status',10);self.handles=self.create_publisher(PoseArray,'/cart_grasp/handle_candidates',10);self.obs=self.create_publisher(String,'/cart_grasp/observation',10)
            self.create_subscription(CameraInfo,self.c['info_topic'],self.on_info,qos_profile_sensor_data)
            self.rgb=Subscriber(self,Image,self.c['rgb_topic'],qos_profile=qos_profile_sensor_data);self.depth=Subscriber(self,Image,self.c['depth_topic'],qos_profile=qos_profile_sensor_data);self.sync=ApproximateTimeSynchronizer([self.rgb,self.depth],queue_size=3,slop=self.c['max_pair_skew_s']);self.sync.registerCallback(self.on_pair);self.create_timer(.2,self.watchdog)
            self.emit('WAITING_FOR_RGBD',hardware_output=False)
        def emit(self,state,**kw):
            msg=String();msg.data=json.dumps({'state':state,'hardware_output':False,**kw});self.status.publish(msg)
        def on_info(self,msg):self.info=msg
        def watchdog(self):
            if time.monotonic()-self.last_valid>1.0:self.tracker.clear();self.emit('NO_FRESH_METRIC_OBSERVATION')
        @staticmethod
        def seconds(stamp):return stamp.sec+stamp.nanosec*1e-9
        def on_pair(self,rgb,depth):
            if time.monotonic()-self.last_run<1/max(.1,min(5.,self.c['max_rate_hz'])):return
            self.last_run=time.monotonic()
            try:
                now=self.get_clock().now().nanoseconds*1e-9;stamp=self.seconds(rgb.header.stamp);ds=self.seconds(depth.header.stamp)
                if stamp<=self.last_stamp or stamp<=0 or not 0<=now-stamp<=self.c['max_age_s'] or not 0<=now-ds<=self.c['max_age_s'] or abs(stamp-ds)>self.c['max_pair_skew_s']:raise Rejected('Stale/future/duplicate RGB-D timestamps')
                self.last_stamp=stamp
                if self.c['camera_tf_calibrated'] is not True:raise Rejected('Camera-to-base transform/floor not calibrated')
                if self.info is None or self.seconds(self.info.header.stamp)<=0 or not 0<=now-self.seconds(self.info.header.stamp)<=1.:raise Rejected('Need fresh CameraInfo')
                frame=self.c['camera_optical_frame']
                if rgb.header.frame_id!=frame or depth.header.frame_id!=frame or self.info.header.frame_id!=frame:raise Rejected('RGB, aligned depth and intrinsics frame mismatch')
                if (self.info.height,self.info.width)!=(rgb.height,rgb.width) or (depth.height,depth.width)!=(rgb.height,rgb.width):raise Rejected('Resolution mismatch')
                distortion=None
                if any(abs(v)>1e-8 for v in self.info.d):
                    if self.c.get('image_geometry','rectified')!='raw' or self.info.distortion_model!='plumb_bob':raise Rejected('Nonzero distortion requires explicit raw image mode and plumb_bob CameraInfo')
                    distortion=list(self.info.d)
                tf=self.buffer.lookup_transform(self.c['base_frame'],frame,Time.from_msg(rgb.header.stamp),timeout=Duration(seconds=.03));ts=self.seconds(tf.header.stamp)
                if ts!=0 and abs(stamp-ts)>self.c['tf_max_age_s']:raise Rejected('Camera TF stale')
                t=tf.transform;matrix=np.eye(4);matrix[:3,3]=[t.translation.x,t.translation.y,t.translation.z];q=np.array([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]);
                if abs(np.linalg.norm(q)-1)>.001:raise Rejected('Invalid camera quaternion')
                matrix[:3,:3]=Rotation.from_quat(q).as_matrix()
                if depth.encoding not in ('16UC1','32FC1'):raise Rejected('Unsupported depth encoding')
                bgr=self.bridge.imgmsg_to_cv2(rgb,'bgr8');d=self.bridge.imgmsg_to_cv2(depth,'passthrough');points,_=cloud_from_rgbd(bgr,d,np.array(self.info.k).reshape(3,3),matrix,self.cfg['perception'],.001 if depth.encoding=='16UC1' else 1.,self.c.get('aligned_depth_verified',self.c['rectified_aligned_depth']),distortion=distortion);observation=self.detector.estimate(points,stamp);stable=self.tracker.update(observation);self.last_valid=time.monotonic()
                msg=String();msg.data=json.dumps({**observation.to_dict(),'stable':stable,'frame_id':self.c['base_frame'],'hardware_output':False});self.obs.publish(msg)
                poses=PoseArray();poses.header=rgb.header;poses.header.frame_id=self.c['base_frame']
                for point in observation.handles:
                    p=Pose();p.position.x,p.position.y,p.position.z=map(float,point);p.orientation.w=1.;poses.poses.append(p)
                self.handles.publish(poses);self.emit('STABLE_HANDLES' if stable else 'TRACKING',residual_m=observation.residual_m)
            except Exception as error:self.tracker.clear();self.emit('REJECTED',reason=str(error))
    rclpy.init(args=args);node=Observer()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()
