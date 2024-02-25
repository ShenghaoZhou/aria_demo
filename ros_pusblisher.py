import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import aria.sdk as aria

def create_img_msg(bridge, aria_img_data, timestamp):
    # Convert the image to a ROS Image message
    img_msg = bridge.cv2_to_imgmsg(aria_img_data, encoding="passthrough")
    img_msg.header.stamp = rospy.Time.from_sec(timestamp / 1e9)
    return img_msg

def create_imu_msg(aria_imu_data):
    
    timstamp = aria_imu_data.capture_timestamp_ns
    accel = aria_imu_data.accel_msec2
    gyro = aria_imu_data.gyro_radsec
    # Create an instance of the Imu message
    imu_msg = Imu()

    imu_msg.header.stamp = rospy.Time.from_sec(timstamp / 1e9)

    # Fill in the data for the Imu message
    # Here we are just filling in dummy data for the example
    imu_msg.angular_velocity.x = gyro[0]
    imu_msg.angular_velocity.y = gyro[1]
    imu_msg.angular_velocity.z = gyro[2]

    imu_msg.linear_acceleration.x = accel[0]
    imu_msg.linear_acceleration.y = accel[1]
    imu_msg.linear_acceleration.z = accel[2]
    return imu_msg


def setup_streamline_manager(streaming_interface='usb', profile_name='profile12'):
    device_client = aria.DeviceClient()

    client_config = aria.DeviceClientConfig()
    # if args.device_ip:
    #     client_config.ip_v4_address = args.device_ip
    device_client.set_client_config(client_config)
    device = device_client.connect()
    streaming_manager = device.streaming_manager
    streaming_config = aria.StreamingConfig()
    streaming_config.profile_name = profile_name
    # Note: by default streaming uses Wifi
    if streaming_interface == "usb":
        streaming_config.streaming_interface = aria.StreamingInterface.Usb
    streaming_config.security_options.use_ephemeral_certs = True
    streaming_manager.start_streaming()
    streaming_manager.streaming_config = streaming_config
    streaming_state = streaming_manager.streaming_state
    print(f"Streaming state: {streaming_state}")
    return streaming_manager


class DataQueuingObserver:
    def __init__(self):
            self.img_data = None
            self.imu_data = None

    def on_image_received(self, image, record):
        self.img_data = (image, record.capture_timestamp_ns, record.camera_id)
    
    def on_imu_received(self, samples, imu_idx):
        # need to use imu_idx to determine IMU
        self.imu_data = (samples, imu_idx)


def setup_ros_node(freq=10):
    # Initialize the ROS node
    rospy.init_node('aria_publisher_node', anonymous=True)

    # Create a publisher object
    pub_imu = rospy.Publisher('imu_data', Imu, queue_size=10)
    pub_img = rospy.Publisher('camera_image', Image, queue_size=10)

    bridge = CvBridge()
    # Set the rate of publishing
    rate = rospy.Rate(freq) # 10hz
    return pub_imu, pub_img, bridge, rate

def main():

    pub_imu, pub_img, bridge, rate = setup_ros_node(freq=1000)

    streamline_manager = setup_streamline_manager()

    observer = DataQueuingObserver()
    streaming_client = streamline_manager.streaming_client
    streaming_client.set_streaming_client_observer(observer)
    streaming_client.subscribe()

    # Main loop for ROS 
    while not rospy.is_shutdown():
        if observer.img_data is not None:
            img_msg = create_img_msg(bridge, observer.img_data)
            pub_img.publish(img_msg)
            observer.img_data = None
        elif observer.imu_data is not None:
            for imu_data in observer.imu_data:
                imu_msg = create_imu_msg(imu_data)
                pub_imu.publish(imu_msg)
            observer.imu_data = None
            
        # Sleep for the remaining time to hit our 10hz publish rate
        rate.sleep()
