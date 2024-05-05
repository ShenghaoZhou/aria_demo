#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import aria.sdk as aria
from common import ctrl_c_handler
import numpy as np


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


def setup_streaming_manager(streaming_interface='usb', profile_name='profile12'):
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

    streaming_manager.streaming_config = streaming_config

    streaming_manager.start_streaming()

    streaming_state = streaming_manager.streaming_state
    print(f"Streaming state: {streaming_state}")
    return streaming_manager, device_client, device

# proposal: keep a list of data
class DataQueuingObserver:
    def __init__(self, pub_img, pub_imu, bridge):
        self.img_data = None
        self.imu_data = None
        self.pub_img = pub_img
        self.pub_imu = pub_imu
        self.bridge = bridge

    def on_image_received(self, image, record):
        # self.img_data = (image, record.capture_timestamp_ns, record.camera_id)
        timestamp = record.capture_timestamp_ns
        camera_id = record.camera_id
        if camera_id == aria.CameraId.Slam1:
            img_msg = create_img_msg(
                self.bridge, np.rot90(image, -1), timestamp)
            self.pub_img.publish(img_msg)


    def on_imu_received(self, samples, imu_idx):
        # need to use imu_idx to determine IMU
        # self.imu_data = (samples, imu_idx)
        if imu_idx == 0:
            for imu_data in samples:
                imu_msg = create_imu_msg(imu_data)
                self.pub_imu.publish(imu_msg)


def setup_ros_node(freq=10):
    # Initialize the ROS node
    rospy.init_node('aria_publisher_node', anonymous=True)

    # Create a publisher object
    pub_imu = rospy.Publisher('imu_data', Imu, queue_size=1000)
    pub_img = rospy.Publisher('camera_image', Image, queue_size=1000)

    bridge = CvBridge()
    # Set the rate of publishing
    rate = rospy.Rate(freq)  # 10hz
    return pub_imu, pub_img, bridge, rate


def main():

    pub_imu, pub_img, bridge, rate = setup_ros_node(freq=10)

    streaming_manager, device_client, device = setup_streaming_manager()

    observer = DataQueuingObserver(pub_img, pub_imu, bridge)
    streaming_client = streaming_manager.streaming_client
    config = streaming_client.subscription_config
    config.subscriber_data_type = (
        aria.StreamingDataType.Imu | aria.StreamingDataType.Slam
    )
    config.message_queue_size[aria.StreamingDataType.Imu] = 10000
    config.message_queue_size[aria.StreamingDataType.Slam] = 100
    streaming_client.subscription_config = config
    streaming_client.set_streaming_client_observer(observer)
    streaming_client.subscribe()

    print('start streaming')
    # Main loop for ROS
    with ctrl_c_handler() as ctrl_c:
        while not (rospy.is_shutdown() or ctrl_c):
            # if observer.img_data is not None:
            #     img_data, timestamp, camera_id = observer.img_data
            #     if camera_id == aria.CameraId.Slam1:
            #         img_msg = create_img_msg(
            #             bridge, np.rot90(img_data, -1), timestamp)
            #         pub_img.publish(img_msg)
            #         observer.img_data = None
            # elif observer.imu_data is not None:
            #     imu_data_batch, imu_idx = observer.imu_data
            #     if imu_idx == 0:
            #         for imu_data in imu_data_batch:
            #             imu_msg = create_imu_msg(imu_data)
            #             pub_imu.publish(imu_msg)
            #         observer.imu_data = None

            # dummy loop
            print("publishing")
            rate.sleep()

    print("Stop streaming")
    streaming_client.unsubscribe()
    streaming_manager.stop_streaming()
    device_client.disconnect(device)


if __name__ == "__main__":
    main()
