import cv2
from cv2.wechat_qrcode import WeChatQRCode
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge
from std_msgs.msg import Int32MultiArray

class RGBNode(Node):
    def __init__(self):
        super().__init__('rgb_node')
        self.get_logger().info("二维码检测节点已初始化")
        qos_profile = rclpy.qos.QoSProfile(
            depth=10,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
        )
        self.subscription = self.create_subscription(
            ROSImage,  
            '/rgb_camera/image_raw',
            self.image_callback,
            qos_profile
        )
        self.signal_publisher=self.create_publisher(Int32MultiArray,'Qrcode1',10)
        self.bridge = CvBridge()  
        # 初始化 WeChatQRCode 检测器
        self.detector = WeChatQRCode(
            detector_prototxt_path="/home/nav/cyberdog_nudt/right_all/detect/QR_row/detect.prototxt",
            detector_caffe_model_path="/home/nav/cyberdog_nudt/right_all/detect/QR_row/detect.caffemodel",
            super_resolution_prototxt_path="/home/nav/cyberdog_nudt/right_all/detect/QR_row/sr.prototxt",
            super_resolution_caffe_model_path="/home/nav/cyberdog_nudt/right_all/detect/QR_row/sr.caffemodel"
        )

    def image_callback(self, msg):
        #self.get_logger().info("已收到图像信息")
        # 将 ROS 图像消息转换为 OpenCV 图像
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        #cv2.imshow("Original Image", cv_image)
        
        # 使用 WeChatQRCode 进行二维码检测与解码
        res, points = self.detector.detectAndDecode(cv_image)
        
        # 初始化返回的布尔值变量
        result_bool = None
        
        if res:
            print("解码数据:", res)
            print("二维码位置:", points)
            
            # 根据解码结果设置布尔值
            if res[0] == 'A-1':
                result_bool = True  # 返回 Bool 值为 1
            elif res[0] == 'A-2':
                result_bool = False  # 返回 Bool 值为 0
            else:
                print("解码结果不符合预期")
        else:
            print("未检测到二维码")
        self.send_signal(result_bool)
        # 显示结果图像
        #cv2.imshow("Result Image", cv_image)
        #cv2.waitKey(10)
        
        # 返回布尔值（如果需要）
        return result_bool
    def send_signal(self,result):
        # 发布信号消息
        msg = Int32MultiArray()
        # 确保 result 是布尔值或 None
        if result is not None:
            msg.data = [1 if result else 2]  # 发布 1 或 2
        else:
            msg.data = [0]  # 默认值为 0
        self.signal_publisher.publish(msg)
        #print("Qrcode signal sent with data:", msg.data)

def main(args=None):
    rclpy.init(args=args)
    rgb_node = RGBNode()
    print('-----------rgb_node is ok-----------')
    try:
        rclpy.spin(rgb_node)
    except KeyboardInterrupt:
        pass
    rgb_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()