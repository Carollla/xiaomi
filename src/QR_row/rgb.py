import cv2
from cv2.wechat_qrcode import WeChatQRCode
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge
from std_msgs.msg import String
import os

class RGBNode(Node):
    def __init__(self):
        super().__init__('rgb_node')

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
        self.bridge = CvBridge()  
        self.arrow_direction_publisher = self.create_publisher(String, 'arrow_direction', qos_profile)

        # 获取当前脚本的绝对路径
        script_dir = os.path.dirname(os.path.abspath(__file__))
        model_dir = os.path.join(script_dir)
        #print(model_dir)
        #print(os.path.join(model_dir, "detect.caffemodel"))
        # 初始化 WeChatQRCode 检测器
        self.detector = WeChatQRCode(
            detector_caffe_model_path=os.path.join(model_dir, "detect.caffemodel"),
            detector_prototxt_path=os.path.join(model_dir, "detect.prototxt"),
            super_resolution_caffe_model_path=os.path.join(model_dir, "sr.caffemodel"),
            super_resolution_prototxt_path=os.path.join(model_dir, "sr.prototxt")
        )

        # 标志变量，用于控制是否继续检测
        self.arrow_detected = False
        self.qrcode_detected = False

    def preprocess(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_green = np.array([35, 43, 46])
        upper_green = np.array([77, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)
        green_img = cv2.bitwise_and(img, img, mask=mask)
        gray = cv2.cvtColor(green_img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 1)
        _, thresh = cv2.threshold(blurred, 50, 255, cv2.THRESH_BINARY)
        kernel = np.ones((3, 3), np.uint8)
        dilated = cv2.dilate(thresh, kernel, iterations=2)
        eroded = cv2.erode(dilated, kernel, iterations=1)
        return eroded

    def find_tip(self, points, convex_hull):
        length = len(points)
        indices = np.setdiff1d(range(length), convex_hull)
        if len(indices) != 2:
            return None
        tip_index = indices[0] if points[indices[0], 1] < points[indices[1], 1] else indices[1]
        return points[tip_index]

    def detect_arrow(self, img):
        processed_image = self.preprocess(img)
        contours, hierarchy = cv2.findContours(processed_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        for cnt in contours:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.025 * peri, True)
            hull = cv2.convexHull(approx, returnPoints=False)
            sides = len(hull)
            if 6 > sides > 3 and sides + 2 == len(approx):
                arrow_tip = self.find_tip(approx[:, 0, :], hull.squeeze())
                if arrow_tip is not None:
                    arrow_dir = np.array(arrow_tip) - np.array(approx.mean(axis=0)[0])
                    arrow_direction = "Right" if arrow_dir[0] > 0 else "Left"
                    self.publish_arrow_direction(arrow_direction)
                    print(f"箭头方向: {arrow_direction}")
                    cv2.drawContours(img, [approx], -1, (0, 255, 0), 3)
                    cv2.circle(img, tuple(arrow_tip), 5, (0, 0, 255), -1)
                    return True
        return False

    def detect_qrcode(self, img):
        res, points = self.detector.detectAndDecode(img)
        if res:
            print("解码数据:", res)
            print("二维码位置:", points)
            if points is not None:
                points = np.array(points).astype(int)
                cv2.polylines(img, [points], True, (0, 255, 0), 3)
            return True
        return False

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # 检测绿色箭头
        if not self.arrow_detected:
            if self.detect_arrow(cv_image):
                self.arrow_detected = True
            else:
                print("未检测到箭头")

        # 检测二维码
        if not self.qrcode_detected:
            if self.detect_qrcode(cv_image):
                self.qrcode_detected = True
            else:
                print("未检测到二维码")

        # 显示结果图像
        # cv_image = cv2.resize(cv_image, (640, 480))  # 调整显示窗口大小
        # cv2.imshow("Result Image", cv_image)
        # cv2.waitKey(10)

    def publish_arrow_direction(self, direction):
        msg = String()
        msg.data = direction
        self.arrow_direction_publisher.publish(msg)

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