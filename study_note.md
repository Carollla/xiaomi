- robot_control_cmd_lcmt 类：
    这是控制命令的数据结构
    包含了所有控制参数：模式、步态、速度、姿态等
    可以自定义各种控制命令
- robot_control_response_lcmt 类：
    这是机器人响应的数据结构
    包含了状态反馈：执行进度、错误信息等
    用于监控命令执行情况

- Robot_Ctrl 类：
    这是一个完整的机器人控制接口
    提供了命令发送、响应接收、状态监控等功能
    可以直接用于开发上层应用
- RobotControlNode 类：
    这是一个ROS2节点，与Robot_Ctrl功能类似，但集成在ROS2生态系统中
    将ROS2的Twist消息转换为LCM控制命令，接受的是Twist消息，但发送的是lcm
    适合与ROS2生态系统集成，有什么导航、传感器之类的;

