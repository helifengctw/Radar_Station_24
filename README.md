# Radar_Station_24
Private radar station repository for CKYF Robomaster 2024

please contact with me if you have any problems about this project (emial: hlfctw@proton.me)

代码说明：
1，雷达比赛启动程序：把雷达传感器端放上雷达摆放平台，运算端放到下面，连好线。打开终端，执行scripts/sensor_qidong脚本，然后再开一个终端，执行ros2 run utilities pick_up_point命令，进行pnp选点。

2，radar24_ws中除了lidar_localization_ros2和ndt_omp_ros2完全没用到外，其他的都用到了，必须根据scripts/sensor_qidong里面的东西一个一个看。

3，雷达站有左右两个相机和一个激光雷达，相机目前是一个看远一个看近，激光雷达是几乎覆盖全场。左右两个相机的运行代码几乎一样，所以将相机驱动、yolo识别、激光雷达驱动和获取深度包放在一个ros2组件中（component_sensor_far/component_sensor_close）,两个组件的代码相同，参数和命名空间不同，其中只有一个组件驱动激光雷达，另一个组件不驱动激光雷达，只订阅重映射到该命名空间的点云话题。

4，采用组件的原因是采用进程内通信，将图像话题和识别等放在一个进程内，但是我也没具体测试组件的优化作用到底大不大。

5，雷达的工作原理和算法可以参考战队仓库的22年雷达站的开源文档。

以下是各个包的具体说明：

a、bayer_camera_driver:采用bayer格式驱动相机获得图像，这个包的关键点在于相机曝光、增益等参数的调节，调不好的话对导致图像中运动物体的拖影长，画面太暗，帧率太低等问题。

b、yolov5_detector：主要涉及yolov5、KM_matching等算法。其中有两层yolov5神经网络，分别识别相机画面中的红方和蓝方的车，和第一次网络识别到的车中的装甲板数字。采用trt部署的yolov5，是从github上tensorrtx仓库改进过来的。KM_matching是用来进行帧之间的运动预测匹配，来增强识别的连续性。最后将识别出的框和车的id发送出来。

c、get_depth：订阅激光雷达的点云话题和yolo的识别结果话题。将点云投影的相机图像画面，并根据yolo的识别框按照一定策略取出对应车的深度。最后将车的框的深度和图像坐标发送出去。

d、small_map：订阅车的框的深度和图像坐标，并将其投影到小地图坐标系上，再加以判断，筛选出敌方车辆，将车在小地图上的坐标发送出去。并根据裁判系统的一些信息进行自主决策，判断出发双倍易伤的时机，再发送出去。

e、robot_serial：串口通信包，订阅小地图的处理结果，并与裁判系统通信，收发车辆的坐标和场地信息。

f、pnp_solver：进行pnp标定的计算，订阅utilites/pick_up_point的选点消息，进行pnp计算。同时，记录并更新各个参数，在small_map启动时，为其提供所需参数以进行计算。

g、utilites：工具包，目前只有pick_up_point一个节点，用于准备阶段进行pnp选点，并将点的信息发送出去。

