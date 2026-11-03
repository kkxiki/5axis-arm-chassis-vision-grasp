固定标定位姿说明

这些位姿来自 Jetson: /home/nvidia/calibration/right_final/samples.json
姿态格式: x,y,z,qx,qy,qz,qw；平移单位 m。
标定板: 5x7 内角点，单格 30.5mm。
本次最终标定使用样本: 1,2,3,4,6,7,8,9,10,11,12。
剔除样本: 0,5。
下次重新标定时，优先运行 used_in_best_calibration=true 的点位。
注意: 回放前必须确认相机安装、标定板位置、机械臂基座、现场安全区域都与本次一致。
