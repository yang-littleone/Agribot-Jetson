# 项目介绍

当前项目为我个人在读研期间毕设所做课题，目的是实现农业小车在玉米行间自主行走。

环境：ubuntun 22.04 + ros2 humble

当前已完成：

- 小车在gazebo仿真环境中运行
- 给小车添加livox-mid360雷达的仿真包，雷达数据的仿真
- 完成仿真环境下fast-lio2的建图
- 使用fast-lio2的建图完成纯追踪算法的行间行走（还不好使）
- 完成雷达实物下fast-lio2的建图

# 使用

mid360的使用：

:red_circle:注意：进行mid360配置的时候，一定注意install目录中share中的config是否也进行同步配置。因为有时候在功能包配置完了，但是share中未同步。



# 特别感谢

实现mid360的仿真：https://github.com/LihanChen2004/livox_laser_simulation_ros2.git