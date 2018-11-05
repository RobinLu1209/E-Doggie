# 基于ESP32的智能导盲陪伴犬E-Doggie

本项目为上海交通大学 电子信息与电气工程学院 信息工程（卓越工程师班）EE397智能系统设计课程项目作业，为了更好地管理代码和记录任务进程，我们建立了本项目说明。

## 项目背景
导盲犬给我们身边的盲人朋友带来了巨大的便利，但是他也有一些问题亟待解决：

* 1.在部分公共场所，限制了部分导盲犬的进入。

* 2.成熟导盲犬要经过14个月培训，花费12万到15万左右，价格非常昂贵。

* 3.导盲犬熟悉的环境有限，限制了盲人探索更广阔的世界。

> 因此，我们希望通过开发出这一款基于ESP32的智能导盲陪伴犬E-Doggie能够帮助每一位盲人朋友，同时作为一个家庭智能机器陪伴犬，也帮助到盲人朋友的家庭。为了方便我们可以更好地操作，我们还将辅助配套一款微信小程序来控制小车。同时，通过调研发现，现在的盲人朋友由于有手机读屏等功能已经可以帮助他们使用一些手机软件，所以手机软件的编写也非常的重要。

## 产品选型
* 关于**开发板**我们选择了乐鑫公司的ESP-WROVER-KIT作为我们的主开发板：
![image](https://github.com/RobinLu1209/E-Doggie/blob/master/ESP-WROVER-KIT.png)

------
## 11月3日-11月6日小长假待办事项
- [ ] 安装Arduino开发环境并学习使用
- [ ] 搭建小车模型，设计开发板摆放位置
- [ ] 完成PID算法的实现，并调试参数使得自平衡算法效果较优

## 一些非常好用的材料和网页链接
2018/11/4

[一文读懂PID控制算法](https://blog.csdn.net/qq_25352981/article/details/81007075)

2018/11/5

[ESP-WROVER-KIT V4.1 入门指南](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/get-started/get-started-wrover-kit.html)

[ESP32环境搭建(arduino)](https://blog.csdn.net/qq_35174914/article/details/79328043)

[平衡小车与电机PID系列视频教程](http://training.eeworld.com.cn/video/14838)
