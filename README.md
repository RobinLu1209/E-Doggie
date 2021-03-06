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
![image](https://github.com/RobinLu1209/E-Doggie/blob/master/picture/%E5%BE%AE%E4%BF%A1%E6%88%AA%E5%9B%BE_20181201162024.png)
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

[A4950电机驱动模块使用手册](https://wenku.baidu.com/view/b36c01766d175f0e7cd184254b35eefdc9d31519.html)

[ESP32乐鑫相关]https://docs.espressif.com/projects/esp-idf/zh_CN/latest/get-started/get-started-wrover-kit.html#get-started-esp-wrover-kit-v4-1-camera-header）

2018/11/28

[串口调试助手](https://github.com/RobinLu1209/E-Doggie/raw/master/serial.rar)

2018/12/1

尝试两块Arduino RoMeo BLE开发板，参数和相关介绍可见[详细的官方wiki](http://wiki.dfrobot.com.cn/index.php/(SKU:DFR0305)RoMeo_BLE%E6%8E%A7%E5%88%B6%E5%99%A8V1.0_%E5%85%BC%E5%AE%B9Arduino)

关于其蓝牙控制app以及相关可参见[GoBLE说明](http://wiki.dfrobot.com.cn/index.php/%E6%B5%B7%E7%9B%97%E8%88%B9%E5%A5%97%E4%BB%B6%E8%BF%9B%E9%98%B6%E6%95%99%E7%A8%8B_%E6%89%8B%E6%9C%BA%E9%81%A5%E6%8E%A7%E5%8A%9F%E8%83%BD)


## Tips:

IO21和IO22引脚是SDA和SCL的默认接口。[esp32在arduino1.8下的I2C引脚](https://blog.csdn.net/quangui666/article/details/81483645) 已将esp32头文件上传。

#define PWMB 15     //Wrover是15，DevKitC是33 

Wrover的晶振是40MHz,DevKitC是80MHz
