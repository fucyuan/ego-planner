以下是你提供的README的中文翻译版本：

# 注意！
我们最近开发的规划器 [EGO-Swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm) 是EGO-Planner的升级版本。它更为健壮和安全，因此我们推荐使用EGO-Swarm。
如果你只有一架无人机，只需在EGO-Swarm的launch文件中将 `drone_id` 设置为 `0` 即可。
当然，一些话题名称相较于EGO-Planner有所更改，你可以使用`rqt_graph`和`rosnode info <package name>`来检查。

# 三分钟快速上手
测试通过的编译环境包括安装了ROS的Ubuntu **16.04、18.04 和 20.04**。
你只需要按顺序执行以下命令：

```
sudo apt-get install libarmadillo-dev
git clone https://github.com/ZJU-FAST-Lab/ego-planner.git
cd ego-planner
catkin_make
source devel/setup.bash
roslaunch ego_planner simple_run.launch
```

如果你的网络访问GitHub较慢，建议使用Gitee镜像库：[https://gitee.com/iszhouxin/ego-planner](https://gitee.com/iszhouxin/ego-planner)，它们会自动同步。

如果你觉得这个项目对你有帮助，请给我们一个星星 :star:，谢谢！:grinning:

# 致谢
- 本仓库的框架基于 [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)，由周博瑜开发，取得了在四旋翼局部规划方面的优秀表现。
- 我们使用的L-BFGS求解器来自 [LBFGS-Lite](https://github.com/ZJU-FAST-Lab/LBFGS-Lite)。它是一个轻量级的C++单文件头文件，易于使用。
- 仿真中生成的地图来自 [mockamap](https://github.com/HKUST-Aerial-Robotics/mockamap)，由吴威廉开发。
- 硬件架构基于 [Teach-Repeat-Replan](https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan) 的开源实现。

# EGO-Planner 
EGO-Planner: 一种无需ESDF的基于梯度的四旋翼局部规划器

**EGO-Planner** 是一种轻量级的基于梯度的局部规划器，不需要ESDF构建，与某些最先进的方法相比（如EWOK和Fast-Planner），显著减少了计算时间。总规划时间仅为 **约1毫秒**，且无需计算ESDF。

<p align="center">
<img src="pictures/title.gif" width="413" height="232" border="5" />
<img src="pictures/comp.jpg" width="413" height="232" border="5" />
<img src="pictures/indoor.gif" width="413" height="232" border="5" />
<img src="pictures/outdoor.gif" width="413" height="232" border="5" />
</p>

**视频链接：** [YouTube](https://youtu.be/UKoaGW7t7Dk)，[哔哩哔哩](https://www.bilibili.com/video/BV1VC4y1t7F4/) （适用于中国大陆）

## 1. 相关论文
EGO-Planner: 一种无需ESDF的基于梯度的四旋翼局部规划器，作者：周鑫，王哲培，徐超，高飞 (被RA-L接收)。 [arXiv预印本](https://arxiv.org/abs/2008.08835)，[IEEE Xplore](https://ieeexplore.ieee.org/abstract/document/9309347)，和 [IEEE Spectrum报告](https://spectrum.ieee.org/automaton/robotics/robotics-hardware/video-friday-mit-media-lab-tf8-bionic-ankle)。

## 2. 标准编译

**要求：** ubuntu 16.04、18.04或20.04，安装有ros-desktop-full。

**步骤1**. 安装 [Armadillo](http://arma.sourceforge.net/)，它是 **uav_simulator** 所需的依赖库。
```
sudo apt-get install libarmadillo-dev
```

**步骤2**. 从GitHub或Gitee克隆代码。这两个仓库会自动同步。

从GitHub克隆：
```
git clone https://github.com/ZJU-FAST-Lab/ego-planner.git
```

或者从Gitee克隆：
```
git clone https://gitee.com/iszhouxin/ego-planner.git
```

**步骤3**. 编译：
```
cd ego-planner
catkin_make -DCMAKE_BUILD_TYPE=Release
```

**步骤4**. 运行。

在 _ego-planner/_ 文件夹中打开终端，使用rviz进行可视化和交互：
```
source devel/setup.bash
roslaunch ego_planner rviz.launch
```

在另一个终端中，在 _ego-planner/_ 目录下运行仿真中的规划器：
```
source devel/setup.bash
roslaunch ego_planner run_in_sim.launch
```

然后你可以按照下方的动图控制无人机。

<p align="center">
<img src="pictures/sim_demo.gif" width="640" height="438" border="5" />
</p>

## 3. 使用集成开发环境 (IDE)
我们推荐使用 [vscode](https://code.visualstudio.com/)，项目文件已包含在代码中，即 _.vscode_ 文件夹。该文件夹默认是 **隐藏的**。

按照以下步骤为IDE配置自动代码补全和跳转功能。整个过程大约需要3分钟。

**步骤1**. 在vscode中安装C++和CMake扩展。

**步骤2**. 使用以下命令重新编译代码：
```
catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes
```
这将导出一个编译命令文件，帮助vscode识别代码结构。

**步骤3**. 启动vscode并选择 _ego-planner_ 文件夹打开：
```
code ~/<......>/ego-planner/
```

在vscode中按 **Ctrl+Shift+B** 编译代码。这个命令已在 _.vscode/tasks.json_ 中定义。你可以在 **"args"** 后添加自定义参数。默认是 **"-DCMAKE_BUILD_TYPE=Release"**。

**步骤4**. 关闭并重新启动vscode，你将看到vscode已经理解了代码结构，并能执行自动补全和跳转。

## 4. 使用GPU或不使用GPU
本仓库中的 **local_sensing** 包提供了GPU和CPU两个不同版本。默认情况下使用CPU版本，以提高兼容性。通过修改 **local_sensing** 包中的 _CMakeList.txt_ 文件可以切换：

将
```
set(ENABLE_CUDA false)
```
改为
```
set(ENABLE_CUDA true)
```
将启用CUDA来生成深度图像，如同真实深度相机一样。

请记得修改以下代码中的`arch`和`code`标志：
```
set(CUDA_NVCC_FLAGS 
    -gencode arch=compute_61,code=sm_61;
)
```
如果你使用的Nvidia显卡型号不同，可能会遇到编译错误或者无法正确生成深度图像，可以通过以下链接查看正确的`code`参数：[link1](https://arnon.dk/matching-sm-architectures-arch-and-gencode-for-various-nvidia-cards/) 或 [link2](https://github.com/tpruvot/ccminer/wiki/Compatibility)。

别忘了重新编译代码！

**local_sensing** 是模拟的传感器。如果 `ENABLE_CUDA` 设置为 **true**，它将模拟通过GPU生成的立体相机深度测量。如果 `ENABLE_CUDA` 为 **false**，它将发布点云而不进行光线投射。我们的局部映射模块会自动选择深度图像或点云作为输入。

安装CUDA的更多信息请参考 [CUDA ToolKit](https://developer.nvidia.com/cuda-toolkit)。

## 5. 充分利用CPU性能
由于我们的规划器计算时间极短，操作系统可能无法及时提升CPU频率，导致计算时间延长且不稳定。

因此，我们建议手动将CPU频率设置为最高。首先安装工具：
```
sudo apt install cpufrequtils
```
然后将CPU频率设置为最大：
```
sudo cpufreq-set -g performance
```
更多信息请参考：[http://www.thinkwiki.org/wiki/How_to_use_cpufrequtils](http://www.thinkwiki.org/wiki/How_to_use_cpufrequtils)。

注意，高负载下CPU频率可能仍会因温度过高而降低。

# 改进的ROS-RealSense驱动
我们修改了ros-realsense驱动，启用了激光发射器每隔一帧闪烁功能，使设备能够借助发射器输出高质量的深度图像，同时提供不受激光干扰

的双目图像。

<p align="center">
<img src="pictures/realsense.PNG" width="640" height="158" border="5" />
</p>

这个ros驱动修改自 [https://github.com/IntelRealSense/realsense-ros](https://github.com/IntelRealSense/realsense-ros)，与librealsense2 2.30.0兼容。测试在Intel RealSense D435和D435i上进行。

参数 `emitter_on_off` 用于开启/关闭此功能。请注意，开启此功能时，设备的输出帧率将减半，因为设备使用一半的流进行深度估计，另一半输出为双目灰度图。此外，参数 `depth_fps` 和 `infra_fps` 必须一致，且 `enable_emitter` 也必须设置为true。

## 安装
librealsense2 2.30.0的驱动需要明确安装。在x86 CPU上，安装过程非常简单，仅需5分钟。

首先，移除当前安装的驱动：
```
sudo apt remove librealsense2-utils
```
或者如果你从源代码安装了librealsense，可以手动移除相应文件。

然后，可以安装2.30.0版本的库：
```
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
```

对于Ubuntu 16.04：
```
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
```

对于Ubuntu 18.04：
```
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
```

接着安装：
```
sudo apt-get install librealsense2-dkms
sudo apt install librealsense2=2.30.0-0~realsense0.1693
sudo apt install librealsense2-gl=2.30.0-0~realsense0.1693
sudo apt install librealsense2-utils=2.30.0-0~realsense0.1693
sudo apt install librealsense2-dev=2.30.0-0~realsense0.1693
sudo apt remove librealsense2-udev-rules
sudo apt install librealsense2-udev-rules=2.30.0-0~realsense0.1693
```

你可以通过以下命令验证安装：
```
realsense-viewer
```

## 运行
如果一切正常，你可以使用`catkin_make`编译ros-realsense包 _modified_realsense2_camera.zip_，然后运行ros realsense节点：
```
roslaunch realsense_camera rs_camera.launch
```

然后你会以默认30Hz的速度接收深度流和双目流。

# 许可证
源代码依据 [GPLv3](http://www.gnu.org/licenses/) 许可证发布。

# 维护
我们仍在不断扩展系统功能和提高代码可靠性。

如有任何技术问题，请联系周鑫 (iszhouxin@zju.edu.cn) 或高飞 (fgaoaa@zju.edu.cn)。

如有商业合作需求，请联系高飞 (fgaoaa@zju.edu.cn)。