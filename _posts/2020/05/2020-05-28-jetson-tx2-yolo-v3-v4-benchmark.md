---
layout: single
title: "Jetson TX2: framerate comparison between YOLOv4 and YOLOv3-tyny"
author: bartoszptak
excerpt: "How does YOLOv4 work on NVIDIA Jetson TX2? We compare with YOLOv3-tyny to choose an effective and fast fast object detection system."
modified: 2020-05-28
tags: [nvidia, jetson, tx2, yolov3, yolov4, fps, benchmark]
category: edge-devices
image: "assets/images/posts/2020/05/jetson-tx2-yolo-v3-v4-benchmark/predictions.jpg"
---

[YOLO](https://pjreddie.com/darknet/yolo/) is an efficient and fast object detection system. Recently a new version has appeared - [YOLOv4](https://arxiv.org/pdf/2004.10934.pdf). How does it work on NVIDIA Jetson TX2? Time to check!

# Benchmark setup
* prints the name, version and other details about the current machine and the operating system running on it:

```bash
$ uname -a

Linux antmicro-tx2-baseboard 4.9.140-tegra #2 SMP PREEMPT Tue May 19 16:58:27 CEST 2020 aarch64 aarch64 aarch64 GNU/Linux
```

* check Linux for Tegra version (we have R32.3.1)

```bash
$ cat /etc/nv_tegra_release

# R32 (release), REVISION: 3.1, GCID: 18186506, BOARD: t186ref, EABI: aarch64, DATE: Tue Dec 10 07:03:07 UTC 2019
```

* set the highest performance settings with `sudo nvpmodel -m 0` and check the frequencies:

```bash
$ sudo nvpmodel -q --verbose

NVPM VERB: Config file: /etc/nvpmodel.conf
NVPM VERB: parsing done for /etc/nvpmodel.conf
NVPM VERB: Current mode: NV Power Mode: MAXN
0
NVPM VERB: PARAM CPU_ONLINE: ARG CORE_1: PATH /sys/devices/system/cpu/cpu1/online: REAL_VAL: 1 CONF_VAL: 1
NVPM VERB: PARAM CPU_ONLINE: ARG CORE_2: PATH /sys/devices/system/cpu/cpu2/online: REAL_VAL: 1 CONF_VAL: 1
NVPM VERB: PARAM CPU_ONLINE: ARG CORE_3: PATH /sys/devices/system/cpu/cpu3/online: REAL_VAL: 1 CONF_VAL: 1
NVPM VERB: PARAM CPU_ONLINE: ARG CORE_4: PATH /sys/devices/system/cpu/cpu4/online: REAL_VAL: 1 CONF_VAL: 1
NVPM VERB: PARAM CPU_ONLINE: ARG CORE_5: PATH /sys/devices/system/cpu/cpu5/online: REAL_VAL: 1 CONF_VAL: 1
NVPM VERB: PARAM CPU_A57: ARG MIN_FREQ: PATH /sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq: REAL_VAL: 345600 CONF_VAL: 0
NVPM VERB: PARAM CPU_A57: ARG MAX_FREQ: PATH /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq: REAL_VAL: 2035200 CONF_VAL: 2147483647
NVPM VERB: PARAM CPU_DENVER: ARG MIN_FREQ: PATH /sys/devices/system/cpu/cpu1/cpufreq/scaling_min_freq: REAL_VAL: 345600 CONF_VAL: 0
NVPM VERB: PARAM CPU_DENVER: ARG MAX_FREQ: PATH /sys/devices/system/cpu/cpu1/cpufreq/scaling_max_freq: REAL_VAL: 2035200 CONF_VAL: 2147483647
NVPM VERB: PARAM GPU_POWER_CONTROL_ENABLE: ARG GPU_PWR_CNTL_EN: PATH /sys/devices/gpu.0/power/control: REAL_VAL: auto CONF_VAL: on
NVPM VERB: PARAM GPU: ARG MIN_FREQ: PATH /sys/devices/17000000.gp10b/devfreq/17000000.gp10b/min_freq: REAL_VAL: 114750000 CONF_VAL: 0
NVPM VERB: PARAM GPU: ARG MAX_FREQ: PATH /sys/devices/17000000.gp10b/devfreq/17000000.gp10b/max_freq: REAL_VAL: 1300500000 CONF_VAL: 2147483647
NVPM VERB: PARAM GPU_POWER_CONTROL_DISABLE: ARG GPU_PWR_CNTL_DIS: PATH /sys/devices/gpu.0/power/control: REAL_VAL: auto CONF_VAL: auto
NVPM VERB: PARAM EMC: ARG MAX_FREQ: PATH /sys/kernel/nvpmodel_emc_cap/emc_iso_cap: REAL_VAL: 0 CONF_VAL: 0
```

* check CUDA version (we have 10.0):

```bash
$ nvcc --version

nvcc: NVIDIA (R) Cuda compiler driver
Copyright (c) 2005-2019 NVIDIA Corporation
Built on Mon_Mar_11_22:13:24_CDT_2019
Cuda compilation tools, release 10.0, V10.0.326
```

* display camera parameters (we have `1920x1080` resolution for `/dev/video0`)

```bash
$ gst-device-monitor-1.0 Video/Source

Device found:

	name  : vi-output, ov5640 32-003c
	class : Video/Source
	caps  : video/x-raw, format=(string)UYVY, width=(int)1920, height=(int)1080, framerate=(fraction)30/1;
	properties:
		udev-probed = true
		device.bus_path = platform-15700000.vi
		sysfs.path = /sys/devices/13e10000.host1x/15700000.vi/video4linux/video0
		device.subsystem = video4linux
		device.product.name = "vi-output\,\ ov5640\ 32-003c"
		device.capabilities = :capture:
		device.api = v4l2
		device.path = /dev/video0
		v4l2.device.driver = tegra-video
		v4l2.device.card = "vi-output\,\ ov5640\ 32-003c"
		v4l2.device.bus_info = platform:15700000.vi:0
		v4l2.device.version = 264588 (0x0004098c)
		v4l2.device.capabilities = 2216689665 (0x84200001)
		v4l2.device.device_caps = 69206017 (0x04200001)
	gst-launch-1.0 v4l2src ! ...

```

# Build from sources
We use the implementation from [AlexeyAB/darknet](https://github.com/AlexeyAB/darknet). This is currently the official implementation of YOLO v4. We recommend reading all the information on that page.

* download sources
```
git clone https://github.com/AlexeyAB/darknet.git
cd darknet/
```

* edit Makefile
```
1 GPU=1
2 CUDNN=1
3 CUDNN_HALF=0
4 OPENCV=1
...
45 ARCH= -gencode arch=compute_62,code=[sm_62,compute_62]
```

* build sources
```
make
```

# Download configs and pretrained weights

|         Model         	| Size 	| BFLOPS 	| mAP@0.5 	|   Config   	|   Weights  	|
|:---------------------:	|:----:	|:------:	|:-------:	|:----------:	|:----------:	|
|         YOLOv4        	|  512 	|  91.1  	|  64.9%  	|    [link](https://raw.githubusercontent.com/AlexeyAB/darknet/master/cfg/yolov4.cfg)    	|    [gdrive](https://drive.google.com/file/d/1cewMfusmPjYWbrnuJRuKhPMwRe_b9PaT/view)    	|
|         YOLOv4        	|  416 	|  60.1  	|  62.8%  	| (as above) 	| (as above) 	|
|         YOLOv4        	|  320 	|  35.5  	|  60.0%  	| (as above) 	| (as above) 	|
| EfficientNetB0-Yolov3 	|  416 	|   3.7  	|  45.5%  	|    [link](https://raw.githubusercontent.com/AlexeyAB/darknet/master/cfg/enet-coco.cfg)    	|    [gdrive](https://drive.google.com/file/d/1FlHeQjWEQVJt0ay1PVsiuuMzmtNyv36m/view)    	|
|      YOLOv3-tyny      	|  416 	|   3.5  	|  33.1%  	|    [link](https://raw.githubusercontent.com/AlexeyAB/darknet/master/cfg/yolov3-tiny-prn.cfg)    	|    [gdrive](https://drive.google.com/file/d/18yYZWyKbo4XSDVyztmsEcF9B_6bxrhUY/view)    	|

To download configs, just use `wget "<file_url>"`. For downloading from Google Drive in the console you can install `sudo pip install gdown` and download using: `gdown https://drive.google.com/uc?id=<gdrive_file_id>`.

# Modify config files
We set `batch=1` and `subdivisions=1` in all files. The `width`x`height` resolution was changed depending on the test, e.g. `512`x`512`, `416`x`416` or `320`x`320`. You can use any resolution that is a square and a multiple of 32.

```
[net]
batch=1
subdivisions=1
# Training
#batch=64
#subdivisions=8
width=416
height=416
channels=3
```

# Benchmark
* the following command was used in the benchmark:

```bash
./darknet detector demo cfg/coco.data <config_file> <weights_file> -benchmark -c <camera_id>
```

* YOLO detects the graphics card:

```
 CUDA-version: 10000 (10000), cuDNN: 7.6.3, GPU count: 1  
 OpenCV version: 4.1.1
 Demo
 0 : compute_capability = 620, cudnn_half = 0, GPU: NVIDIA Tegra X2
 ```

* and video stream

 ```
 Video stream: 1920 x 1080
 ```

# Results

|         Model         	| Size 	| BFLOPS 	| mAP@0.5 	|   AVG_FPS   	|
|:---------------------:	|:----:	|:------:	|:-------:	|:----------:	|
|         YOLOv4        	|  512 	|  91.1  	|  64.9%  	|    4.3       	|
|         YOLOv4        	|  416 	|  60.1  	|  62.8%  	|    5.4     	|
|         YOLOv4        	|  320 	|  35.5  	|  60.0%  	| 	 7.6	 	|
| EfficientNetB0-Yolov3 	|  416 	|   3.7  	|  45.5%  	|    0.7       	|
|      YOLOv3-tyny      	|  416 	|   3.5  	|  33.1%  	|   57.0    	|

Not cool. The results are not satisfactory. Maybe it would work much better for fewer classes? Maybe changing the parameters would lead to optimization? Why is YOLO with EfficientNet backbone so weak? We currently have no idea.