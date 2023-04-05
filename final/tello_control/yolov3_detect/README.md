# yolo_v3的**CPU**前向代码

本代码适用于python2和python3，CPU上可以运行。

## 1.安装

pytorch安装参考官网： https://pytorch.org/get-started/locally/ 

- 如果你在conda环境下安装，新建一个环境之后，执行：

```
conda install pytorch torchvision cpuonly -c pytorch

conda install -yc anaconda future numpy opencv matplotlib tqdm pillow
conda install -yc conda-forge scikit-image tensorboard pycocotools
conda install -yc spyder-ide spyder-line-profiler

git clone https://github.com/zoeyuchao/yolov3_detect.git
```

- 如果不在conda下，直接是系统版本的python，那么参考：
  - python2

```
pip install torch==1.3.0+cpu torchvision==0.4.1+cpu -f https://download.pytorch.org/whl/torch_stable.html

git clone https://github.com/zoeyuchao/yolov3_detect.git

pip install -U -r requirements.txt
```
  -   python3

```
pip3 install torch==1.3.0+cpu torchvision==0.4.1+cpu -f https://download.pytorch.org/whl/torch_stable.html

git clone https://github.com/zoeyuchao/yolov3_detect.git

pip3 install -U -r requirements.txt
```

## 2.测试

安装完之后执行：

```
cd ~/yolov3_detect
python detect.py
```

## 3.Tips

1.在data文件夹中添加xxx.data和xxx.names文件(示例为球类检测，ball.data和ball.name已经添加到data文件夹中,更新了fire.data和ball.name)

2.在weights文件夹中添加训练好的best.pt

  - 提供的ball.pt：

    - 百度网盘链接：链接：https://pan.baidu.com/s/1nrLffPnwhslNZHZPpmUAoA  提取码：lf6y 

    - 清华网盘链接：https://cloud.tsinghua.edu.cn/f/f6748453cced47608edb/?dl=1
  
  - 提供的fire_new.pt:
  
    - 百度网盘链接：https://pan.baidu.com/s/1TNjLhGkkbnlyqa_A7wiI3g 提取码：715s 
    
    - 清华网盘链接：https://cloud.tsinghua.edu.cn/f/b2b5fb05152a4059b3f3/?dl=1

3.在cfg文件夹中添加训练网络时修改过的xxx.cfg文件（示例球类检测的yolov3.cfg已经添加至cfg文件夹中,更新fire）

4.可能遇到no model named xxx，解决方法 pip install xxx

5.detect.py程序中

    if __name__ == '__main__':
    img = cv2.imread('data/samples/three_balls_2.jpg')#可修改为自己的图片
    with torch.no_grad():
     	result_obj=detect(img)
     	print(result_obj)
     	print(result_obj.shape)

**result_obj**

返回值为（n,7）维tensor矩阵，其中n为所识别图片中的目标物体数，如出现一个足球，一个篮球，则n为2

每行包含7列，前4列为xyxy,即矩形框角点的左边，第5列，第6类分别为不同指标下该类别的置信度，可以第6列为准

第7列为识别的物体类别，0,1,2,3分别对应于ball.name中的类别顺序。
