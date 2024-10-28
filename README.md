***
# 说明
此工程用于实现ROS1下用TensorRT部署pointpillars模型实现对pointcloud2点云的3D目标检测，源码参考自https://github.com/NVIDIA-AI-IOT/ros2_tao_pointpillars?tab=readme-ov-file<br>
工程中model下提供的模型文件来自https://catalog.ngc.nvidia.com/orgs/nvidia/teams/tao/models/pointpillarnet/files<br>
建议基于自己的数据集使用 https://developer.nvidia.com/tao-toolkit 重新训练生成.onnx模型文件
***
# 环境准备
* CUDA
* TensorRT 8 或 TensorRT 10
* TensorRT OSS 对应你的TensorRT版本
***
# Step1
修改CMakeLists文件以适配你的CUDA，TensorRT系统环境
***
# Step2
```bash
catkin_make
source devel/setup.bash
```
***
# Step3(可选)
```bash
cd model
trtexec --onnx=pointpillars_deployable.onnx --saveEngine=trt.engine --fp16
```
生成TensorRT可加载的.engine文件
***
# Step4
发布你的pointcloud2点云话题，修改`cfg`文件夹下`params.yaml`文件
|参数名称|功能描述|备注|
|---|---|---|
|nms_iou_thresh|非极大值抑制(nms)算法iou阈值| - |
|pre_nms_top_n|非极大值抑制(nms)之前要考虑的最大候选框数量| - |
|class_names|pointpillars模型检测类别| - |
|lidar_input_topic|接收lidar点云话题| - |
|model_path|.onnx模型文件路径|如果已执行`Step3`则此处可以不填，此时需要填入`engine_path`|
|engine_path|.engine引擎文件路径|如果未执行`Step3`则此处可以不填，此时需要填入`model_path`|
|data_type|模型推理精度|可选`fp16`或`fp32`|
|intensity_scale|点云反射强度归一化|如果模型是在点强度在[0.0-1.0]范围内的数据上训练的，并且推理时的输入数据的强度在[1-255]范围内，则此参数应设置为255.0，以便输入数据与训练数据匹配。|
***
# Step5
```bash
roslaunch lidar_3d_detector run.launch
```
***
