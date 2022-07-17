# PID_yaml
## yaml
利用yaml文件存储参数，可以便于后续进行参数的调整。
* yaml文本规范  
注意yaml文件的书写对空格非常敏感，具体细节见[yaml书写规范](https://blog.csdn.net/liukuan73/article/details/78031693)  
* 利用 yaml-cpp 读取  
Node 是 yaml-cpp 中的核心概念，它用于存储解析后的 yaml 信息,可以利用以下读取参数  
`YAML::Node config = YAML::LoadFile("../config.yaml");`  
`int a=config["age"].as<int>();`  
这样可以读取config.yaml中的age的参数。  
**注意：在头文件引用yaml.h后需要在CMakeList.txt文件中将可执行文件链接上libyaml-cpp.so**
##  PID
pid的参数中有ki(积分)，kd(微分)，kp(误差)，调整顺序如下：  
* 调节Kp使发生1/4幅度的震荡，ki,kd均置零。  
* 使得Kp变为原来的一半。  
* 逐步增大Ki使得重新出现震荡。  
* 调节Kd使系统稳定。  
可以设定pid的输出的极值使系统不至于失控。
