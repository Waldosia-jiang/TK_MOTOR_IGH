# EtherCAT-IGH主站

## 环境安装：

操作系统：Ubuntu20.04

IGH安装，参考：https://zhuanlan.zhihu.com/p/7019519900

安装各种依赖：

```bash
sudo apt-get install build-essential bc curl ca-certificates fakeroot gnupg2 libssl-dev lsb-release libelf-dev bison flex cmake libeigen3-dev dwarves zstd libncurses-dev
```

### 1、下载IGH

官网地址：https://gitlab.com/etherlab.org/ethercat/-/tree/stable-1.5?ref_type=heads

![alt text](image/image6.png)

点击之后开始下载

### 2、解压

下载完成后，找到刚刚下载的文件目录，解压到自己能找到的目录，比如桌面

然后进入解压完成的目录鼠标右键，在终端中打开

![alt text](image/image7.png)

### 3、生成配置文件

输入`./bootstrap `生成`.configure`

```bash
./bootstrap
```

![alt text](image/image8.png)

### 4、配置

```bash
./configure --enable-8139too=no  --disable-eoe
```

![alt text](image/image9.png)

等待配置完成

![alt text](image/image10.png)

### 5、编译modules

```bash
sudo make all modules
```

![alt text](image/image11.png)

### 6、安装modules

```bash
sudo make modules_install install
```

![alt text](image/image12.png)

### 7、生成模块依赖信息

```bash
sudo depmod
```

![alt text](image/image13.png)

### 8、链接init脚本

```bash
sudo ln -s /usr/local/etc/init.d/ethercat /etc/init.d/ethercat
```
或者

```bash
sudo ln -s ${PREFIX}/etc/init.d/ethercat /etc/init.d/ethercat
```

### 9、自定义 sysconfig 文件

```bash
sudo mkdir /etc/sysconfig

sudo cp /usr/local/etc/sysconfig/ethercat /etc/sysconfig/ethercat
```
或者

```bash
sudo mkdir /etc/sysconfig

sudo cp ${PREFIX}/etc/sysconfig/ethercat /etc/sysconfig/ethercat
```

### 10、配置ethercat device

```bash
sudo gedit /etc/sysconfig/ethercat
sudo gedit /usr/local/etc/sysconfig/ethercat
```
运行后可能会等待一段时间

![alt text](image/image14.png)

新建终端并输入 `ip a` 查看MAC地址
 
![alt text](image/image15.png)

复制粘贴到刚刚打开的ethercat文件中，并需要修改 `DEVICE_MODULES`

![alt text](image/image16.png)

```bash
# 原本：
MASTER0_DEVICE=""
DEVICE_MODULES=""

# 修改后：
MASTER0_DEVICE="00:0c:29:11:0a:90"
DEVICE_MODULES="generic"
```

修改完后保存退出

### 11、启动igh

```bash
sudo /etc/init.d/ethercat start
```

![alt text](image/image17.png)

### 12、设置开机自启（可选）

如需 EtherCAT 主站在开机后自动启动：

```bash
sudo update-rc.d ethercat defaults
```

取消开机自启：
```bash
sudo update-rc.d ethercat remove
```

**注意**：ethercat_motor_node 启动时会检测从站数量，预期 3 个从站在线。若不符合预期，请检查上述步骤 1) 是否已执行 `sudo /etc/init.d/ethercat start`；2) 是否已设置开机自启；3) 网线连接和从站供电。

## 编译项目：

在终端进入项目目录后，执行以下命令：

```bash
# 创建build目录
mkdir build
# 进入build目录
cd build
# 配置
cmake ..
# 编译
make
```
![alt text](image/image1.png)

按下回车后开始编译

![alt text](image/image2.png)

等待编译完成

## 硬件连接

连接从站和电脑网口

![alt text](image/image4.png)

然后连接电机和从站电源，电源可以使用Type-C接口提供，也可以由GH1.24-4PIN电源接口提供。

![alt text](image/image5.png)

## 启动IGH

再运行IGH主站程序前，需要启动IGH。

输入以下命令：

```bash
sudo /etc/init.d/ethercat start
```
输入sudo密码后回车，等待启动完成

![alt text](image/image3.png)

然后输入以下指令运行IGH主站程序

```bash
sudo ./master_stack_test
```

## 二次开发

控制电机操作需要在`do_ec_task`函数中完成

```c
static void do_ec_task()
{
    ethercat_data_exchange()
    
    ecrt_master_receive(master);
    ecrt_domain_process(domain);

    check_domain_state();
    check_master_state();

    // 控制电机，比如
    set_motor_speed(&tx_msg[0], 1, 1, 10, 40, 2);

    // 如果只有一个从站，可以简写成
    // set_motor_speed(tx_msg, 1, 1, 10, 40, 2);
 

    RV_can_data_repack_all(rx_msg, comm_ack, rv_motor_msg, SlaveNum, 1);

    ecrt_domain_queue(domain);
    ecrt_master_send(master);
}
```

### 控制多个电机

一块EtherCAT转接板最多可以控制6个电机，通道（passage）1~3需要接在CAN1上，通道4~6需要接到CAN2上面

示例代码：

```c
static void do_ec_task()
{
    ethercat_data_exchange()

    ecrt_master_receive(master);
    ecrt_domain_process(domain);

    check_domain_state();
    check_master_state();

    // 控制多个电机
    set_motor_speed(&tx_msg[0], 1, 6,  5, 40, 2);
    set_motor_speed(&tx_msg[0], 2, 5, 10, 40, 2);
    set_motor_speed(&tx_msg[0], 3, 4, 15, 40, 2);
    set_motor_speed(&tx_msg[0], 4, 3, 20, 40, 2);
    set_motor_speed(&tx_msg[0], 5, 2, 25, 40, 2);
    set_motor_speed(&tx_msg[0], 6, 1, 30, 40, 2);

    // 只有一个从站可以简写成
    // set_motor_speed(tx_msg, 1, 6,  5, 40, 2);
    // set_motor_speed(tx_msg, 2, 5, 10, 40, 2);
    // ...

    RV_can_data_repack_all(rx_msg, comm_ack, rv_motor_msg, SlaveNum, 1);

    ecrt_domain_queue(domain);
    ecrt_master_send(master);
}
```

ID为6、5、4的电机接在CAN1上面，ID为3、2、1的电机接到CAN2上面。

### 连接多个从站

修改宏定义`SlaveNum`即可

```c
// 修改前
#define SlaveNum 1
// 修改后
#define SlaveNum 3
```

```c
static void do_ec_task()
{
    ethercat_data_exchange()
    
    ecrt_master_receive(master);
    ecrt_domain_process(domain);

    check_domain_state();
    check_master_state();

    // 第一个从站
    set_motor_speed(&tx_msg[0], 1, 1, 5, 40, 2);
    // 第二个从站
    set_motor_speed(&tx_msg[1], 1, 1, 5, 40, 2);
    // 第三个从站
    set_motor_speed(&tx_msg[2], 1, 1, 5, 40, 2);
    
    RV_can_data_repack_all(rx_msg, comm_ack, rv_motor_msg, SlaveNum, 1);

    ecrt_domain_queue(domain);
    ecrt_master_send(master);
}
```
