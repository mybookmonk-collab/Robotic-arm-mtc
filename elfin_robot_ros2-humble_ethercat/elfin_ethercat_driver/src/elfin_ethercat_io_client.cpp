/*
Created on Mon Sep 17 11:15 2018

@author: Cong Liu

 Software License Agreement (BSD License)

 Copyright (c) 2018, Han's Robot Co., Ltd.
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
  * Neither the name of the copyright holders nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/
// author: Cong Liu

#include <elfin_ethercat_driver/elfin_ethercat_io_client.h>

namespace elfin_ethercat_driver {

ElfinEtherCATIOClient::ElfinEtherCATIOClient(EtherCatManager *manager, int slave_no, const rclcpp::Node::SharedPtr& nh, std::string io_port_name):
    manager_(manager), slave_no_(slave_no), io_nh_(nh)
{
    // init pdo_input and output
    std::string name_pdo_input[5]={"Digital_Input", "Analog_Input_channel1", "Analog_Input_channel2",
                                  "Smart_Camera_X", "Smart_Camera_Y"};
    uint8_t channel_pdo_input[5]={0, 4, 8, 12, 16};
    pdo_input.clear();
    ElfinPDOunit unit_tmp;
    for(unsigned i=0; i<5; ++i)
    {
        unit_tmp.name=name_pdo_input[i];
        unit_tmp.channel=channel_pdo_input[i];
        pdo_input.push_back(unit_tmp);
    }

    std::string name_pdo_output[1]={"Digital_Output"};
    uint8_t channel_pdo_output[1]={0};
    pdo_output.clear();
    for(unsigned i=0; i<1; ++i)
    {
        unit_tmp.name=name_pdo_output[i];
        unit_tmp.channel=channel_pdo_output[i];
        pdo_output.push_back(unit_tmp);
    }


    // Initialize services
    read_claw_=io_nh_->create_service<elfin_robot_msgs::srv::GripperReadData>("read_claw", std::bind(&ElfinEtherCATIOClient::readCLAW_cb, this,std::placeholders::_1,std::placeholders::_2));
    write_claw_=io_nh_->create_service<elfin_robot_msgs::srv::GripperWriteData>("write_claw", std::bind(&ElfinEtherCATIOClient::writeCLAW_cb, this,std::placeholders::_1,std::placeholders::_2));
    read_sdo_=io_nh_->create_service<elfin_robot_msgs::srv::ElfinIODRead>("read_di", std::bind(&ElfinEtherCATIOClient::readSDO_cb, this,std::placeholders::_1,std::placeholders::_2));
    read_do_=io_nh_->create_service<elfin_robot_msgs::srv::ElfinIODRead>("read_do", std::bind(&ElfinEtherCATIOClient::readDO_cb, this,std::placeholders::_1,std::placeholders::_2));
    write_sdo_=io_nh_->create_service<elfin_robot_msgs::srv::ElfinIODWrite>("write_do", std::bind(&ElfinEtherCATIOClient::writeSDO_cb, this,std::placeholders::_1,std::placeholders::_2));
    get_txsdo_server_=io_nh_->create_service<std_srvs::srv::SetBool>("get_io_txpdo", std::bind(&ElfinEtherCATIOClient::getTxSDO_cb, this,std::placeholders::_1,std::placeholders::_2));
    get_rxsdo_server_=io_nh_->create_service<std_srvs::srv::SetBool>("get_io_rxpdo", std::bind(&ElfinEtherCATIOClient::getRxSDO_cb, this,std::placeholders::_1,std::placeholders::_2));

}

ElfinEtherCATIOClient::~ElfinEtherCATIOClient()
{

}

int16_t ElfinEtherCATIOClient::readInput_unit(int n)
{

    int16_t map;
    map = (manager_->readSDO<int16_t>(4, 0x6001, 0x01)); // read the end DI
    return map;
}
// ros2 service call /write_claw elfin_robot_msgs/srv/GripperWriteData "{'slave_id':1,'function_code':6,'value':5000,'speed':80,'force':50}"
bool ElfinEtherCATIOClient::write_claw(int32_t slave_id, int32_t functionCode, int32_t address, int32_t value) {
    // 1. 设置从站ID
    manager_->writeSDO<int32_t>(4, 0x7003, 0x01, slave_id);
    usleep(10000); // 10ms 延时

    // 2. 设置功能码（6=写入单个寄存器）
    manager_->writeSDO<int32_t>(4, 0x7004, 0x01, functionCode);
    usleep(10000);

    // 3. 设置寄存器地址
    manager_->writeSDO<int32_t>(4, 0x7005, 0x01, address);
    usleep(10000);

    // 4. 设置数据长度（1=16位值）
    manager_->writeSDO<int32_t>(4, 0x7006, 0x01, 1);
    usleep(10000);

    // 5. 写入实际值（16位）
    manager_->writeSDO<int16_t>(4, 0x7007, 0x01, static_cast<int16_t>(value));
    usleep(10000);

    // 6. 触发操作（1→0）
    manager_->writeSDO<int32_t>(4, 0x7002, 0x01, 1);
    manager_->writeSDO<int32_t>(4, 0x7002, 0x01, 0);
    usleep(10000); // 40ms 等待执行

    return true;
}

int32_t ElfinEtherCATIOClient::read_claw(int32_t slave_id, int32_t functionCode)
{
    // 读取高16位 (0x0609)
    manager_->writeSDO<int32_t>(4,0x7003,0x01,slave_id); // 设置从站ID
    usleep(10000);
    manager_->writeSDO<int32_t>(4,0x7004,0x01,functionCode); // 功能码3:读取寄存器
    usleep(10000);
    manager_->writeSDO<int32_t>(4,0x7005,0x01,0x0609); // 高位寄存器地址
    usleep(10000);
    manager_->writeSDO<int32_t>(4,0x7006,0x01,1); // 数据长度=1(16位)
    usleep(10000);
    manager_->writeSDO<int32_t>(4,0x7002,0x01,1); // 触发读取
    manager_->writeSDO<int32_t>(4,0x7002,0x01,0);
    usleep(20000);
    int32_t high_word = manager_->readSDO<int32_t>(4, 0x6009,0x01);
    usleep(50000);

    // 读取低16位 (0x060A)
    manager_->writeSDO<int32_t>(4,0x7005,0x01,0x060A); // 低位寄存器地址
    usleep(10000);
    manager_->writeSDO<int32_t>(4,0x7002,0x01,1); // 触发读取
    manager_->writeSDO<int32_t>(4,0x7002,0x01,0);
    usleep(40000);
    int32_t low_word = manager_->readSDO<int32_t>(4, 0x6009,0x01);
    usleep(50000);

    // 组合32位位置值 (高16位 << 16 | 低16位)
    int32_t position_value = (high_word << 16) | (low_word & 0xFFFF);
//    std::cout << "Gripper position: high=0x" << std::hex << high_word
//              << ", low=0x" << low_word
//              << ", combined=0x" << position_value << std::dec << std::endl;
    return position_value;
}



int32_t ElfinEtherCATIOClient::readOutput_unit(int n)
{
    int32_t map;
    map = (manager_->readSDO<int32_t>(4, 0x7001, 0x01)) << 12; 
    return map;
}

void ElfinEtherCATIOClient::writeOutput_unit(int n, int32_t val)
{

    manager_->writeSDO<int32_t>(4,0x7001,0x01, val >> 12);
}


// 20201116: read the end SDO
int32_t ElfinEtherCATIOClient::readSDO_unit(int n)
{
    if(n<0 || n>=pdo_input.size())
        return 0x0000;
    // 20201116: build the connection.
    manager_->writeSDO<int>(3,0x3100,0x0,1); // Modbus DO command
    usleep(50000);
    manager_->writeSDO<int32_t>(3,0x3101,0x0,0x010040); // 64 connect to Modbus
    usleep(50000);
    manager_->writeSDO<int32_t>(3,0x3102,0x0,0x010001); // Modbus Addr & count
    usleep(50000);
    // 0x2126, L_4 is end DI, H_4 is button DI.
    int32_t map;
    map = (manager_->readSDO<int32_t>(3, 0x2126, 0x0)) << 16; // read the end DI
    manager_->writeSDO<int>(3,0x3100,0x0,0); // Modbus DO command 0
    usleep(50000);
    return map;
}

// 20201130: read the end DO
int32_t ElfinEtherCATIOClient::readDO_unit(int n)
{
    if(n<0 || n>=pdo_input.size())
        return 0x0000;
    // 20201130: build the connection.
    manager_->writeSDO<int>(3,0x3100,0x0,1); // Modbus DO command
    usleep(50000);
    manager_->writeSDO<int32_t>(3,0x3101,0x0,0x010040); // 64 connect to Modbus
    usleep(50000);
    manager_->writeSDO<int32_t>(3,0x3102,0x0,0x010001); // Modbus Addr & count
    usleep(50000);
    // 0x310C, DO.
    int32_t map;
    map = (manager_->readSDO<int32_t>(4, 0x7001, 0x0)) << 12; // read the end DO
    manager_->writeSDO<int>(3,0x3100,0x0,0); // Modbus DO command 0
    usleep(50000);
    return map;
}

// 20201117: write the end SDO LED and DO
int32_t ElfinEtherCATIOClient::writeSDO_unit(int32_t val)
{
    // 20201119: high the LED and DO of the end
    manager_->writeSDO<int>(3,0x3100,0x0,1); // Modbus DO command
    usleep(50000);
    manager_->writeSDO<int32_t>(3,0x3101,0x0,0x010006); // Modbus SlaveID & Function
    usleep(50000);
    manager_->writeSDO<int32_t>(3,0x3102,0x0,0x010001); // Modbus Addr & count
    usleep(50000);
    manager_->writeSDO<int32_t>(3,0x310C,0x0, val >> 12); // Write the LED and DO
    usleep(50000);
    manager_->writeSDO<int>(3,0x3100,0x0,0); // Modbus DO command 0
    usleep(50000);

    return 0;
}

// 20201120: add the getTxSDO and getRxSDO for confirming the same as old IO
std::string ElfinEtherCATIOClient::getTxSDO()
{
    int length=20;
    uint8_t map[length];
    char temp[8];
    std::string result="slave";
    result.reserve(160);
    result.append("4_txpdo:\n");
    for (unsigned i = 0; i < length; ++i)
    {
        map[i] = 0x00;
        sprintf(temp,"0x%.2x",(uint8_t)map[i]);
        result.append(temp, 4);
        result.append(":");
    }
    result.append("\n");
    return result;
}

std::string ElfinEtherCATIOClient::getRxSDO()
{
    int length=4;
    uint8_t map[length];
    char temp[8];
    std::string result="slave";
    result.reserve(160);
    result.append("4_rxpdo:\n");
    for (unsigned i = 0; i < length; ++i)
    {
        map[i] = 0x00;
        sprintf(temp,"0x%.2x",(uint8_t)map[i]);
        result.append(temp, 4);
        result.append(":");
    }
    result.append("\n");
    return result;

}

bool ElfinEtherCATIOClient::readCLAW_cb(const std::shared_ptr<elfin_robot_msgs::srv::GripperReadData::Request> req, const std::shared_ptr<elfin_robot_msgs::srv::GripperReadData::Response> resp)
{
    // resp->position=read_claw(req->slave_id,req->function_code);
    if(claw_value == -1){
      claw_value = read_claw(req->slave_id,req->function_code);
    }
    resp->position=claw_value;
    return true;
}
//ros2 service call /write_claw elfin_robot_msgs/srv/GripperWriteData "{'slave_id',1,'function_code':6,'addrss':0,'value':5000,'speed':80,'force':50}"
bool ElfinEtherCATIOClient::writeCLAW_cb(const std::shared_ptr<elfin_robot_msgs::srv::GripperWriteData::Request> req,
                                        const std::shared_ptr<elfin_robot_msgs::srv::GripperWriteData::Response> resp) {

    std::thread([this, req, resp]() {
      // 控制逻辑（按文档2顺序执行）
      // 1. 写入位置高位
      write_claw(req->slave_id, 6, POSITION_HIGH_8, (req->value >> 16) & 0xFFFF);

      // 2. 写入位置低位
      write_claw(req->slave_id, 6, POSITION_LOW_8, req->value & 0xFFFF);

      // 3. 写入速度
      write_claw(req->slave_id, 6, SPEED, req->speed);

      // 4. 写入力度
      write_claw(req->slave_id, 6, FORCE, req->force);

      // 5. 触发运动
      write_claw(req->slave_id, 6, MOTION_TRIGGER, 1);
      int32_t expect_value = static_cast<int32_t>(std::round(req->value * 0.12288889));
      int32_t value = (expect_value - this->claw_value) / 10;
      for (int i = 0; i < 10; i++) {
        // int32_t value = read_claw(1,3);
        // if(value > 1100){
        //   break;
        // }
        usleep(100000);
        if (((this->claw_value-expect_value) > -10 && (this->claw_value-expect_value) < 10) || i == 9) {
          this->claw_value = expect_value;
        }else{
          this->claw_value += value;
        }
        std::cout << "i=" << i
          << " value=" << value
          << " expect_value=" << expect_value
          << " claw_value=" << this->claw_value
          << std::endl;
      }

    }).detach();

    resp->success = true;
    return true;
}

// 20201116: read the end SDO
bool ElfinEtherCATIOClient::readSDO_cb(const std::shared_ptr<elfin_robot_msgs::srv::ElfinIODRead::Request> req, const std::shared_ptr<elfin_robot_msgs::srv::ElfinIODRead::Response> resp)
{
    resp->digital_input=readInput_unit(elfin_io_txpdo::DIGITAL_INPUT);
    return true;
}

// 20201130: read the end DO
bool ElfinEtherCATIOClient::readDO_cb(const std::shared_ptr<elfin_robot_msgs::srv::ElfinIODRead::Request> req, const std::shared_ptr<elfin_robot_msgs::srv::ElfinIODRead::Response> resp)
{
    resp->digital_input=readOutput_unit(elfin_io_txpdo::DIGITAL_INPUT); // 20201130: digital_input for convenience
    return true;
}

// 20201117: write the end SDO
bool ElfinEtherCATIOClient::writeSDO_cb(const std::shared_ptr<elfin_robot_msgs::srv::ElfinIODWrite::Request> req, const std::shared_ptr<elfin_robot_msgs::srv::ElfinIODWrite::Response> resp)
{
    writeOutput_unit(elfin_io_rxpdo::DIGITAL_OUTPUT,req->digital_output);
    resp->success=true;
    return true;
}

// 20201120: add the getTxSDO and getRxSDO for confirming the same as old IO
bool ElfinEtherCATIOClient::getRxSDO_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp)
{
    if(!req->data)
    {
        resp->success=false;
        resp->message="require's data is false";
        return true;
    }

    resp->success=true;
    resp->message=getRxSDO();
    return true;
}

bool ElfinEtherCATIOClient::getTxSDO_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> resp)
{
    if(!req->data)
    {
        resp->success=false;
        resp->message="require's data is false";
        return true;
    }

    resp->success=true;
    resp->message=getTxSDO();
    return true;
}

}


