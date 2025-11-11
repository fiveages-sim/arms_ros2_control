#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>

#include "rokae/robot.h"
#include <rclcpp/rclcpp.hpp>

using namespace rokae;

namespace DhGripParams {
	std::vector<uint8_t> send_data = { 0x01,0x06,0x01,0x00,0x00,0xA5,0x48,0x4D };//初始化的裸传数据
	std::vector<uint8_t> rev_data = { 0,0,0,0,0,0,0,0 };//接收裸传的返回数据
	std::vector<int>  init_set = { 0x00 };//初始化数据
	std::vector<int>  init_get = { 100 };//获取初始化状态  0-2
	std::vector<int>  grip_status_get = { 0 };//获取夹持状态  0-3
	std::vector<int>  trq_set = { 100 };//设置力 20-100，可以改
	std::vector<int>  vel_set = { 2 };//设置速度 1-100，可以改
	std::vector<int>  pos_set = { 100 };//设置位置 0-1000，可以改
	std::vector<int>  trq_get = { 0 };//存获取到的力
	std::vector<int>  vel_get = { 0 };//存获取到的速度
	std::vector<int>  pos_get = { 0 };//存获取到的位置
	std::vector<int>  pos_now_get = { 0 };//存获取到的实时位置
	int slave_add = 9;
	int init_func_code = 0x10;
	int init_reg_addr = 0x03E8;
}

/*
* @brief DH电爪初始化
*/
void DHGripInit(xMateErProRobot &robot, rclcpp::Node::SharedPtr node) {
	error_code ec;
	//裸传的初始化
	//std::vector<uint8_t> send_data = { 0x01,0x06,0x01,0x00,0x00,0xA5,0x48,0x4D };//初始化的裸传数据
	//robot.XPRS485SendData(send_data.size(), rev_data.size(), send_data, DhGripParams::rev_data,ec);
	int num_write = 1;
	int init_func_code = 0x10;
	int init_reg_addr = 0x03E8;
	std::vector<int>  init_set = { 0x00 };//初始化数据
	robot.XPRWModbusRTUReg(DhGripParams::slave_add, init_func_code, init_reg_addr, "int16", num_write, init_set, false, ec);
    RCLCPP_INFO(node->get_logger(), "查询DH初始化执行结果 1: %d, %d", ec.value(), init_set[0]);
	std::vector<int>  init_set_up = { 0x01 };//初始化数据
	robot.XPRWModbusRTUReg(DhGripParams::slave_add, init_func_code, init_reg_addr, "int16", num_write, init_set_up, false, ec);
	RCLCPP_INFO(node->get_logger(), "查询DH初始化执行结果 2: %d , %d", ec.value(), init_set_up[0]);
} 

/*
* @brief DH电爪运动至
*/
void DHGripMove(xMateErProRobot &robot, int& trq_set, int& vel_set, int& pos_set, rclcpp::Node::SharedPtr node) {
	error_code ec;
	//设置力
	std::vector<int>  trq_set_vec;//设置力 20-100，可以改
	uint16_t trigger = 0x09;
	uint16_t position = pos_set << 8;
	uint16_t speed_force = trq_set << 8 | vel_set;
	trq_set_vec.push_back(trigger);
	trq_set_vec.push_back(position);
	trq_set_vec.push_back(speed_force);

	int num_write = 3;
	int init_func_code = 0x10;
	int init_reg_addr = 0x03E8;

	robot.XPRWModbusRTUReg(DhGripParams::slave_add, init_func_code, init_reg_addr, "int16", num_write, trq_set_vec, false, ec);
    RCLCPP_INFO(node->get_logger(), "设置位置执行结果: %d", ec.value());
}


/*
* @brief DH电爪获取初始化状态
*/
bool DHGripGetInitStatus(xMateErProRobot& robot, uint8_t & init_get, rclcpp::Node::SharedPtr node) {
	error_code ec;
	int num_read = 1;
	int init_func_code = 0x04;
	int init_reg_addr = 0x07D0;
	std::vector<int>  init_get_vec = { -1 };//获取初始化状态  0-2
	robot.XPRWModbusRTUReg(DhGripParams::slave_add, init_func_code, init_reg_addr, "int16", num_read, init_get_vec, false, ec);
	init_get = init_get_vec[0];
	int bit4 = (init_get >> 4) & 1;
    int bit5 = (init_get >> 5) & 1;

    // 合并为整数：bit5 作为高位，bit4 作为低位
    int result = (bit5 << 1) | bit4;
    RCLCPP_INFO(node->get_logger(), "获取初始化状态执行结果:  %d", ec.value());
	return result == 3;
}

/*
* @brief DH电爪获取夹爪夹持状态
*/
void DHGripGetStatus(xMateErProRobot& robot, std::vector<int>& grip_status_get_vec, rclcpp::Node::SharedPtr node) {
	error_code ec;
	int num_read = 3;
	int init_func_code = 0x04;
	int init_reg_addr = 0x07D0;
	grip_status_get_vec = { -1, -1, -1 };//获取夹持状态  0-3
	robot.XPRWModbusRTUReg(DhGripParams::slave_add, init_func_code, init_reg_addr, "int16", num_read, grip_status_get_vec, false, ec);
	uint8_t grip_status_get = grip_status_get_vec[0];
	uint8_t grip_position_get = grip_status_get_vec[1] >> 8;
	uint8_t grip_error_status = grip_status_get_vec[1] & 0xFF;
	uint8_t grip_force_get = grip_status_get_vec[2] >> 8;
	uint8_t grip_speed_get =grip_status_get_vec[2] & 0xFF;
    RCLCPP_INFO(node->get_logger(), "获取夹爪状态执行结果:  %d, 位置: %d, 错误码: %d ,力: %d ,速度: %d", grip_status_get, grip_position_get,grip_error_status,grip_force_get, grip_speed_get);
}


