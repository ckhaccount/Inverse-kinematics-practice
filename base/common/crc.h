/**
 ******************************************************************************
 * @file    crc.cpp/h
 * @brief   CRC calculation. CRC计算
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef CRC_H
#define CRC_H

#include <stdint.h>

// CRC8 Calculate. CRC8计算
uint8_t CRC8_Calc(uint8_t* data, uint32_t len);
// CRC8 verify. CRC8校验函数
uint8_t CRC8_Verify(uint8_t* data, uint32_t len);
// Append CRC8 to the end of data. 添加CRC8到数据的结尾
void CRC8_Append(uint8_t* data, uint32_t len);

// CRC16 Calculate. CRC16计算
uint16_t CRC16_Calc(uint8_t* data, uint32_t len);
// CRC16 verify. CRC16校验函数
uint8_t CRC16_Verify(uint8_t* data, uint32_t len);
// Append CRC16 to the end of data. 添加CRC16到数据的结尾
void CRC16_Append(uint8_t* data, uint32_t len);

#endif  // CRC_H