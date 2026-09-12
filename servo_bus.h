#pragma once

#include <Arduino.h>

void initServoBus();
// 返回 true 表示该行是舵机命令；false 交给综合程序其他命令处理。
bool handleServoCommand(char *line);
