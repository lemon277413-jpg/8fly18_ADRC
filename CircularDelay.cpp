#include "Arduino.h"
#include "CircularDelay.h"

// 构造函数：初始化
CircularDelayLine::CircularDelayLine() {
    for (int i = 0; i < DELAY_BUFFER_SIZE; i++) {
        delayBuffer[i] = 0.0f;
    }
    front = 0;
    rear = DELAY_BUFFER_SIZE - 1;
    bufLen = 0;
}

// 更新数据入队
void CircularDelayLine::update(float newData) {
    rear = (rear + 1) % DELAY_BUFFER_SIZE;
    delayBuffer[rear] = newData;
    
    // 限制 bufLen 最大为 6，队列满了移动 front
    if (bufLen < DELAY_BUFFER_SIZE) {
        bufLen++;
    } else {
        front = (front + 1) % DELAY_BUFFER_SIZE;
    }
}

// 获取延时数据
float CircularDelayLine::getDelayed(float delay_cycles) {
    // 1. 限制延时范围 0~5
    delay_cycles = constrain(delay_cycles, 0.0f, 5.0f);

    // 2. 拆分整数/小数
    int integer = (int)delay_cycles;
    float decimal = delay_cycles - integer;

    // 3. 检查数据是否足够（保留你的思路）
    if (bufLen > integer) {
        // ✅ 核心：从最新数据(rear)往前推 integer 个周期
        int pos = (rear - integer + DELAY_BUFFER_SIZE) % DELAY_BUFFER_SIZE;

        // 整数周期：直接返回
        if (decimal < 0.01f) {
            return delayBuffer[pos];
        }
        // 半周期：正确插值
        else {
            int nextPos = (pos - 1 + DELAY_BUFFER_SIZE) % DELAY_BUFFER_SIZE;
            // ✅ 正确系数：(1-decimal)*当前 + decimal*前一个
            return delayBuffer[pos] * (1 - decimal) + delayBuffer[nextPos] * decimal;
        }
    }
    // 数据不够返回 0
    else {
        return 0.0f;
    }
}