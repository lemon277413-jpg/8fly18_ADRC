#ifndef CircularDelay_h
#define CircularDelay_h

// 最大延时5周期，缓冲区固定6
#define DELAY_BUFFER_SIZE 6

class CircularDelayLine {
private:
  float delayBuffer[DELAY_BUFFER_SIZE];
  int front;
  int rear;
  int bufLen;

public:
  // 构造函数
  CircularDelayLine();
  // 更新新数据
  void update(float newData);
  // 获取延时数据（支持0~5，半周期）
  float getDelayed(float delay_cycles);
};

#endif