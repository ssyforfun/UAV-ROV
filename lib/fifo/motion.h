#ifndef motion_h
#define motion_h

#include <stddef.h>
#include <stdint.h>

typedef struct
{
    uint32_t _time;
    float _value;
} MotionData_t;

class Motion
{
public:
    Motion();
    ~Motion();
    void init(void);

public:
    int write(const uint8_t *data, int data_len);
    int read(uint8_t *buf, int len);
    int read(MotionData_t *buf, int len);
    int peek(uint8_t *buf, int len, int offset);
    int peek(MotionData_t *buf, int len, int offset);
    int peekLatest(uint8_t *buf);
    int peekLast(uint8_t *buf);
    int peekLatest(MotionData_t *buf);
    int peekLast(MotionData_t *buf);
    int pop(int len);
    int occupy(void);
    int reset(void);

private:
    bool _en = false; // 使能
    uint32_t _tick;   // 当en=true时开始计数

public:
    int _dType; // 数据类型: 默认0是位置, 1是速度, 2是转矩
    bool En() { return _en; }
    void En(bool en)
    {
        if (_en != en)
        {
            if (en == true)
            {
                _tick = 0;
            }
            _en = en;
        }
    }
    uint32_t Tick() { return _tick; }
    void TickAdd() { _tick++; }
};

extern Motion motion;

#endif
