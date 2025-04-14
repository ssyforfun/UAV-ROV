#ifndef fifo_hpp
#define fifo_hpp

#include <stddef.h>
#include <stdint.h>

/******************* 模板template ***************************
 *  template：让 类型 变成可变参数
 *    基本问题: 模板不是简单的变量化类型，不要采用.h/.cpp分离式的变成方式，要放到.hpp里面
 *    基本用法： template <typename T, typename T1>
 *              1 template: 关键词
 *              2 typename: 关键词，可以用class替代[同一个意思]
 *              3 T/T1: 代表类型的形参[类型变量], 即用T代替某一种具体的类型
 *              4 函数模板:
 *                template <class T1, class T2>
 *                T1 func(T1 a, T2& b)
 *                {
 *                   // a就是T1类型的变量，b是T2类型的引用
 *                }
 *                调用方法:
 *                int c = 6;
 *                int i = func<int, int>(5, c);
 *              5 类模板
 *                template <class T1, class T2>
 *                class A
 *                {
 *                   T1 a;
 *                   void fun1(T2 b);
 *                }
 *                .cpp实现
 *                template <class T2>
 *                void A<T2>::fun1(T2 b)
 *                {
 *                   //....
 *                }
 *              6 特化，(有点类似类的继承[对虚函数的使用])
 *                1 原型、全特化、偏特化
 *                2 定义一个原型
 *                  template <typename T1, typename T2> struct A;
 *                3 全特化， 指定模板基本类型(float, int是基本类型)
 *                  template <>
 *                  struct <float, int> { };
 *                4 偏特化，指定模板非基本类型(指针是非基本类型)
 *                  template <typename T1>
 *                  struct <T1*, int> { };
 * **********************************************************/

/******************* 多变形参... ****************************
 *  ... ：让函数形参/类型 可以是可变长度的
 *    基本要素: ...可以认为是类型的数组，数组长度不定[0, n]
 *    用法:  1 void fun(int a, ...);
 *           2 template <class ... Ts> struct A; // 泛型
 *             template <> struct A<> {}; // 特化1
 *             template <class T, class ... Ts> struct A<T, Ts...> {} // 特化2
 *    多用于递归里
 * **********************************************************/

template <typename T>
class Fifo
{
public:
    Fifo()
    {
    }
    Fifo(int length, T *memory)
    {
        init(length, memory);
    }
    ~Fifo()
    {
    }

    void init(int length, T *memory)
    {
        _buffer = memory;
        _size = length;
        _writePtr = 0;
        _readPtr = 0;
        _occupy = 0;
        _lock = 0;
    }

public:
    // 剩余的大小, 返回-1代表操作失败
    int remainedSize(void)
    {
        return (_lock) ? -1 : (_size - _occupy);
    }

    // 使用了的大小, 返回-1代表操作失败
    int occupiedSize(void)
    {
        return (_lock) ? -1 : _occupy;
    }

    // 总大小
    int totalSize(void)
    {
        return _size;
    }

    // 删除len长度的数据，和peek一起用，可以构成非阻塞式的read, 返回-1代表操作失败
    int pop(int len)
    {
        if ((len > _size) || (len < 0))
            return -1;

        if (_lock)
            return -1;

        _lock = 1;
        if (_occupy < len)
        {
            _lock = 0;
            return -1;
        }
        else
        {
            _occupy -= len;
            _readPtr += len;
            while (_readPtr >= _size)
            {
                _readPtr -= _size;
            }
        }
        _lock = 0;
        return len;
    }

    // 读取第N个数, 返回NULL代表操作失败
    const T *peek(int index)
    {
        if (_buffer == NULL)
            return NULL;

        if (_lock)
            return NULL;

        if ((index < 0) || (index > (_size - 1)))
            return NULL;

        int tindex = _readPtr + index;
        if (tindex >= _size)
            tindex -= _size;
        if (tindex < 0)
            tindex = 0;
        if (tindex >= _size)
            tindex = _size - 1;
        return (const T *)(&_buffer[tindex]);
    }

    // 读但不改变FIFO状态, 返回1，-1代表操作失败
    int peek(T *data, int len)
    {
        if (_buffer == NULL)
            return -1;

        if (_lock)
            return -1;

        _lock = 1;
        T *des;
        T *src;

        int readptr = _readPtr;
        int occupy = _occupy;

        int i;

        des = data;
        src = &_buffer[readptr];
        int availableLen = (len <= occupy) ? len : occupy; // 可以操作的长度
        if ((availableLen + readptr) <= _size)
        {
            for (i = 0; i < availableLen; i++)
            {
                *des++ = *src++;
            }
        }
        else
        {
            int len1 = _size - readptr;
            for (i = 0; i < len1; i++)
            {
                *des++ = *src++;
            }
            src = _buffer;
            len1 = availableLen - len1;
            for (i = 0; i < len1; i++)
            {
                *des++ = *src++;
            }
        }

        _lock = 0;
        return availableLen;
    }

    // 读, 返回=实际操作长度，-1代表操作失败
    int read(T *data, int len)
    {
        if (_buffer == NULL)
            return -1;

        T *des;
        T *src;
        int i;

        if (_lock == 0)
        {
            _lock = 1;
            des = data;
            src = &_buffer[_readPtr];
            int availableLen = (len <= _occupy) ? len : _occupy; // 可以操作的长度
            if ((availableLen + _readPtr) <= _size)
            {
                for (i = 0; i < availableLen; i++)
                {
                    *des++ = *src++;
                    _occupy--;
                    _readPtr++;
                }
            }
            else
            {
                int len1 = _size - _readPtr;
                for (i = 0; i < len1; i++)
                {
                    *des++ = *src++;
                    _occupy--;
                    _readPtr++;
                }
                _readPtr = 0;
                src = _buffer;
                len1 = availableLen - len1;
                for (i = 0; i < len1; i++)
                {
                    *des++ = *src++;
                    _occupy--;
                    _readPtr++;
                }
            }

            if (_readPtr >= _size)
            {
                _readPtr = 0;
            }
            _lock = 0;
            return availableLen;
        }
        else
        {
            return -1;
        }
    }

    // 读, 返回=当前剩余长度，-1代表操作失败
    int read(T *data)
    {
        if (_buffer == NULL)
            return -1;

        T *des;
        T *src;
        int i;

        if (_occupy == 0)
            return -1;

        if (_lock == 0)
        {
            _lock = 1;
            data = &_buffer[_readPtr++];
            _occupy--;
            if (_readPtr >= _size)
            {
                _readPtr = 0;
            }
            _lock = 0;
            return _occupy;
        }
        else
        {
            return -1;
        }
    }

    // 写, 返回=实际操作长度，-1代表操作失败
    int write(const T *data, int len)
    {
        if (_buffer == NULL)
            return -1;

        T *des;
        const T *src;
        int i;

        if (_lock == 0)
        {
            _lock = 1;
            src = data;
            des = &_buffer[_writePtr];
            int remain = _size - _occupy;
            int availableLen = (len <= remain) ? len : remain; // 可以操作的长度
            if ((availableLen + _writePtr) <= _size)
            {
                for (i = 0; i < availableLen; i++)
                {
                    *des++ = *src++;
                    _occupy++;
                    _writePtr++;
                }
            }
            else
            {
                int len1 = _size - _writePtr;
                for (i = 0; i < len1; i++)
                {
                    *des++ = *src++;
                    _occupy++;
                    _writePtr++;
                }
                _writePtr = 0;
                des = _buffer;
                len1 = availableLen - len1;
                for (i = 0; i < len1; i++)
                {
                    *des++ = *src++;
                    _occupy++;
                    _writePtr++;
                }
            }

            if (_writePtr >= _size)
            {
                _writePtr = 0;
            }
            _lock = 0;
            return availableLen;
        }
        else
        {
            return -1;
        }
    }

    // 写, 返回=实际操作的字节长度，-1代表操作失败
    // 以字节的方式写，这样可以避免write那种方式会碰到的内存不对齐的问题（如果会有采用类型转换的话）
    // lenT: 代表的和write(T*, lenT)的lenT一致，注意不是len_bytes
    int writeByBytes(const uint8_t *databytes, int lenT)
    {
        if (_buffer == NULL)
            return -1;

        uint8_t *des;
        const uint8_t *src;
        uint8_t *bufferByte = (uint8_t *)_buffer;
        int i;

        if (_lock == 0)
        {
            _lock = 1;

            int sizeT = sizeof(T);
            int len = lenT * sizeT;
            int writePrtByte = _writePtr * sizeT;
            int sizeByte = _size * sizeT;
            int occupyByte = _occupy * sizeT;

            int remain = sizeByte - occupyByte;
            int availableLen = (len <= remain) ? len : remain; // 可以操作的长度

            src = databytes;
            des = &bufferByte[writePrtByte];

            if ((availableLen + writePrtByte) <= sizeByte)
            {
                for (i = 0; i < availableLen; i++)
                {
                    *des++ = *src++;
                    occupyByte++;
                    writePrtByte++;
                }
            }
            else
            {
                int len1 = sizeByte - writePrtByte;
                for (i = 0; i < len1; i++)
                {
                    *des++ = *src++;
                    occupyByte++;
                    writePrtByte++;
                }
                writePrtByte = 0;
                des = bufferByte;
                len1 = availableLen - len1;
                for (i = 0; i < len1; i++)
                {
                    *des++ = *src++;
                    occupyByte++;
                    writePrtByte++;
                }
            }

            if (writePrtByte >= sizeByte)
            {
                writePrtByte = 0;
            }

            _writePtr = writePrtByte / sizeT;
            _occupy = occupyByte / sizeT;
            _lock = 0;
            return availableLen;
        }
        else
        {
            return -1;
        }
    }

    // 写, 返回1，-1代表操作失败
    int write(T data)
    {
        if (_buffer == NULL)
            return -1;

        if (_lock == 0)
        {
            _lock = 1;
            int remain = _size - _occupy;
            if (remain > 0)
            {
                _buffer[_writePtr++] = data;
                _occupy++;
                if (_writePtr >= _size)
                    _writePtr = 0;
            }
            _lock = 0;
            return 1;
        }
        else
        {
            return -1;
        }
    }

    // 重置, 返回1，-1代表操作失败
    int reset()
    {
        if (_lock == 0)
        {
            _writePtr = 0;
            _readPtr = 0;
            _occupy = 0;
            return 1;
        }
        else
            return -1;
    }

private:
    T *_buffer = NULL; // 此内存需要外部定义，这里只是一个指针
    int _readPtr;
    int _writePtr;
    int _size;
    int _occupy;

private:
    volatile int _lock;
};

#endif
