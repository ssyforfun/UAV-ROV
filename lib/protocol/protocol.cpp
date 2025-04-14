/*************************如何使用*************************
---如何使用
1 提供基础的浮点转字节函数
2 提供基础通讯协议框架CommunicationFrame，框架的形式为一个帧：
    -- 帧头+命令+数据长度+数据+校验
    -- 每个部分的长度和方法可以根据自己的意思设置，比如：
    ---- (CommunicationFrame)_txFrame.init(_txBuffer, ...) 可以设置不同于默认的帧的格式， 帧格式请参考CommunicationFrame类
    ---- (CommunicationFrame)_txFrame.setHeader可以设置不同于默认的帧头数据
3 提供基础的协议操作CommunicationProtocol
    -- 包含 接收：判断帧的完整性，校验方法，接收处理函数（需要被继承的类重写）
    -- 包含 发送：帧的合成，帧发送方法的包装（需要被继承的类重写）
    -- 校验方法可以被重写[checkAnalysis, checkGen]，默认是求和校验(_isCrcCheck)
4 一定要被重写（或者说要使用该文件，要写的部分）
    -- CommunicationProtocol::dataAnalysis 和 CommunicationProtocol::send 两组方法
 * **********************************************************/

#include "protocol.h"
#include "string.h"

// -------------------- 基础服务部分 ------------------------------------
#pragma region 基础服务部分
void float2bytes(const float *const fval, uint8_t *bval, int floatNum)
{
    FLOAT32_UNION fu32;
    int j = 0;
    for (int i = 0; i < floatNum; i++)
    {
        fu32.fval = fval[i];
        bval[j++] = fu32.uval.u1;
        bval[j++] = fu32.uval.u2;
        bval[j++] = fu32.uval.u3;
        bval[j++] = fu32.uval.u4;
    }
}

void double2bytes(const double *const dval, uint8_t *bval, int doubleNum)
{
    DOUBLE64_UNION du64;
    int j = 0;
    for (int i = 0; i < doubleNum; i++)
    {
        du64.dval = dval[i];
        bval[j++] = du64.luval.u1;
        bval[j++] = du64.luval.u2;
        bval[j++] = du64.luval.u3;
        bval[j++] = du64.luval.u4;
        bval[j++] = du64.luval.u5;
        bval[j++] = du64.luval.u6;
        bval[j++] = du64.luval.u7;
        bval[j++] = du64.luval.u8;
    }
}

void bytes2float(const uint8_t *const bval, float *fval, int floatNum)
{
    FLOAT32_UNION fu32;
    int j = 0;
    for (int i = 0; i < floatNum; i++)
    {
        fu32.uval.u1 = bval[j++];
        fu32.uval.u2 = bval[j++];
        fu32.uval.u3 = bval[j++];
        fu32.uval.u4 = bval[j++];
        fval[i] = fu32.fval;
    }
}

void bytes2double(const uint8_t *const bval, double *dval, int doubleNum)
{
    DOUBLE64_UNION du64;
    int j = 0;
    for (int i = 0; i < doubleNum; i++)
    {
        du64.luval.u1 = bval[j++];
        du64.luval.u2 = bval[j++];
        du64.luval.u3 = bval[j++];
        du64.luval.u4 = bval[j++];
        du64.luval.u5 = bval[j++];
        du64.luval.u6 = bval[j++];
        du64.luval.u7 = bval[j++];
        du64.luval.u8 = bval[j++];
        dval[i] = du64.dval;
    }
}

void uint2bytes(const uint32_t *uval, uint8_t *bval, int uintNum)
{
    int j = 0;
    for (int i = 0; i < uintNum; i++)
    {
        bval[j++] = ((uval[i] >> 24) & 0xFF);
        bval[j++] = ((uval[i] >> 16) & 0xFF);
        bval[j++] = ((uval[i] >> 8) & 0xFF);
        bval[j++] = ((uval[i] >> 0) & 0xFF);
    }
}

void bytes2uint(const uint8_t *bval, uint32_t *uval, int uintNum)
{
    uint32_t tmp;
    int j = 0;
    for (int i = 0; i < uintNum; i++)
    {
        tmp = 0;
        tmp += (bval[j++] << 24);
        tmp += (bval[j++] << 16);
        tmp += (bval[j++] << 8);
        tmp += (bval[j++] << 0);
        uval[i] = tmp;
    }
}
#pragma endregion 基础服务部分

// -------------------- CommunicationFrame 类 ------------------------------------
#pragma region CommunicationFrame 类
CommunicationFrame::CommunicationFrame()
{
    _header = NULL;
}

CommunicationFrame::CommunicationFrame(uint8_t *buf, uint32_t headerlen, uint32_t cmdlen, uint32_t datanumlen, uint32_t checklen)
{
    init(buf, headerlen, cmdlen, datanumlen, checklen);
}

/// @brief 帧的初始化
/// @param buf
/// @param headerlen
/// @param cmdlen
/// @param datanumlen
/// @param checklen
void CommunicationFrame::init(uint8_t *buf, uint32_t headerlen, uint32_t cmdlen, uint32_t datanumlen, uint32_t checklen)
{
    _header = buf;
    _headerLen = headerlen;
    _cmdPtr = _header + _headerLen;
    _cmdLen = cmdlen;
    _dataNum = _cmdPtr + _cmdLen;
    _dataNumLen = datanumlen;
    _dataNumMsbFirst = 1;
    _dataPtr = _dataNum + _dataNumLen;
    _dataLen = 0;
    _checkLen = checklen;
    _checkMsbFirst = 1;
    _headerEnd = _headerLen;
    _cmdEnd = _headerEnd + _cmdLen;
    _dataNumEnd = _cmdEnd + _dataNumLen;
    _index = 0;
    _length = 0;

    for (uint32_t i = 0; i < _headerLen; i++)
    {
        _header[i] = FRAME_HEADER;
    }
}

// 设置帧头, 默认的帧头数据为 0x55
void CommunicationFrame::setHeader(uint8_t *header)
{
    for (uint32_t i = 0; i < _headerLen; i++)
    {
        _header[i] = header[i];
    }
}

#pragma endregion CommunicationFrame 类

// -------------------- CommunicationProtocol 类 ------------------------------------
#pragma region CommunicationProtocol 类
//
CommunicationProtocol::CommunicationProtocol()
{
    _txFrame.init(_txFrameBuffer);
    _rxFrame.init(_rxFrameBuffer);
}

// frame数据生成
int CommunicationProtocol::txConstruct(const uint8_t *const cmd, const uint8_t *const data, uint8_t datalength)
{
    // 命令
    for (uint32_t i = 0; i < _txFrame._cmdLen; i++)
    {
        _txFrame._cmdPtr[i] = cmd[i];
    }

    // 数据长度
    if (_txFrame._dataNumMsbFirst) // 高位在前
    {
        for (uint32_t i = 0; i < _txFrame._dataNumLen; i++)
        {
            _txFrame._dataNum[i] = (datalength >> ((_txFrame._dataNumLen - i - 1) * 8));
        }
    }
    else // 低位在前
    {
        for (uint32_t i = 0; i < _txFrame._dataNumLen; i++)
        {
            _txFrame._dataNum[i] = (datalength >> (i * 8));
        }
    }

    // 数据
    for (uint32_t i = 0; i < datalength; i++)
    {
        _txFrame._dataPtr[i] = data[i];
    }

    // 校验位
    _txFrame._dataLen = datalength;
    _txFrame._index = 0;
    _txFrame._dataEnd = _txFrame._dataNumEnd + _txFrame._dataLen;
    _txFrame._checkEnd = _txFrame._dataEnd + _txFrame._checkLen;
    _txFrame._length = _txFrame._checkEnd;
    checkGen();
    return 0;
}

int CommunicationProtocol::txConstruct(const char *const str)
{
    uint8_t cmd = 's';
    uint8_t *data = (uint8_t *)(str);
    uint32_t datalength = strlen(str);
    return txConstruct(&cmd, data, datalength);
}

int CommunicationProtocol::txConstruct(const uint8_t *const buf, uint32_t len)
{
    uint8_t cmd = buf[0];
    const uint8_t *const data = &buf[1];
    if (len < 1)
        return -1;
    len--; // 减去cmd的长度
    return txConstruct(&cmd, data, len);
}

int CommunicationProtocol::rxAnalysis(char *str)
{
    const uint8_t *const recvBuf = (const uint8_t *const)str;
    uint32_t length = strlen(str);
    return rxAnalysis(recvBuf, length);
}

/**  采用 桢头 + 包长度 的通讯协议
 **/
int CommunicationProtocol::rxAnalysis(const uint8_t *const recvBuf, uint32_t length)
{
    uint8_t dataTmp;
    uint32_t tmp;

    for (uint32_t i = 0; i < length; i++)
    {
        dataTmp = recvBuf[i];

        if (_isFrameData)
        {
            // 帧头 header
            if (_rxFrame._index < _rxFrame._headerEnd)
            { // 处于桢头位置，需要判断桢头是否正确
                if (dataTmp != _rxFrame._header[_rxFrame._index])
                { // 如果桢头对应不上，从当前字节开始认为下一个桢的起点
                    if (dataTmp != _rxFrame._header[0])
                    {
                        _rxFrame._index = 0;
                        continue;
                    }
                    else
                    {
                        _rxFrame._index++;
                        continue;
                    }
                }
                else
                {
                    _rxFrame._index++;
                    continue;
                }
            }

            // 命令 cmd
            if (_rxFrame._index < _rxFrame._cmdEnd)
            {
                _rxFrame._header[_rxFrame._index++] = dataTmp;
                continue;
            }

            // 数据长度
            if (_rxFrame._index < _rxFrame._dataNumEnd)
            {
                _rxFrame._header[_rxFrame._index++] = dataTmp;
                /// 计算数据长度
                if (_rxFrame._index == _rxFrame._dataNumEnd) // 数据长度接收完整了，可以计算数据长度了
                {
                    tmp = 0;
                    if (_rxFrame._dataNumMsbFirst == 0) // 代表长度的数据数组中，高字节在后
                    {
                        for (uint32_t j = 0; j < _rxFrame._dataNumLen; j++)
                        {
                            tmp += (_rxFrame._dataNum[j] << (j * 8));
                        }
                    }
                    else // 代表长度的数据数组中，高字节在前
                    {
                        for (uint32_t j = 0; j < _rxFrame._dataNumLen; j++)
                        {
                            tmp += (_rxFrame._dataNum[j] << ((_rxFrame._dataNumLen - j - 1) * 8));
                        }
                    }
                    _rxFrame._dataLen = tmp;
                    _rxFrame._dataEnd = _rxFrame._dataNumEnd + _rxFrame._dataLen;
                    _rxFrame._checkEnd = _rxFrame._dataEnd + _rxFrame._checkLen;
                    _rxFrame._length = _rxFrame._checkEnd;
                }
                continue;
            }

            // 数据
            if (_rxFrame._index < _rxFrame._dataEnd)
            {
                _rxFrame._header[_rxFrame._index++] = dataTmp;
                continue;
            }

            // 校验
            if (_rxFrame._index < _rxFrame._checkEnd)
            {
                _rxFrame._header[_rxFrame._index++] = dataTmp;
                if (_rxFrame._index == _rxFrame._length)
                {
                    if (checkAnalysis() == 0) // 校验OK
                        dataAnalysis();
                    _rxFrame._index = 0;
                }
                continue;
            }
        }
        else
        {
            _rxFrame._header[_rxFrame._index++] = dataTmp;
        }
    }

    if (!_isFrameData)
    {
        dataAnalysis();
        _rxFrame._dataLen = _rxFrame._index;
        _rxFrame._index = 0;
    }

    return 0;
}

int CommunicationProtocol::getMiniTxFrameLength()
{
    if (_isFrameData)
        return _txFrame._dataNumEnd + _txFrame._checkLen;
    else
        return 0;
}

// 返回0代表成功，其它代表失败
int CommunicationProtocol::checkAnalysis()
{
    if (_isCrcCheck)
        return crcCheckAnalysis();
    else
        return sumCheckAnalysis();
}

// 校验码生成
// 返回0代表成功，其它代表失败
int CommunicationProtocol::checkGen()
{
    if (_isCrcCheck)
        return crcCheckGen();
    else
        return sumCheckGen();
}

// CRC校验 ---- remainder 0xA001
// 返回0代表成功，其它代表失败
int CommunicationProtocol::crcCheckAnalysis()
{
    uint16_t remainder = 0xA001;
    uint16_t ucrc = 0xFFFF;
    int len = _rxFrame._length - _rxFrame._checkLen;
    uint8_t *checkPtr = _rxFrame._header + _rxFrame._dataEnd;

    for (int i = 0; i < len; i++)
    {
        ucrc = (_rxFrame._header[i] ^ ucrc);
        for (int j = 0; j < 8; j++)
        {
            if ((ucrc & 0x0001))
            {
                ucrc = (ucrc >> 1);
                ucrc = (ucrc ^ remainder);
            }
            else
            {
                ucrc = (ucrc >> 1);
            }
        }
    }

    // ucrc = (((ucrc >> 8) & 0xFF) + ((ucrc << 8) & 0xFF00));
    for (int i = 0; i < (int)_rxFrame._checkLen; i++)
    {
        if (_rxFrame._checkMsbFirst) // 高位在前
        {
            if (checkPtr[i] != ((ucrc >> ((_rxFrame._checkLen - i - 1) * 8)) & 0xFF))
            {
                return -1;
            }
        }
        else // 低位在前
        {
            if (checkPtr[i] != ((ucrc >> ((i) * 8)) & 0xFF))
            {
                return -1;
            }
        }
    }

    return 0;
}

// CRC校验 ---- remainder 0xA001
// 返回0代表成功，其它代表失败
int CommunicationProtocol::crcCheckGen()
{
    uint16_t remainder = 0xA001;
    uint16_t ucrc = 0xFFFF;

    uint8_t *checkPtr = _txFrame._header + _txFrame._dataEnd;
    int len = _txFrame._dataEnd;

    for (int i = 0; i < len; i++)
    {
        ucrc = (_txFrame._header[i] ^ ucrc);
        for (int j = 0; j < 8; j++)
        {
            if ((ucrc & 0x0001))
            {
                ucrc = (ucrc >> 1);
                ucrc = (ucrc ^ remainder);
            }
            else
            {
                ucrc = (ucrc >> 1);
            }
        }
    }

    if (_txFrame._checkMsbFirst) // 高位在前
    {
        for (uint32_t i = 0; i < _txFrame._checkLen; i++)
        {
            checkPtr[i] = (ucrc >> ((_txFrame._checkLen - i - 1) * 8)) & 0xFF;
        }
    }
    else // 低位在前
    {
        for (uint32_t i = 0; i < _txFrame._checkLen; i++)
        {
            checkPtr[i] = (ucrc >> (i * 8)) & 0xFF;
        }
    }

    return 0;
}

// 求和校验
// 返回0代表成功，其它代表失败
int CommunicationProtocol::sumCheckAnalysis()
{
    int len = _rxFrame._length - _rxFrame._checkLen;
    uint8_t *checkPtr = _rxFrame._header + _rxFrame._dataEnd;
    int check = 0;

    if (len >= (int)_rxFrame._dataNumEnd)
    {
        for (int i = 0; i < len; i++)
        {
            check += _rxFrame._header[i];
        }

        for (int i = 0; i < (int)_rxFrame._checkLen; i++)
        {
            if (_rxFrame._checkMsbFirst) // 高位在前
            {
                if (checkPtr[i] != ((check >> ((_rxFrame._checkLen - i - 1) * 8)) & 0xFF))
                {
                    return -1;
                }
            }
            else // 低位在前
            {
                if (checkPtr[i] != ((check >> ((i) * 8)) & 0xFF))
                {
                    return -1;
                }
            }
        }

        return 0;
    }
    else
        return -1;
}

// 求和校验
// 返回0代表成功，其它代表失败
int CommunicationProtocol::sumCheckGen()
{
    int check = 0;
    uint8_t *checkPtr = _txFrame._header + _txFrame._dataEnd;

    // 除去校验位，所有数据的和
    for (uint32_t i = 0; i < _txFrame._dataEnd; i++)
    {
        check += _txFrame._header[i];
    }

    if (_txFrame._checkMsbFirst) // 高位在前
    {
        for (uint32_t i = 0; i < _txFrame._checkLen; i++)
        {
            checkPtr[i] = (check >> ((_txFrame._checkLen - i - 1) * 8)) & 0xFF;
        }
    }
    else // 低位在前
    {
        for (uint32_t i = 0; i < _txFrame._checkLen; i++)
        {
            checkPtr[i] = (check >> (i * 8)) & 0xFF;
        }
    }

    return 0;
}

uint8_t CommunicationProtocol::send(uint8_t *buf, uint32_t length)
{
    if (length >= 1)
        return send(buf, &buf[1], length - 1);
    else
        return 1;
}
uint8_t CommunicationProtocol::send(char *str)
{
    uint8_t cmd = 's';
    uint32_t datalength = strlen(str);
    uint8_t *data = (uint8_t *)(str);
    return send(&cmd, data, datalength);
}

uint8_t CommunicationProtocol::send(char cmd, uint8_t *data, uint32_t datalength)
{
    return send((uint8_t *)(&cmd), data, datalength);
}

// ----------- 一定要被重写的部分 -----------------------------------------------
// virtual func part
// uint8_t CommunicationProtocol::send(uint8_t *cmd, uint8_t *data, uint32_t datalength)
// {
//     return 0;
// }

// void CommunicationProtocol::dataAnalysis()
// {
// }
#pragma endregion CommunicationProtocol 类
