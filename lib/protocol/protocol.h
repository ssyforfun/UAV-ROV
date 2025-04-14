#ifndef protocol_h
#define protocol_h

#include <stddef.h>
#include <stdint.h>

#define PROTOCOL_BUFFER_SIZE 256
#define FRAME_HEADER 0x55

// float    32bit,  31          30 23         22 0
//                  sign          E            M
//        val = (-1)^sign *  2^(E - 127)  *  (1 + M)
typedef struct
{
	uint8_t u1;
	uint8_t u2;
	uint8_t u3;
	uint8_t u4;
} FLOATBYTE_STRUCT;

typedef union
{
	float fval;
	FLOATBYTE_STRUCT uval;
	uint32_t val32;
} FLOAT32_UNION;

typedef struct
{
	uint8_t u1;
	uint8_t u2;
	uint8_t u3;
	uint8_t u4;
	uint8_t u5;
	uint8_t u6;
	uint8_t u7;
	uint8_t u8;
} DOUBLEBYTE_STRUCT;

typedef union
{
	double dval;
	DOUBLEBYTE_STRUCT luval;
	uint64_t val64;
} DOUBLE64_UNION;

void float2bytes(const float *const fval, uint8_t *bval, int floatNum);
void double2bytes(const double *const dval, uint8_t *bval, int doubleNum);
void bytes2float(const uint8_t *const bval, float *fval, int floatNum);
void bytes2double(const uint8_t *const bval, double *dval, int doubleNum);
void uint2bytes(const uint32_t* uval, uint8_t *bval, int uintNum);
void bytes2uint(const uint8_t *bval, uint32_t* uval, int uintNum);

class CommunicationFrame
{
public:
	CommunicationFrame();
	CommunicationFrame(uint8_t *buf, uint32_t headerlen = 2, uint32_t cmdlen = 1, uint32_t datanumlen = 1, uint32_t checklen = 1);
	void init(uint8_t *buf, uint32_t headerlen = 2, uint32_t cmdlen = 1, uint32_t datanumlen = 1, uint32_t checklen = 1);
	void setHeader(uint8_t *header); // 设置帧头
	const uint8_t *getFrame(int *length)
	{
		*length = _length;
		return (const uint8_t *)_header;
	}

public:
	uint8_t *_header;	 // 帧头的匹配字符地址
	uint32_t _headerLen; // 帧头长度
	uint8_t *_cmdPtr;  // 帧CMD的地址，CMD是COMMAND的缩写
	uint32_t _cmdLen; // 帧CMD占用的字节数
	uint8_t *_dataNum;         // 帧数据长度的地址
	uint32_t _dataNumLen;	  // 帧数据长度占用的字节数
	uint8_t _dataNumMsbFirst; // 是否是先高字节后低字节
	uint8_t *_dataPtr;         // 数据帧地址
	uint32_t _checkLen;		// 检验位占用的字节数
	uint8_t _checkMsbFirst; // 是否是先高字节后低字节
	uint32_t _headerEnd;	// 帧头结尾所在的索引位置
	uint32_t _cmdEnd;		//
	uint32_t _dataNumEnd;

	uint32_t _dataLen; // 帧数据占用的字节数
	uint32_t _index;   // 代表帧长度的字节索引
	uint32_t _length;  // 代表帧长度的字节数
	uint32_t _dataEnd;
	uint32_t _checkEnd;
};

class CommunicationProtocol
{
public:
	CommunicationProtocol();

public:
	virtual uint8_t send(uint8_t *buf, uint32_t length);
	virtual uint8_t send(char *str);
	virtual uint8_t send(uint8_t *cmd, uint8_t *data, uint32_t datalength) = 0;
	virtual uint8_t send(char cmd, uint8_t *data, uint32_t datalength);

protected:
	virtual void dataAnalysis() = 0;

public:
    bool _isFrameData = true;

public:
	bool _isCrcCheck = false; // 默认是sum校验
protected:
	virtual int checkAnalysis();	// 校验函数
	virtual int checkGen();			// 校验生成函数
	virtual int sumCheckGen();		// 求和校验
	virtual int sumCheckAnalysis(); // 求和校验
	virtual int crcCheckGen();		// crc校验
	virtual int crcCheckAnalysis(); // crc校验

public:
	// 接收数据的数据分析
	virtual int rxAnalysis(const uint8_t *const recvBuf, uint32_t length); // 数据处理函数
protected:
	virtual int rxAnalysis(char *str);
	// 用户发送数据的协议组成
	virtual int txConstruct(const uint8_t *const cmd, const uint8_t *const data, uint8_t datalength); // 帧生成
	virtual int txConstruct(const char *const str);
	virtual int txConstruct(const uint8_t *const buf, uint32_t len);

public:
	// const uint8_t *const getTxBufPtr() { return _txFrameBuffer; };
	// const uint8_t *const getRxBufPtr() { return _rxFrameBuffer; };
	int getMiniTxFrameLength();
	// int getTxFrameLength() { return _txFrame._length; }

protected:
	CommunicationFrame _txFrame;
	CommunicationFrame _rxFrame;
	uint8_t _txFrameBuffer[PROTOCOL_BUFFER_SIZE];
	uint8_t _rxFrameBuffer[PROTOCOL_BUFFER_SIZE];
};

#endif