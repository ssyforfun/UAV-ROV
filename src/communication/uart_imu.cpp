#include "uart_imu.h"
#include "main.h"

UartIMU::UartIMU()
{
}

// 重写了 init 函数，因为要初始化imu的帧头， 匹配 IMU901 模块
void UartIMU::init(uint8_t *txheader, int sendSize, UART_HandleTypeDef *huart)
{
	_txFrame.setHeader(txheader);
	UartComm::init(sendSize, huart);
}

// ----------------- 接收处理 ---------------------------------
void UartIMU::dataAnalysis()
{
	switch (_rxFrame._cmdPtr[0])
	{
	case ZITAI_CMD:
		data_zitai();
		break;
	case SIYUANSHU_CMD:
		data_siyuanshu();
		break;
	case GYRO_ACC_CMD:
		data_gyro_acc();
		break;
	case MAGNET_CMD:
		data_magnet();
		break;
	case PRESSURE_CMD:
		data_pressure();
		break;
	case PORT_STATUS_CMD:
		break;
	default:
		break;
	}
}

void UartIMU::data_zitai()
{
	float zitai[3];
	uint8_t tmpH, tmpL;

	if (_rxFrame._dataLen == 6)
	{
		tmpL = _rxFrame._dataPtr[0];
		tmpH = _rxFrame._dataPtr[1];
		zitai[0] = (float)((int16_t)(tmpH << 8) | tmpL) / 32768.0f * 180.0f; // X

		tmpL = _rxFrame._dataPtr[2];
		tmpH = _rxFrame._dataPtr[3];
		zitai[1] = (float)((int16_t)(tmpH << 8) | tmpL) / 32768.0f * 180.0f; // Y

		tmpL = _rxFrame._dataPtr[4];
		tmpH = _rxFrame._dataPtr[5];
		zitai[2] = (float)((int16_t)(tmpH << 8) | tmpL) / 32768.0f * 180.0f; // Z

		for (int i = 0; i < 3; i++)
			imuHandle.zitai[i] = zitai[i];
	}
}
void UartIMU::data_siyuanshu(void)
{
	float siyuanshu[4];
	uint8_t tmpH, tmpL;

	if (_rxFrame._dataLen == 8)
	{
		tmpL = _rxFrame._dataPtr[0];
		tmpH = _rxFrame._dataPtr[1];
		siyuanshu[0] = (float)((int16_t)(tmpH << 8) | tmpL) / 32768.0f;

		tmpL = _rxFrame._dataPtr[2];
		tmpH = _rxFrame._dataPtr[3];
		siyuanshu[1] = (float)((int16_t)(tmpH << 8) | tmpL) / 32768.0f;

		tmpL = _rxFrame._dataPtr[4];
		tmpH = _rxFrame._dataPtr[5];
		siyuanshu[2] = (float)((int16_t)(tmpH << 8) | tmpL) / 32768.0f;

		tmpL = _rxFrame._dataPtr[6];
		tmpH = _rxFrame._dataPtr[7];
		siyuanshu[3] = (float)((int16_t)(tmpH << 8) | tmpL) / 32768.0f;

		for (int i = 0; i < 4; i++)
			imuHandle.siyuanshu[i] = siyuanshu[i];
	}
}
void UartIMU::data_gyro_acc(void)
{
	float gyro[3], acc[3];
	uint8_t tmpH, tmpL;

	if (_rxFrame._dataLen == 12)
	{
		for (int i = 0; i < 3; i++)
		{
			tmpL = _rxFrame._dataPtr[i * 2];
			tmpH = _rxFrame._dataPtr[i * 2 + 1];
			acc[i] = (float)((int16_t)(tmpH << 8) | tmpL) / 32768.0f * IMU_ACC_RANGE;
		}

		for (int i = 0; i < 3; i++)
		{
			tmpL = _rxFrame._dataPtr[i * 2 + 6];
			tmpH = _rxFrame._dataPtr[i * 2 + 7];
			gyro[i] = (float)((int16_t)(tmpH << 8) | tmpL) / 32768.0f * IMU_GYPO_RANGE;
		}

		for (int i = 0; i < 3; i++)
		{
			imuHandle.acc[i] = acc[i];
			imuHandle.gyro[i] = gyro[i];
		}
	}
}
void UartIMU::data_magnet(void)
{
	float magnet[3];
	float temperature;
	uint8_t tmpH, tmpL;

	if (_rxFrame._dataLen == 8)
	{
		for (int i = 0; i < 3; i++)
		{
			tmpL = _rxFrame._dataPtr[i * 2];
			tmpH = _rxFrame._dataPtr[i * 2 + 1];
			magnet[i] = (float)((int16_t)(tmpH << 8) | tmpL);
		}

		tmpL = _rxFrame._dataPtr[6];
		tmpH = _rxFrame._dataPtr[7];
		temperature = (float)((int16_t)(tmpH << 8) | tmpL) / 100.0f;

		for (int i = 0; i < 3; i++)
		{
			imuHandle.magnet[i] = magnet[i];
		}
		imuHandle.temperature = temperature * 0.01f + imuHandle.temperature * 0.99f;
	}
}
void UartIMU::data_pressure(void)
{
	float pressure;
	float altitude;
	float temperature;
	uint8_t tmpH, tmpL;
	int itmp;

	if (_rxFrame._dataLen == 10)
	{
		tmpL = _rxFrame._dataPtr[0];
		tmpH = _rxFrame._dataPtr[1];
		itmp = ((int)(tmpH << 8) | tmpL);
		tmpL = _rxFrame._dataPtr[2];
		tmpH = _rxFrame._dataPtr[3];
		pressure = ((((int)(tmpH << 8) | tmpL) << 16) + itmp);

		tmpL = _rxFrame._dataPtr[4];
		tmpH = _rxFrame._dataPtr[5];
		itmp = ((int)(tmpH << 8) | tmpL);
		tmpL = _rxFrame._dataPtr[6];
		tmpH = _rxFrame._dataPtr[7];
		altitude = ((((int)(tmpH << 8) | tmpL) << 16) + itmp);

		tmpL = _rxFrame._dataPtr[8];
		tmpH = _rxFrame._dataPtr[9];
		temperature = (float)((int16_t)(tmpH << 8) | tmpL) / 100.0f;

		imuHandle.pressure = pressure;
		imuHandle.altitude = altitude * 1e-3f;
		imuHandle.temperature = temperature * 0.01f + imuHandle.temperature * 0.99f;
	}
}

// ----------------- 发送处理 ----------------------------------
// baudrate=4:115200,  8:9600,  6:38400
void UartIMU::imu_baud_set(uint8_t baudrate) // 设为115200
{
	send(IMUCMD_BAUD & 0x7F, &baudrate, 1);
}

// 回传速率设置
// [0-9]
// 1: 200Hz, 6:10Hz, 9:1Hz, 4:50Hz, 5:20Hz, 3:100Hz, 2:125Hz
void UartIMU::imu_returnrate_set(uint8_t returnrate) // 设置回传速度
{
	send(IMUCMD_RETURNRATE & 0x7F, &returnrate, 1);
}

// 回传内容设置 0不上传;  1上传
// bit 0: 姿态
// bit 1: 四元数
// bit 2: 陀螺仪和加速度
// bit 3: 磁力计
// bit 4: 气压计
// bit 5: 端口状态
// bit 6: 数据匿名
// bit 7: reserved to 0
void UartIMU::imu_returncontext_set(uint8_t returncontext) // 设置回传内容
{
	send(IMUCMD_RETURNSET & 0x7F, &returncontext, 1);
}

// 设置算法 1:9轴 or 0:6轴
void UartIMU::imu_alg_set(uint8_t alg)
{
	send(IMUCMD_ALG & 0x7F, &alg, 1);
}

// 保存设置的内容到imu模块的flash
void UartIMU::imu_save(void) // 保存设置
{
	uint8_t buf;
	send(IMUCMD_SAVE & 0x7F, &buf, 1);
}

// range [0, 3]
// 0: 250dps
// 1: 500dps
// 2: 1000dps
// 3: 2000dps (default)
void UartIMU::imu_gyro_range_set(int range)
{
	if (range > 3)
		return;
	uint8_t buf = range;
	send(IMUCMD_GYRO_RANGE & 0x7F, &buf, 1);
}

// range [0, 3], unit G = 9.8m/s^2
// 0: 2G
// 1: 4G (default)
// 2: 8G
// 3: 16G
void UartIMU::imu_acc_range_set(int range)
{
	if (range > 3)
		return;
	uint8_t buf = range;
	send(IMUCMD_ACC_RANGE & 0x7F, &buf, 1);
}

void UartIMU::imu_led_set(bool on)
{
	uint8_t buf = on ? 0 : 1;
	send(0x0F & 0x7F, &buf, 1);
}
