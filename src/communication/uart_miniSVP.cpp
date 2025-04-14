#include "uart_miniSVP.h"
#include "main.h"
#include "math.h"

UartMiniSVP::UartMiniSVP()
{
}

int UartMiniSVP::rxAnalysis(const uint8_t *const recvBuf, uint32_t length)
{
	for (uint32_t i = 0; i < length; i++)
	{
		if (_rxFrame._index > 255)
			_rxFrame._index = 0;

		_rxFrame._header[_rxFrame._index++] = recvBuf[i];
		if (recvBuf[i] == 0x0A) // 一个帧结束符
		{
			dataAnalysis();
			_rxFrame._index = 0;
		}
	}
	return 0;
}

void UartMiniSVP::dataAnalysis()
{
	if (_rxFrame._index == 0)
		return;
	if (_rxFrame._header[_rxFrame._index - 2] != 0x0D) // 一个帧的结束符标记倒数第二位
		return;

	svpData();
}

// 压力、温度、电导数据提取
void UartMiniSVP::svpData()
{
	if (_rxFrame._index < 6) // 数据长度不满足 3data+2tab+2end minimum
		return;

	// 找到2个0x09制表符
	int tabcount, tabindex[2];
	tabcount = 0;
	for (uint32_t i = 0; i < _rxFrame._index; i++)
	{
		if (_rxFrame._header[i] == 0x09)
		{
			if (tabcount < 2)
				tabindex[tabcount] = i;
			tabcount++;
		}
	}
	if (tabcount != 2) // 如果不是2个制表符则代表出错
		return;

	// 制表符位置不对
	if ((tabindex[0] == 0) || ((uint32_t)tabindex[1] == (_rxFrame._index - 3)))
	{
		return;
	}

	bool valid;
	float pressure = 0, temperature = 0, conductivity = 0;
	pressure = string2float(_rxFrame._header, 0, tabindex[0], &valid);
	if (valid)
		temperature = string2float(_rxFrame._header, tabindex[0] + 1, tabindex[1], &valid);
	if (valid)
		conductivity = string2float(_rxFrame._header, tabindex[1] + 1, _rxFrame._index - 2, &valid);
	if (valid)
	{
		this->pressure = pressure;
		this->temperature = temperature;
		this->conductivity = conductivity;
	}
}

// 如果发生错误，输出0，并且valid=false
float UartMiniSVP::string2float(uint8_t *str, int start, int end, bool *valid)
{
	int val = 0;
	bool isneg = false;
	bool epos = false;
	int eposcount = 0;

	if (valid != NULL)
		*valid = false;

	int len = end - start;
	if (len <= 0)
		return 0;

	for (int i = start; i < end; i++)
	{
		if (str[i] == '.')
		{
			epos = true;
		}
		else if ((str[i] <= 0x39) && (str[i] >= 0x30))
		{
			val *= 10;
			val += (str[i] - '0');
			if (epos)
				eposcount++;
		}
		else if (i == start)
		{
			if ((str[start] == '-') || (str[start] == '+'))
			{
				if (str[start] == '-')
					isneg = true;
			}
			else
				return 0;
		}
		else
			return 0;
	}

	float fval;
	if (epos)
	{
		fval = val;
		for (int i = 0; i < eposcount; i++)
		{
			fval *= 0.1f;
		}
	}
	else
		fval = val;

	if (isneg)
		fval = 0 - fval;

	if (valid != NULL)
		*valid = true;
	return fval;
}
