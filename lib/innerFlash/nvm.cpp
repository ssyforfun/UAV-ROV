#include "nvm.h"

/**
 * @brief flash erase single sector
 * 
 * @param addr 
 * @return int 0:success, -1:falied, others:falied
 */
int NVM_eraseSector(uint32_t addr)
{
    uint32_t sector = GetSector(addr);

    FLASH_EraseInitTypeDef erase_struct = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
        .Banks = 0, // only used for mass erase
        .Sector = sector,
        .NbSectors = 1,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3};

    uint32_t error;
    HAL_FLASH_Unlock(); // 要先解锁才能操作
    HAL_FLASH_ClearError();
    if (HAL_FLASHEx_Erase(&erase_struct, &error) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return -1;
    }
    HAL_FLASH_Lock(); // 最后一定要上锁
    return 0;
}

/**
 * @brief flash read
 * 
 * @param data 以字节为单位
 * @param addr 
 * @param length 以字节为单位
 * @return int 0:success, -1:falied, others:falied
 */
int NVM_read(uint8_t *data, uint32_t addr, uint32_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        data[i] = *(__IO uint8_t *)(addr + i);
    }
    return 0;
}

/**
 * @brief flash write
 * 
 * @param data 以字节为单位
 * @param addr 
 * @param length 以字节为单位
 * @return int 0:success, -1:falied, others:falied
 */
int NVM_write(uint8_t *data, uint32_t addr, uint32_t length)
{
    uint8_t *datacheck = data;
    uint32_t addrcheck = addr;
    uint32_t lengthcheck = length;

    HAL_FLASH_Unlock();
    HAL_FLASH_ClearError();

    // 刚开始按字节写最多前3个数据，然后采用按word写中间的数据，最后再用byte写剩余最多后3个数据
    // 这样做的目的是实现了对齐，同时尽量加快了写的速度 word speed > byte speed
    // handle unaligned start,
    for (; (addr & 0x3) && length; ++data, ++addr, --length)
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, addr, *data) != HAL_OK)
            goto fail;

    // write 32-bit values (64-bit doesn't work)
    for (; length >= 4; data += 4, addr += 4, length -= 4)
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, *(uint32_t *)data) != HAL_OK)
            goto fail;

    // handle unaligned end
    for (; length; ++data, ++addr, --length)
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, addr, *data) != HAL_OK)
            goto fail;

    // 校验 看看写入的数据是否准确
    for (uint32_t i = 0; i < lengthcheck; i++)
    {
        if (datacheck[i] != *(__IO uint8_t *)(addrcheck + i))
            goto fail;
    }

    HAL_FLASH_Lock();
    return 0;
fail:
    HAL_FLASH_Lock();
    return HAL_FLASH_GetError(); // non-zero
}

/**
  * @brief  Gets the sector of a given address
  * @param  None
  * @retval The sector of a given address
  */
uint32_t GetSector(uint32_t Address)
{
    uint32_t sector = 0;

    if ((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
    {
        sector = FLASH_SECTOR_0;
    }
    else if ((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
    {
        sector = FLASH_SECTOR_1;
    }
    else if ((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
    {
        sector = FLASH_SECTOR_2;
    }
    else if ((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
    {
        sector = FLASH_SECTOR_3;
    }
    else if ((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
    {
        sector = FLASH_SECTOR_4;
    }
    else if ((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
    {
        sector = FLASH_SECTOR_5;
    }
    else if ((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
    {
        sector = FLASH_SECTOR_6;
    }
    else if ((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
    {
        sector = FLASH_SECTOR_7;
    }
    else if ((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
    {
        sector = FLASH_SECTOR_8;
    }
    else if ((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
    {
        sector = FLASH_SECTOR_9;
    }
    else if ((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
    {
        sector = FLASH_SECTOR_10;
    }
    else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_11) */
    {
        sector = FLASH_SECTOR_11;
    }

    return sector;
}

/**
  * @brief  Gets sector Size
  * @param  None
  * @retval The size of a given sector
  */
uint32_t GetSectorSize(uint32_t Sector)
{
    uint32_t sectorsize = 0x00;

    if ((Sector == FLASH_SECTOR_0) || (Sector == FLASH_SECTOR_1) || (Sector == FLASH_SECTOR_2) || (Sector == FLASH_SECTOR_3))
    {
        sectorsize = 16 * 1024;
    }
    else if (Sector == FLASH_SECTOR_4)
    {
        sectorsize = 64 * 1024;
    }
    else
    {
        sectorsize = 128 * 1024;
    }
    return sectorsize;
}

