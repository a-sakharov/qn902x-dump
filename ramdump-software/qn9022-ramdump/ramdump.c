#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdbool.h>
#include "crc16-ccitt-algorithm.h"

#ifdef __unix__
#include <unistd.h>
#include <errno.h>
#define Sleep(x) usleep(x*1000)
#elif WIN32
#include <windows.h>
#else
#define no sleep function
#endif

#define BOORLOADER_HEADCODE         0x71

#define B_C_CMD                     0x33
#define SET_BR_CMD                  0x34
#define SET_FLASH_CLK_CMD           0x35
#define RD_BL_VER_CMD               0x36
#define RD_CHIP_ID_CMD              0x37
#define RD_FLASH_ID_CMD             0x38
#define SET_APP_LOC_CMD             0x39
#define SETUP_FLASH_CMD             0x3A
#define SET_ST_ADDR_CMD             0x3B
#define SET_APP_SIZE_CMD            0x3C
#define SET_APP_CRC_CMD             0x3E
#define SET_APP_IN_FLASH_ADDR_CMD   0x40
#define SE_FLASH_CMD                0x42
#define BE_FLASH_CMD                0x43
#define CE_FLASH_CMD                0x44
#define PROGRAM_CMD                 0x45
#define RD_CMD                      0x46
#define VERIFY_CMD                  0x47
#define PROTECT_CMD                 0x48
#define RUN_APP_CMD                 0x49
#define REBOOT_CMD                  0x4A
#define WR_RANDOM_DATA_CMD          0x4B
#define SET_APP_IN_RAM_ADDR_CMD     0x4C
#define SET_APP_RESET_ADDR_CMD      0x4D

#define BOOTLOADER_CONFIRM_OK       0x01
#define BOOTLOADER_CONFIRM_FAIL     0x02

#define BOOTLOADER_RESULT_SUCCESS   0x03
#define BOOTLOADER_RESULT_FAIL      0x04

#define APP_RESET_ADDRESS           0x10000000
#define RAM_OFFSET                  0x00000000
#define FAILFALSE(x) if(!(x)){printf("Fail at line %d, error: %s (%s)\n", __LINE__, #x, strerror(errno)); return false;}

typedef void* TRANSPORT;

bool QN902x_bl_EnableProtection(TRANSPORT device);
bool QN902x_bl_ReadBlVersion(TRANSPORT device, uint32_t *ver);
bool QN902x_bl_ReadFlashId(TRANSPORT device, uint32_t *id);
bool QN902x_bl_ReadChipId(TRANSPORT device, uint32_t *id);
bool QN902x_bl_ReadNvds(TRANSPORT device, uint8_t *buffer, uint32_t size);
bool QN902x_bl_SetAppResetAddress(TRANSPORT device, uint32_t addr);
bool QN902x_bl_SetStartAddress(TRANSPORT device, uint32_t addr);
bool QN902x_bl_SetAppInRamAddress(TRANSPORT device, uint32_t addr);
bool QN902x_bl_SetBaudRate(TRANSPORT device, uint32_t baud);
bool QN902x_bl_SetAppCrc(TRANSPORT device, uint32_t crc);
bool QN902x_bl_SetAppSize(TRANSPORT device, uint32_t size);
bool QN902x_bl_Verify(TRANSPORT device);
bool QN902x_bl_RunApp(TRANSPORT device);
bool QN902x_bl_Reboot(TRANSPORT device);
bool QN902x_bl_ChipErase(TRANSPORT device);
bool QN902x_bl_SetAppLocation(TRANSPORT device, bool shoudUseFlash);
bool QN902x_bl_Program(TRANSPORT device, uint8_t *data, uint16_t size);
bool QN902x_bl_SetupFlashCmd(TRANSPORT device, uint8_t commands[8]);
bool QN902x_bl_WriteRandomData(TRANSPORT device, uint8_t commands[12]);

bool QN902x_bl_command(TRANSPORT device, uint8_t command, uint32_t send_length, uint8_t *send, uint32_t *recv_length, uint8_t *recv, bool check_exe_result);
bool QN902x_bl_InvokeBootloader(TRANSPORT device);
uint16_t QN902x_bl_crc(size_t data_len, uint8_t *data);

bool readout_binary(char *name, size_t *outsize, uint8_t **data);

bool QN902x_subbl_RamLoadLaunchBinaryProtectFlash(TRANSPORT device, uint8_t * image, size_t image_sz);
bool QN902x_subbl_NvdsDump(TRANSPORT device);

bool QN902x_ramdump_Write(TRANSPORT device, uint32_t addr, uint32_t data);
bool QN902x_ramdump_Read(TRANSPORT device, uint32_t addr, uint32_t read_bytes, uint8_t *buffer);
bool QN902x_ramdump_ReadFlash(TRANSPORT device, uint32_t addr, uint32_t read_bytes, uint8_t * buffer);
bool QN902x_ramdump_DumpRam(TRANSPORT device, uint32_t addr, uint32_t length, char *filename);
bool QN902x_ramdump_DumpFlash(TRANSPORT device, uint32_t addr, uint32_t length, char* filename);

bool Transport_OpenDevice(TRANSPORT *device, char *id);
bool Transport_CloseDevice(TRANSPORT device);
bool Transport_Recv(TRANSPORT device, uint8_t *buffer, uint32_t buffer_len);
bool Transport_Send(TRANSPORT device, uint8_t *buffer, uint32_t buffer_len);
bool Transport_Reset(TRANSPORT device);
bool Transport_Unreset(TRANSPORT device);
bool Transport_ReadLine(TRANSPORT device, char *buffer, uint32_t buffer_max);
bool Transport_GetStats(TRANSPORT device, uint32_t *recv_quenue);
bool Transport_SetBaudRate(TRANSPORT device, uint32_t baudrate);

#define BAUD_RATE_115200 0x0000082c
#define BAUD_RATE_9600   0x0000680b

//#define LOG_EXCHANGE

int main(int argc, char **argv)
{
    TRANSPORT device;

    char binary[] = "loader.bin";
    char* device_id;

    FAILFALSE(argc > 1);

    device_id = argv[argc-1];

    FAILFALSE(Transport_OpenDevice(&device, device_id));
    printf("IO device initialized\n");
    bool loaded_success = false;

    size_t i;
    size_t loader_sz;
    uint8_t *loader;
    FAILFALSE(readout_binary(binary, &loader_sz, &loader));
    FAILFALSE(loader_sz < 0x1000);
    for (i = 0; i < 5; ++i)
    {
        if (QN902x_subbl_RamLoadLaunchBinaryProtectFlash(device, loader, loader_sz))
        {
            loaded_success = true;
            break;
        }
    }
    free(loader);

    if (!loaded_success)
    {
        printf("Error loading FW\n");
    }
    else
    {
        printf("Application loaded\n");

        //FAILFALSE(QN902x_ramdump_DumpRam(device, 0x10000000, 0x00010000, "ram_0.bin"));
        //FAILFALSE(QN902x_ramdump_DumpRam(device, 0x10000000, 0x00010000, "ram_1.bin"));
        //FAILFALSE(QN902x_ramdump_DumpRam(device, 0x10000000, 0x00010000, "ram_2.bin"));

        //FAILFALSE(QN902x_ramdump_DumpRam(device, 0x00000000, 0x01000000 - 1, "BootLoader_0.bin"));
        //FAILFALSE(QN902x_ramdump_DumpRam(device, 0x00000000, 0x01000000 - 1, "BootLoader_1.bin"));
        //FAILFALSE(QN902x_ramdump_DumpRam(device, 0x00000000, 0x01000000 - 1, "BootLoader_2.bin"));

        //FAILFALSE(QN902x_ramdump_DumpRam(device, 0x01000000, 0x00018000 - 1, "ROM_0.bin"));
        //FAILFALSE(QN902x_ramdump_DumpRam(device, 0x01000000, 0x00018000 - 1, "ROM_1.bin"));
        //FAILFALSE(QN902x_ramdump_DumpRam(device, 0x01000000, 0x00018000 - 1, "ROM_2.bin"));

        FAILFALSE(QN902x_ramdump_DumpFlash(device, 0x00000000, 0x00020000, "flash_0.bin"));
        FAILFALSE(QN902x_ramdump_DumpFlash(device, 0x00000000, 0x00020000, "flash_1.bin"));
        
    }

    Transport_CloseDevice(device);

    return 0;
}

/* Subfunctions, using bootloader commands */

bool QN902x_subbl_RamLoadLaunchBinaryProtectFlash(TRANSPORT device, uint8_t *image, size_t image_sz)
{
    uint32_t loader_crc;
    uint8_t flash_command_set_no_erase[8] =
    {
        0x05, //real: 0x05, Read Status Register 
        0x05, //real: 0x06, Write Enable 
        0x05, //real: 0x20, Sector Erase 
        0x05, //real: 0x52, Block Erase 
        0x05, //real: 0x60, Chip Erase 
        0xB9, //real: 0xB9, Deep Power Down 
        0xAB, //real: 0xAB, Release form Deep Power Down 
        0x01, //real: 0x01, reserved, the value is not 0x00 and 0xFF 
    };
           
    uint8_t flash_command_set_original[8] =
    {
        0x05, //real: 0x05, Read Status Register 
        0x06, //real: 0x06, Write Enable 
        0xDC, //real: 0x20, Sector Erase 
        0x4E, //real: 0x52, Block Erase 
        0x5C, //real: 0x60, Chip Erase 
        0xB9, //real: 0xB9, Deep Power Down 
        0xAB, //real: 0xAB, Release form Deep Power Down 
        0x01, //real: 0x01, reserved, the value is not 0x00 and 0xFF 
    };

    loader_crc = QN902x_bl_crc(image_sz, image);

    FAILFALSE(QN902x_bl_InvokeBootloader(device)); //Build Connection
    printf("Bootloader mode entered\n");

    uint32_t bl_ver;
    FAILFALSE(QN902x_bl_ReadBlVersion(device, &bl_ver)); //Read BL Version
    printf("Bootloader version: %.8X\n", bl_ver);

    uint32_t chip_id;
    FAILFALSE(QN902x_bl_ReadChipId(device, &chip_id)); //Read Chip ID
    printf("Chip ID %.8X\n", chip_id);

    uint32_t flash_id;
    FAILFALSE(QN902x_bl_ReadFlashId(device, &flash_id)); //Read Flash ID
    printf("Flash ID %.8X\n", flash_id);


    FAILFALSE(QN902x_bl_SetupFlashCmd(device, flash_command_set_no_erase));

    FAILFALSE(QN902x_bl_SetBaudRate(device, BAUD_RATE_115200)); //Set UART Baudrate
    FAILFALSE(Transport_SetBaudRate(device, 115200));//for 16M crystal


    FAILFALSE(QN902x_bl_SetAppLocation(device, false)); //Set APP LOC(SRAM)

    FAILFALSE(QN902x_bl_SetStartAddress(device, RAM_OFFSET));//set start addr

    FAILFALSE(QN902x_bl_SetAppResetAddress(device, APP_RESET_ADDRESS));//set app reset addr

    FAILFALSE(QN902x_bl_SetAppInRamAddress(device, APP_RESET_ADDRESS));

    FAILFALSE(QN902x_bl_SetAppSize(device, image_sz));//set app size

    FAILFALSE(QN902x_bl_SetAppCrc(device, loader_crc));//set app crc


    uint16_t loaded = 0;
    uint16_t to_load;
    while (loaded < (uint16_t)image_sz)
    {
        if ((uint16_t)image_sz - loaded > 0x100)
        {
            to_load = 0x100;
        }
        else
        {
            to_load = (uint16_t)image_sz - loaded;
        }

        FAILFALSE(QN902x_bl_Program(device, image + loaded, to_load));//program
        loaded += to_load;
    }

    FAILFALSE(QN902x_bl_SetupFlashCmd(device, flash_command_set_original));

    FAILFALSE(QN902x_bl_Verify(device));//verify

    FAILFALSE(QN902x_bl_RunApp(device));//run

    return true;
}

bool QN902x_subbl_NvdsDump(TRANSPORT device)
{
    //untested
    uint32_t addr = 0x00000000;
    FAILFALSE(QN902x_bl_SetStartAddress(device, addr));
    uint8_t readbuf[0x10];
    uint32_t readed_total = 0;
    while (QN902x_bl_ReadNvds(device, readbuf, sizeof(readbuf)) && readed_total < 0x1000)
    {
        printf("%.8X: ", addr);
        size_t i;
        for (i = 0; i < sizeof(readbuf); ++i)
        {
            printf("%.2hhX ", readbuf[i]);
        }
        printf("\n");

        addr += sizeof(readbuf);
        readed_total += sizeof(readbuf);
    }
    return true;
}

/* Direct bootloader commands */

bool QN902x_bl_EnableProtection(TRANSPORT device)
{
    FAILFALSE(QN902x_bl_command(device, PROTECT_CMD, 0, NULL, NULL, NULL, true));

    return true;
}

bool QN902x_bl_ReadBlVersion(TRANSPORT device, uint32_t *ver)
{
    uint32_t recved = 4;
    FAILFALSE(QN902x_bl_command(device, RD_BL_VER_CMD, 0, NULL, &recved, (uint8_t*)ver, false));
    FAILFALSE(recved == 4);

    return true;
}

bool QN902x_bl_ReadChipId(TRANSPORT device, uint32_t*id)
{
    uint32_t recved = 4;
    FAILFALSE(QN902x_bl_command(device, RD_CHIP_ID_CMD, 0, NULL, &recved, (uint8_t*)id, false));
    FAILFALSE(recved == 4);

    return true;
}

bool QN902x_bl_ReadFlashId(TRANSPORT device, uint32_t*id)
{
    uint32_t recved = 4;
    FAILFALSE(QN902x_bl_command(device, RD_FLASH_ID_CMD, 0, NULL, &recved, (uint8_t*)id, false));
    FAILFALSE(recved == 4);

    return true;
}

bool QN902x_bl_ReadNvds(TRANSPORT device, uint8_t *buffer, uint32_t size)
{
    uint32_t recved = size;

    FAILFALSE(QN902x_bl_command(device, RD_CMD, sizeof(size), (uint8_t*)&size, &recved, buffer, false));
    FAILFALSE(recved == size);

    return true;
}

bool QN902x_bl_SetAppInRamAddress(TRANSPORT device, uint32_t addr)
{
    FAILFALSE(QN902x_bl_command(device, SET_APP_IN_RAM_ADDR_CMD, sizeof(addr), (uint8_t*)&addr, NULL, NULL, true));

    return true;
}

bool QN902x_bl_SetStartAddress(TRANSPORT device, uint32_t addr)
{
    FAILFALSE(QN902x_bl_command(device, SET_ST_ADDR_CMD, sizeof(addr), (uint8_t*)&addr, NULL, NULL, false));

    return true;
}

bool QN902x_bl_SetAppResetAddress(TRANSPORT device, uint32_t addr)
{
    FAILFALSE(QN902x_bl_command(device, SET_APP_RESET_ADDR_CMD, sizeof(addr), (uint8_t*)&addr, NULL, NULL, true));

    return true;
}

bool QN902x_bl_SetAppLocation(TRANSPORT device, bool shoudUseFlash)
{
    uint32_t param = !!shoudUseFlash;
    FAILFALSE(QN902x_bl_command(device, SET_APP_LOC_CMD, sizeof(param), (uint8_t*)&param, NULL, NULL, false));

    return true;
}

bool QN902x_bl_SetAppSize(TRANSPORT device, uint32_t size)
{
    FAILFALSE(QN902x_bl_command(device, SET_APP_SIZE_CMD, sizeof(size), (uint8_t*)&size, NULL, NULL, true));

    return true;
}

bool QN902x_bl_SetAppCrc(TRANSPORT device, uint32_t crc)
{
    FAILFALSE(QN902x_bl_command(device, SET_APP_CRC_CMD, sizeof(crc), (uint8_t*)&crc, NULL, NULL, true));

    return true;
}

bool QN902x_bl_SetBaudRate(TRANSPORT device, uint32_t baud)
{
    FAILFALSE(QN902x_bl_command(device, SET_BR_CMD, sizeof(baud), (uint8_t*)&baud, NULL, NULL, false));

    return true;
}

bool QN902x_bl_Program(TRANSPORT device, uint8_t *data, uint16_t size)
{
    FAILFALSE(QN902x_bl_command(device, PROGRAM_CMD, size, data, NULL, NULL, true));

    return true;
}

bool QN902x_bl_Verify(TRANSPORT device)
{
    FAILFALSE(QN902x_bl_command(device, VERIFY_CMD, 0, NULL, NULL, NULL, true));

    return true;
}

bool QN902x_bl_RunApp(TRANSPORT device)
{
    FAILFALSE(QN902x_bl_command(device, RUN_APP_CMD, 0, NULL, NULL, NULL, false));

    return true;
}

bool QN902x_bl_Reboot(TRANSPORT device)
{
    FAILFALSE(QN902x_bl_command(device, REBOOT_CMD, 0, NULL, NULL, NULL, false));

    return true;
}

bool QN902x_bl_ChipErase(TRANSPORT device)
{
    FAILFALSE(QN902x_bl_command(device, CE_FLASH_CMD, 0, NULL, NULL, NULL, true));

    return true;
}

bool QN902x_bl_SetupFlashCmd(TRANSPORT device, uint8_t commands[8])
{
    FAILFALSE(QN902x_bl_command(device, SETUP_FLASH_CMD, 8, commands, NULL, NULL, true));

    return true;
}

bool QN902x_bl_WriteRandomData(TRANSPORT device, uint8_t commands[12])
{
    FAILFALSE(QN902x_bl_command(device, WR_RANDOM_DATA_CMD, 12, commands, NULL, NULL, true));

    return true;
}

/* Utility functions for bootloader functions */

bool QN902x_bl_InvokeBootloader(TRANSPORT device)
{
    uint8_t invokebl[1];

    memset(invokebl, B_C_CMD, sizeof(invokebl));

    uint8_t invoke_attempt = 0;
    uint8_t read_attempt = 0;
    uint8_t recv = 0;
    uint32_t rx;

    for(invoke_attempt=0; invoke_attempt < 100 && recv != BOOTLOADER_CONFIRM_OK; invoke_attempt++)
    {
        FAILFALSE(Transport_Unreset(device));//unreset
        Sleep(10);
        FAILFALSE(Transport_Reset(device));//reset
        Sleep(10);
        FAILFALSE(Transport_Unreset(device));//unreset

        for (read_attempt = 0; read_attempt < 5; ++read_attempt)
        {
            FAILFALSE(Transport_Send(device, invokebl, sizeof(invokebl)));

            FAILFALSE(Transport_GetStats(device, &rx));

            if (rx)
            {
                if (!Transport_Recv(device, &recv, 1))
                {
                    continue;
                }

                if (recv == BOOTLOADER_CONFIRM_OK)
                {
                    break;
                }
            }

            Sleep(5);
        }
    }

    FAILFALSE(recv == BOOTLOADER_CONFIRM_OK);

    return true;
}

bool QN902x_bl_command(TRANSPORT device, uint8_t command, uint32_t send_length, uint8_t *send, uint32_t *recv_length, uint8_t *recv, bool check_exe_result)
{
    uint8_t packet_send[0x1FF] = {
        BOORLOADER_HEADCODE,
        command,
        (send_length >> 0) & 0xFF,
        (send_length >> 8) & 0xFF,
        (send_length >> 16) & 0xFF,
    };
    size_t packet_send_pos = 5;

    FAILFALSE(send_length <= (sizeof(packet_send) - 7));

    memcpy(packet_send + packet_send_pos, send, send_length);
    packet_send_pos += send_length;

    uint16_t crc16;
    crc16 = QN902x_bl_crc(packet_send_pos - 1, packet_send + 1);

    packet_send[packet_send_pos++] = (crc16 >> 0) & 0xFF;
    packet_send[packet_send_pos++] = (crc16 >> 8) & 0xFF;

    FAILFALSE(Transport_Send(device, packet_send, packet_send_pos));

    uint8_t packet_recv[0xFFF];
    size_t packet_recv_pos = 0;

    size_t retry = 0;

    //CONFIRM + RESULT + HEAD + COMMAND + LEN1 + LEN2 + LEN3 + DATA[0..0xFFFFFF] + CRC1 + CRC2

    uint32_t rx;
    uint32_t recv_packet_size;
    bool got_confirm = false;
    bool got_exe_status = false;
    bool crc16_valid = false;
    while (retry++<1000)
    {
        FAILFALSE(Transport_GetStats(device, &rx));

        if (rx)
        {
            //read
            uint32_t toread;
            toread = rx;
            if (sizeof(packet_recv) - packet_recv_pos < rx)
            {
                toread = sizeof(packet_recv) - packet_recv_pos < rx;
            }

            if (toread)
            {
                if (!got_confirm || (check_exe_result && !got_exe_status))
                {
                    toread = 1;
                }
            }

            FAILFALSE(Transport_Recv(device, packet_recv + packet_recv_pos, toread));

            packet_recv_pos += toread;

            if (packet_recv_pos > 0 && got_confirm == false)
            {
                FAILFALSE(packet_recv[0] == BOOTLOADER_CONFIRM_OK);
                memmove(packet_recv, packet_recv + 1, packet_recv_pos - 1);
                packet_recv_pos--;
                got_confirm = true;
            }

            if (packet_recv_pos > 0 && check_exe_result == true && got_exe_status == false)
            {
                FAILFALSE(packet_recv[0] == BOOTLOADER_RESULT_SUCCESS);
                memmove(packet_recv, packet_recv + 1, packet_recv_pos - 1);
                packet_recv_pos--;
                got_exe_status = true;
            }

            if (packet_recv_pos)
            {
                FAILFALSE(packet_recv[0] == BOORLOADER_HEADCODE);
            }

            if (got_confirm)
            {
                if (!check_exe_result || got_exe_status)
                {
                    if (!recv)
                    {
                        return true;
                    }
                }
            }

            if (packet_recv_pos > 3)
            {
                recv_packet_size = ((uint32_t)packet_recv[2] << 0) | ((uint32_t)packet_recv[3] << 8) | ((uint32_t)packet_recv[4] << 16);

                if (packet_recv_pos >= (1 + 1 + 3 + recv_packet_size + 2))
                {
                    crc16 = QN902x_bl_crc(1 + 3 + recv_packet_size, packet_recv + 1);

                    uint16_t crc16_recv;
                    crc16_recv = ((uint16_t)packet_recv[1 + 1 + 3 + recv_packet_size + 0] << 0) | ((uint16_t)packet_recv[1 + 1 + 3 + recv_packet_size + 1] << 8);

                    if (crc16 == crc16_recv)
                    {
                        crc16_valid = true;
                    }
                    else
                    {
                        crc16_valid = false;
                    }
                    break;
                }
            }
        }
        Sleep(1);
    }

    if (!crc16_valid)
    {
        crc16_valid = 0;
    }

    FAILFALSE(crc16_valid);
    FAILFALSE(recv_packet_size <= *recv_length);

    *recv_length = recv_packet_size;
    memcpy(recv, packet_recv + 5, *recv_length);

    return true;
}

uint16_t QN902x_bl_crc(size_t data_len, uint8_t *data)
{
    uint16_t crc16;
    size_t i;

    crc16 = 0;// crc16_ccitt_init();
    for (i = 0; i < data_len; i++)
    {
        crc16 = crc16_ccitt_update(data[i], crc16);
    }
    crc16 = crc16_ccitt_finalize(crc16);

    return crc16;
}

/* System functions */

bool readout_binary(char *name, size_t *outsize, uint8_t **data)
{
    FILE *f;

    f = fopen(name, "rb");
    if (!f)
    {
        return false;
    }

    fseek(f, 0, SEEK_END);
    *outsize = ftell(f);
    fseek(f, 0, SEEK_SET);

    *data = malloc(*outsize);
    if (!*data)
    {
        fclose(f);
        return false;
    }

    fread(*data, 1, *outsize, f);

    fclose(f);

    return true;
}

/* High-level functions, using RAMDUMP interface */

bool QN902x_ramdump_DumpRam(TRANSPORT device, uint32_t addr, uint32_t length, char* filename)
{
    FILE* out;
    uint8_t packet[0x20];
    uint32_t dumped = 0;
    time_t start;
    out = fopen(filename, "wb");
    FAILFALSE(out);

    start = time(NULL);
    while (dumped < length)
    {
        FAILFALSE(QN902x_ramdump_Read(device, addr + dumped, sizeof(packet), packet));
#if 0
        printf("%.8X  |  ", addr + dumped);
        uint32_t i;
        for (i=0; i< sizeof(packet); ++i)
        {
            printf("%.2X ", packet[i]);
        }
        printf(" |  ");
        for (i = 0; i < sizeof(packet); ++i)
        {
            printf("%c", isgraph(packet[i]) ? packet[i] : '.');
        }
        printf("\n");
#else
        if (dumped % 0x400 == 0)
        {
            double percent;
            percent = ((double)dumped) / (((double)length) / 100.0);
            time_t time_elapsed;
            time_elapsed = time(NULL) - start;

            printf("%ds, address %.8X (%.1f%%). eta %ds\n", (int)time_elapsed, dumped + addr, percent, (int)((100. / percent)* time_elapsed) - (int)time_elapsed);
        }
#endif
        dumped += sizeof(packet);
        fwrite(packet, 1, sizeof(packet), out);
    }

    fclose(out);

    return true;
}

bool QN902x_ramdump_DumpFlash(TRANSPORT device, uint32_t addr, uint32_t length, char* filename)
{
    FILE* out;
    uint8_t packet[0x100];
    uint32_t dumped = 0;
    time_t start;
    out = fopen(filename, "wb");
    FAILFALSE(out);

    FAILFALSE(addr + 0x30000000UL < 0x3FFFFFE8);
    FAILFALSE((addr + length + 0x30000000UL) < 0x3FFFFFE8);

    start = time(NULL);
    while (dumped < length)
    {
        FAILFALSE(QN902x_ramdump_ReadFlash(device, 0x1000, 0x10, packet));
        FAILFALSE(QN902x_ramdump_ReadFlash(device, addr + dumped, sizeof(packet), packet));
#if 0
        printf("%.8X  |  ", addr + dumped);
        uint32_t i;
        for (i = 0; i < sizeof(packet); ++i)
        {
            printf("%.2X ", packet[i]);
        }
        printf(" |  ");
        for (i = 0; i < sizeof(packet); ++i)
        {
            printf("%c", isgraph(packet[i]) ? packet[i] : '.');
        }
        printf("\n");
#else
        if (dumped % 0x400 == 0)
        {
            double percent;
            percent = ((double)dumped) / (((double)length) / 100.0);
            time_t time_elapsed;
            time_elapsed = time(NULL) - start;

            printf("%ds, address %.8X (%.1f%%). eta %ds\n", (int)time_elapsed, dumped + addr, percent, (int)((100. / percent) * time_elapsed) - (int)time_elapsed);
        }
#endif
        dumped += sizeof(packet);
        fwrite(packet, 1, sizeof(packet), out);
    }

    fclose(out);

    return true;
}

bool QN902x_ramdump_Write(TRANSPORT device, uint32_t addr, uint32_t data)
{
    char request[32];

    snprintf(request, sizeof(request), "w%X.%X\n", addr, data);
    FAILFALSE(Transport_Send(device, (uint8_t*)request, strlen(request)));

    char readout_text[32];
    FAILFALSE(Transport_ReadLine(device, readout_text, sizeof(readout_text)));

    FAILFALSE(strcmp(readout_text, "OK") == 0);

    return true;
}

bool QN902x_ramdump_Read(TRANSPORT device, uint32_t addr, uint32_t read_bytes, uint8_t* buffer)
{
    char request[32];

    FAILFALSE((read_bytes % 4) == 0);

    snprintf(request, sizeof(request), "r%X.%X\n", addr, read_bytes);
    FAILFALSE(Transport_Send(device, (uint8_t*)request, strlen(request)));

    char *readout_text;
    size_t alloc_size = read_bytes * 3 + 16;
    readout_text = malloc(alloc_size);
    FAILFALSE(readout_text);
    if (!Transport_ReadLine(device, readout_text, alloc_size))
    {
        free(readout_text);
        return false;
    }

    uint32_t buffer_pos = 0;

    char* text_p = readout_text;
    while (*text_p)
    {
        uint8_t byte;

        if (strchr("abcdef", *text_p))
        {
            *text_p -= 'a' - 'A';
        }
        if (*text_p >= 'A' && *text_p <= 'F')
        {
            byte = *text_p - 'A' + 0x0a;
        }
        else if (*text_p >= '0' && *text_p <= '9')
        {
            byte = *text_p - '0';
        }
        else
        {
            free(readout_text);
            return false;
        }
        byte <<= 4;
        text_p++;

        if (strchr("abcdef", *text_p))
        {
            *text_p -= 'a' - 'A';
        }
        if (*text_p >= 'A' && *text_p <= 'F')
        {
            byte |= *text_p - 'A' + 0x0a;
        }
        else if (*text_p >= '0' && *text_p <= '9')
        {
            byte |= *text_p - '0';
        }
        else
        {
            free(readout_text);
            return false;
        }
        text_p++;

        if (*text_p == ' ')
        {
            text_p++;
        }
        else
        {
            free(readout_text);
            return false;
        }

        buffer[buffer_pos++] = byte;
    }

    free(readout_text);
    return true;
}

bool QN902x_ramdump_ReadFlash(TRANSPORT device, uint32_t addr, uint32_t read_bytes, uint8_t* buffer)
{
    char request[32];

    FAILFALSE((read_bytes % 4) == 0);

    snprintf(request, sizeof(request), "f%X.%X\n", addr, read_bytes);
    FAILFALSE(Transport_Send(device, (uint8_t*)request, strlen(request)));

    char* readout_text;
    size_t alloc_size = read_bytes * 3 + 16;
    readout_text = malloc(alloc_size);
    FAILFALSE(readout_text);
    if (!Transport_ReadLine(device, readout_text, alloc_size))
    {
        free(readout_text);
        return false;
    }

    uint32_t buffer_pos = 0;

    char* text_p = readout_text;
    while (*text_p)
    {
        uint8_t byte;

        if (strchr("abcdef", *text_p))
        {
            *text_p -= 'a' - 'A';
        }
        if (*text_p >= 'A' && *text_p <= 'F')
        {
            byte = *text_p - 'A' + 0x0a;
        }
        else if (*text_p >= '0' && *text_p <= '9')
        {
            byte = *text_p - '0';
        }
        else
        {
            free(readout_text);
            return false;
        }
        byte <<= 4;
        text_p++;

        if (strchr("abcdef", *text_p))
        {
            *text_p -= 'a' - 'A';
        }
        if (*text_p >= 'A' && *text_p <= 'F')
        {
            byte |= *text_p - 'A' + 0x0a;
        }
        else if (*text_p >= '0' && *text_p <= '9')
        {
            byte |= *text_p - '0';
        }
        else
        {
            free(readout_text);
            return false;
        }
        text_p++;

        if (*text_p == ' ')
        {
            text_p++;
        }
        else
        {
            free(readout_text);
            return false;
        }

        buffer[buffer_pos++] = byte;
    }

    free(readout_text);
    return true;
}

/* Transport functions */

bool Transport_ReadLine(TRANSPORT device, char* buffer, uint32_t buffer_max)
{
    uint32_t readed_total = 0;
    uint32_t index = 0;

    while (readed_total < buffer_max && index++ < 100)
    {
        uint32_t rx;
        FAILFALSE(Transport_GetStats(device, &rx));

        if (rx)
        {
            uint32_t toread;
            if (buffer_max - readed_total >= rx)
            {
                toread = rx;
            }
            else
            {
                toread = buffer_max - readed_total;
            }
            FAILFALSE(Transport_Recv(device, (uint8_t*)(buffer + readed_total), toread));
            readed_total += toread;

            uint32_t i;
            for (i = 0; i < readed_total; ++i)
            {
                if (buffer[i] == '\n')
                {
                    buffer[i] = '\0';
                    return true;
                }
            }
        }
        Sleep(1);
    }

    return false;
}

/* Change functions below for adapting to new system  */

#ifdef USE_FTDI_TRANSPORT

#include <ftd2xx.h>

bool Transport_Send(TRANSPORT device, uint8_t *buffer, uint32_t buffer_len)
{
    DWORD written;

    FAILFALSE(FT_Write(*(FT_HANDLE*)device, (LPVOID)buffer, (DWORD)buffer_len, &written) == FT_OK);

    FAILFALSE(written == buffer_len);

#ifdef LOG_EXCHANGE
    {
        printf("> ");
        size_t i;
        for (i = 0; i < buffer_len; ++i)
        {
            printf("%.2hhX ", buffer[i]);
        }
        printf("\n");
    }
#endif

    return true;
}

bool Transport_Recv(TRANSPORT device, uint8_t *buffer, uint32_t buffer_len)
{
    DWORD readed;

    FAILFALSE(FT_Read(*(FT_HANDLE*)device, (LPVOID)buffer, (DWORD)buffer_len, &readed) == FT_OK);

    FAILFALSE(readed == buffer_len);

#ifdef LOG_EXCHANGE
    {
        printf("< ");
        size_t i;
        for (i = 0; i < buffer_len; ++i)
        {
            printf("%.2hhX ", buffer[i]);
        }
        printf("\n");
    }
#endif

    return true;
}

bool Transport_GetStats(TRANSPORT device, uint32_t *recv_quenue)
{
    DWORD tx;
    DWORD rx;
    DWORD event;

    FAILFALSE(FT_GetStatus(*(FT_HANDLE*)device, &rx, &tx, &event) == FT_OK);

    *recv_quenue = rx;

    return true;
}

bool Transport_OpenDevice(TRANSPORT *device, char *id)
{
    FT_HANDLE dev;

    FAILFALSE(FT_OpenEx(id, FT_OPEN_BY_SERIAL_NUMBER, &dev) == FT_OK);

    FAILFALSE(FT_SetBaudRate(dev, FT_BAUD_9600) == FT_OK);//for 16M crystal

    FAILFALSE(FT_SetDataCharacteristics(dev, FT_BITS_8, FT_STOP_BITS_1, FT_PARITY_NONE) == FT_OK);

    FAILFALSE(FT_SetFlowControl(dev, FT_FLOW_NONE, 0, 0) == FT_OK);

    FAILFALSE(FT_Purge(dev, FT_PURGE_RX | FT_PURGE_TX) == FT_OK);

    Sleep(10);

    *device = malloc(sizeof(FT_HANDLE));
    if (!*device)
    {
        FT_Close(dev);
        return false;
    }

    **(FT_HANDLE**)device = dev;

    return true;
}

bool Transport_CloseDevice(TRANSPORT device)
{
    FT_Close(*(FT_HANDLE*)device);
    free(device);

    return true;
}

bool Transport_Reset(TRANSPORT device)
{
    FAILFALSE(FT_SetRts(*(FT_HANDLE*)device) == FT_OK);//reset

    return true;
}

bool Transport_Unreset(TRANSPORT device)
{
    FAILFALSE(FT_ClrRts(*(FT_HANDLE*)device) == FT_OK);//unreset

    return true;
}

bool Transport_SetBaudRate(TRANSPORT device, uint32_t baudrate)
{
    switch (baudrate)
    {
        case 9600:
            FAILFALSE(FT_SetBaudRate(*(FT_HANDLE*)device, FT_BAUD_9600) == FT_OK);//unreset
            break;

        case 115200:
            FAILFALSE(FT_SetBaudRate(*(FT_HANDLE*)device, FT_BAUD_115200) == FT_OK);//unreset
            break;

        default:
            return true;
            break;
    }

    return true;
}
#elif USE_WINAPI_COM_PORT

#include <windows.h>

bool Transport_Send(TRANSPORT device, uint8_t* buffer, uint32_t buffer_len)
{
    DWORD written;

    FAILFALSE(WriteFile(*(HANDLE*)device, buffer, buffer_len, &written, NULL) == TRUE);

    FAILFALSE(written == buffer_len);

#ifdef LOG_EXCHANGE
    {
        printf("> ");
        size_t i;
        for (i = 0; i < buffer_len; ++i)
        {
            printf("%.2hhX ", buffer[i]);
        }
        printf("\n");
    }
#endif

    return true;
}

bool Transport_Recv(TRANSPORT device, uint8_t* buffer, uint32_t buffer_len)
{
    DWORD readed;

    FAILFALSE(ReadFile(*(HANDLE*)device, buffer, buffer_len, &readed, NULL) == TRUE);

    FAILFALSE(readed == buffer_len);

#ifdef LOG_EXCHANGE
    {
        printf("< ");
        size_t i;
        for (i = 0; i < buffer_len; ++i)
        {
            printf("%.2hhX ", buffer[i]);
        }
        printf("\n");
    }
#endif

    return true;
}

bool Transport_GetStats(TRANSPORT device, uint32_t* recv_quenue)
{
    COMSTAT stat;

    FAILFALSE(ClearCommError(*(HANDLE*)device, NULL, &stat) );

    *recv_quenue = stat.cbInQue;

    return true;
}

bool Transport_OpenDevice(TRANSPORT* device, char* id)
{
    HANDLE dev;
    DCB dcb_com_params =
    {
        .DCBlength = sizeof(dcb_com_params),
    };
    COMMTIMEOUTS timeouts =
    {
        .ReadIntervalTimeout = 50,
        .ReadTotalTimeoutConstant = 50,
        .ReadTotalTimeoutMultiplier = 10,
        .WriteTotalTimeoutConstant = 50,
        .WriteTotalTimeoutMultiplier = 10,
    };
    char com_port_path[32];

    snprintf(com_port_path, sizeof(com_port_path)/sizeof(*com_port_path), "\\\\.\\%s", id);
    dev = CreateFileA(com_port_path, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (dev == INVALID_HANDLE_VALUE)
    {
        return false;
    }

    if (GetCommState(dev, &dcb_com_params) == FALSE)
    {
        CloseHandle(dev);
        return false;
    }

    dcb_com_params.BaudRate = CBR_9600;
    dcb_com_params.fParity = FALSE;
    dcb_com_params.fOutxCtsFlow = FALSE;
    dcb_com_params.fOutxDsrFlow = FALSE;
    dcb_com_params.fDtrControl = DTR_CONTROL_DISABLE;
    dcb_com_params.fDsrSensitivity = FALSE;
    dcb_com_params.fTXContinueOnXoff = FALSE;
    dcb_com_params.fOutX = FALSE;
    dcb_com_params.fInX = FALSE;
    dcb_com_params.fErrorChar = FALSE;
    dcb_com_params.fNull = FALSE;
    dcb_com_params.fRtsControl = RTS_CONTROL_DISABLE;
    dcb_com_params.fAbortOnError = FALSE;
    dcb_com_params.ByteSize = 8;
    dcb_com_params.Parity = NOPARITY;
    dcb_com_params.StopBits = ONESTOPBIT;

    if (SetCommState(dev, &dcb_com_params) == FALSE)
    {
        CloseHandle(dev);
        return false;
    }

    if (SetCommTimeouts(dev, &timeouts) == FALSE)
    {
        CloseHandle(dev);
        return false;
    }

    Sleep(10);

    *device = malloc(sizeof(HANDLE));
    if (!*device)
    {
        CloseHandle(dev);
        return false;
    }

    **(HANDLE**)device = dev;

    return true;
}

bool Transport_CloseDevice(TRANSPORT device)
{
    CloseHandle(*(HANDLE*)device);

    free(device);

    return true;
}

bool Transport_Reset(TRANSPORT device)
{
    DCB dcb_com_params =
    {
        .DCBlength = sizeof(dcb_com_params),
    };

    FAILFALSE(GetCommState(*(HANDLE*)device, &dcb_com_params) == TRUE);

    FAILFALSE(EscapeCommFunction(*(HANDLE*)device, SETRTS));

    dcb_com_params.fRtsControl = RTS_CONTROL_ENABLE; //if not then RTS will fall
    FAILFALSE(SetCommState(*(HANDLE*)device, &dcb_com_params) == TRUE);

    return true;
}

bool Transport_Unreset(TRANSPORT device)
{
    DCB dcb_com_params =
    {
        .DCBlength = sizeof(dcb_com_params),
    };

    FAILFALSE(GetCommState(*(HANDLE*)device, &dcb_com_params) == TRUE);

    FAILFALSE(EscapeCommFunction(*(HANDLE*)device, CLRRTS));

    dcb_com_params.fRtsControl = RTS_CONTROL_DISABLE; //if not then RTS will fall
    FAILFALSE(SetCommState(*(HANDLE*)device, &dcb_com_params) == TRUE);

    return true;
}

bool Transport_SetBaudRate(TRANSPORT device, uint32_t baudrate)
{
    DCB dcb_com_params =
    {
        .DCBlength = sizeof(dcb_com_params),
    };

    FAILFALSE(GetCommState(*(HANDLE*)device, &dcb_com_params) == TRUE);

    dcb_com_params.fRtsControl = RTS_CONTROL_DISABLE; //if not then RTS will fall

    switch (baudrate)
    {
    case 9600:
        dcb_com_params.BaudRate = CBR_9600;
        break;

    case 115200:
        dcb_com_params.BaudRate = CBR_115200;
        break;

    default:
        return true;
        break;
    }

    FAILFALSE(SetCommState(*(HANDLE*)device, &dcb_com_params) == TRUE);

    return true;
}

#elif USE_LINUX_TTY

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>

bool Transport_Send(TRANSPORT device, uint8_t* buffer, uint32_t buffer_len)
{
    FAILFALSE(write(*(int*)device, buffer, buffer_len) == buffer_len);

#ifdef LOG_EXCHANGE
    {
        printf("> ");
        size_t i;
        for (i = 0; i < buffer_len; ++i)
        {
            printf("%.2hhX ", buffer[i]);
        }
        printf("\n");
    }
#endif

    return true;
}

bool Transport_Recv(TRANSPORT device, uint8_t* buffer, uint32_t buffer_len)
{
    FAILFALSE(read(*(int*)device, buffer, buffer_len) == buffer_len);

#ifdef LOG_EXCHANGE
    {
        printf("< ");
        size_t i;
        for (i = 0; i < buffer_len; ++i)
        {
            printf("%.2hhX ", buffer[i]);
        }
        printf("\n");
    }
#endif

    return true;
}

bool Transport_GetStats(TRANSPORT device, uint32_t* recv_quenue)
{
    int available;

    FAILFALSE(ioctl(*(int*)device, FIONREAD, &available) == 0);

    *recv_quenue = available;

    return true;
}

bool Transport_OpenDevice(TRANSPORT* device, char* id)
{
    int fd;
    struct termios serial_port_settings;

    fd = open(id, O_RDWR | O_NOCTTY);

    FAILFALSE(fd != -1);

    FAILFALSE(tcgetattr(fd, &serial_port_settings) == 0);

    FAILFALSE(cfsetispeed(&serial_port_settings, B9600) == 0);
    FAILFALSE(cfsetispeed(&serial_port_settings, B9600) == 0);
/*
    serial_port_settings.c_cflag &= ~CRTSCTS;
    serial_port_settings.c_cflag |= CREAD | CLOCAL;
    serial_port_settings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    serial_port_settings.c_iflag &= ~(IXON | IXOFF | IXANY);
*/

    serial_port_settings.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    serial_port_settings.c_cflag &= ~CSIZE;
    serial_port_settings.c_cflag |= CS8;         /* 8-bit characters */
    serial_port_settings.c_cflag &= ~PARENB;     /* no parity bit */
    serial_port_settings.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    serial_port_settings.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    serial_port_settings.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    serial_port_settings.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    serial_port_settings.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    serial_port_settings.c_cc[VMIN] = 1;
    serial_port_settings.c_cc[VTIME] = 1;

    FAILFALSE(tcsetattr(fd, TCSANOW, &serial_port_settings) == 0);

    Sleep(10);

    *device = malloc(sizeof(int));
    if (!*device)
    {
        close(fd);
        return false;
    }

    **(int**)device = fd;

    return true;
}

bool Transport_CloseDevice(TRANSPORT device)
{
    close(*(int*)device);

    free(device);

    return true;
}

bool Transport_Reset(TRANSPORT device)
{
    int param = TIOCM_RTS;

    FAILFALSE(ioctl(*(int*)device, TIOCMBIS, &param) == 0);

    return true;
}

bool Transport_Unreset(TRANSPORT device)
{
    int param = TIOCM_RTS;

    FAILFALSE(ioctl(*(int*)device, TIOCMBIC, &param) == 0);

    return true;
}

bool Transport_SetBaudRate(TRANSPORT device, uint32_t baudrate)
{
    struct termios serial_port_settings;

    FAILFALSE(tcgetattr(*(int*)device, &serial_port_settings) == 0);

    switch (baudrate)
    {
    case 9600:
        FAILFALSE(cfsetispeed(&serial_port_settings, B9600) == 0);
        FAILFALSE(cfsetospeed(&serial_port_settings, B9600) == 0);
        break;

    case 115200:
        FAILFALSE(cfsetispeed(&serial_port_settings, B115200) == 0);
        FAILFALSE(cfsetospeed(&serial_port_settings, B115200) == 0);
        break;

    default:
        return true;
        break;
    }

    FAILFALSE(tcsetattr(*(int*)device, TCSANOW, &serial_port_settings) == 0);

    return true;
}


#else
#error No transport api
#endif
