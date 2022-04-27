#include "QN9020.h"
#include <stdbool.h>

void setup_peripherials();
void send_line(char *buffer);
int get_line(char *buffer, int buffer_len);
int parse_line(char *command, uint8_t *mode, uint32_t *address, uint32_t *data);
void transmit_byte_as_ascii_space(uint8_t byte);
void read_flash(uint32_t addr, uint32_t *pBuf, uint32_t nByte);

int main (void)
{
    setup_peripherials();

    char command[32];
    while(1)
    {
        if(!get_line(command, sizeof(command)))
        {
            continue;
        }

        uint8_t  mode;//0 - read, 1 - write
        uint32_t address;//address
        uint32_t data;//count of bytes to read or data to write

        if(!parse_line(command, &mode, &address, &data))
        {
            continue;
        }

        if(mode == 1)
        {
            //write
            *((uint32_t*)address) = data;
             send_line("OK\n");
        }
        else if(mode == 0)
        {
            //read
            uint32_t *ptr = (uint32_t*)address;
            int i;
            uint32_t word;
            for(i=0; i<data>>2; ++i)
            {
                word = ptr[i];
                transmit_byte_as_ascii_space((word >> 0) & 0xFF);
                transmit_byte_as_ascii_space((word >> 8) & 0xFF);
                transmit_byte_as_ascii_space((word >> 16) & 0xFF);
                transmit_byte_as_ascii_space((word >> 24) & 0xFF);
            }

            send_line("\n");
        }
	else if(mode == 2)
	{
            if(data <= 0x100)
            {
                int i;
                uint8_t buffer[0x100];
                read_flash(address, (uint32_t*)buffer, data);

                for(i=0; i<data; ++i)
                {
                     transmit_byte_as_ascii_space(buffer[i]);
                }
                send_line("\n");
            }
	}
    }
}

void transmit_byte_as_ascii_space(uint8_t byte)
{
    char tb[4];
    char alphabet[] = "0123456789ABCDEF";

    tb[0] = ' ';
    tb[1] = ' ';
    tb[2] = ' ';
    tb[3] = '\0';

    tb[0] = alphabet[(byte >> 4) & 0x0F];
    tb[1] = alphabet[(byte >> 0) & 0x0F];
    send_line(tb);
}

unsigned char char_to_nibble(char c)
{
    if (c >= '0' && c <= '9')
    {
        return c - '0';
    }
    else if (c >= 'a' && c <= 'f')
    {
        return c - 'a' + 0x0a;
    }
    else if (c >= 'A' && c <= 'F')
    {
        return c - 'A' + 0x0a;
    }
    else
    {
        return 0xff;
    }
}

int parse_line(char* command, uint8_t* mode, uint32_t* address, uint32_t* data)
{
    //command is like
    //r00000000.11111111 - read 0x11111111 bytes from address 0x00000000
    //w00000000.11111111 - write word 0x11111111 to address 0x00000000
    //f00000000.11111111 - read 0x11111111 bytes from flash address 0x00000000

    int pos = 0;
    if (command[pos] == 'r' || command[pos] == 'R')
    {
        *mode = 0;
    }
    else if (command[pos] == 'w' || command[pos] == 'W')
    {
        *mode = 1;
    }
    else if (command[pos] == 'f' || command[pos] == 'F')
    {
        *mode = 2;
    }
    else
    {
        return 0;
    }

    pos++;

    int sz;
    uint8_t orb;

    *address = 0;
    sz = 0;
    while (sz < 8)
    {
        orb = char_to_nibble(command[pos]);
        if (orb > 0x0f) break;

        *address <<= 4;
        *address |= orb;

        sz++;
        pos++;
    }

    if (sz == 0)
    {
        return 0;
    }

    if (command[pos] != '.')
    {
        return 0;
    }
    pos++;

    *data = 0;
    sz = 0;

    while (sz < 8)
    {
        orb = char_to_nibble(command[pos]);
        if (orb > 0x0f) break;

        *data <<= 4;
        *data |= orb;

        sz++;
        pos++;
    }

    if (command[pos] != '\n' && command[pos] != '\0')
    {
        return 0;
    }

    return 1;
}
int get_line(char *buffer, int buffer_len)
{
    int i=0;
    while(i<buffer_len-1)
    {
        while(!(QN_UART0->FLAG & UART_MASK_RX_IF));
        buffer[i]=QN_UART0->RXD;
        if(buffer[i] == '\n')
        {
            buffer[i+1] = '\0';
            return 1;
        }
        i++;
    }
    return 0;
}

void send_line(char *buffer)
{
    int i = 0;
    while(buffer[i])
    {
        while(!(QN_UART0->FLAG & UART_MASK_TX_IF));
        QN_UART0->TXD = buffer[i++];
    }
}

void setup_peripherials()
{
    QN_SYSCON->IVREF_X32 = SYSCON_MASK_DVDD12_SW_EN;

    QN_SYSCON->CRSS = 0xff7a0000;

    QN_SYSCON->CMDCR = 0;
    QN_SYSCON->CMDCR |= SYSCON_MASK_APB_DIV_BYPASS;  //APB bypass divider
    QN_SYSCON->CMDCR |= SYSCON_MASK_USART0_DIV_BYPASS;  //UART0 bypass divider

    QN_SYSCON->PMCR0 = P00_UART0_TXD_PIN_CTRL | P17_UART0_RXD_PIN_CTRL;//UART0 tx+rx to p0_0+p1_7
    QN_SYSCON->PPCR0 = 0x80000002;//pull-up P0_0 P1_7

    QN_SYSCON->CRSC = SYSCON_MASK_GATING_UART0 | SYSCON_MASK_GATING_SPI_AHB;//enable UART0 clock

    QN_UART0->BAUD = 0x00000416;//115200
    QN_UART0->CR = UART_MASK_UART_EN | UART_MASK_TX_EN | UART_MASK_RX_EN | UART_MASK_BIT_ORDER | UART_MASK_LEVEL_INV | UART_MASK_OVS;
}


#define RD_FLASH_ST_CMD         0x0500

void sf_ctrl_SetCRWithMask(QN_SF_CTRL_TypeDef *SF_CTRL, uint32_t mask, uint32_t value)
{
    SF_CTRL->CTRL_STAT = ((SF_CTRL->CTRL_STAT) & (~mask)) | (mask & value);
}

bool is_flash_busy(void)
{
    uint32_t status;

    sf_ctrl_SetCRWithMask(QN_SF_CTRL, SF_CTRL_MASK_RD_STAT_CMD, RD_FLASH_ST_CMD);

    status = QN_SF_CTRL->FLASH_SR;

    return ((status & 0x01) == 0x1);
}


void read_flash(uint32_t addr, uint32_t *pBuf, uint32_t nByte)
{
    addr += QN_FLASH_BASE;   //get the data register address of flash control register
    while(is_flash_busy());  //wait the flash is free.
    QN_SF_CTRL->DATA_LEN = nByte;
    nByte >>= 2;               //nByte must is 4 integer times
    while (nByte--)
    {
        *pBuf++ = *(uint32_t *)addr;
        addr += 4;
    }
}
