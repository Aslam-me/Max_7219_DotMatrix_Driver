#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <fcntl.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include "cp437font.h"
#include "Dot_Max7219_Driver.h"

#define MAX7219_REG_NOOP 0x00
#define MAX7219_REG_DIGIT0 0x01
#define MAX7219_REG_DIGIT1 0x02
#define MAX7219_REG_DIGIT2 0x03
#define MAX7219_REG_DIGIT3 0x04
#define MAX7219_REG_DIGIT4 0x05
#define MAX7219_REG_DIGIT5 0x06
#define MAX7219_REG_DIGIT6 0x07
#define MAX7219_REG_DIGIT7 0x08
#define MAX7219_REG_DECODEMODE 0x09
#define MAX7219_REG_INTENSITY 0x0A
#define MAX7219_REG_SCANLIMIT 0x0B
#define MAX7219_REG_SHUTDOWN 0x0C
#define MAX7219_REG_DISPLAYTEST 0x0F

#define CMD_WRITE 0x40

#define NUMBER_OF_DISPLAY 0x4
#define TOTAL_INITIAL_CMD_COUNT 0x5

int spi_fd;
struct sigaction act;
static char *spiDevice = "/dev/spidev0.0";
static uint8_t spiBPW = 8;
static uint32_t spiSpeed = 500000;
static uint16_t spiDelay = 0;

int SPI_Open(char* dev)
{
  if((spi_fd = open(dev, O_RDWR)) < 0)
  {
    printf("error opening %s\n",dev);
    return -1;
  }
  return 0;
}

void SPI_writeBytes(uint8_t* data, uint8_t Lenght)
{
  uint8_t spiBufTx [8];
  uint8_t spiBufRx [8];
  uint8_t count;
  struct spi_ioc_transfer spi;
  memset (&spi, 0, sizeof(spi));
  memcpy (spiBufTx, data, Lenght);
  
  printf("SPI Data : ");
  for(count =0; count<Lenght; count++)
  {
    printf(" %x",spiBufTx[count]);
  }
  printf("\n");
  
  spi.tx_buf =(unsigned long)spiBufTx;
  spi.rx_buf =(unsigned long)spiBufRx;
  spi.len = Lenght;
  
  spi.delay_usecs = spiDelay;
  spi.speed_hz = spiSpeed;
  spi.bits_per_word = spiBPW;
  ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi);
}

void Send_Command_To_All_Dispaly(uint8_t Reg, uint8_t data)
{
  uint8_t Tx_Data[NUMBER_OF_DISPLAY*2];
  uint8_t Display_Count;
  
  

  for(Display_Count = 0; Display_Count <NUMBER_OF_DISPLAY ; Display_Count++)
  {
    Tx_Data[Display_Count*2]    = Reg;
    Tx_Data[(Display_Count*2)+1] = data;
  }
  
  SPI_writeBytes(Tx_Data, (NUMBER_OF_DISPLAY*2));

}

void clearDisplay(void)
{

  uint8_t CMD_Count, Reg, data;
  
  
  for(CMD_Count = 1; CMD_Count < 9; CMD_Count++)
  {
    Reg =   CMD_Count;
    data =  0x0;
    Send_Command_To_All_Dispaly(Reg,data);
  }
  
  //Send_Command_To_All_Dispaly(MAX7219_REG_SHUTDOWN, 0x1);
}


void initialiseDisplay(void)
{
  uint8_t CMD_Count, Reg, data;
  
  const uint8_t Init_CMD[5][2] = {{MAX7219_REG_SCANLIMIT,    0x7},
                                  {MAX7219_REG_DECODEMODE,   0x0},
                                  {MAX7219_REG_SHUTDOWN,     0x1},
                                  {MAX7219_REG_DISPLAYTEST,  0x0},
                                  {MAX7219_REG_INTENSITY,    0x7}};
  
  SPI_Open(spiDevice);
  
  for(CMD_Count = 0; CMD_Count < TOTAL_INITIAL_CMD_COUNT; CMD_Count++)
  {
    Reg =   Init_CMD[CMD_Count][0];
    data =  Init_CMD[CMD_Count][1];
    Send_Command_To_All_Dispaly(Reg,data);
  }
  
  clearDisplay();

}

void Rotate_Font(const uint8_t* In_data, uint8_t* Out_data)
{
  uint8_t i,j,Cur_row;
  
  for(i=0; i<8; i++)
    Out_data[i] = 0x0;
  
  for(i=0; i<8; i++)
  {
    Cur_row = In_data[i];
    for(j=0;j<8;j++)
    {
      Out_data[j] |= ((Cur_row & (1<<j)) >> j) << (7-i);
    }
  }
}

void Display_Test(uint8_t Test_font)
{
  uint8_t Tx_Data[NUMBER_OF_DISPLAY*2];
  uint8_t Display_Count, Row_count,i;
  
  uint8_t Font_to_print[NUMBER_OF_DISPLAY][8];
  
  for(i = 0; i<NUMBER_OF_DISPLAY; i++)
    Rotate_Font(cp437_font[Test_font+i],Font_to_print[i]);
  
  for(Row_count =1; Row_count<9;Row_count++)
  {
    for(Display_Count = 0; Display_Count <NUMBER_OF_DISPLAY ; Display_Count++)
    {
      Tx_Data[Display_Count*2]    = Row_count;
      Tx_Data[(Display_Count*2)+1] = Font_to_print[Display_Count][Row_count-1];
    }
    
    SPI_writeBytes(Tx_Data, (NUMBER_OF_DISPLAY*2));
  }
}

int main()
{
  uint8_t i = 48;
  initialiseDisplay();
  //Display_Test(65);
  //clearDisplay(&header);

  while (1)
  {

    clearDisplay();
    Display_Test(i);
    sleep(2);
    i+=4;
  }

  return 0;
}

