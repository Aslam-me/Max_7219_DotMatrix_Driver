#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/delay.h>
/******/
#include <linux/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/spi/spi.h>
#include "cp437font.h"
#include "Dot_Max7219_Driver.h"
/***************/

/* Meta Information */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mohammad Aslam");
MODULE_DESCRIPTION("A driver to Display text at Dot Matrix Display");

/* Variables for device and device class */
static dev_t DotMatrixDisplay_device_nr;
static struct class *DotMatrixDisplay_class;
static struct cdev DotMatrixDisplay_device;
static char DotMatrixDisplay_buffer[255];

#define DRIVER_NAME "DotMatrixDisplay"
#define DRIVER_CLASS "DotMatrixDisplay_class"

/***********************************************************/

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

static uint8_t spiBPW = 8;
static uint32_t spiSpeed = 500000;
static uint16_t spiDelay = 0;

#define SPI_BUS_NUM 0
static struct spi_device *Max7219_DotMatrixDriver;

int SPI_Init(void)
{
	struct spi_master *master;

	/* Parameters for SPI device */
	struct spi_board_info spi_device_info = {
		.modalias = "Max7219_DotMatrix",
		.max_speed_hz = spiSpeed,
		.bus_num = SPI_BUS_NUM,
		.chip_select = SPI_BUS_NUM,
		.mode = 3,
	};

	/* Get access to spi bus */
	master = spi_busnum_to_master(SPI_BUS_NUM);
	/* Check if we could get the master */
	if(!master) {
		printk("There is no spi bus with Nr. %d\n", SPI_BUS_NUM);
		return -1;
	}

	/* Create new SPI device */
	Max7219_DotMatrixDriver = spi_new_device(master, &spi_device_info);
	if(!Max7219_DotMatrixDriver) {
		printk("Could not create device!\n");
		return -1;
	}

	Max7219_DotMatrixDriver -> bits_per_word = spiBPW;

	/* Setup the bus for device's parameters */
	if(spi_setup(Max7219_DotMatrixDriver) != 0){
		printk("Could not change bus setup!\n");
		spi_unregister_device(Max7219_DotMatrixDriver);
		return -1;
	}

	
	return 0;
}

void SPI_writeBytes(uint8_t* data, uint8_t Lenght)
{
  spi_write(Max7219_DotMatrixDriver, data, Lenght);
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


int Initialise_Display(void)
{
  uint8_t CMD_Count, Reg, data;
  int ret;
  
  const uint8_t Init_CMD[5][2] = {{MAX7219_REG_SCANLIMIT,    0x7},
                                  {MAX7219_REG_DECODEMODE,   0x0},
                                  {MAX7219_REG_SHUTDOWN,     0x1},
                                  {MAX7219_REG_DISPLAYTEST,  0x0},
                                  {MAX7219_REG_INTENSITY,    0x7}};
  
  ret = SPI_Init();
  
  //Only execute if SPI can be intialised
  if(ret >= 0)
  {
    for(CMD_Count = 0; CMD_Count < TOTAL_INITIAL_CMD_COUNT; CMD_Count++)
    {
      Reg =   Init_CMD[CMD_Count][0];
      data =  Init_CMD[CMD_Count][1];
      Send_Command_To_All_Dispaly(Reg,data);
    }
    
    clearDisplay();
  }
  
  return ret;

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

uint8_t Display_data_for_4_Display(uint32_t* In_Data)
{
  uint8_t Tx_Data[NUMBER_OF_DISPLAY*2];
  uint8_t Display_Count, Row_count;
  
  
  for(Row_count =1; Row_count<9;Row_count++)
  {
    for(Display_Count = 0; Display_Count <NUMBER_OF_DISPLAY ; Display_Count++)
    {
      Tx_Data[Display_Count*2]    = Row_count;
      Tx_Data[(Display_Count*2)+1] = In_Data[Row_count-1]>>((NUMBER_OF_DISPLAY-Display_Count-1)*8);
    }
    
    SPI_writeBytes(Tx_Data, (NUMBER_OF_DISPLAY*2));
  }
  return 0;
}

void Scroll_text(const uint8_t* In_ptr)
{
  uint8_t i,j,k,t,r;
  uint8_t Char_data[8];
  //uint8_t Temp_Char_data[8];
  uint32_t Data[8] = {0};
  
  printk("Text Receive to display : %s",In_ptr);
  
  k=0;
  r=0;
  Rotate_Font(cp437_font[In_ptr[r]],Char_data);

  do
  {
    
    
    for(i=0;i<32;i++)
    {
      for(j=0;j<8;j++)
      {
        Data[j] = Data[j] << 1;
        t=(Char_data[j] & (1<<(7-k)))? 1: 0;
        Data[j] |= t<<0;
        
        //Data[j] |= (1<<0);
      }
      k++;
      if(k == 8)
      {
        k=0;
        r++;
        Rotate_Font(cp437_font[In_ptr[r]],Char_data);
      }
      Display_data_for_4_Display(Data);
      msleep(30);
      
    }
  }while(In_ptr[r]!=NULL);

}


/************************************************************/

/**
 * @brief Write data to buffer
 */
static ssize_t driver_write(struct file *File, const char *user_buffer, size_t count, loff_t *offs) {
	int to_copy, not_copied, delta, i;

	/* Get amount of data to copy */
	to_copy = min(count, sizeof(DotMatrixDisplay_buffer));

	/* Copy data to user */
	not_copied = copy_from_user(DotMatrixDisplay_buffer, user_buffer, to_copy);

	/* Calculate data */
	delta = to_copy - not_copied;

	/* Set the new data to the display */

	Scroll_text(DotMatrixDisplay_buffer[i]);

	return delta;
}

/**
 * @brief This function is called, when the device file is opened
 */
static int driver_open(struct inode *device_file, struct file *instance) {
	printk("dev_nr - open was called!\n");
	return 0;
}

/**
 * @brief This function is called, when the device file is opened
 */
static int driver_close(struct inode *device_file, struct file *instance) {
	printk("dev_nr - close was called!\n");
	return 0;
}

static struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = driver_open,
	.release = driver_close,
	.write = driver_write
};

/**
 * @brief This function is called, when the module is loaded into the kernel
 */
static int __init ModuleInit(void) {
	int i, ret;
	
	/* Initialize DotMatrixDisplay */
	ret = Initialise_Display();
	
	//If error already found don't move further 
	if(ret < 0)
	  return ret;

	/* Allocate a device nr */
	if( alloc_chrdev_region(&DotMatrixDisplay_device_nr, 0, 1, DRIVER_NAME) < 0) {
		printk("Device Nr. could not be allocated!\n");
		return -1;
	}
	printk("read_write - Device Nr. Major: %d, Minor: %d was registered!\n", DotMatrixDisplay_device_nr >> 20, DotMatrixDisplay_device_nr && 0xfffff);

	/* Create device class */
	if((DotMatrixDisplay_class = class_create(THIS_MODULE, DRIVER_CLASS)) == NULL) {
		printk("Device class can not e created!\n");
		goto ClassError;
	}

	/* create device file */
	if(device_create(DotMatrixDisplay_class, NULL, DotMatrixDisplay_device_nr, NULL, DRIVER_NAME) == NULL) {
		printk("Can not create device file!\n");
		goto FileError;
	}

	/* Initialize device file */
	cdev_init(&DotMatrixDisplay_device, &fops);

	/* Regisering device to kernel */
	if(cdev_add(&DotMatrixDisplay_device, DotMatrixDisplay_device_nr, 1) == -1) {
		printk("DotMatrix-driver - Registering of device to kernel failed!\n");
		goto AddError;
	}

	
	
	//Test
	Scroll_text("Hello, Kernel!");

	return 0;

AddError:
	device_destroy(DotMatrixDisplay_class, DotMatrixDisplay_device_nr);
FileError:
	class_destroy(DotMatrixDisplay_class);
ClassError:
	unregister_chrdev_region(DotMatrixDisplay_device_nr, 1);
	return -1;
}

/**
 * @brief This function is called, when the module is removed from the kernel
 */
static void __exit ModuleExit(void) {
	
	clearDisplay();
	
	if(Max7219_DotMatrixDriver)
		spi_unregister_device(Max7219_DotMatrixDriver);
	
	cdev_del(&DotMatrixDisplay_device);
	device_destroy(DotMatrixDisplay_class, DotMatrixDisplay_device_nr);
	class_destroy(DotMatrixDisplay_class);
	unregister_chrdev_region(DotMatrixDisplay_device_nr, 1);
	printk("Goodbye, Kernel\n");
}

module_init(ModuleInit);
module_exit(ModuleExit);


