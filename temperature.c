/*!
 *  @brief Example shows basic application to configure and read the temperature.
 */

#include <stdio.h>
#include <stdint.h>
#include "bmp280.h"
#include <sys/ioctl.h>
#include <stdlib.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <time.h>

void delay_ms(uint32_t period_ms);

int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t spi_reg_write(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t spi_reg_read(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);

 void print_rslt(const char api_name[], int8_t rslt);

//////
int g_i2cFid; // I2C Linux device handle

// open the Linux device
void i2cOpen()
{
  g_i2cFid = open("/dev/i2c-8", O_RDWR);
  if (g_i2cFid < 0) {
    perror("i2cOpen");
    exit(1);
  }
  else
  {
      printf("I2C geöffnet\n\r");
      printf("Device_Nr.:%i\n\r", g_i2cFid);
      
  }
}

// set the I2C slave address for all subsequent I2C device transfers
void i2cSetAddress(int address)
{
  if (ioctl(g_i2cFid, I2C_SLAVE, address) < 0) {
    perror("i2cSetAddress");
    exit(1);
  }
  else
  {
      printf("I2C-Adresse: 0x%x\n\r", address);
  }
}

/////// Pit
// Device ID 0x58 auslesen

void checkID(void)
{
	int wert;
    uint8_t reg[1];
    reg[0]=0xd0;
    write(g_i2cFid, reg, 1);
    wert = read(g_i2cFid, reg, 1);
    //read(g_i2cFid, reg_data, length)
    
    //wert = bus_read( 0x76, 0xd0, reg, 1)

	printf("ID: 0x%x\n\r", reg[0]);
}

/*
 * Read operation in either I2C or SPI
 *
 * param[in]        dev_addr        I2C or SPI device address
 * param[in]        reg_addr        register address
 * param[out]       reg_data    pointer to the memory to be used to store
 *                                  the read data
 * param[in]        length        number of bytes to be read
 *
 * return          result of the bus communication function
 */
//int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
int8_t bus_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data,
    uint16_t length)
{
  int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

  uint8_t reg[1];
  reg[0]=reg_addr;

    printf("in_read\n\r");
    printf("Addr:%x\n\r", i2c_addr);
    printf("RegAddr:%x\n\r", reg_addr);
    
    printf("lenght:%i\n\r", length);
    
    
  if (write(g_i2cFid, reg, 1) != 1) {
    perror("user_i2c_read_reg Schreibfehler");
    printf("reg:%i\n\r", reg[0]);
    rslt = 1;
  }
  else
  {
      printf("ERFOLG! Write in Reg 1\n\r");
      printf("length:%i\n\r", length);
  }

  if (read(g_i2cFid, reg_data, length) != length) {
    perror("user_i2c_read_data");
      printf("length_reg_data:%i\n\r", length);
    rslt = 1;
  }
    /*else
    {
        printf("ERFOLG! Write Reg Data\n\r");
    }
   */
  return rslt;
}

/*
 * Write operation in either I2C or SPI
 *
 * param[in]        dev_addr        I2C or SPI device address
 * param[in]        reg_addr        register address
 * param[in]        reg_data.       pointer to the data to be written
 * param[in]        length        number of bytes to be written
 *
 * return          result of the bus communication function
 */
int8_t bus_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data,
    uint16_t length)
{
  printf("in_write\n\r\n");
  int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

  uint8_t reg[16];  //[16];
  reg[0]=reg_addr;
  int i;

  for (i=1; i<length +1; i++)
{
    reg[i] = reg_data[i-1];
    delay_ms(500);
}

printf("absolut_vor_write\n\r\n");
printf("Device:%i\n\r", g_i2cFid);
printf("length:%i\n\r", length);
printf("reg[0]:%i\n\r", reg[0]);
printf("reg[1]:%i\n\r", reg[1]);
printf("reg[2]:%i\n\r", reg[2]);
printf("reg[3]:%i\n\r", reg[3]);
printf("reg[4]:%i\n\r", reg[4]);
printf("Zähler i:%i\n\r", i);

  // Device Handle von g_i2cFid = 3
  if (write(g_i2cFid, reg, length+1) != length+1) {
      printf("Ultimativ nach write\n\r");
    perror("user_i2c_write");
    rslt = 1;
    exit(1);
  }
    else
    {
        printf("Write erfolgreich!\n\r");
    }

  return rslt;
}

/*
 * System specific implementation of sleep function
 *
 * param[in]       t_ms    time in milliseconds
 *
 * return          none
 */
void delay_ms(uint32_t t_ms)
{
  struct timespec ts;
  ts.tv_sec = t_ms / 1000;
  /* mod because nsec must be in the range 0 to 999999999 */
  ts.tv_nsec = (t_ms % 1000) * 1000000L;
    //printf("%lu\n\r", &ts);
  nanosleep(&ts, NULL);
}
//////

int main(void)
{
    int8_t rslt;
    struct bmp280_dev bmp;
    struct bmp280_config conf;
    struct bmp280_uncomp_data ucomp_data;
    int32_t temp32;
    double temp;

    delay_ms(1000);
    // I2C öffnen
    i2cOpen();
    delay_ms(1000);
    // I2C Adresse
    i2cSetAddress(0x76);
    // ID auslesen
    delay_ms(1000);
    checkID();
    delay_ms(1000);
    printf("Nach delay\n\r");
    
    /* Map the delay function pointer with the function responsible for implementing the delay */
    bmp.delay_ms = delay_ms;

    /* Assign device I2C address based on the status of SDO pin (GND for PRIMARY(0x76) & VDD for SECONDARY(0x77)) */
    bmp.dev_id = BMP280_I2C_ADDR_PRIM;

    /* Select the interface mode as I2C */
    bmp.intf = BMP280_I2C_INTF;

    /* Map the I2C read & write function pointer with the functions responsible for I2C bus transfer */
    bmp.read = bus_read; //i2c_reg_read;
    bmp.write = bus_write; //i2c_reg_write;

    /* To enable SPI interface: comment the above 4 lines and uncomment the below 4 lines */
    /*
     * bmp.dev_id = 0;
     * bmp.read = spi_reg_read;
     * bmp.write = spi_reg_write;
     * bmp.intf = BMP280_SPI_INTF;
     */
    printf("Vor checkID()\n\r");
    checkID();
    rslt = bmp280_init(&bmp);
    checkID();
    print_rslt(" bmp280_init status", rslt);
    printf("Nach bmp280_init\n\r");
    checkID();


    /* Always read the current settings before writing, especially when
     * all the configuration is not modified
     */
    printf("Vor bmp280_get_config status\n\r");
    rslt = bmp280_get_config(&conf, &bmp);
    print_rslt(" bmp280_get_config status", rslt);
    printf("Nach bmp280_get_config status\n\r");

    /* configuring the temperature oversampling, filter coefficient and output data rate */
    /* Overwrite the desired settings */
    conf.filter = BMP280_FILTER_COEFF_2;

    /* Temperature oversampling set at 4x */
    conf.os_temp = BMP280_OS_4X;

    /* Pressure over sampling none (disabling pressure measurement) */
    conf.os_pres = BMP280_OS_NONE;

    /* Setting the output data rate as 1HZ(1000ms) */
    conf.odr = BMP280_ODR_1000_MS;
    rslt = bmp280_set_config(&conf, &bmp);
    print_rslt(" bmp280_set_config status", rslt);

    /* Always set the power mode after setting the configuration */
    rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);
    print_rslt(" bmp280_set_power_mode status", rslt);
    while (1)
    {
        /* Reading the raw data from sensor */
        rslt = bmp280_get_uncomp_data(&ucomp_data, &bmp);

        /* Getting the 32 bit compensated temperature */
        rslt = bmp280_get_comp_temp_32bit(&temp32, ucomp_data.uncomp_temp, &bmp);

        /* Getting the compensated temperature as floating point value */
        rslt = bmp280_get_comp_temp_double(&temp, ucomp_data.uncomp_temp, &bmp);
        printf("UT: %ld, T32: %ld, T: %f \r\n", ucomp_data.uncomp_temp, temp32, temp);

        /* Sleep time between measurements = BMP280_ODR_1000_MS */
        bmp.delay_ms(5000); //(1000);
    }

    return 0;
}

/*!
 *  @brief Function that creates a mandatory delay required in some of the APIs such as "bmg250_soft_reset",
 *      "bmg250_set_foc", "bmg250_perform_self_test"  and so on.
 *
 *  @param[in] period_ms  : the required wait time in milliseconds.
 *  @return void.
 *
 */
////  ersetzt durch _sleep

/*!
 *  @brief Function for writing the sensor's registers through I2C bus.
 *
 *  @param[in] i2c_addr : sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[in] reg_data : Pointer to the data buffer whose value is to be written.
 *  @param[in] length   : No of bytes to write.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

    /* Implement the I2C write routine according to the target machine. */
    return -1;
}

/*!
 *  @brief Function for reading the sensor's registers through I2C bus.
 *
 *  @param[in] i2c_addr : Sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] length   : No of bytes to read.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

    /* Implement the I2C read routine according to the target machine. */
    return -1;
}

/*!
 *  @brief Function for writing the sensor's registers through SPI bus.
 *
 *  @param[in] cs           : Chip select to enable the sensor.
 *  @param[in] reg_addr     : Register address.
 *  @param[in] reg_data : Pointer to the data buffer whose data has to be written.
 *  @param[in] length       : No of bytes to write.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t spi_reg_write(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

    /* Implement the SPI write routine according to the target machine. */
    return -1;
}

/*!
 *  @brief Function for reading the sensor's registers through SPI bus.
 *
 *  @param[in] cs       : Chip select to enable the sensor.
 *  @param[in] reg_addr : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] length   : No of bytes to read.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t spi_reg_read(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

    /* Implement the SPI read routine according to the target machine. */
    return -1;
}

/*!
 *  @brief Prints the execution status of the APIs.
 *
 *  @param[in] api_name : name of the API whose execution status has to be printed.
 *  @param[in] rslt     : error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
void print_rslt(const char api_name[], int8_t rslt)
{
    if (rslt != BMP280_OK)
    {
        printf("%s\t", api_name);
        if (rslt == BMP280_E_NULL_PTR)
        {
            printf("Error [%d] : Null pointer error\r\n", rslt);
        }
        else if (rslt == BMP280_E_COMM_FAIL)
        {
            printf("Error [%d] : Bus communication failed\r\n", rslt);
        }
        else if (rslt == BMP280_E_IMPLAUS_TEMP)
        {
            printf("Error [%d] : Invalid Temperature\r\n", rslt);
        }
        else if (rslt == BMP280_E_DEV_NOT_FOUND)
        {
            printf("Error [%d] : Device not found\r\n", rslt);
        }
        else
        {
            /* For more error codes refer "*_defs.h" */
            printf("Error [%d] : Unknown error code\r\n", rslt);
        }
    }
}
