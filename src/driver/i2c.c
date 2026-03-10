#include "FreeRTOS.h"
#include "task.h"
#include "libopencm3/cm3/common.h"
#include <libopencm3/stm32/f1/memorymap.h>
#include "libopencm3/stm32/f1/i2c.h"
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>

#include "i2c.h"

#define systicks xTaskGetTickCount
diff_ticks(TickType_t early,TickType_t later) {

	if ( later >= early )
		return later - early;
	return ~(TickType_t)0 - early + 1 + later;
}

static const char *i2c_error_msg[] = {
    "I2C clock timeout",
    "I2C address timeout",
    "I2C address NACK",
    "I2C write timeout",
    "I2C read timeout"
};

jmp_buf i2c_exception;

/*********************************************************************
 * Return a character string message for I2C_Fails code
 *********************************************************************/
const char* i2c_error(I2C_errors error_code) {
    int icode = (int)error_code;
    if (icode < 0 || icode >= sizeof(I2C_errors)/sizeof(I2C_errors[0])) {
        return "Unknown I2C error";
    }
    return i2c_error_msg[icode];
}

/*********************************************************************
 * Reset the I2C Peripheral
 *********************************************************************/
static void i2c_reset(uint32_t i2c) {
    switch (i2c) {
        case I2C1:
            rcc_periph_reset_pulse(RST_I2C1);
            break;
        case I2C2:
            rcc_periph_reset_pulse(RST_I2C2);
            break;
        #if defined (I2C3_BASE)
        case I2C3:
            rcc_periph_reset_pulse(RST_I2C3);
            break;
        #endif
        default:
            /* Invalid I2C peripheral, trigger an exception */
            longjmp(i2c_exception, I2C_ck);
            break;
    }
}

/*********************************************************************
 * Configure I2C device for 100 kHz, 7-bit addresses
 *********************************************************************/
void i2c_configure(I2C_control* dev, uint32_t i2c, uint32_t ticks) {
    dev->device = i2c;
    dev->timeout = ticks;

    i2c_peripheral_disable(dev->device); // disable peripheral before configuring
    i2c_reset(dev->device); // reset peripheral to clear any pending conditions
    I2C_CR1(dev->device) &= ~I2C_CR1_STOP; // clear stop bit to avoid generating a stop condition when the peripheral is enabled
    i2c_set_standard_mode(dev->device); // 100 kHz mode
    i2c_set_clock_frequency(dev->device, 36); // APB1 clock is 36 MHz, so set frequency to 36 MHz / 36 = 1 MHz
    i2c_set_trise(dev->device, 36); // maximum rise time is 1000 ns, which corresponds to 36 cycles at 36 MHz
    i2c_set_dutycycle(dev->device, I2C_CCR_DUTY_DIV2); // 50% duty cycle
    i2c_set_ccr(dev->device, 180); // 180 cycles at 36 MHz gives 100 kHz SCL frequency
    i2c_set_own_7bit_slave_address(dev->device, 0x00); // set own address to 0 since we won't be using slave mode
    i2c_peripheral_enable(dev->device); // enable peripheral after configuring
}

/*********************************************************************
 * Return when I2C is not busy
 *********************************************************************/
void i2c_wait_busy(I2C_control* dev) {
    while (I2C_SR2(dev->device) & I2C_SR2_BUSY) 
        taskYIELD(); //I2C is busy, yield to other tasks while waiting
}

/*********************************************************************
 * Start I2C Read/Write Transaction with indicated 7-bit address:
 *********************************************************************/
void i2c_start_addr(I2C_control* dev, uint8_t addr, int rw) {
    TickType_t start_time = systicks();

    i2c_wait_busy(dev); // wait until I2C is not busy before starting a new transaction
	uart_send_string("[LOG] [i2c_start_addr] After checking busy flag\n");
    I2C_SR1(dev->device) &= ~I2C_SR1_AF; // clear address NACK flag before starting transaction
    i2c_clear_stop(dev->device); // clear stop bit to avoid generating a stop condition when the transaction is started
    if (rw == I2C_READ) {
        i2c_enable_ack(dev->device); // enable ACK for read transactions
    }
    i2c_send_start(dev->device); // generate start condition

    // loop until ready:
    while ( !(I2C_SR1(dev->device) & I2C_SR1_SB) /* && (I2C_SR2(dev->device) & (I2C_SR2_MSL | I2C_SR2_BUSY))*/) {
        if (diff_ticks(start_time, systicks()) > dev->timeout) {
            longjmp(i2c_exception, I2C_addr_timeout); // timeout waiting for start condition
        }
        taskYIELD(); // yield to other tasks while waiting
    }
	uart_send_string("[LOG] [i2c_start_addr] After checking SB bit\n");

    // Send address and r/w flag:
    i2c_send_7bit_address(dev->device, addr, rw == I2C_READ ? I2C_READ : I2C_WRITE);

    // Wait until completion, NAK or timeout
	start_time = systicks();
	while ( !(I2C_SR1(dev->device) & I2C_SR1_ADDR) ) {
		if ( I2C_SR1(dev->device) & I2C_SR1_AF ) {
			i2c_send_stop(dev->device);
			(void)I2C_SR1(dev->device);
			(void)I2C_SR2(dev->device); 	// Clear flags
			// NAK Received (no ADDR flag will be set here)
			longjmp(i2c_exception,I2C_addr_nack); 
		}
		if ( diff_ticks(start_time,systicks()) > dev->timeout )
			longjmp(i2c_exception,I2C_addr_timeout); 
		taskYIELD();
	}
	uart_send_string("[LOG] [i2c_start_addr] Address received\n");

    (void)I2C_SR2(dev->device); // clear ADDR flag by reading SR2 after SR1 indicates address sent
}

/*********************************************************************
 * Write one byte of data
 *********************************************************************/
void i2c_write(I2C_control *dev,uint8_t byte) {
	TickType_t t0 = systicks();

	i2c_send_data(dev->device,byte);
	while ( !(I2C_SR1(dev->device) & (I2C_SR1_BTF)) ) {
		if ( diff_ticks(t0,systicks()) > dev->timeout )
			longjmp(i2c_exception,I2C_write_timeout);
		taskYIELD();
	}
}

/*********************************************************************
 * Read one byte of data. Set lastflag=true, if this is the last/only
 * byte being read.
 *********************************************************************/
uint8_t i2c_read(I2C_control *dev,bool lastf) {
	TickType_t t0 = systicks();

	if ( lastf )
		i2c_disable_ack(dev->device);	// Reading last/only byte

	while ( !(I2C_SR1(dev->device) & I2C_SR1_RxNE) ) {
		if ( diff_ticks(t0,systicks()) > dev->timeout )
			longjmp(i2c_exception,I2C_read_timeout);
		taskYIELD();
	}
	
	return i2c_get_data(dev->device);
}

/*********************************************************************
 * Write one byte of data, then initiate a repeated start for a
 * read to follow.
 *********************************************************************/
void i2c_write_restart(I2C_control *dev,uint8_t byte,uint8_t addr) {
	TickType_t t0 = systicks();

	taskENTER_CRITICAL();
	i2c_send_data(dev->device,byte);
	// Must set start before byte has written out
	i2c_send_start(dev->device);
	taskEXIT_CRITICAL();

	// Wait for transmit to complete
	while ( !(I2C_SR1(dev->device) & (I2C_SR1_BTF)) ) {
		if ( diff_ticks(t0,systicks()) > dev->timeout )
			longjmp(i2c_exception,I2C_write_timeout);
		taskYIELD();
	}

	// Loop until restart ready:
	t0 = systicks();
        while ( !((I2C_SR1(dev->device) & I2C_SR1_SB) 
	  && (I2C_SR2(dev->device) & (I2C_SR2_MSL|I2C_SR2_BUSY))) ) {
		if ( diff_ticks(t0,systicks()) > dev->timeout )
			longjmp(i2c_exception,I2C_addr_timeout);
		taskYIELD();
	}

	// Send Address & Read command bit
	i2c_send_7bit_address(dev->device,addr,I2C_READ);

	// Wait until completion, NAK or timeout
	t0 = systicks();
	while ( !(I2C_SR1(dev->device) & I2C_SR1_ADDR) ) {
		if ( I2C_SR1(dev->device) & I2C_SR1_AF ) {
			i2c_send_stop(dev->device);
			(void)I2C_SR1(dev->device);
			(void)I2C_SR2(dev->device); 	// Clear flags
			// NAK Received (no ADDR flag will be set here)
			longjmp(i2c_exception,I2C_addr_nack); 
		}
		if ( diff_ticks(t0,systicks()) > dev->timeout )
			longjmp(i2c_exception,I2C_addr_timeout); 
		taskYIELD();
	}

	(void)I2C_SR2(dev->device);		// Clear flags
}