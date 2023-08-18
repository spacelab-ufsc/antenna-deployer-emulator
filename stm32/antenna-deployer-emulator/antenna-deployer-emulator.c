#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/assert.h>

void i2c_slave_init(uint8_t ownaddress);

static uint8_t rx_buffer[2];
static uint8_t tx_buffer[2];
static uint8_t *rx_pos;
static uint8_t *tx_pos;

int main(void)
{
   //set STM32 to 72 MHz
   rcc_clock_setup_in_hse_8mhz_out_72mhz();

   // Enable GPIOC clock
   rcc_periph_clock_enable(RCC_GPIOC);
   //Set GPIO13 (inbuild led connected) to 'output push-pull'
   gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
                 GPIO13);

   //switch led off
   gpio_set(GPIOC, GPIO13);

   //initialize i2c slave
   i2c_slave_init(0x32);

   while( 1 )
     {
        /* Check for a command */
        if (rx_buffer[0] == 0xC0) //Read temperature command
        {
            gpio_toggle(GPIOC, GPIO13);
            tx_buffer[0] = 0x0A;
            tx_buffer[1] = 0xBC;

            rx_buffer[0] = 0x00;
            rx_buffer[1] = 0x00;
        }
        else if (rx_buffer[0] == 0xA2) //Report antenna deployment activation count
        {
            gpio_toggle(GPIOC, GPIO13);

            rx_buffer[0] = 0x00;
            rx_buffer[1] = 0x00;
        }
        
   return 0;
   }
}   

void i2c_slave_init(uint8_t ownaddress)
{
   /* Enables clock for gpio_b and i2c_1 modules */
   rcc_periph_clock_enable(RCC_GPIOB);
   rcc_periph_clock_enable(RCC_I2C1);

   /* Enables Interruptions for i2c_1 module */
   nvic_enable_irq(NVIC_I2C1_EV_IRQ);

   /* configure i2c pins */
   gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                 GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO_I2C1_SDA); //PB7
   gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                 GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO_I2C1_SCL); //PB6

   i2c_reset(I2C1);
   i2c_peripheral_disable(I2C1);

   i2c_set_speed(I2C1, i2c_speed_sm_100k, I2C_CR2_FREQ_36MHZ);
   i2c_set_own_7bit_slave_address(I2C1, ownaddress);
   i2c_enable_interrupt(I2C1, I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN);
   i2c_peripheral_enable(I2C1);

   i2c_enable_ack(I2C1);
}

//i2c1 event ISR
void i2c1_ev_isr(void)
{
   uint32_t sr1, sr2;

   sr1 = I2C_SR1(I2C1);

   // Address matched (Slave)
   if (sr1 & I2C_SR1_ADDR)
     {
        // Start of a transaction
        rx_pos = rx_buffer;
        tx_pos = tx_buffer;

        //Clear the ADDR sequence by reading SR2.
        sr2 = I2C_SR2(I2C1);
        (void) sr2;

        //gpio_toggle(GPIOC, GPIO13);
     }
   // Receive buffer not empty
   else if (sr1 & I2C_SR1_RxNE)
     {
        //read bytes from slave
        *(rx_pos++) = i2c_get_data(I2C1);

     }
   // Transmit buffer empty & Data byte transfer not finished
   else if ((sr1 & I2C_SR1_TxE) && !(sr1 & I2C_SR1_BTF))
     {
        //send data to master in MSB order
        i2c_send_data(I2C1, *tx_pos);
        *tx_pos++ = 0x00; /*Clears after sending */
     }
   // done by master by sending STOP
   //this event happens when slave is in Recv mode at the end of communication
   else if (sr1 & I2C_SR1_STOPF)
     {
        i2c_peripheral_enable(I2C1);
     }
   //this event happens when slave is in transmit mode at the end of communication
   else if (sr1 & I2C_SR1_AF)
     {
        //(void) I2C_SR1(I2C1);
        I2C_SR1(I2C1) &= ~(I2C_SR1_AF);
     }
}