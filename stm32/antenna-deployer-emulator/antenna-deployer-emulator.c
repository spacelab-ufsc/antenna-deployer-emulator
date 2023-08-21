#include <libopencm3/cm3/assert.h>
#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>

// Antenna valid commands
#define RESET 0xAA
#define ARM 0xAD
#define DISARM 0xAC
#define DEPLOY_ANT_1 0xA1
#define DEPLOY_ANT_2 0xA2
#define DEPLOY_ANT_3 0xA3
#define DEPLOY_ANT_4 0xA4
#define DEPLOY_SEQUENCIAL 0xA5
#define DEPLOY_ANT_1_OVERRIDE 0xBA
#define DEPLOY_ANT_2_OVERRIDE 0xBB
#define DEPLOY_ANT_3_OVERRIDE 0xBC
#define DEPLOY_ANT_4_OVERRIDE 0xBD
#define DEPLOY_CANCEL 0xA9
#define MEASURE_TEMPERATURE 0xC0
#define REPORT_DEPLOY_COUNTER_1 0xB0
#define REPORT_DEPLOY_COUNTER_2 0xB1
#define REPORT_DEPLOY_COUNTER_3 0xB2
#define REPORT_DEPLOY_COUNTER_4 0xB3
#define REPORT_DEPLOY_TIMER_1 0xB4
#define REPORT_DEPLOY_TIMER_2 0xB5
#define REPORT_DEPLOY_TIMER_3 0xB6
#define REPORT_DEPLOY_TIMER_4 0xB7
#define REPORT_DEPLOY_STATUS 0xC3
#define TIME_STEP 3200

void i2c_slave_init(uint8_t ownaddress);
void sequencial(void);
void command_decode(uint8_t *response);
void delay_us(uint32_t us);
void delay_setup(void);

uint8_t deploy_counter_1 = 0;
uint8_t deploy_counter_2 = 0;
uint8_t deploy_counter_3 = 0;
uint8_t deploy_counter_4 = 0;
uint8_t sq_counter = 0;
uint16_t it_counter = 0; /* Each bit of this variable controls counters repetitons */
uint32_t deploy_timer_1 = 0;
uint32_t deploy_timer_2 = 0;
uint32_t deploy_timer_3 = 0;
uint32_t deploy_timer_4 = 0;
uint32_t time_to_burn = 0;
volatile uint32_t myticks_ = 0;
static uint8_t rx_buffer[2];
static uint8_t tx_buffer[2];
static uint8_t *rx_pos;
static uint8_t *tx_pos;

int main(void)
{
    // set STM32 to 72 MHz
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    // Enable GPIOC clock
    rcc_periph_clock_enable(RCC_GPIOC);
    // Set GPIO13 (inbuild led connected) to 'output push-pull'
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

    // switch led off
    gpio_set(GPIOC, GPIO13);

    // initialize i2c slave
    i2c_slave_init(0x32);
    delay_setup();

    while (1)
    {
        /* Check for a valid command */
        command_decode(tx_buffer); /* Receives as parameter buffer to store responses*/

        /* Delay to simulate antenna behavior */
        delay_us(4);
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
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
                  GPIO_I2C1_SDA); // PB7
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
                  GPIO_I2C1_SCL); // PB6

    i2c_reset(I2C1);
    i2c_peripheral_disable(I2C1);

    i2c_set_speed(I2C1, i2c_speed_sm_100k, I2C_CR2_FREQ_36MHZ);
    i2c_set_own_7bit_slave_address(I2C1, ownaddress);
    i2c_enable_interrupt(I2C1, I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN);
    i2c_peripheral_enable(I2C1);

    i2c_enable_ack(I2C1);
}

// i2c1 event ISR
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

        // Clear the ADDR sequence by reading SR2.
        sr2 = I2C_SR2(I2C1);
        (void)sr2;

        // gpio_toggle(GPIOC, GPIO13);
    }

    // Receive buffer not empty
    else if (sr1 & I2C_SR1_RxNE)
    {
        // read bytes from slave
        *(rx_pos++) = i2c_get_data(I2C1);

        // Resets the command reptition control variable
        it_counter = 0x0000;
    }

    // Transmit buffer empty & Data byte transfer not finished
    else if ((sr1 & I2C_SR1_TxE) && !(sr1 & I2C_SR1_BTF))
    {
        // send data to master in MSB order
        i2c_send_data(I2C1, *tx_pos);
        *tx_pos++ = 0x00; /*Clears after sending */
    }

    // done by master by sending STOP
    // this event happens when slave is in Recv mode at the end of communication
    else if (sr1 & I2C_SR1_STOPF)
    {
        i2c_peripheral_enable(I2C1);
    }

    // this event happens when slave is in transmit mode at the end of
    // communication
    else if (sr1 & I2C_SR1_AF)
    {
        //(void) I2C_SR1(I2C1);
        I2C_SR1(I2C1) &= ~(I2C_SR1_AF);
    }
}

/* Simulates the sequencial deployment */
void sequencial(void)
{
    /* For each case increment the respective deployment counter  */
    switch (sq_counter)
    {
    case 0:
        /* Increment counter and setup for next antenna */
        deploy_counter_1++;
        sq_counter++;
        break;
    case 1:
        /* Increment counter and setup for next antenna */
        deploy_counter_2++;
        sq_counter++;
        break;
    case 2:
        /* Increment counter and setup for next antenna */
        deploy_counter_3++;
        sq_counter++;
        break;
    case 3:
        /* Increment counter and reset for next sequencial deploy */
        deploy_counter_4++;
        sq_counter = 0;
        break;
    default:
        /* Impossible to match unless something breaks */
        sq_counter = 0;
        break;
    }
}

/* Decide the action upon the command received */
void command_decode(uint8_t *response)
{

    /* Switch statement to check all valid commands and how to treat then */
    switch (rx_buffer[0])
    {
    case ARM:
        /* do nothing, since there is nothing to increment or respond */
        response[0] = 0x00;
        response[1] = 0x00;
        break;
    case RESET:
        /* do nothing, since there is nothing to increment or respond */
        response[0] = 0x00;
        response[1] = 0x00;
        break;
    case DISARM:
        /* do nothing, since there is nothing to increment or respond */
        response[0] = 0x00;
        response[1] = 0x00;
        break;
    case DEPLOY_ANT_1:
        /* Updating the counters, necessary for other command responses */

        /* Separating counter from timer */
        if (!(it_counter & 0x0001))
        {
            deploy_counter_1++;

            /* Avoids counter getting incremented while waiting for new commands */
            it_counter |= 0x0001;

            /* Receives command parameter send by i2c */
            time_to_burn = (uint32_t)rx_buffer[1] * TIME_STEP;
        }
        deploy_timer_1++;
        response[0] = 0x00;
        response[1] = 0x00;
        break;
    case DEPLOY_ANT_2:
        /* Updating the counters, necessary for other command responses */

        /* Separating counter from timer */
        if (!(it_counter & 0x0002))
        {
            deploy_counter_2++;

            /* Avoids counter getting incremented while waiting for new commands */
            it_counter |= 0x0002;

            /* Receives command parameter send by i2c */
            time_to_burn = (uint32_t)rx_buffer[1] * TIME_STEP;
        }
        deploy_timer_2++;
        response[0] = 0x00;
        response[1] = 0x00;
        break;
    case DEPLOY_ANT_3:
        /* Updating the counters, necessary for other command responses */

        /* Separating counter from timer */
        if (!(it_counter & 0x0004))
        {
            deploy_counter_3++;

            /* Avoids counter getting incremented while waiting for new commands */
            it_counter |= 0x0004;

            /* Receives command parameter send by i2c */
            time_to_burn = (uint32_t)rx_buffer[1] * TIME_STEP;
        }
        deploy_timer_3++;
        response[0] = 0x00;
        response[1] = 0x00;
        break;
    case DEPLOY_ANT_4:
        /* Updating the counters, necessary for other command responses */

        /* Separating counter from timer */
        if (!(it_counter & 0x0008))
        {
            deploy_counter_4++;

            /* Avoids counter getting incremented while waiting for new commands */
            it_counter |= 0x0008;

            /* Receives command parameter send by i2c */
            time_to_burn = (uint32_t)rx_buffer[1] * TIME_STEP;
        }
        deploy_timer_4++;
        response[0] = 0x00;
        response[1] = 0x00;
        break;
    case DEPLOY_SEQUENCIAL:
        /* Contains the logic for sequencial deployment */

        /* Separating counter from timer */
        if (!(it_counter & 0x0100))
        {
            sequencial(); /* Call sequencial deployment */

            /* Avoids counter getting incremented while waiting for new commands */
            it_counter |= 0x0100;

            /* Receives command parameter send by i2c */
            time_to_burn = (uint32_t)rx_buffer[1] * TIME_STEP;
        }
        response[0] = 0x00;
        response[1] = 0x00;
        break;
    case DEPLOY_ANT_1_OVERRIDE:
        /* Updating the counters, necessary for other command responses */

        /* Separating counter from timer */
        if (!(it_counter & 0x0010))
        {
            deploy_counter_1++;

            /* Avoids counter getting incremented while waiting for new commands */
            it_counter |= 0x0010;

            /* Receives command parameter send by i2c */
            time_to_burn = (uint32_t)rx_buffer[1] * TIME_STEP;
        }
        deploy_timer_1++;
        response[0] = 0x00;
        response[1] = 0x00;
        break;
    case DEPLOY_ANT_2_OVERRIDE:
        /* Updating the counters, necessary for other command responses */

        /* Separating counter from timer */
        if (!(it_counter & 0x0020))
        {
            deploy_counter_2++;

            /* Avoids counter getting incremented while waiting for new commands */
            it_counter |= 0x0020;

            /* Receives command parameter send by i2c */
            time_to_burn = (uint32_t)rx_buffer[1] * TIME_STEP;
        }
        deploy_timer_2++;
        response[0] = 0x00;
        response[1] = 0x00;
        break;
    case DEPLOY_ANT_3_OVERRIDE:
        /* Updating the counters, necessary for other command responses */

        /* Separating counter from timer */
        if (!(it_counter & 0x0040))
        {
            deploy_counter_3++;

            /* Avoids counter getting incremented while waiting for new commands */
            it_counter |= 0x0040;

            /* Receives command parameter send by i2c */
            time_to_burn = (uint32_t)rx_buffer[1] * TIME_STEP;
        }
        deploy_timer_3++;
        response[0] = 0x00;
        response[1] = 0x00;
        break;
    case DEPLOY_ANT_4_OVERRIDE:
        /* Updating the counters, necessary for other command responses */

        /* Separating counter from timer */
        if (!(it_counter & 0x0080))
        {
            deploy_counter_4++;

            /* Avoids counter getting incremented while waiting for new commands */
            it_counter |= 0x0080;

            /* Receives command parameter send by i2c */
            time_to_burn = (uint32_t)rx_buffer[1] * TIME_STEP;
        }
        deploy_timer_4++;
        response[0] = 0x00;
        response[1] = 0x00;
        break;
    case DEPLOY_CANCEL:
        /* do nothing, since there is nothing to increment or respond */
        response[0] = 0x00;
        response[1] = 0x00;
        break;
    case MEASURE_TEMPERATURE:
        /* Simulates reading the temperature and responding its value */
        response[0] = 0xFF;
        response[1] = 0x03;
        break;
    case REPORT_DEPLOY_TIMER_1:
        /* Simulates responding deploy time using two bytes */
        deploy_timer_1 = deploy_timer_1;
        response[0] = (deploy_timer_1 & 0xFF00) >> 8;
        response[1] = (deploy_timer_1 & 0xFF);
        deploy_timer_1 = 0;
        break;
    case REPORT_DEPLOY_TIMER_2:
        /* Simulates responding deploy time using two bytes */
        deploy_timer_2 = deploy_timer_2;
        response[0] = (deploy_timer_2 & 0xFF00) >> 8;
        response[1] = (deploy_timer_2 & 0xFF);
        deploy_timer_2 = 0;
        break;
    case REPORT_DEPLOY_TIMER_3:
        /* Simulates responding deploy time using two bytes */
        deploy_timer_3 = deploy_timer_3;
        response[0] = (deploy_timer_3 & 0xFF00) >> 8;
        response[1] = (deploy_timer_3 & 0xFF);
        deploy_timer_3 = 0;
        break;
    case REPORT_DEPLOY_TIMER_4:
        /* Simulates responding deploy time using two bytes */
        deploy_timer_4 = deploy_timer_4;
        response[0] = (deploy_timer_4 & 0xFF00) >> 8;
        response[1] = (deploy_timer_4 & 0xFF);
        deploy_timer_4 = 0;
        break;
    case REPORT_DEPLOY_COUNTER_1:
        /* Simulates responding deploy counter using one byte */
        response[0] = deploy_counter_1;
        response[1] = 0x00;
        break;
    case REPORT_DEPLOY_COUNTER_2:
        /* Simulates responding deploy counter using one byte */
        response[0] = deploy_counter_2;
        response[1] = 0x00;
        break;
    case REPORT_DEPLOY_COUNTER_3:
        /* Simulates responding deploy counter using one byte */
        response[0] = deploy_counter_3;
        response[1] = 0x00;
        break;
    case REPORT_DEPLOY_COUNTER_4:
        /* Simulates responding deploy counter using one byte */
        response[0] = deploy_counter_4;
        response[1] = 0x00;
        break;
    case REPORT_DEPLOY_STATUS:
        /* Simulates responding the deploy status using two bytes */
        response[0] = 0xFF;
        response[1] = 0xFF;
        break;
    default:
        /* Used for debug purposes, literally means invalid command */
        response[0] = 0x00;
        response[1] = 0x00;
    }
}
/* Setup the timer used for delay */
void delay_setup(void)
{
    rcc_periph_clock_enable(RCC_TIM4); /* Enable clock for timer */

    timer_set_prescaler(TIM4, 0); /* Set prescaler for timer */
    timer_set_period(TIM4, 71);   /* Set period to create a microsecond interrupt */

    TIM_EGR(TIM4) = TIM_EGR_UG;   /* Necessary to uptade registers */
    TIM_CR1(TIM4) |= TIM_CR1_URS; /* Configures interrupt to only occur after overflow */

    timer_enable_irq(TIM4, TIM_DIER_UIE); /* Enables interrupt for timer */
    nvic_enable_irq(NVIC_TIM4_IRQ);       /* Necessary to enable interrupt for timer */
}
/* Timer event isr */
void tim4_isr(void)
{
    TIM_SR(TIM4) &= ~TIM_SR_UIF; /* Clears interrupt flag */
    myticks_++;                  /* Sinalizes that 1 microsecond passed */
}

/* Create delays in microseconds */
void delay_us(uint32_t us)
{
    myticks_ = 0;                 /* Resets counter */
    TIM_CR1(TIM4) |= TIM_CR1_CEN; /* Starts the counting */

    while (myticks_ < us)
        ; /* Lock the process until "us" microseconds passed */

    TIM_CR1(TIM4) &= ~TIM_CR1_CEN; /* Clear the starting flag */
}
