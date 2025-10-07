#include "lora.h"
#include "SoftSPI.h"
#include "dummy_defines.h"
#include "logger.h"
#include "system.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>

// #define DATASHEET_RESET

static int __implicit;
static long __tx_frequency;
static long __rx_frequency;
static int __initialized = 0;

static LoraPortDefines port;

/*
 * Asynchronous API
 */
// static void (*__callback)(void) = NULL;

/**
 * Returns non-zero value if the hardware had been initialized
 */
int lora_initialized(void)
{
   if(__initialized <= 0) return 0;
   return 1;
}

/**
 * Define pins and spi channel for interfacing with the transceiver.
 * @param reg: the register to write to
 * @param buf: the buffer to write from
 * @param len: the length of the buffer  
*/
void lora_spi_write(uint8_t reg, uint8_t * buf, size_t len) {

    ADD_TO_STACK_DEPTH();
    level_log(TRACE, "LORA_SPI - Writing...");

    /* Start a data transmission by pulling the Chip Select low*/
    lora_write_chip_select(LOW);

    /* Indicate a write by setting the MSB of the address byte */
    reg |= 0x80;

    /* Write the register over SPI with the write bit set to indicate write to the register */
    fast_SPI_Write(reg, SOFT_SPI_MSB_FIRST);
    
    /* Write the buffer over the SPI data line */
    for(int i = 0; i < len; i++) {
        fast_SPI_Write(*(buf + i), SOFT_SPI_MSB_FIRST);
    }

    /* End the transmission by setting the Chip Select high */
    // __delay_us(100); // Minimum time before CS goes high after falling edge of clock is 100us. (see section 2.5.6 of the datasheet)
    lora_write_chip_select(HIGH);
    *(port.mosi_port) |= (1 << port.mosi_pin);

    level_log(TRACE, "LORA_SPI - Write complete");
    REMOVE_FROM_STACK_DEPTH();
}

/**
 * Read the data coming over SPI and store it in a buffer
 * This is a dangerous function because if the buffer smaller than the length provided
 * @param reg: the register to read from
 * @param buf: the buffer to read to
 * @param len: the length of the read buffer
*/
void lora_spi_read(uint8_t reg, uint8_t * buf, size_t len) {
    uint8_t indata;

    ADD_TO_STACK_DEPTH();
    level_log(TRACE, "LORA_SPI - Reading...");
    if(len > MAX_BUFFER_SIZE) {level_log(ERROR, "Cannot write more than %d bytes to the I2C buffer", MAX_BUFFER_SIZE);}
    

    /* Start a data transmission by pulling the Chip Select low*/
    // __delay_us(30); // Minimum time before clock goes high after falling edge of CS is 30us. (see section 2.5.6 of the datasheet)
    lora_write_chip_select(LOW);

    /* indicate a read by clearing the MSB of the address byte*/
    reg &= 0x7f;

    /* Write the register over SPI with the write bit set to indicate a read to the register */
    fast_SPI_Write(reg, SOFT_SPI_MSB_FIRST);
    
    /* Write the buffer over the SPI data line */
    for(int i = 0; i < len; i++) {
        indata = fast_SPI_Read();
        memset( (buf + i), indata, 1);
    }

    /* End the transmission by setting the Chip Select high */
    // __delay_us(100); // Minimum time before CS goes high after falling edge of clock is 100us. (see section 2.5.6 of the datasheet)
    lora_write_chip_select(HIGH);
    *(port.mosi_port) |= (1 << port.mosi_pin);

    level_log(TRACE, "LORA_SPI - Read complete");
    REMOVE_FROM_STACK_DEPTH();
}


/**
 * Write a value to a register.
 * @param reg Register index.
 * @param val Value to write.
 */
void lora_write_reg(uint8_t reg, uint8_t val)
{
    ADD_TO_STACK_DEPTH();
    level_log(TRACE, "LORA - Writing to register: 0x%X", reg);

    // LATA |= (1 << 2);

    lora_spi_write(reg, &val, 1);

    // LATA &= ~(1 << 2);

    level_log(TRACE , "LORA - Wrote 0x%X to register", val);
    REMOVE_FROM_STACK_DEPTH();
}

/**
 * Read the current value of a register.
 * @param reg Register index.
 * @return Value of the register.
 */
uint8_t lora_read_reg(uint8_t reg)
{
    ADD_TO_STACK_DEPTH();
    level_log(TRACE , "LORA - Reading from register 0x%X...", reg);
    uint8_t in;

    /* Since this is a hard-coded register-write,
    The write length will always be 2  */
    lora_spi_read(reg, &in, 1);

    level_log(TRACE , "LORA - Read 0x%X from register", in);
    REMOVE_FROM_STACK_DEPTH();

    return in;
}

/**
 * Perform physical reset on the Lora chip
 */
void lora_reset(void)
{
    ADD_TO_STACK_DEPTH();
    level_log(TRACE, "LORA - RESETTING CHIP...");
    
    if(__initialized) {
        level_log(TRACE, " LORA already powered. manipulating reset line...");
        /* If the LoRa is already initialized, then the processor should manipulate the reset line
           as per section 7 of the datasheet */
        LORA_RST_SetDigitalOutput();
        LORA_RST_SetLow();
        __delay_us(110); // Datasheet says wait 100us, so be a little safer and wait 110us 
        // LORA_RST_SetHigh();
        LORA_RST_SetDigitalInput();
        __delay_ms(6); // Datasheet says delay 5ms, so we are waiting one more milisecond to be safe.
    } else {

    level_log(TRACE, " LORA reset on powerup. listening for the reset line...");
    /* If the LORA is not initialized */

        /* Set VDD low to start a proper powerup sequence */
        *(port.vdd_port) |= (1u << port.vdd_pin);

        __delay_ms(20); // Make sure everything is powered off before starting the power-up sequence

        LORA_RST_SetDigitalInput();

        /* Set VDD inn low (which gives power to the LoRa Chip) while RESET is set as an input */
        *(port.vdd_port) &= ~(1u << port.vdd_pin);
        // LORA_VDD_SetHigh();

        #ifndef DATASHEET_RESET
        LORA_RST_SetDigitalOutput();
        __delay_ms(1);
        LORA_RST_SetLow();
        __delay_us(100);
        LORA_RST_SetDigitalInput();
        #endif // PBP RESET ROUTUINE

        /* Wait for the RESET pin to be driven high by the LORA */
        #ifdef DATASHEET_RESET
        uint8_t lora_reset_value = LORA_RST_GetValue();
        while(lora_reset_value == 0){
            __delay_ms(1);
            lora_reset_value = LORA_RST_GetValue(); // Brandon, before you comment on this code being ineffieicent because it could just be written while( ~(LORA_RST_GetValue()) ) {...}, that doesn't work because the compiler doesn't "see" the value of the macro-like function, LORA_RST_GetValue(), as something that can change, so this gets compiled as while(1) because the LORA_RST pin is already low when this while loop runs. Basically that value only gets checked once, hence the volatile variable.
            level_log(TRACE, "RESET line is %d", lora_reset_value);
        }
        #endif // DATASHEET_RESET 

        /* Reset the bit with the OR assignment operator */
        // *(port.reset_port) |= (1u << port.reset_pin);

        /* After the reset is driven high by the RA-01, wait 20ms before doing anything else */
        __delay_ms(20);

        level_log(TRACE, "LORA - RESET CHIP");
        REMOVE_FROM_STACK_DEPTH();
    }
}

/**
 * Configure explicit header mode.
 * Packet size will be included in the frame.
 */
void lora_explicit_header_mode(void)
{
   __implicit = 0;
   lora_write_reg(REG_MODEM_CONFIG_1, lora_read_reg(REG_MODEM_CONFIG_1) & 0xfe);
}

/**
 * Configure implicit header mode.
 * All packets will have a predefined size.
 * @param size Size of the packets.
 */
void lora_implicit_header_mode(uint8_t size)
{
   __implicit = 1;
   lora_write_reg(REG_MODEM_CONFIG_1, lora_read_reg(REG_MODEM_CONFIG_1) | 0x01);
   lora_write_reg(REG_PAYLOAD_LENGTH, size);
}

/**
 * Sets the radio transceiver in idle mode.
 * Must be used to change registers and access the FIFO.
 */
void lora_idle(void)
{
   lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

/**
 * Sets the radio transceiver in sleep mode.
 * Low power consumption and FIFO is lost.
 */
void lora_sleep(void)
{
   lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

/**
 * Sets the radio transceiver in receive mode.
 * Incoming packets will be received.
 */
void lora_receive(void)
{
   lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

/**
 * Configure power level for transmission
 * @param level 2-17, from least to most power
 */
void lora_set_tx_power(uint8_t level)
{
   // RF9x module uses PA_BOOST pin
   if (level < 2) level = 2;
   else if (level > 17) level = 17;
   
   lora_write_reg(REG_PA_CONFIG, PA_BOOST | (level - 2));
}

/**
 * Set carrier frequency.
 * @param frequency Frequency in Hz
 */
void lora_set_frequency(long frequency)
{
    ADD_TO_STACK_DEPTH();
    level_log(TRACE , "LORA - Setting frequency to 0x%llx", (long long)(frequency));
    // __tx_frequency = frequency;

   unsigned long long frf = (((unsigned long long)(frequency) << 19) / 32000000);

   lora_write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16));
   lora_write_reg(REG_FRF_MID, (uint8_t)(frf >> 8));
   lora_write_reg(REG_FRF_LSB, (uint8_t)(frf >> 0));
   level_log(TRACE , "LORA - Set Frequency");
   REMOVE_FROM_STACK_DEPTH();
}

/**
 * Set spreading factor.
 * @param sf 6-12, Spreading factor to use.
 */
void lora_set_spreading_factor(int sf)
{
    ADD_TO_STACK_DEPTH();
    level_log(TRACE , "LORA - Setting spreading factor to %d", sf);
    if (sf < 6) sf = 6;
    else if (sf > 12) sf = 12;

    if (sf == 6) {
       lora_write_reg(REG_DETECTION_OPTIMIZE, 0xc5);
       lora_write_reg(REG_DETECTION_THRESHOLD, 0x0c);
    } else {
       lora_write_reg(REG_DETECTION_OPTIMIZE, 0xc3);
       lora_write_reg(REG_DETECTION_THRESHOLD, 0x0a);
    }

    lora_write_reg(REG_MODEM_CONFIG_2, (lora_read_reg(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
    level_log(TRACE , "LORA - Set Frequency");
    REMOVE_FROM_STACK_DEPTH();
}

/**
 * Set bandwidth (bit rate)
 * @param sbw Bandwidth in Hz (up to 500000)
 */
void lora_set_bandwidth(long sbw)
{
    ADD_TO_STACK_DEPTH();
    level_log(TRACE , "LORA - Setting bandwidth to %d", sbw);
    uint8_t bw;
 
    if (sbw <= (long)7.8E3) bw = 0;
    else if (sbw <= (long)10.4E3) bw = 1;
    else if (sbw <= (long)15.6E3) bw = 2;
    else if (sbw <= (long)20.8E3) bw = 3;
    else if (sbw <= (long)31.25E3) bw = 4;
    else if (sbw <= (long)41.7E3) bw = 5;
    else if (sbw <= (long)62.5E3) bw = 6;
    else if (sbw <= (long)125E3) bw = 7;
    else if (sbw <= (long)250E3) bw = 8;
    else bw = 9;
    
    lora_write_reg(REG_MODEM_CONFIG_1, (uint8_t)((uint8_t)(lora_read_reg(REG_MODEM_CONFIG_1) & 0x0f)) | (uint8_t)(bw << 4));
    level_log(TRACE , "LORA - Set bandwidth");
    REMOVE_FROM_STACK_DEPTH();
}

/**
 * Set coding rate 
 * @param denominator 5-8, Denominator for the coding rate 4/x
 */ 
void lora_set_coding_rate(int denominator)
{
    ADD_TO_STACK_DEPTH();
    level_log(TRACE , "LORA - Setting coding rate to %d", denominator);
    if (denominator < 5) denominator = 5;
    else if (denominator > 8) denominator = 8;

    uint8_t cr = (uint8_t)(denominator - 4);

    lora_write_reg(REG_MODEM_CONFIG_1, (uint8_t)((uint8_t)(lora_read_reg(REG_MODEM_CONFIG_1) & 0xf1)) | (uint8_t)(cr << 1));
    level_log(TRACE , "LORA - Set coding rate");
    REMOVE_FROM_STACK_DEPTH();
}

/**
 * Set the size of preamble.
 * @param length Preamble length in symbols.
 */
void lora_set_preamble_length(long length)
{
   lora_write_reg(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
   lora_write_reg(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

/**
 * Change radio sync word.
 * @param sw New sync word to use.
 * @remark the word referenced in set_sync_WORD is actually a byte. Don't be decieved
 */
void lora_set_sync_word(uint8_t sw)
{
   lora_write_reg(REG_SYNC_WORD, sw);
}

/**
 * Enable appending/verifying packet CRC.
 */
void lora_enable_crc(void)
{
   lora_write_reg(REG_MODEM_CONFIG_2, lora_read_reg(REG_MODEM_CONFIG_2) | 0x04);
}

/**
 * Disable appending/verifying packet CRC.
 */
void lora_disable_crc(void)
{
   lora_write_reg(REG_MODEM_CONFIG_2, lora_read_reg(REG_MODEM_CONFIG_2) & 0xfb);
}

void lora_write_chip_select(uint8_t pin_state){
        if(pin_state){
            *(port.select_port) |= (1 << port.select_pin);
            // LORA_CS_SetHigh();
        } else {
            *(port.select_port) &= ~(1 << port.select_pin);
            // LORA_CS_SetLow();
        }
}

/**
 * Set the Clock, MOSI, and MISO pins to their idle state of zero
*/
void lora_pins_idle(){
    *(port.select_port) |= (1 << port.select_pin);
    *(port.clock_port) &= ~(1 << port.clock_pin);

    __delay_us(10);

    *(port.mosi_port) |= (1 << port.mosi_pin);
    *(port.miso_port) &= ~(1 << port.miso_pin);
}

/**
 * Perform hardware initialization.
 */
int lora_init(LoraDefines args, LoraConfigs config)
{
    ADD_TO_STACK_DEPTH();
    level_log(TRACE , "LORA - Initialize");

    port = args.port;
    /* set the pins to the idle state before initializing */
    lora_pins_idle();

    /*
     * Perform hardware reset/initialization.
     */
    /* This basically mimics the SoftSPI_Init function */
    SoftSPI_InitDataOutPin(port.mosi_port, port.mosi_pin);
    SoftSPI_InitDataInPin(port.miso_port, port.miso_pin);
    SoftSPI_InitClockPin(port.clock_port, port.clock_pin);
    SoftSPI_InitSelectPin(port.select_port, port.select_pin);

    /* Set the talk and recieve frequencies */
    __tx_frequency = args.tx_frequency;
    __rx_frequency = args.rx_frequency;

    /* Set pin hardware configuration for the PIC */
    lora_write_chip_select(1);
    lora_reset();

    /* Put the Lora in Standby mode so that we can write to the device regiseters */
    // lora_idle();
    // uint8_t mode_try_ctr = 10;
    // while(mode_try_ctr--) {
        lora_write_reg(REG_OP_MODE, 0x80);
    //     __delay_ms(5);
    // }

    uint8_t mode = lora_read_reg(REG_OP_MODE);
    level_log(INFO, "Mode is set to: 0x%x", mode);

    /*
     * Check version.
     */
    #warning LoRa version is being checked. This program WILL crash if the version is wrong, or if the SPI read is wrong. 
    uint8_t version = lora_read_reg(REG_VERSION);
    assert(version == 0x12);

    /*
     * Default configuration.
     */
    lora_write_reg(REG_FIFO_RX_BASE_ADDR, 0);

    lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0);
    lora_write_reg(REG_LNA, lora_read_reg(REG_LNA) | 0x03);

    /* We are making the assumption here that the user wants to set these configs directly */
    lora_write_reg(REG_MODEM_CONFIG_1, config.RegModemConfig1);
    lora_write_reg(REG_MODEM_CONFIG_2, config.RegModemConfig2);
    lora_write_reg(REG_MODEM_CONFIG_3, config.RegModemConfig3);

    lora_set_tx_power(args.tx_power); // should probably be set to 17 to be safe...
    lora_set_bandwidth(args.bandwidth); // Modifies config 1 - default should be 125000
    lora_set_frequency(args.tx_frequency); // Does not modify a config register - default should be 425000000
    lora_set_coding_rate(args.coding_rate); // Modifies config 1 - default should be 5
    lora_set_spreading_factor(args.spread_factor); // Modifies config 2 - default should be 10

    /* Don't make the LoRa Idle right now... */
    // lora_idle();

    __initialized = 1;

    level_log(TRACE , "LORA - Initialization complete");
    REMOVE_FROM_STACK_DEPTH();

    return __initialized;
}

/**
 * Send a packet.
 * @param buf Data to be sent
 * @param size Size of data.
 */
void lora_send_packet(uint8_t *buf, size_t size)
{
    ADD_TO_STACK_DEPTH();
    level_log(TRACE, "LORA - sending packet...");
    /* Set frequency to sending frequency of 425000000 */
    lora_set_frequency(__tx_frequency);

    /*
     * Transfer data to radio.
     */
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    lora_write_reg(REG_FIFO_ADDR_PTR, 0);

    for(int i=0; i<size; i++) 
       lora_write_reg(REG_FIFO, *buf++);

    /* if the size is greater than 255, this will write the wrong value to the LoRa*/
    lora_write_reg(REG_PAYLOAD_LENGTH, (uint8_t)size);

    /*
     * Start transmission and wait for conclusion.
     */
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
    while((lora_read_reg(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0){
       __delay_us(100);
    }
    lora_write_reg(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);

    level_log(TRACE, "LORA - sent packet");
    REMOVE_FROM_STACK_DEPTH();
}

/**
 * Read a received packet.
 * @param buf Buffer for the data.
 * @param size Available size in buffer (bytes).
 * @return Number of bytes received (zero if no packet available).
 */
int lora_receive_packet(uint8_t *buf, size_t size)
{
    ADD_TO_STACK_DEPTH();
    level_log(TRACE, "LORA - receiving packet...");

   int i, len = 0;
    
    /* Set frequency to listening frequency of Whatever... */
    lora_set_frequency(__rx_frequency);

    /*
    * Check interrupts.
    */
    uint8_t irq = lora_read_reg(REG_IRQ_FLAGS);
    lora_write_reg(REG_IRQ_FLAGS, irq);
    
    if((irq & IRQ_RX_DONE_MASK) == 0)  {
        REMOVE_FROM_STACK_DEPTH();
        return -1;
    }
    if(irq & IRQ_PAYLOAD_CRC_ERROR_MASK) {
        REMOVE_FROM_STACK_DEPTH();
        return -1;
    }
    /*
    * Find packet size.
    */
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    if (__implicit) len = lora_read_reg(REG_PAYLOAD_LENGTH);
    else len = lora_read_reg(REG_RX_NB_BYTES);

    /*
    * Transfer data from radio.
    */
    lora_write_reg(REG_FIFO_ADDR_PTR, lora_read_reg(REG_FIFO_RX_CURRENT_ADDR));
    if(len > size) len = (int)size;
    for(i = 0; i < len; i++) {
        *(buf++) = lora_read_reg(REG_FIFO);
    }

    level_log(TRACE, "LORA - received packet");
    REMOVE_FROM_STACK_DEPTH();
    return len;
}

/**
 * Returns non-zero if there is data to read (packet received).
 */
int lora_received(void)
{
   int m = lora_read_reg(REG_IRQ_FLAGS) & IRQ_RX_DONE_MASK;
   if(m) return 1;
   return 0;
}

/**
 * Suspend the current thread until a packet arrives or a timeout occurs.
 * @param timeout Timeout in ms.
 */
void lora_wait_for_packet(int timeout)
{
   lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
   lora_write_reg(REG_IRQ_FLAGS_MASK, 0x9f);
   lora_write_reg(REG_DIO_MAPPING_1, 0x00);

   lora_receive();
}


/**
 * Define a callback function for packet reception.
 * @param cb Callback function to use (NULL to cancel callbacks).
 */
// void lora_on_receive(void (*cb)(void))
// {
//    if((cb == NULL) && (__callback != NULL)) {
      
      
//       __callback = NULL;
//       return;
//    }

//    if(__callback == NULL) {
//       __callback = cb;
      
//       return;
//    }

//    __callback = cb;
// }

/**
 * Return last packet's RSSI.
 */
int lora_packet_rssi(void)
{
   int v = lora_read_reg(REG_PKT_RSSI_VALUE);
   return v - (__tx_frequency < (long)868E6 ? 164 : 157);
}

/**
 * Return last packet's SNR (signal to noise ratio).
 */
float lora_packet_snr(void)
{
   int v = lora_read_reg(REG_PKT_SNR_VALUE);
   return ((int8_t)v) * ((float)0.25);
}

/**
 * Shutdown hardware.
 */
void lora_close(void)
{
    lora_sleep();

    // Add power down here //
}

void lora_dump_registers(void)
{
    uint8_t i;
    for(i=0; i<0x26; i++) {
       printf("%02x -> %02x\n", i, lora_read_reg(i));
    }
}

