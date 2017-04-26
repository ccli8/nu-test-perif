#include "mbed.h"

#define MYCONF_TRAN_UNIT_T          uint32_t
#define MYCONF_BITS_TRAN_UNIT_T     (sizeof (MYCONF_TRAN_UNIT_T) * 8)
#define MYCONF_DMA_USAGE            DMA_USAGE_NEVER
//DMA_USAGE_NEVER,DMA_USAGE_ALWAYS
#define MYCONF_DEBUG                1

// SPI
#define MYCONF_SPI_AUTOSS           1
// NOTE: CS# Deselect Time (tSHSL)
#define MYCONF_SPI_tSHSL_US         0
#define MYCONF_SPI_ECHO_PLUS        5
// I2C
#define MYCONF_I2C_ADDR             (0x90)


#if defined(TARGET_NUMAKER_PFM_NUC472)
//Serial
#define SERIAL_RX   PF_0
#define SERIAL_TX   PD_15
#define SERIAL_CTS  PD_13
#define SERIAL_RTS  PD_14
// SPI
#define SPI_MOSI    PF_0
#define SPI_MISO    PD_15
#define SPI_SCLK    PD_14
#define SPI_SSEL    PD_13
// I2C
#define I2C_SDA     PD_12
#define I2C_SCL     PD_10
// InterruptIn
#define BTN1        SW1
#define BTN2        SW2

#elif defined(TARGET_NUMAKER_PFM_M453)
//Serial
#define SERIAL_RX   D0
#define SERIAL_TX   D1
#define SERIAL_CTS  NC
#define SERIAL_RTS  NC
// SPI
#define SPI_MOSI    PD_13
#define SPI_MISO    PD_14
#define SPI_SCLK    PD_15
#define SPI_SSEL    PD_12
// I2C
#define I2C_SDA     PE_5
#define I2C_SCL     PE_4
// InterruptIn
#define BTN1        SW2
#define BTN2        SW3

#elif defined(TARGET_NUMAKER_PFM_M487)
//Serial
#define SERIAL_RX   PG_7
#define SERIAL_TX   PG_8
#define SERIAL_CTS  PA_2
#define SERIAL_RTS  PA_3
// SPI
#define SPI_MOSI    PD_9
#define SPI_MISO    PD_1
#define SPI_SCLK    PD_2
#define SPI_SSEL    PD_8
// I2C
#define I2C_SDA     PA_2
#define I2C_SCL     PA_3
// InterruptIn
#define BTN1        NC
#define BTN2        NC

#elif defined(TARGET_NUMAKER_PFM_NANO130)
//Serial
#define SERIAL_RX   PB_0
#define SERIAL_TX   PB_1
#define SERIAL_CTS  NC
#define SERIAL_RTS  NC
// SPI
#define SPI_MOSI    PE_4
#define SPI_MISO    PE_3
#define SPI_SCLK    PE_2
#define SPI_SSEL    PE_1
// I2C
#define I2C_SDA     PA_8
#define I2C_SCL     PA_9
// InterruptIn
#define BTN1        NC
#define BTN2        NC

#endif

static void test_serial_tx_attach(void);
static void test_serial_rx_attach(void);
static void test_serial_txrx_attach(void);
static void serial_tx_callback(RawSerial *serial);
static void serial_rx_callback(RawSerial *serial);
static void test_serial_tx_async(void);
static void test_serial_rx_async(void);
static void test_serial_tx_async_n_tx_attach(void);
static void serial_async_callback(int event);
static void test_spi_master(void);
static void test_spi_master_async(void);
static void spi_master_async_callback(int event);
static void test_spi_slave(void);
static void test_i2c_master(void);
static void test_i2c_master_async(void);
static void i2c_master_async_callback(int event);
static void test_i2c_slave(void);
static void test_interruptin(void);
static void my_gpio_irq_rise(void);
static void my_gpio_irq_fall(void);

static DigitalOut led1(LED_RED, 1);
static DigitalOut led2(LED_GREEN, 1);
static DigitalOut led3(LED_BLUE, 1);

static union {
    struct {
        char buf[32];
        char buf2[32];
    } i2c_test_ctx;
    struct {
        MYCONF_TRAN_UNIT_T buf[32];
        MYCONF_TRAN_UNIT_T buf2[32];
    } spi_test_ctx;
};

volatile int callback_event = 0;

#if MYCONF_DEBUG
int MY_BUF[64];
int MY_BUF_POS = 0;
#endif

int main()
{
    //test_serial_tx_attach();
    //test_serial_rx_attach();
    test_serial_txrx_attach();
    //test_serial_tx_async();
    //test_serial_rx_async();
    //test_serial_tx_async_n_tx_attach();
    //test_spi_master();
    //test_spi_master_async();
    //test_spi_slave();
    //test_i2c_master();
    //test_i2c_master_async();
    //test_i2c_slave();
    //test_interruptin();
}

static char serial_buf_tx[] = "123456780000000000111111111122222222223333333333444444444455555555556666666666END\n";
//static char serial_buf_tx[23] = "12345678901234567890\n\n";
//static char serial_buf_tx[11] = "123456789\n";
static char serial_buf_rx[23];
static volatile int my_serial_event = 0;

static void test_serial_tx_attach(void)
{
    static RawSerial my_serial(SERIAL_TX, SERIAL_RX);
    
    Callback<void()> callback(&serial_tx_callback, &my_serial);
    my_serial.attach(callback, mbed::SerialBase::TxIrq);
    
    while (1) {
        my_serial.putc('.');
        wait(1.0);
    }
}

static void test_serial_rx_attach(void)
{
    static RawSerial my_serial(SERIAL_TX, SERIAL_RX);
    
    Callback<void()> callback(&serial_rx_callback, &my_serial);
    my_serial.attach(callback, mbed::SerialBase::RxIrq);
    
    while (1) {
        wait(1.0);
    }
}

static void test_serial_txrx_attach(void)
{
    static RawSerial my_serial(SERIAL_TX, SERIAL_RX);
    
    Callback<void()> tx_callback(&serial_tx_callback, &my_serial);
    Callback<void()> rx_callback(&serial_rx_callback, &my_serial);  
    my_serial.attach(tx_callback, mbed::SerialBase::TxIrq);
    my_serial.attach(rx_callback, mbed::SerialBase::RxIrq);
    
    while (1) {
        wait(1.0);
    }
}

static void serial_tx_callback(RawSerial *serial)
{
    led2 = ! led2;
}

static void serial_rx_callback(RawSerial *serial)
{
    led3 = ! led3;
    // NOTE: On Nuvoton targets, no H/W IRQ to match RxIrq. Simulation of RxIrq requires the call to Serial::getc(). 
    serial->putc(serial->getc());
}

static void test_serial_tx_async(void)
{   
    static RawSerial my_serial(SERIAL_TX, SERIAL_RX);
    event_callback_t event_callback(serial_async_callback);
    
REPEAT:
    my_serial_event = 0;
    //my_serial.set_dma_usage_tx(DMA_USAGE_NEVER);
    my_serial.set_dma_usage_tx(DMA_USAGE_ALWAYS);
    
    //my_serial.set_flow_control(SerialBase::RTSCTS, SERIAL_RTS, SERIAL_CTS);
    
    my_serial.write((const uint8_t *) serial_buf_tx, sizeof (serial_buf_tx) - 1, event_callback, SERIAL_EVENT_TX_COMPLETE);
    
    while (! my_serial_event) {
        wait_ms(200);
    }
    
    if (my_serial_event & SERIAL_EVENT_TX_COMPLETE) {
        printf("SERIAL_EVENT_TX_COMPLETE event received\n");
        goto REPEAT;
    }
}

static void test_serial_rx_async(void)
{   
    static RawSerial my_serial(SERIAL_TX, SERIAL_RX);
    event_callback_t event_callback(serial_async_callback);
    
REPEAT:
    my_serial_event = 0;
    memset(serial_buf_rx, 0x00, sizeof (serial_buf_rx));
    //my_serial.set_dma_usage_tx(DMA_USAGE_NEVER);
    my_serial.set_dma_usage_rx(DMA_USAGE_ALWAYS);
    
    //my_serial.set_flow_control(SerialBase::RTSCTS, SERIAL_RTS, SERIAL_CTS);
    
    //my_serial.read((uint8_t *) serial_buf_rx, sizeof (serial_buf_rx) - 1, event_callback, SERIAL_EVENT_RX_COMPLETE, SERIAL_RESERVED_CHAR_MATCH);    
    my_serial.read((uint8_t *) serial_buf_rx, sizeof (serial_buf_rx) - 1, event_callback, SERIAL_EVENT_RX_COMPLETE | SERIAL_EVENT_RX_CHARACTER_MATCH, 'q');
    
    while (! my_serial_event) {
        wait_ms(200);
    }
    
    if (my_serial_event & SERIAL_EVENT_RX_COMPLETE) {
        serial_buf_rx[sizeof (serial_buf_rx) - 1] = 0;
        printf("%s\n", serial_buf_rx);
        goto REPEAT;
    }
    if (my_serial_event & SERIAL_EVENT_RX_OVERRUN_ERROR) {
        printf("SERIAL_EVENT_RX_OVERRUN_ERROR\n");
    }
    if (my_serial_event & SERIAL_EVENT_RX_FRAMING_ERROR) {
        printf("SERIAL_EVENT_RX_FRAMING_ERROR\n");
    }
    if (my_serial_event & SERIAL_EVENT_RX_PARITY_ERROR) {
        printf("SERIAL_EVENT_RX_PARITY_ERROR\n");
    }
    if (my_serial_event & SERIAL_EVENT_RX_OVERFLOW) {
        printf("SERIAL_EVENT_RX_OVERFLOW\n");
    }
    if (my_serial_event & SERIAL_EVENT_RX_CHARACTER_MATCH) {
        printf("%s\n", serial_buf_rx);
        goto REPEAT;
    }
}

void test_serial_tx_async_n_tx_attach(void)
{
    static RawSerial my_serial(SERIAL_TX, SERIAL_RX);
    
    Callback<void()> callback(&serial_tx_callback, &my_serial);
    my_serial.attach(callback, mbed::SerialBase::TxIrq);
    
    event_callback_t event_callback(serial_async_callback);
    
REPEAT:
    wait(1.0);
    
    my_serial.putc('.');
    // Wait for UART TX FIFO empty
    wait_ms(100);
    
    my_serial_event = 0;
    my_serial.set_dma_usage_tx(DMA_USAGE_NEVER);
    //my_serial.set_dma_usage_tx(DMA_USAGE_ALWAYS);
    
    my_serial.write((const uint8_t *) serial_buf_tx, sizeof (serial_buf_tx) - 1, event_callback, SERIAL_EVENT_TX_COMPLETE);
    
    while (! my_serial_event) {
        wait_ms(200);
    }
    
    if (my_serial_event & SERIAL_EVENT_TX_COMPLETE) {
        my_serial.printf("SERIAL_EVENT_TX_COMPLETE event received\n");
        goto REPEAT;
    }
}

void serial_async_callback(int event)
{
    my_serial_event = event;
}

#if DEVICE_SPI

static void test_spi_master(void)
{
#if MYCONF_SPI_AUTOSS
    static SPI spi_master(SPI_MOSI, SPI_MISO, SPI_SCLK, SPI_SSEL);
#else
    static SPI spi_master(SPI_MOSI, SPI_MISO, SPI_SCLK);
    static DigitalOut cs(SPI_SSEL);
#endif

    int n_round = 0;
    int data = 0;
    int res = 0;
    
    spi_master.format(MYCONF_BITS_TRAN_UNIT_T);    // n bits per SPI frame
    
    // NOTE: Run spi_master first and then run spi_slave 3 secs. This is to keep cs inactive until spi_slave is ready.
#if (! MYCONF_SPI_AUTOSS)
    cs = 1;
#endif
    wait(3);
	
REPEAT:

    data = 0;
    for (int i = 0; i < 30; i ++) {
#if (! MYCONF_SPI_AUTOSS)
        cs = 0;
#if MYCONF_SPI_tSHSL_US
        wait_us(MYCONF_SPI_tSHSL_US);
#endif
#endif
        res = spi_master.write(data);
#if (! MYCONF_SPI_AUTOSS)
        cs = 1;
#if MYCONF_SPI_tSHSL_US
        wait_us(MYCONF_SPI_tSHSL_US);
#endif
#endif

        if (i >= 2 && res != (data + MYCONF_SPI_ECHO_PLUS - 2)) {
            printf("i=%d, res=0x%08x, data=0x%08x\n", i, res, data);
            printf("%s Round %d FAILED\n", __func__, n_round ++);
            while (1);
        }
        
        data ++;
    }

    printf("%s Round %d OK\n", __func__, n_round ++);
    goto REPEAT;
}

static void test_spi_master_async(void)
{
#if MYCONF_SPI_AUTOSS
    static SPI spi_master(SPI_MOSI, SPI_MISO, SPI_SCLK, SPI_SSEL);
#else
    static SPI spi_master(SPI_MOSI, SPI_MISO, SPI_SCLK);
    static DigitalOut cs(SPI_SSEL);
#endif

    int n_round = 0;
    
    // With NUMAKER-PFM-NUC472 as SPI slave, test fails with default 1 MHz SPI clock.
    spi_master.frequency(500000);
    
    spi_master.set_dma_usage(MYCONF_DMA_USAGE);
    spi_master.format(MYCONF_BITS_TRAN_UNIT_T);    // n bits per SPI frame
    // Fill in transmit buffer
    for (int i = 0; i < sizeof (spi_test_ctx.buf) / sizeof (spi_test_ctx.buf[0]); i ++) {
        spi_test_ctx.buf[i] = i;
    }
    
    // NOTE: Run spi_master first and then run spi_slave 3 secs. This is to keep cs inactive until spi_slave is ready.
#if (! MYCONF_SPI_AUTOSS)
    cs = 1;
#endif
    wait(3);

REPEAT:

    // Clear receive buffer
    memset(spi_test_ctx.buf2, 0xFF, sizeof (spi_test_ctx.buf2));
    
#if (! MYCONF_SPI_AUTOSS)
    cs = 0;
#if MYCONF_SPI_tSHSL_US
    wait_us(MYCONF_SPI_tSHSL_US);
#endif
#endif

    callback_event = 0;
    event_callback_t callback(spi_master_async_callback);
    if (spi_master.transfer(spi_test_ctx.buf, sizeof (spi_test_ctx.buf) / sizeof (spi_test_ctx.buf[0]), spi_test_ctx.buf2, sizeof (spi_test_ctx.buf2) / sizeof (spi_test_ctx.buf2[0]), callback, SPI_EVENT_ALL)) {
        printf("%s FAILED\n", __func__);
#if (! MYCONF_SPI_AUTOSS)
        cs = 1;
#endif
        while (1);
    }
    
    // Wait for asynchronous transfer done
    while (callback_event == 0);
    
#if (! MYCONF_SPI_AUTOSS)
    cs = 1;
#if MYCONF_SPI_tSHSL_US
    wait_us(MYCONF_SPI_tSHSL_US);
#endif
#endif

    if (callback_event & SPI_EVENT_ERROR) {
        printf("SPI_EVENT_ERROR\n");
        while (1);
    }
    if (callback_event & SPI_EVENT_RX_OVERFLOW) {
        printf("SPI_EVENT_RX_OVERFLOW\n");
        while (1);
    }
    if (callback_event & SPI_EVENT_COMPLETE) {
        printf("SPI_EVENT_COMPLETE\n");
        if (memcmp(spi_test_ctx.buf + MYCONF_SPI_ECHO_PLUS, spi_test_ctx.buf2 + 2, sizeof (spi_test_ctx.buf) - sizeof (MYCONF_TRAN_UNIT_T) * MYCONF_SPI_ECHO_PLUS)) {
            printf("%s Round %d FAILED\n", __func__, n_round ++);
            while (1);
        }
        else {
            printf("%s Round %d OK\n", __func__, n_round ++);
            goto REPEAT;
        }
    }
}

static void spi_master_async_callback(int event)
{
    callback_event = event;
}

static void test_spi_slave(void)
{
    static SPISlave spi_slave(SPI_MOSI, SPI_MISO, SPI_SCLK, SPI_SSEL);
    int resp = -1;

    spi_slave.format(MYCONF_BITS_TRAN_UNIT_T);              // n bits per SPI frame
    // NOTE: 
    // As STATUS[SLVURIF] is set, next data in transmit buffer are not transmitted correctly:
    // On NUC472, next data in transmit buffer can be transmitted out but is incorrect.
    // On M453, next data in transmit buffer cannot be transmitted out until next CS select/deselect. Clear this flag doesn't recover.
    spi_slave.reply(resp);                                  // Prime SPI with first reply
    spi_slave.reply(resp);                                  // Prime SPI with first reply
    
    while(1) {
        if (spi_slave.receive()) {
            resp = spi_slave.read();                        // Read byte from master
#if MYCONF_DEBUG
            if (MY_BUF_POS < sizeof (MY_BUF) / sizeof (MY_BUF[0])) {
                MY_BUF[MY_BUF_POS ++] = resp;
            }
#endif
            
            spi_slave.reply(resp + MYCONF_SPI_ECHO_PLUS);   // Make this the next reply
        }
    }
}

#endif  // #if DEVICE_SPI

static void test_i2c_master(void)
{
    static I2C i2c_master(I2C_SDA, I2C_SCL);
    
    int n_round = 0;
    
    // Fill in transmit buffer
    for (int i = 0; i < sizeof (i2c_test_ctx.buf) / sizeof (i2c_test_ctx.buf[0]); i ++) {
        i2c_test_ctx.buf[i] = i;
    }

REPEAT:
    i2c_master.write(MYCONF_I2C_ADDR, i2c_test_ctx.buf, sizeof (i2c_test_ctx.buf), 1);
    memset(i2c_test_ctx.buf2, 0xFF, sizeof (i2c_test_ctx.buf2));
    i2c_master.read(MYCONF_I2C_ADDR, i2c_test_ctx.buf2, sizeof (i2c_test_ctx.buf2), 1);
    
    i2c_master.write(MYCONF_I2C_ADDR, i2c_test_ctx.buf2, sizeof (i2c_test_ctx.buf2), 1);
    i2c_master.write(MYCONF_I2C_ADDR, i2c_test_ctx.buf2, sizeof (i2c_test_ctx.buf2), 1);
    i2c_master.write(MYCONF_I2C_ADDR, i2c_test_ctx.buf2, sizeof (i2c_test_ctx.buf2), 1);
    memset(i2c_test_ctx.buf2, 0xFF, sizeof (i2c_test_ctx.buf2));
    i2c_master.read(MYCONF_I2C_ADDR, i2c_test_ctx.buf2, sizeof (i2c_test_ctx.buf2), 1);
    
    i2c_master.write(MYCONF_I2C_ADDR, i2c_test_ctx.buf2, sizeof (i2c_test_ctx.buf2));
    memset(i2c_test_ctx.buf2, 0xFF, sizeof (i2c_test_ctx.buf2));
    i2c_master.read(MYCONF_I2C_ADDR, i2c_test_ctx.buf2, sizeof (i2c_test_ctx.buf2));
    
    i2c_master.write(MYCONF_I2C_ADDR, i2c_test_ctx.buf2, sizeof (i2c_test_ctx.buf2));
    i2c_master.write(MYCONF_I2C_ADDR, i2c_test_ctx.buf2, sizeof (i2c_test_ctx.buf2));
    i2c_master.write(MYCONF_I2C_ADDR, i2c_test_ctx.buf2, sizeof (i2c_test_ctx.buf2));
    memset(i2c_test_ctx.buf2, 0xFF, sizeof (i2c_test_ctx.buf2));
    i2c_master.read(MYCONF_I2C_ADDR, i2c_test_ctx.buf2, sizeof (i2c_test_ctx.buf2));
    
    i2c_master.write(MYCONF_I2C_ADDR, i2c_test_ctx.buf2, sizeof (i2c_test_ctx.buf2));
    memset(i2c_test_ctx.buf2, 0xFF, sizeof (i2c_test_ctx.buf2));
    i2c_master.read(MYCONF_I2C_ADDR, i2c_test_ctx.buf2, sizeof (i2c_test_ctx.buf2));
    memset(i2c_test_ctx.buf2, 0xFF, sizeof (i2c_test_ctx.buf2));
    i2c_master.read(MYCONF_I2C_ADDR, i2c_test_ctx.buf2, sizeof (i2c_test_ctx.buf2));
    memset(i2c_test_ctx.buf2, 0xFF, sizeof (i2c_test_ctx.buf2));
    i2c_master.read(MYCONF_I2C_ADDR, i2c_test_ctx.buf2, sizeof (i2c_test_ctx.buf2));
    
    char *buf_pos = i2c_test_ctx.buf;
    char *buf_end = i2c_test_ctx.buf + sizeof (i2c_test_ctx.buf) / sizeof (i2c_test_ctx.buf[0]);
    char *buf2_pos = i2c_test_ctx.buf2;
    int data;
    int data2;
    while (buf_pos < buf_end) {
        data = *buf_pos ++;
        data2 = *buf2_pos ++;
        if (data2 != (data + 5)) {
            printf("%s Round %d FAILED\n", __func__, n_round ++);
            while (1);
        }
    }

    printf("%s Round %d OK\n", __func__, n_round ++);
    goto REPEAT;
}

static void test_i2c_master_async(void)
{
    static I2C i2c_master(I2C_SDA, I2C_SCL);
    
    int n_round = 0;

    // Fill in transmit buffer
    for (int i = 0; i < sizeof (i2c_test_ctx.buf) / sizeof (i2c_test_ctx.buf[0]); i ++) {
        i2c_test_ctx.buf[i] = i;
    }
    
REPEAT:
    // Clear receive buffer
    memset(i2c_test_ctx.buf2, 0xFF, sizeof (i2c_test_ctx.buf2));
    
    callback_event = 0;
    event_callback_t callback(i2c_master_async_callback);
    if (i2c_master.transfer(MYCONF_I2C_ADDR, i2c_test_ctx.buf, sizeof (i2c_test_ctx.buf) / sizeof (i2c_test_ctx.buf[0]), i2c_test_ctx.buf2, sizeof (i2c_test_ctx.buf2) / sizeof (i2c_test_ctx.buf2[0]), callback, I2C_EVENT_ALL, 0)) {
        printf("%s FAILED\n", __func__);
        while (1);
    }
    
    // Wait for asynchronous transfer done
    while (callback_event == 0);

    if (callback_event & I2C_EVENT_ERROR) {
        printf("I2C_EVENT_ERROR\n");
        while (1);
    }
    if (callback_event & I2C_EVENT_ERROR_NO_SLAVE) {
        printf("I2C_EVENT_ERROR_NO_SLAVE\n");
        while (1);
    }
    if (callback_event & I2C_EVENT_TRANSFER_EARLY_NACK) {
        printf("I2C_EVENT_TRANSFER_EARLY_NACK\n");
        while (1);
    }
    if (callback_event & I2C_EVENT_TRANSFER_COMPLETE) {
        printf("I2C_EVENT_TRANSFER_COMPLETE\n");
        if (memcmp(i2c_test_ctx.buf + 1, i2c_test_ctx.buf2, sizeof (i2c_test_ctx.buf) - sizeof (char) * 1)) {
            printf("%s Round %d FAILED\n", __func__, n_round ++);
            while (1);
        }
        else {
            printf("%s Round %d OK\n", __func__, n_round ++);
            goto REPEAT;
        }
    }
}

static void i2c_master_async_callback(int event)
{
    callback_event = event;
}

static void test_i2c_slave(void)
{
    static I2CSlave i2c_slave(I2C_SDA, I2C_SCL);

    i2c_slave.address(MYCONF_I2C_ADDR);

    while (1) {
        int addr_status = i2c_slave.receive();
        switch (addr_status) {
            case I2CSlave::ReadAddressed:
                i2c_slave.write(i2c_test_ctx.buf, sizeof (i2c_test_ctx.buf));
                break;
            case I2CSlave::WriteAddressed:
                i2c_slave.read(i2c_test_ctx.buf, sizeof (i2c_test_ctx.buf));
                for(int i = 0; i < sizeof (i2c_test_ctx.buf); i ++) {
                    i2c_test_ctx.buf[i] ++;
                }
                break;
        }
    }
}


static void test_interruptin(void)
{
    static InterruptIn int_in1(BTN1);
    static InterruptIn int_in2(BTN2);

    //TESTTEST
    //int_in1.mode(Quasi);
    //int_in2.mode(Quasi);
    
    int_in1.rise(my_gpio_irq_rise);
    int_in1.fall(my_gpio_irq_fall);
    
    int_in2.rise(my_gpio_irq_rise);
    int_in2.fall(my_gpio_irq_fall);
}

static void my_gpio_irq_rise(void)
{
    led2 = ! led2;
    //printf("Detected GPIO IRQ Rise\n");
}
static void my_gpio_irq_fall(void)
{
    led2 = ! led2;
    //printf("Detected GPIO IRQ Fall\n");
}