//#include "libc/syscall.h"
//#include "string.h"
//#include "libusart_fields.h"
//#include "libusart.h"
//#include "libdrviso7816.h"
//#include "libc/semaphore.h"
//#include "generated/smartcard.h"
//#include "generated/led0.h"
//#include "generated/dfu_button.h"

#include "platform.h"
#include "libdrviso7816.h"

#define SMARTCARD_USART 0

cb_usart_getc_t platform_SC_usart_getc = NULL;
cb_usart_putc_t platform_SC_usart_putc = NULL;
static void platform_smartcard_irq(uint32_t status, uint32_t data);

static usart_config_t smartcard_usart_config = {
        .set_mask = USART_SET_ALL,
        .mode = SMARTCARD,
        .usart = SMARTCARD_USART,

        /* To be filled later depending on the clock configuration */
        .baudrate = 0,

        /* Word length = 9 bits (8 bits + parity) */
        .word_length = USART_CR1_M_9,

        /* 1 stop bit instead of 1.5 is necessary for some cards that use a very short delay between USART characters ... */
        .stop_bits = USART_CR2_STOP_1BIT,

        /* 8 bits plus parity  (parity even) */
        .parity = USART_CR1_PCE_EN | USART_CR1_PS_EVEN,

        /* Hardware flow control disabled, smartcard mode enabled, smartard
         * NACK enabled, HDSEL and IREN disabled, error interrupts enabled (for framing
         * error) */
        .hw_flow_control = USART_CR3_CTSE_CTS_DIS | USART_CR3_RTSE_RTS_DIS |
                           USART_CR3_SCEN_EN | USART_CR3_NACK_EN | USART_CR3_HDSEL_DIS |
                           USART_CR3_IREN_DIS | USART_CR3_EIE_EN,

        /* TX and RX are enabled, parity error interrupt enabled */
        .options_cr1 = USART_CR1_TE_EN | USART_CR1_RE_EN | USART_CR1_PEIE_EN |
                       USART_CR1_RXNEIE_EN | USART_CR1_TCIE_EN,

        /* LINEN disabled, USART clock enabled, CPOL low, CPHA 1st edge, last bit clock pulse enabled */
        .options_cr2 = USART_CR2_LINEN_DIS | USART_CR2_CLKEN_PIN_EN |
                       USART_CR2_CPOL_DIS | USART_CR2_CPHA_DIS | USART_CR2_LBCL_EN,

        /* To be filled later depending on the clock configuration */
        .guard_time_prescaler = 0,

        /* Send/Receive/Error IRQ callback */
        .callback_irq_handler = platform_smartcard_irq,

        /* Receive and send function pointers */
        .callback_usart_getc_ptr = &platform_SC_usart_getc,
        .callback_usart_putc_ptr = &platform_SC_usart_putc,
};

/* Initialize the CONTACT pin */

void platform_smartcard_register_user_handler_action(void (*action)(void))
{
	return;
}



void exti_handler(uint8_t irq __attribute__((unused)),
                  uint32_t status __attribute__((unused)),
                  uint32_t data __attribute__((unused)))
{
	return;
}

uint8_t platform_early_gpio_init(void)
{
  return 0;
}


static inline void toggle_smartcard_led_on(void){
	return;
}

static inline void toggle_smartcard_led_off(void){
	return;
}

static inline void toggle_smartcard_led(){
	return;
}

void platform_set_smartcard_rst(uint8_t val)
{
}

void platform_set_smartcard_vcc(uint8_t val)
{
}


static uint8_t platform_early_usart_init(drv7816_map_mode_t map_mode)
{
  return 0;
}

int platform_smartcard_map(void)
{
    return 0;
}

int platform_smartcard_unmap(void)
{
    return 0;
}


/* The SMARTCARD_CONTACT pin is at state high (pullup to Vcc) when no card is
 * not present, and at state low (linked to GND) when the card is inserted.
 */
uint8_t platform_is_smartcard_inserted(void)
{
	return 1;
}

void platform_smartcard_lost(void)
{
}

/* Initialize the USART in smartcard mode as
 * described in the datasheet, as well as smartcard
 * associated GPIOs.
 */
static int platform_smartcard_clocks_init(usart_config_t *config, uint32_t *target_freq, uint8_t target_guard_time, uint32_t *etu)
{
        return 0;
}

static volatile uint8_t platform_SC_pending_receive_byte = 0;
static volatile uint8_t platform_SC_pending_send_byte = 0;
static volatile uint8_t platform_SC_byte = 0;


int platform_smartcard_early_init(drv7816_map_mode_t map_mode)
{
	return 0;
}

int platform_smartcard_init(void){
	/* Reinitialize global variables */
	platform_SC_pending_receive_byte = 0;
	platform_SC_pending_send_byte = 0;
	platform_SC_byte = 0;

	return 0;
}

void platform_smartcard_reinit(void){
	return;
}

/* Adapt clocks and guard time depending on what has been received */
int platform_SC_adapt_clocks(uint32_t *etu, uint32_t *frequency){
	return 0;
}

/*
 * Low level related functions: we handle the low level USAT/smartcard
 * bytes send and receive stuff here.
 */

/* The following buffer is a circular buffer holding the received bytes when
 * an asynchronous burst of ISRs happens (i.e. when sending/receiving many bytes
 * in a short time slice).
 */
static uint8_t received_SC_bytes[64];
volatile unsigned int received_SC_bytes_start = 0;
volatile unsigned int received_SC_bytes_end   = 0;
/* The mutex for handling the reception ring buffer between ISR and main thread */
static volatile uint32_t SC_mutex;

volatile unsigned int received = 0;

static void platform_smartcard_irq(uint32_t status __attribute__((unused)), uint32_t data){
	/* Dummy read variable */
	uint8_t dummy_usart_read = 0;
	/* Check if we have a parity error */
	if ((get_reg(&status, USART_SR_PE)) && (platform_SC_pending_send_byte != 0)) {
		/* Parity error, program a resend */
		platform_SC_pending_send_byte = 3;
		/* Dummy read of the DR register to ACK the interrupt */
		dummy_usart_read = data & 0xff;
		return;
	}

	/* Check if we have a framing error */
	if ((get_reg(&status, USART_SR_FE)) && (platform_SC_pending_send_byte != 0)) {
		/* Frame error, program a resend */
		platform_SC_pending_send_byte = 4;
		/* Dummy read of the DR register to ACK the interrupt */
		dummy_usart_read = data & 0xff;
		return;
	}

	/* We have sent our byte */
	if ((get_reg(&status, USART_SR_TC)) && (platform_SC_pending_send_byte != 0)) {
		/* Clear TC, not needed here (done in posthook) */
		/* Signal that the byte has been sent */
		platform_SC_pending_send_byte = 2;
		return;
	}

	/* We can actually read data */
	if (get_reg(&status, USART_SR_RXNE)){
		/* We are in our sending state, no need to treceive anything */
		if(platform_SC_pending_send_byte != 0){
			return;
		}
		/* Lock the mutex */
		if(!mutex_trylock(&SC_mutex)){
			/* We should not be blocking when locking the mutex since we are in ISR mode!
			 * This means that we will miss bytes here ... But this is better than corrupting our
			 * reception ring buffer!
			 */
			return;
		}

		/* Check if we overflow */
		/* We have no more room to store bytes, just give up ... and
		 * drop the current byte
		 */
		if(((received_SC_bytes_end + 1) % sizeof(received_SC_bytes)) == received_SC_bytes_start){
			/* Unlock the mutex */
            mutex_unlock(&SC_mutex);
            dummy_usart_read = data & 0xff;
            return;
		}
		if(received_SC_bytes_end >= sizeof(received_SC_bytes)){
			/* This check should be unnecessary due to the modulus computation
			 * performed ahead (which is the only update to received_SC_bytes_end),
			 * but better safe than sorry!
             *
			 */

            mutex_unlock(&SC_mutex);
			/* Overflow, get out */
			return;
		}
		received_SC_bytes[received_SC_bytes_end] = data & 0xff;
		/* Wrap up our ring buffer */
		received_SC_bytes_end = (received_SC_bytes_end + 1) % sizeof(received_SC_bytes);
		platform_SC_pending_receive_byte = 1;

		/* Unlock the mutex */
		mutex_unlock(&SC_mutex);

		return;
	}

	return;
}

/* Set the direct convention at low level */
int platform_SC_set_direct_conv(void){
	return 0;
}

/* Set the inverse convention at low level */
int platform_SC_set_inverse_conv(void){
	uint32_t old_mask;
	usart_config_t *config = &smartcard_usart_config;
	uint64_t t, start_tick, curr_tick;
	/* Dummy read variable */
	uint8_t dummy_usart_read = 0;

	/* Flush the pending received byte from the USART block */
	dummy_usart_read = (*usart_get_data_addr(SMARTCARD_USART)) & 0xff;
	/* ACK the pending parity errors */
	dummy_usart_read = get_reg(usart_get_status_addr(smartcard_usart_config.usart), USART_SR_PE);

	/* Reconfigure the usart with an ODD parity */
	if(config->mode != SMARTCARD){
		goto err;
	}
	old_mask = config->set_mask;
	/* Adapt the configuration at the USART level */
	config->set_mask = USART_SET_PARITY;
	config->parity = USART_CR1_PCE_EN | USART_CR1_PS_ODD,
	usart_init(&smartcard_usart_config);
	config->set_mask = old_mask;

	/* Get the pending byte again (with 9600 ETU at 372 timeout) to send the proper
	 * parity ACK to the card and continue to the next bytes ...
	 */
        t = ((uint64_t)9600 * 372 * 1000) / 3500000;
        start_tick = platform_get_microseconds_ticks();
        curr_tick = start_tick;
	while(platform_SC_getc((uint8_t*)&dummy_usart_read, 0, 0)){
		if((curr_tick - start_tick) > t){
			goto err;
		}
		curr_tick = platform_get_microseconds_ticks();
	}

	return 0;

err:
	return -1;
}

/* Low level flush of our receive/send state, in order
 * for the higher level to be sure that everything is clean
 */
void platform_SC_flush(void){
	/* Flushing the receive/send state is only a matter of cleaning
	 * our ring buffer!
	 */
	/* Lock the mutex */
	mutex_lock(&SC_mutex);
	platform_SC_pending_receive_byte = platform_SC_pending_send_byte = 0;
	received_SC_bytes_start = received_SC_bytes_end = 0;
	mutex_unlock(&SC_mutex);
	/* Toggle the smartcard led */
	toggle_smartcard_led();
}

/* Low level char PUSH/POP functions */
/* Smartcard putc and getc handling errors:
 * The getc function is non blocking */
int platform_SC_getc(uint8_t *c,
                     uint32_t timeout __attribute__((unused)),
                     uint8_t reset __attribute__((unused)))
{
    int ret = -1;

	if(c == NULL){
		goto invalid_input;
	}
	if(platform_SC_pending_receive_byte != 1) {
		goto invalid_input;
	}
	/* Lock the mutex */
    mutex_lock(&SC_mutex);

	/* Read our ring buffer to check if something is ready */
	if(received_SC_bytes_start == received_SC_bytes_end){
		/* Ring buffer is empty */
        goto err;
	}
	/* Data is ready, go ahead ... */
	if(received_SC_bytes_start >= sizeof(received_SC_bytes)) {
		/* This check should be unnecessary due to the modulus computation
		 * performed ahead (which is the only update to received_SC_bytes_start),
		 * but better safe than sorry!
		 */
		goto err;
	}
	*c = received_SC_bytes[received_SC_bytes_start];
	/* Wrap up our ring buffer */
	received_SC_bytes_start = (received_SC_bytes_start + 1) % sizeof(received_SC_bytes);
	/* Tell that there is no more byte when our ring buffer is empty */
	if(received_SC_bytes_start == received_SC_bytes_end){
		platform_SC_pending_receive_byte = 0;
	}
    ret = 0;

err:
    mutex_unlock(&SC_mutex);

invalid_input:

	return ret;
}

/* The putc function is non-blocking and checks
 * for errors. In the case of errors, try to send the byte again.
 */
int platform_SC_putc(uint8_t c,
                     uint32_t timeout __attribute__((unused)),
                     uint8_t reset){
	if(reset){
		platform_SC_pending_send_byte = 0;
		return 0;
	}
	if((platform_SC_pending_send_byte == 0) || (platform_SC_pending_send_byte >= 3)){
		platform_SC_pending_send_byte = 1;
		/* Push the byte on the line */
		(*usart_get_data_addr(SMARTCARD_USART)) = c;
		return -1;
	}
	if(platform_SC_pending_send_byte == 2){
		/* The byte has been sent */
		platform_SC_pending_send_byte = 0;
		return 0;
	}

	return -1;
}

/* Get ticks/time in microseconds */
static volatile uint64_t ticks = 0;
uint64_t platform_get_microseconds_ticks(void){
	//uint64_t tick = 0;
	//sys_get_systick(&tick, PREC_MICRO);
	ticks++;
	return ticks;
}


void platform_SC_reinit_smartcard_contact(void){
	return;
}
void platform_SC_reinit_iso7816(void){
	platform_SC_pending_receive_byte = 0;
	platform_SC_pending_send_byte = 0;
	platform_SC_byte = 0;
	received_SC_bytes_start = received_SC_bytes_end = 0;
	mutex_init(&SC_mutex);
	usart_init(&smartcard_usart_config);
	return;
}

int platform_smartcard_set_1ETU_guardtime(void){
        /* We are already at 1 ETU guard time, so return OK */
        return 0;
}
