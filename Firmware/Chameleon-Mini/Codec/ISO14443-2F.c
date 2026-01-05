//
// Created by l on 7/5/24.
//

#include "ISO14443-2F.h"
#include "../System.h"
#include "../Application/Application.h"
#include "../LEDHook.h"
#include "Codec.h"
#include "Log.h"

// ------------------------ LEGIC PRIME PRNG SECTION -------------------------
static struct legicPRNG_t {
    uint8_t a;
    uint8_t b;
    size_t step;
} legicPRNG;

void LegicPrimePRNGInit(uint8_t iv){
    legicPRNG.a = iv;
    legicPRNG.b = (iv << 1) | 1;
    legicPRNG.step = 0;
}
void LegicPrimePRNGAdvance(size_t steps){
    while(steps--){
        uint8_t new_b_bit = legicPRNG.b ^ (legicPRNG.b >> 2) ^ (legicPRNG.b >> 3) ^ (legicPRNG.b >> 7);
        legicPRNG.b = (new_b_bit << 7) | (legicPRNG.b >> 1);

        uint8_t new_a_bit = legicPRNG.a ^ (legicPRNG.a >> 6);
        legicPRNG.a = (new_a_bit << 6) | legicPRNG.a  >> 1;
    }
}
uint8_t LegicPrimePRNGGetBit(void){
    uint8_t index = (legicPRNG.a ^ 0x1C) >> 2; // Flip the bits 2,3 and 4 from a and move them, so they are LSB
    index = ((index & 4) >> 2) | (index & 2) | ((index & 1) << 2); // Reverse their direction
    return (legicPRNG.b >> index) & 1; // Select only one bit from b according to index made from a
}



// ------------------------ LEGIC CODEC SECTION -------------------------------
/* Sampling is done using internal clock, synchronized to the field modulation.
 * For that we need to convert the bit rate for the internal clock. */
// F_CPU = 2 * 13 560 000UL = Speed of the CPU, in Hz
// CODEC_CARRIER_FREQ = 13 560 000
// READER_SIGNAL_SAMPLE_RATE_IN_SYSTEM_CYCLES = (2 * 13 560 000 * ISO14443F_BIT_RATE_CYCLES) / 13 560 000 = 2 * ISO14443F_BIT_RATE_CYCLES

// Our "Bitrate in cycles" is F_CPU / bitrate
// But our "bitrate" is variable. Received bit 1 takes 80us HIGH and a bit 0 takes 40us HIGH.
// After both, there needs to be 20us of LOW.
// So we can effectively say that 1 is composited of 80us of HIGH and 20us LOW which takes 100us in total
// and 0 is composited of 40us of HIGH and 20us of LOW which takes 60us in total.
// GCD of 100 and 60 is 20, so we need to measure every 20us to be sure we are synced.
// Thus, every time we measure LOW, we'll look into our memory and if we encountered
// 4 HIGHs before, we have just received a 1 or if we read just 2 HIGHs, we have received a 0.
// So effectively we need to sample each 20us which makes our bitrate 50 kbps
// which in turn makes our BIT_RATE_CYCLES 542 (.4)
//
//
// v ^
// o |
// l |          1                 0                   1                     0
// t |  +----------------+    +--------+    +--*----*----*----*--+  L  +--*----*--+  L
// a |  |                |    |        |    |  *    *    *    *  |  *  |  *    *  |  *
// g |  |                |    |        |    |  *    *    *    *  |  *  |  *    *  |  *
// e |  |                +----+        +----+  H    H    H    H  +--*--+  H    H  +--*--
//   +-------------------------------------------------------------------------------------------> time
//
// a dash takes 5us, stars symbolise a measurement that should be every 20us
// we decode HHHHL as 1 and HHL as 0

#define READER_SIGNAL_SAMPLE_RATE_IN_SYSTEM_CYCLES		((uint16_t) (((uint64_t) F_CPU * ISO14443F_BIT_RATE_CYCLES) / CODEC_CARRIER_FREQ))
// This is 30 microseconds
#define FIRST_SAMPLING_OFFSET_IN_SYSTEM_CYCLES 659
#define SAMPLING_OFFSET_IN_SYSTEM_CYCLES 244
#define TRANSMIT_RATE_IN_SYSTEM_CYCLES  1361
#define ISO14443A_MIN_BITS_PER_FRAME		7

static volatile struct {
//    volatile bool LoadmodFinished;
} Flags = { 0 };

// Enum for states of the receive functions that demodulate data from the reader
// The cycle should be DONT -> DO -> END -> DONT
typedef enum {
    DONT_RECEIVE,
    DO_RECEIVE,
    END_RECEIVE //Give away control after all data have been received
} ReceiveStateType;


typedef enum {
    TRANSMIT_NONE,
    TRANSMIT_START,
    TRANSMIT_BIT,
    TRANSMIT_END
} TransmitStateType;

/* Define pseudo variables to use fast register access. This is useful for global vars */
#define ReceiveStateRegister	Codec8Reg0
#define TransmitStateRegister	Codec8Reg1
#define SampleIdxRegister Codec8Reg2
#define SampleRegister	Codec8Reg3
#define BitSent			CodecCount16Register1
#define BitCount		CodecCount16Register2
#define ParityBufferPtr	CodecPtrRegister2

/* Nastav pin PE0 na HIGH
* Používáno pro můj debug, PE0 není nijak potřeba pro reálný provoz karty
*/
INLINE void set_PE0_high(void){
    PORTE.DIRSET |= PIN0_bm;
    PORTE.OUTSET |= PIN0_bm;
}

/* Nastav pin PE0 na LOW
 * Používáno pro můj debug, PE0 není nijak potřeba pro reálný provoz karty
 * */
INLINE void set_PE0_low(void){
    PORTE.OUTCLR |= PIN0_bm;
}

INLINE void ISO14443_F_DEMOD_END(void) {

    SampleIdxRegister = 0;
    /* Disable demodulation interrupt */
    if (1) {
        CODEC_TIMER_SAMPLING.CTRLA = TC_CLKSEL_OFF_gc; /* Disconnect system clock from demod timer */
        CODEC_TIMER_SAMPLING.CTRLD = TC_EVACT_OFF_gc; /* Remove action from timer */
        CODEC_TIMER_SAMPLING.INTCTRLB = TC_CCAINTLVL_OFF_gc; /* Disable CCA interrupts */
        CODEC_TIMER_SAMPLING.INTFLAGS = TC0_CCAIF_bm; /* Clear CCA interrupt flag */
    }

    /* By this time, the LOADMOD timer is aligned to the last modulation
     * edge of the reader. So we disable the auto-synchronization and
     * let it count the frame delay time in the background, and generate
     * an interrupt once it has reached the FDT. */
    CODEC_TIMER_LOADMOD.CTRLD = TC_EVACT_OFF_gc; /* Disable restarts on modulation ends */
    CODEC_TIMER_LOADMOD.PERBUF = TRANSMIT_RATE_IN_SYSTEM_CYCLES; /* Prepare to change state every 100 us */
    CODEC_TIMER_LOADMOD.PER = 4334; /* +- 330 microseconds */ //TODO: Try 4334 for 320 microseconds as measured on Kaba
    CODEC_TIMER_LOADMOD.INTFLAGS = TC0_OVFIF_bm; /* Clear overflow interrupt flag */
    CODEC_TIMER_LOADMOD.INTCTRLA = TC_OVFINTLVL_HI_gc; /* Set overflow interrupt level to high */

    TransmitStateRegister = TRANSMIT_START;
//    TransmitSynced = 0;
    ReceiveStateRegister = END_RECEIVE;
    //Zero out unused bytes for logging
    for (uint16_t i = 0; i < (BitCount % 8); i++){
        CodecBuffer[(BitCount + 7) / 8] &= ~(1u << i);
    }
    LogEntry(LOG_INFO_CODEC_RX_DATA, CodecBuffer, (BitCount+7)/8 );
}

// v ^                                          Trigger here
// o |                                                |
// l |  Reader charging card/previous communication   V            1                 0
// t |------------------------------------------------|    +----------------+    +--------+
// a |                                                |    |                |    |        |
// g |                                                |    |                |    |        |
// e |                                                |----|                +----+        +-----
//   +-------------------------------------------------------------------------------------------> time
void EnableFirstModulationPauseInterrupt(void){
    /* Start looking out for modulation pause via interrupt. */
    CODEC_DEMOD_IN_PORT.INTFLAGS = PORT_INT0IF_bm; /* Clear the Interrupt 0 flag on the DEMOD port (Port B) */
    /* Set Pin 1 as source for Interrupt 0 on the DEMOD port (Port B) */
    CODEC_DEMOD_IN_PORT.INT0MASK = CODEC_DEMOD_IN_MASK0;
}

/* Funkce které vyčistí nastavení po tom co demodulujeme bordel */
INLINE void ISO14443_F_GARBAGE(void){
    SampleIdxRegister = 0;
    CODEC_TIMER_SAMPLING.CTRLA = TC_CLKSEL_OFF_gc; /* Disconnect system clock from demod timer */
    CODEC_TIMER_SAMPLING.CTRLD = TC_EVACT_OFF_gc; /* Remove action from timer */
    CODEC_TIMER_SAMPLING.INTCTRLB = TC_OVFINTLVL_OFF_gc; /* Disable CCA interrupts */
    CODEC_TIMER_SAMPLING.INTFLAGS = TC0_OVFIF_bm; /* Clear OVF interrupt flag */
    EnableFirstModulationPauseInterrupt();
}

void PrepareLoadmodTimer(void){
    CODEC_TIMER_LOADMOD.CTRLA = CODEC_TIMER_CARRIER_CLKSEL; /* Use Carrier wave as timer source */
    CODEC_TIMER_LOADMOD.CTRLD = TC_EVACT_RESTART_gc | CODEC_TIMER_MODEND_EVSEL; /* Restart time on modulation ends */
    CODEC_TIMER_LOADMOD.INTCTRLA = TC_OVFINTLVL_OFF_gc; /* Disable interrupt on overflow */
    CODEC_TIMER_LOADMOD.CNT = 0;
    CODEC_TIMER_LOADMOD.PER = 0xFFFF; /* Set period to too high of a value */
}

INLINE void StartDemod(void) {
    PrepareLoadmodTimer();

    /* Activate Power for demodulator */
    CodecSetDemodPower(true);
    ParityBufferPtr = &CodecBuffer[ISO14443A_BUFFER_PARITY_OFFSET];
    ReceiveStateRegister = DO_RECEIVE;
    SampleRegister = 0;
    SampleIdxRegister = 0;
    BitCount = 0;

    EnableFirstModulationPauseInterrupt();
}

/* This handles the interrupt raised after first event on the DEMOD pin
 * Starts CODEC_TIMER_SAMPLING for sampling the reader's data */
ISR_SHARED isr_ISO14443_F_CODEC_DEMOD_IN_INT0_VECT(void) {
    /* Configure sampling-timer free running and sync to first modulation-pause. */
    /* CodecInitCommon(); sets Event channel 0 to signal the beginning (rising edge) of a modulation pause and Event channel 1 to
     * signal the end (falling edge) of a modulation pause. */
    CODEC_TIMER_SAMPLING.CNT = 0; /* Reset the timer's initial value*/
    CODEC_TIMER_SAMPLING.PER = FIRST_SAMPLING_OFFSET_IN_SYSTEM_CYCLES; /* Set the timer's period, so we land +-10us into readers data signal */
    CODEC_TIMER_SAMPLING.PERBUF = READER_SIGNAL_SAMPLE_RATE_IN_SYSTEM_CYCLES; /* Set the timer's next period, so we sample each 20us */
    CODEC_TIMER_SAMPLING.CTRLA = TC_CLKSEL_DIV1_gc;  /* Select the system clock (with no prescaler) as the timer source */
    CODEC_TIMER_SAMPLING.CTRLD = TC_EVACT_OFF_gc; /* Turn of any event actions */
    CODEC_TIMER_SAMPLING.INTCTRLA = TC_OVFINTLVL_HI_gc; /* Mark timer overflow interrupt as high level */
    CODEC_TIMER_SAMPLING.INTFLAGS = TC0_OVFIF_bm; /* Clear timer overflow interrupt flag */

    /* Disable this interrupt. From now on we will sample the field using our CODEC_TIMER_SAMPLING */
    CODEC_DEMOD_IN_PORT.INT0MASK = 0;

}
void disable_loadmod_timer(void){
    CODEC_TIMER_LOADMOD.CTRLA = TC_CLKSEL_OFF_gc;
    CODEC_TIMER_LOADMOD.INTCTRLA = TC_OVFINTLVL_OFF_gc;
}

void set_bit_on_position_in_buffer_to_value(volatile uint8_t * buffer, uint16_t position, uint8_t value){
    uint16_t byte_offset = position / 8;
    uint8_t bit_offset = position % 8;

    if (value){
        buffer[byte_offset] |= (1 << bit_offset);        // Set bit to 1
    } else {
        buffer[byte_offset] &= ~(1 << bit_offset);        // Set bit to 0
    }
}

uint8_t get_bit_on_position_in_buffer(const uint8_t * buffer, uint16_t position){
    uint16_t byte_offset = position / 8;
    uint8_t bit_offset = position % 8;

    return (buffer[byte_offset] & (1 << bit_offset)) >> bit_offset;
}

// This function translates raw signal from the SamplePin to logical bits by reading SamplePin's value
// if 2 highs and a low are observed, it stores a logical 0 into CodecBuffer
// if 4 highs and a low are observed, it stores a logical 1 into CodecBuffer
INLINE void demodulate_reader_bit(void){
    SampleIdxRegister++;

    uint8_t SamplePin = CODEC_DEMOD_IN_PORT.IN & CODEC_DEMOD_IN_MASK;

    /* Shift sampled bit into sampling register */
    // Sample pin očekávám z téhle negace, že mi vrací 1 když naměřil LOW a 0 když naměřil HIGH

    SampleRegister = (SampleRegister << 1) | (!SamplePin ? 0x01 : 0x00);

    if (!(SampleRegister & 0x1)) { // if last read bit is a zero
        //SynchronizeSamplingTimerToDemodEnd();
        if (!(SampleRegister ^ 0x1E)) {
            // We have read a 1
            set_bit_on_position_in_buffer_to_value(CodecBuffer, BitCount, 1);
            BitCount++;
        } else if (!(SampleRegister ^ 0x06)) {
            // We have read a 0
            set_bit_on_position_in_buffer_to_value(CodecBuffer, BitCount, 0);
            BitCount++;
        } else {
            ISO14443_F_GARBAGE();
        }
        SampleRegister = 0;
        SampleIdxRegister = 0;
    }

    if (SampleIdxRegister > 4) {
        // No more bits to be read or an error occurred during transmission as 5x HIGH should not happen
        if (BitCount > 0){
            ISO14443_F_DEMOD_END();
        } else {
            ISO14443_F_GARBAGE();
        }
    }
}

//This triggers every 20us and samples the readers field
ISR_SHARED isr_ISO14443_F_CODEC_TIMER_SAMPLING_OVF_VECT(void){
    CODEC_TIMER_SAMPLING.INTFLAGS = TC0_OVFIF_bm; /* Clear timer overflow interrupt flag */

    if (Flags.PRNGInitialized){
        Flags.TickCounter++;
        if (Flags.TickCounter == 5){
//            LegicPrimePRNGAdvance(1);
            Flags.TickCounter = 0;
        }
    }
    if (ReceiveStateRegister == DO_RECEIVE) {
        demodulate_reader_bit();
    }

}

// Modulate as a card to send card response
ISR_SHARED isr_ISO14443_F_CODEC_TIMER_LOADMOD_OVF_VECT(void) {
    static void *JumpTable[] = {
            [TRANSMIT_NONE] = && TRANSMIT_NONE_LABEL,
            [TRANSMIT_START] = && TRANSMIT_START_LABEL,
            [TRANSMIT_BIT] = && TRANSMIT_BIT_LABEL,
            [TRANSMIT_END] = && TRANSMIT_END_LABEL
    };

    //TODO: Přidej label ochranu
    if ((TransmitStateRegister >= TRANSMIT_NONE) && (TransmitStateRegister <= TRANSMIT_END)) {
        goto *JumpTable[TransmitStateRegister];
    } else {
        TerminalSendString("ERROR: Jump to unregistered label!\r\n");
        return;
    }

TRANSMIT_NONE_LABEL:
//    TerminalSendString("Transmit label none \r\n");
    return;

TRANSMIT_START_LABEL:
    TransmitStateRegister = TRANSMIT_BIT;
    BitSent = 0;
    CodecSetSubcarrier(CODEC_SUBCARRIERMOD_OOK, ISO14443F_SUBCARRIER_DIVIDER);
    CodecStartSubcarrier();
    /* Fallthrough */
TRANSMIT_BIT_LABEL:
    CodecSetLoadmodState(get_bit_on_position_in_buffer(CodecBuffer, BitSent));
    BitSent++;
    if (BitSent >= BitCount){
        TransmitStateRegister = TRANSMIT_END;
    }
    return;

TRANSMIT_END_LABEL:
    TransmitStateRegister = TRANSMIT_NONE;
    CodecSetLoadmodState(false);
    CodecSetSubcarrier(CODEC_SUBCARRIERMOD_OFF, 0);

    disable_loadmod_timer();
    StartDemod();
}

void ISO14443FCodecInit(void) {
    TerminalSendString("Legic ISO14443FCodecInit\r\n");

    /* Initialize some global vars and start looking out for reader commands */
    //Flags.LoadmodFinished = 0;
    ReceiveStateRegister = DONT_RECEIVE;
    TransmitStateRegister = TRANSMIT_NONE;

    isr_func_CODEC_DEMOD_IN_INT0_VECT = &isr_ISO14443_F_CODEC_DEMOD_IN_INT0_VECT;
    isr_func_CODEC_TIMER_SAMPLING_OVF_vect = &isr_ISO14443_F_CODEC_TIMER_SAMPLING_OVF_VECT;
    isr_func_CODEC_TIMER_LOADMOD_OVF_VECT = &isr_ISO14443_F_CODEC_TIMER_LOADMOD_OVF_VECT;
    isr_func_CODEC_TIMER_SAMPLING_CCA_vect = &isr_ISO14443_F_CODEC_TIMER_SAMPLING_CCA_VECT;
    CodecInitCommon();
    StartDemod();
}

void ISO14443FCodecDeInit(void) {
    /* Gracefully shutdown codec */
    CODEC_DEMOD_IN_PORT.INT0MASK = 0;
    //Flags.LoadmodFinished = 0;
    ReceiveStateRegister = DONT_RECEIVE;
    TransmitStateRegister = TRANSMIT_NONE;


    CODEC_TIMER_SAMPLING.CTRLA = TC_CLKSEL_OFF_gc;
    CODEC_TIMER_SAMPLING.CTRLD = TC_EVACT_OFF_gc;
    CODEC_TIMER_SAMPLING.INTCTRLB = TC_CCAINTLVL_OFF_gc;
    CODEC_TIMER_SAMPLING.INTFLAGS = TC0_CCAIF_bm;


    CODEC_TIMER_LOADMOD.CTRLA = TC_CLKSEL_OFF_gc;
    CODEC_TIMER_LOADMOD.CTRLD = TC_EVACT_OFF_gc;
    CODEC_TIMER_LOADMOD.INTCTRLA = TC_OVFINTLVL_OFF_gc;
    CODEC_TIMER_LOADMOD.INTFLAGS = TC0_OVFIF_bm;

    CodecSetSubcarrier(CODEC_SUBCARRIERMOD_OFF, 0);
    CodecSetDemodPower(false);
    CodecSetLoadmodState(false);

}

void ISO14443FCodecTask(void) {

    if (ReceiveStateRegister == END_RECEIVE) {

        // Zablikej, že jsme přijali data
        LEDHook(LED_CODEC_RX, LED_PULSE);
        // Zaloguj přijatá data - TODO: bity ukládáme jako byty
        //LogEntry(LOG_INFO_CODEC_RX_DATA, CodecBuffer, BitCount);

        uint16_t AnswerBitCount;
        // Zavolej aplikační vrstvu
        AnswerBitCount = ApplicationProcess(CodecBuffer, BitCount);

        if (AnswerBitCount != ISO14443F_APP_NO_RESPONSE) {
            ReceiveStateRegister = DONT_RECEIVE;
            LogEntry(LOG_INFO_CODEC_TX_DATA, CodecBuffer, (AnswerBitCount + 7) / 8);
            BitCount = AnswerBitCount;
            TransmitStateRegister = TRANSMIT_START;
        } else {
            /* No data to be processed. Disable loadmodding and start listening again */
            disable_loadmod_timer();
            StartDemod();
        }
    }
}

