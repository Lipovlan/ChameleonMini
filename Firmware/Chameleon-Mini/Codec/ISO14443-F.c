//
// Created by l on 7/5/24.
//

#include "ISO14443-F.h"
#include "../System.h"
#include "../Application/Application.h"
#include "../LEDHook.h"
#include "Codec.h"
#include "Log.h"



/* Sampling is done using internal clock, synchronized to the field modulation.
 * For that we need to convert the bit rate for the internal clock. */
// F_CPU = 2 * 13 560 000UL = Speed of the CPU, in Hz
// CODEC_CARRIER_FREQ = 13 560 000
// SAMPLE_RATE_IN_SYSTEM_CYCLES = (2 * 13 560 000 * ISO14443F_BIT_RATE_CYCLES) / 13 560 000 = 2 * ISO14443F_BIT_RATE_CYCLES

// Our "Bitrate in cycles" is F_CPU / bitrate
// But our "bitrate" is variable. Received bit 1 takes 80us HIGH and a bit 0 takes 40us HIGH.
// After both, there needs to be 20us of LOW.
// So we can effectively say that 1 is composited of 80us of HIGH and 20us LOW which takes 100us in total
// and 0 is composited of 40us of HIGH and 20us of LOW which takes 60us in total.
// GCD of 100 and 60 is 20, so we need to measure every 20us to be sure we are synced.
// Thus, every time we measure LOW, we'll look into our memory and if we encountered
// 4 HIGHs before, we have just received a 1 or if we read just 3 HIGHs, we have received a 0.
// So effectively we need to sample each 20us which makes our bitrate 50 kbps
// which in turn makes our BIT_RATE_CYCLES 542 (.4)
//
//
// v |
// o |
// l |          1                 0                   1                        0
// t |  +----------------+    +--------+    +--*----*----*----*--+  L  +--*----*----*--+  L
// a |  |                |    |        |    |  *    *    *    *  |  *  |  *    *    *  |  *
// g |  |                |    |        |    |  *    *    *    *  |  *  |  *    *    *  |  *
// e |  |                +----+        +----+  H    H    H    H  +--*--+  H    H    H  +--*--
//   +-------------------------------------------------------------------------------------------- time
//
// a dash takes 10us, stars symbolise a measurement that should be every 20us
// we decode HHHHL as 1 and HHHL as 0

//#define SAMPLE_RATE_IN_SYSTEM_CYCLES		((uint16_t) (((uint64_t) F_CPU * ISO14443A_BIT_RATE_CYCLES) / CODEC_CARRIER_FREQ) )
#define SAMPLE_RATE_IN_SYSTEM_CYCLES		((uint16_t) (((uint64_t) F_CPU * ISO14443F_BIT_RATE_CYCLES) / CODEC_CARRIER_FREQ) )

#define ISO14443A_MIN_BITS_PER_FRAME		7

static volatile struct {
    volatile bool DemodFinished;
    volatile bool LoadmodFinished;
} Flags = { 0 };

typedef enum {
    TRANSMIT_NONE,
    TRANSMIT_START,
    TRANSMIT_END
} StateType;

/* Define pseudo variables to use fast register access. This is useful for global vars */
#define DataRegister	Codec8Reg0
// TODO: Nepoužíváme StateRegister
#define StateRegister	Codec8Reg1
#define ParityRegister	Codec8Reg2
#define SampleIdxRegister Codec8Reg2
#define SampleRegister	Codec8Reg3
#define BitSent			CodecCount16Register1
#define BitCount		CodecCount16Register2
#define CodecBufferPtr	CodecPtrRegister1
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
    TerminalSendString("DEMOD END \r\n");

    SampleIdxRegister = 0;
    Flags.DemodFinished = 1;
    /* Disable demodulation interrupt */
    CODEC_TIMER_SAMPLING.CTRLA = TC_CLKSEL_OFF_gc; /* Disconnect system clock from demod timer */
    CODEC_TIMER_SAMPLING.CTRLD = TC_EVACT_OFF_gc; /* Remove action from timer */
    CODEC_TIMER_SAMPLING.INTCTRLB = TC_CCAINTLVL_OFF_gc; /* Disable CCA interrupts */
    CODEC_TIMER_SAMPLING.INTFLAGS = TC0_CCAIF_bm; /* Clear CCA interrupt flag */


    /* Enable loadmodulation interrupt */
    // TODO: Počkej nějakej čas než začneš samplovat?
    CODEC_TIMER_LOADMOD.PER = 1230;
    /* By this time, the FDT timer is aligned to the last modulation
     * edge of the reader. So we disable the auto-synchronization and
     * let it count the frame delay time in the background, and generate
     * an interrupt once it has reached the FDT. */
    CODEC_TIMER_LOADMOD.CNT = 0;
    CODEC_TIMER_LOADMOD.PER = 0xFFFF;
    CODEC_TIMER_LOADMOD.CTRLA = CODEC_TIMER_CARRIER_CLKSEL;
    CODEC_TIMER_LOADMOD.CTRLD = TC_EVACT_OFF_gc;
    CODEC_TIMER_LOADMOD.INTFLAGS = TC0_OVFIF_bm;
    CODEC_TIMER_LOADMOD.INTCTRLA = TC_OVFINTLVL_HI_gc;
}

/* Funkce které vyčistí nastavení po tom co demodulujeme bordel */
INLINE void ISO14443_F_GARBAGE(void){
    //TODO: Nic z tohohle není nejspíš správně, mělo by to jen vyčistit nastavení
    TerminalSendString("GARBAGE \r\n");

    ISO14443_F_DEMOD_END();
    //    Flags.DemodFinished = 1;
//    /* No carrier modulation for 3 sample points. EOC! */
//    /* Disable demodulation interrupt */
//    /* Sets timer off for TCD0, disabling clock source. We're done receiving data from reader and don't need to probe the antenna anymore*/
//    CODEC_TIMER_SAMPLING.CTRLA = TC_CLKSEL_OFF_gc;
//    /* Clear Compare Channel C (CCC) interrupt Flags */
//   CODEC_TIMER_SAMPLING.INTFLAGS = TC0_CCAIF_bm;
//
//   // TODO: Počkej nějakej čas než začneš samplovat?
//    CODEC_TIMER_LOADMOD.PER = 1230;
//
//   /* By this time, the FDT timer is aligned to the last modulation
//    * edge of the reader. So we disable the auto-synchronization and
//    * let it count the frame delay time in the background, and generate
//    * an interrupt once it has reached the FDT. */
//    /* Enable loadmodulation interrupt */
//    /* Disable the event action for TCE0 */
//    CODEC_TIMER_LOADMOD.CTRLD = TC_EVACT_OFF_gc;
//    /* Clear TCE0 interrupt flags for Capture Channel B */
//    CODEC_TIMER_LOADMOD.INTFLAGS = TC0_OVFIF_bm;
//    CODEC_TIMER_LOADMOD.INTCTRLA = TC_OVFINTLVL_HI_gc;
}
static void StartDemod(void) {
    TerminalSendString("Legic start demod\r\n");

    /* Activate Power for demodulator */
    CodecSetDemodPower(true);

    CodecBufferPtr = CodecBuffer;
    ParityBufferPtr = &CodecBuffer[ISO14443A_BUFFER_PARITY_OFFSET];
    DataRegister = 0;
    SampleRegister = 0;
    SampleIdxRegister = 0;
    BitCount = 0;
    StateRegister = TRANSMIT_NONE;

    /* Configure sampling-timer free running and sync to first modulation-pause. */
    CODEC_TIMER_SAMPLING.CNT = 0; /* Reset the timer's initial value*/
    CODEC_TIMER_SAMPLING.PER = SAMPLE_RATE_IN_SYSTEM_CYCLES - 1; /* Set the timer's period one smaller because PER is 0-based*/
    CODEC_TIMER_SAMPLING.CTRLA = TC_CLKSEL_DIV1_gc;  /* Select the system clock (with no prescaler) as the timer source */
    /* Set up  timer's action/delay/source:
     *   - Event Action: Restart waveform period (set CNT to zero, direction to addition and clear all timer outputs)
     *   - Event Delay: None
     *   - Event Source Select: Trigger on CODEC_TIMER_MODSTART_EVSEL = TC_EVSEL_CH0_gc = Event Channel 0 */
    CODEC_TIMER_SAMPLING.CTRLD = TC_EVACT_RESTART_gc | CODEC_TIMER_MODSTART_EVSEL;

    /* Temporarily disable Compare Channel A (CCA) interrupts.
     * They'll be enabled later in isr_ISO14443_F_CODEC_DEMOD_IN_INT0_VECT once we sensed the reader is sending data and we're in sync with the pulses */
    CODEC_TIMER_SAMPLING.INTCTRLB = TC_CCAINTLVL_HI_gc;
    CODEC_TIMER_SAMPLING.INTFLAGS = TC0_CCAIF_bm; /* Clear CCA interrupt flag */
    CODEC_TIMER_SAMPLING.CCA = 0xFFFF; /* Disable CCA interrupt */

    /* Start looking out for modulation pause via interrupt. */
    /* Clear the interrupt flag on the port */
    CODEC_DEMOD_IN_PORT.INTFLAGS = PORT_INT0IF_bm;
    /* Set pin 1 as source for interrupt 0*/
    CODEC_DEMOD_IN_PORT.INT0MASK = CODEC_DEMOD_IN_MASK0;
}

/* This handles the interrupt raised after first event on the DEMOD pin
 * Starts CODEC_TIMER_SAMPLING for sampling the reader's data */
ISR_SHARED isr_ISO14443_F_CODEC_DEMOD_IN_INT0_VECT(void) {

//    CODEC_TIMER_SAMPLING.INTFLAGS = TC0_CCAIF_bm; /* Clear CCA interrupt flag */
//    CODEC_TIMER_SAMPLING.INTCTRLB = TC_CCAINTLVL_HI_gc; /* Re-enable CCA interrupts */
    //TODO: WIP
//    CODEC_TIMER_SAMPLING.CNT = 0;

//    /* This is the first edge of the first modulation-pause after StartDemod.
//     * Now we have time to start
//     * demodulating beginning from one bit-width after this edge. */
//
//    /* Sampling timer has been preset in StartDemod() to sample-rate and has automatically synced
//     * to THIS first modulation pause. Thus after exactly one bit-width from here,
//     * an OVF is generated. We want to start sampling with the next bit and use the
//     * XYZBUF mechanism of the xmega to automatically double the sampling rate on the
//     * next overflow. For this we have to temporarily deactivate the automatical alignment
//     * in order to catch next overflow event for updating the BUF registers.*/
    CODEC_TIMER_SAMPLING.CTRLD = TC_EVACT_OFF_gc;
    CODEC_TIMER_SAMPLING.PERBUF = SAMPLE_RATE_IN_SYSTEM_CYCLES - 1; /* Perioda samplování v CPU cyklech */
    /* Počet CPU cyklů po kterých začne samplování. Ty se počítají (nevím) buď
     * a) hned po sestupné hraně DEMOD pinu
     * b) jednu periodu po vzestupné hraně DEMOD pinu - spíš asi tohle*/
    CODEC_TIMER_SAMPLING.CCABUF = SAMPLE_RATE_IN_SYSTEM_CYCLES / 2 ;

//    /* Setup Frame Delay Timer and wire to EVSYS. Frame delay time is
//     * measured from last change in RF field, therefore we use
//     * the event channel 1 (end of modulation pause) as the restart event.
//     * The preliminary frame delay time chosen here is irrelevant, because
//     * the correct FDT gets set automatically after demodulation. */
//    CODEC_TIMER_LOADMOD.CNT = 0;
//    CODEC_TIMER_LOADMOD.PER = 0xFFFF;
//    CODEC_TIMER_LOADMOD.CTRLD = TC_EVACT_RESTART_gc | CODEC_TIMER_MODEND_EVSEL;
//    CODEC_TIMER_LOADMOD.INTCTRLA = TC_OVFINTLVL_OFF_gc;
//    CODEC_TIMER_LOADMOD.INTFLAGS = TC0_OVFIF_bm;
//    CODEC_TIMER_LOADMOD.CTRLA = CODEC_TIMER_CARRIER_CLKSEL;

    /* Disable this interrupt. From now on we will sample the field using our CODEC_TIMER_SAMPLING */
    CODEC_DEMOD_IN_PORT.INT0MASK = 0;

}

// Sampling with timer and demod
ISR_SHARED isr_ISO14443_F_CODEC_TIMER_SAMPLING_CCA_VECT(void){
    set_PE0_high();
    SampleIdxRegister++;

    uint8_t SamplePin = CODEC_DEMOD_IN_PORT.IN & CODEC_DEMOD_IN_MASK;

    /* Shift sampled bit into sampling register */
    // Sample pin očekávám z téhle negace, že mi vrací 1 když naměřil LOW a 0 když naměřil HIGH

    SampleRegister = (SampleRegister << 1) | (!SamplePin ? 0x01 : 0x00);

    if (SampleIdxRegister > 5){
        // No more bits to be read or an error occurred during transmission as 4x HIGH should not happen
        /* No carrier modulation for 3 sample points. EOC! */
       ISO14443_F_DEMOD_END();
    }

    if (!(SampleRegister & 0x1)){ //if last read bit is a zero
        if (!(SampleRegister ^ 0x1E)) {
            // We have read a 1
            *CodecBufferPtr = 0x01;
            CodecBufferPtr++;
            BitCount++;
        } else if (!(SampleRegister ^ 0x06)) {
            // We have read a 0
            *CodecBufferPtr = 0x00;
            CodecBufferPtr++;
            BitCount++;
        } else {
            ISO14443_F_GARBAGE();
        }
        SampleRegister = 0;
        SampleIdxRegister = 0;
    }


    /* Make sure the sampling timer gets automatically aligned to the
     * modulation pauses by using the RESTART event.
     * This can be understood as a "poor mans PLL" and makes sure that we are
     * never too far out the bit-grid while sampling. */
    CODEC_TIMER_SAMPLING.CTRLD = TC_EVACT_RESTART_gc | CODEC_TIMER_MODEND_EVSEL;
    set_PE0_low();
}

// Modulate as a card to send card response
ISR_SHARED isr_ISO14443_F_CODEC_TIMER_LOADMOD_OVF_VECT(void) {
    static void *JumpTable[] = {
            [TRANSMIT_NONE] = && TRANSMIT_NONE_LABEL,
            [TRANSMIT_START] = && TRANSMIT_START_LABEL,
            [TRANSMIT_END] = && TRANSMIT_END_LABEL
    };

    //TODO: Přidej label ochranu
//    if ((StateRegister >= TRANSMIT_NONE) && (StateRegister <= TRANSMIT_END)) {
//        goto *JumpTable[StateRegister];
//    } else {
//        return;
//    }
    goto *JumpTable[StateRegister];

TRANSMIT_NONE_LABEL:
    TerminalSendString("Transmit label none \r\n");
    return;

TRANSMIT_START_LABEL:
    TerminalSendString("Transmit label start \r\n");
    StateRegister = TRANSMIT_END;
    set_PE0_high();
    CodecSetLoadmodState(true);
    CodecStartSubcarrier();
    CODEC_TIMER_LOADMOD.PER = ISO14443F_BIT_RATE_CYCLES - 1;
    return;

TRANSMIT_END_LABEL:
    TerminalSendString("Transmit label end \r\n");
    StateRegister = TRANSMIT_NONE;
    CodecSetLoadmodState(false);
    CodecSetSubcarrier(CODEC_SUBCARRIERMOD_OFF, ISO14443F_SUBCARRIER_DIVIDER);

    CODEC_TIMER_LOADMOD.CTRLA = TC_CLKSEL_OFF_gc;
    CODEC_TIMER_LOADMOD.INTCTRLA = 0;

    Flags.LoadmodFinished = 1;
    set_PE0_low();
    return;

}

void ISO14443FCodecInit(void) {
    TerminalSendString("Legic ISO14443FCodecInit\r\n");

    /* Initialize some global vars and start looking out for reader commands */
    Flags.DemodFinished = 0;
    Flags.LoadmodFinished = 0;

    isr_func_TCD0_CCC_vect = &isr_Reader14443_2A_TCD0_CCC_vect;
    isr_func_CODEC_DEMOD_IN_INT0_VECT = &isr_ISO14443_F_CODEC_DEMOD_IN_INT0_VECT;
    isr_func_CODEC_TIMER_LOADMOD_OVF_VECT = &isr_ISO14443_F_CODEC_TIMER_LOADMOD_OVF_VECT;
    isr_func_CODEC_TIMER_SAMPLING_CCA_vect = &isr_ISO14443_F_CODEC_TIMER_SAMPLING_CCA_VECT;
    CodecInitCommon();
    StartDemod();
}

void ISO14443FCodecDeInit(void) {
    TerminalSendString("Legic ISO14443FCodecDeInit\r\n");

    /* Gracefully shutdown codec */
    CODEC_DEMOD_IN_PORT.INT0MASK = 0;

    Flags.DemodFinished = 0;
    Flags.LoadmodFinished = 0;

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

    if (Flags.DemodFinished) {
        // Reset demod flag so we can demod again
        Flags.DemodFinished = 0;

        // Pošli dekódovaná data na sériovou linku
        TerminalSendString("Data sampled: ");
        uint8_t *bufind = CodecBuffer;
        while (bufind != CodecBufferPtr) {
            char c[10];
            sprintf(c, "%x ", *bufind);
            TerminalSendString(c);
            bufind++;
        }
        TerminalSendString("\r\nLegic DemodFinished\r\n");

        // Zablikej, že jsme přijali data
        LEDHook(LED_CODEC_RX, LED_PULSE);
        // Zaloguj přijatá data - TODO: bity ukládáme jako byty
        LogEntry(LOG_INFO_CODEC_RX_DATA, CodecBuffer, BitCount);

        uint16_t AnswerBitCount;
        // Zavolej aplikační vrstvu
        // AnswerBitCount = ApplicationProcess(CodecBuffer, BitCount);
        // TODO: Prozatím vynutíme konkrétní data na odeslání
        uint8_t tmpbf[] = {0x1, 0x0, 0x0, 0x1, 0x1, 0x0};
        memcpy(CodecBuffer, tmpbf, 6);
        AnswerBitCount = BitCount ? ISO14443F_APP_NO_RESPONSE : ISO14443F_APP_NO_RESPONSE;

        if (AnswerBitCount != ISO14443F_APP_NO_RESPONSE) {
            TerminalSendString("APP response \r\n");

//            // Zablikej, že vysíláme
//            LEDHook(LED_CODEC_TX, LED_PULSE);
//            // Zaloguj data co odesíláme - TODO: bity jsou jako byty
//            LogEntry(LOG_INFO_CODEC_TX_DATA, CodecBuffer, AnswerBitCount);
            BitCount = AnswerBitCount;
            CodecBufferPtr = CodecBuffer;
            CodecSetSubcarrier(CODEC_SUBCARRIERMOD_OOK, ISO14443F_SUBCARRIER_DIVIDER);
            StateRegister = TRANSMIT_START;
        } else {
            TerminalSendString("No APP response \r\n");

            /* No data to be processed. Disable loadmodding and start listening again */
            CODEC_TIMER_LOADMOD.CTRLA = TC_CLKSEL_OFF_gc;
            CODEC_TIMER_LOADMOD.INTCTRLA = 0;
            StateRegister = TRANSMIT_NONE;

            StartDemod();
        }

        /* Reception finished. Process the received bytes */
//        uint16_t DemodBitCount = BitCount;
//        uint16_t AnswerBitCount = ISO14443A_APP_NO_RESPONSE;

//        if (DemodBitCount >= ISO14443A_MIN_BITS_PER_FRAME) {
//            // For logging data
//            LogEntry(LOG_INFO_CODEC_RX_DATA, CodecBuffer, (DemodBitCount + 7) / 8);
//            LEDHook(LED_CODEC_RX, LED_PULSE);
//
//            /* Call application if we received data */
//            AnswerBitCount = ApplicationProcess(CodecBuffer, DemodBitCount);
//
//            if (AnswerBitCount & ISO14443A_APP_CUSTOM_PARITY) {
//                /* Application has generated it's own parity bits.
//                 * Clear this option bit. */
//                AnswerBitCount &= ~ISO14443A_APP_CUSTOM_PARITY;
//                ParityBufferPtr = &CodecBuffer[ISO14443A_BUFFER_PARITY_OFFSET];
//            } else {
//                /* We have to generate the parity bits ourself */
//                ParityBufferPtr = 0;
//            }
//        }
//
//        if (AnswerBitCount != ISO14443A_APP_NO_RESPONSE) {
//            LogEntry(LOG_INFO_CODEC_TX_DATA, CodecBuffer, (AnswerBitCount + 7) / 8);
//            LEDHook(LED_CODEC_TX, LED_PULSE);
//
//            BitCount = AnswerBitCount;
//            CodecBufferPtr = CodecBuffer;
//            CodecSetSubcarrier(CODEC_SUBCARRIERMOD_OOK, ISO14443F_SUBCARRIER_DIVIDER);
//
//            StateRegister = LOADMOD_START;
//        } else {
//            /* No data to be processed. Disable loadmodding and start listening again */
//            CODEC_TIMER_LOADMOD.CTRLA = TC_CLKSEL_OFF_gc;
//            CODEC_TIMER_LOADMOD.INTCTRLA = 0;
//
//            StartDemod();
//        }
//    }

        if (Flags.LoadmodFinished) {
            TerminalSendString("Legic LoadmodFinished\r\n");

            Flags.LoadmodFinished = 0;
            /* Load modulation has been finished. Stop it and start to listen
             * for incoming data again. */
            StartDemod();
        }
    }
}

