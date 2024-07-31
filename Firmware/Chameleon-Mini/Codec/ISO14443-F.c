//
// Created by l on 7/5/24.
//

#include "ISO14443-F.h"

// ============================== From ISO14443-2A.c

#include "../System.h"
#include "../Application/Application.h"
#include "../LEDHook.h"
#include "Codec.h"
#include "Log.h"

/* Sampling is done using internal clock, synchronized to the field modulation.
 * For that we need to convert the bit rate for the internal clock. */
#define SAMPLE_RATE_SYSTEM_CYCLES		((uint16_t) (((uint64_t) F_CPU * ISO14443A_BIT_RATE_CYCLES) / CODEC_CARRIER_FREQ) )

#define ISO14443A_MIN_BITS_PER_FRAME		7

static volatile struct {
    volatile bool DemodFinished;
    volatile bool LoadmodFinished;
} Flags = { 0 };

typedef enum {
    /* Demod */
    DEMOD_DATA_BIT,
    DEMOD_PARITY_BIT,

    /* Loadmod */
    LOADMOD_FDT,
    LOADMOD_START,
    LOADMOD_START_BIT0,
    LOADMOD_START_BIT1,
    LOADMOD_DATA0,
    LOADMOD_DATA1,
    LOADMOD_PARITY0,
    LOADMOD_PARITY1,
    LOADMOD_STOP_BIT0,
    LOADMOD_STOP_BIT1,
    LOADMOD_FINISHED
} StateType;

/* Define pseudo variables to use fast register access. This is useful for global vars */
#define DataRegister	Codec8Reg0
#define StateRegister	Codec8Reg1
#define ParityRegister	Codec8Reg2
#define SampleIdxRegister Codec8Reg2
#define SampleRegister	Codec8Reg3
#define BitSent			CodecCount16Register1
#define BitCount		CodecCount16Register2
#define CodecBufferPtr	CodecPtrRegister1
#define ParityBufferPtr	CodecPtrRegister2

static void StartDemod(void) {
    /* Activate Power for demodulator */
    CodecSetDemodPower(true);

    CodecBufferPtr = CodecBuffer;
    ParityBufferPtr = &CodecBuffer[ISO14443A_BUFFER_PARITY_OFFSET];
    DataRegister = 0;
    SampleRegister = 0;
    SampleIdxRegister = 0;
    BitCount = 0;
    StateRegister = DEMOD_DATA_BIT;

    /* Configure sampling-timer free running and sync to first modulation-pause. */
    CODEC_TIMER_SAMPLING.CNT = 0;                               // Reset the timer count
    CODEC_TIMER_SAMPLING.PER = SAMPLE_RATE_SYSTEM_CYCLES - 1;   // Set Period regisiter
    CODEC_TIMER_SAMPLING.CCA = 0xFFFF; /* CCA Interrupt is not active! */
    CODEC_TIMER_SAMPLING.CTRLA = TC_CLKSEL_DIV1_gc;
    CODEC_TIMER_SAMPLING.CTRLD = TC_EVACT_RESTART_gc | CODEC_TIMER_MODSTART_EVSEL;
    CODEC_TIMER_SAMPLING.INTFLAGS = TC0_CCAIF_bm;
    CODEC_TIMER_SAMPLING.INTCTRLB = TC_CCAINTLVL_HI_gc;

    /* Start looking out for modulation pause via interrupt. */
    CODEC_DEMOD_IN_PORT.INTFLAGS = PORT_INT0IF_bm;
    CODEC_DEMOD_IN_PORT.INT0MASK = CODEC_DEMOD_IN_MASK0;
}

// ============================== end ISO14443-2A.c

//TODO: Clean this up
char legic_log_str[64];

void ISO14443FCodecInit(void) {
    sprintf(legic_log_str, "ISO14443-F Initialized");
    LogEntry(LOG_INFO_GENERIC, legic_log_str, strlen(legic_log_str));

    // ============================== From ISO14443-2A.c
    /* Initialize some global vars and start looking out for reader commands */

    Flags.DemodFinished = 0;
    Flags.LoadmodFinished = 0;

    isr_func_TCD0_CCC_vect = &isr_Reader14443_2A_TCD0_CCC_vect;
    isr_func_CODEC_DEMOD_IN_INT0_VECT = &isr_ISO14443_2A_TCD0_CCC_vect;
    isr_func_CODEC_TIMER_LOADMOD_OVF_VECT = &isr_ISO14443_2A_CODEC_TIMER_LOADMOD_OVF_VECT;
    CodecInitCommon();
    StartDemod();
    // ============================== end ISO14443-2A.c

}

void ISO14443FCodecDeInit(void) {
    sprintf(legic_log_str, "ISO14443-F Deinitialized");
    LogEntry(LOG_INFO_GENERIC, legic_log_str, strlen(legic_log_str));

    // ============================== From ISO14443-2A.c
    /* Gracefully shutdown codec */
    CODEC_DEMOD_IN_PORT.INT0MASK = 0;
    LEDHook(LED_POWERED, LED_TOGGLE);

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
    // ============================== end ISO14443-2A.c

}

void ISO14443FCodecTask(void) {
    // ============================== From ISO14443-2A.c
    if (Flags.DemodFinished) {
        Flags.DemodFinished = 0;
        /* Reception finished. Process the received bytes */
        uint16_t DemodBitCount = BitCount;
        uint16_t AnswerBitCount = ISO14443A_APP_NO_RESPONSE;

        if (DemodBitCount >= ISO14443A_MIN_BITS_PER_FRAME) {
            // For logging data
            LogEntry(LOG_INFO_CODEC_RX_DATA, CodecBuffer, (DemodBitCount + 7) / 8);
            LEDHook(LED_CODEC_RX, LED_PULSE);

            /* Call application if we received data */
            AnswerBitCount = ApplicationProcess(CodecBuffer, DemodBitCount);

            if (AnswerBitCount & ISO14443A_APP_CUSTOM_PARITY) {
                /* Application has generated it's own parity bits.
                 * Clear this option bit. */
                AnswerBitCount &= ~ISO14443A_APP_CUSTOM_PARITY;
                ParityBufferPtr = &CodecBuffer[ISO14443A_BUFFER_PARITY_OFFSET];
            } else {
                /* We have to generate the parity bits ourself */
                ParityBufferPtr = 0;
            }
        }

        if (AnswerBitCount != ISO14443A_APP_NO_RESPONSE) {
            LogEntry(LOG_INFO_CODEC_TX_DATA, CodecBuffer, (AnswerBitCount + 7) / 8);
            LEDHook(LED_CODEC_TX, LED_PULSE);

            BitCount = AnswerBitCount;
            CodecBufferPtr = CodecBuffer;
            CodecSetSubcarrier(CODEC_SUBCARRIERMOD_OOK, ISO14443A_SUBCARRIER_DIVIDER);

            StateRegister = LOADMOD_START;
        } else {
            /* No data to be processed. Disable loadmodding and start listening again */
            CODEC_TIMER_LOADMOD.CTRLA = TC_CLKSEL_OFF_gc;
            CODEC_TIMER_LOADMOD.INTCTRLA = 0;

            StartDemod();
        }
    }

    if (Flags.LoadmodFinished) {
        Flags.LoadmodFinished = 0;
        /* Load modulation has been finished. Stop it and start to listen
         * for incoming data again. */
        StartDemod();
    }
    // ============================== end ISO14443-2A.c

}