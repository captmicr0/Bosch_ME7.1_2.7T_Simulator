#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/structs/dma.h"
#include "engine_profile.h"
#include "dac_stream.pio.h"
#include "cam_stream.pio.h"

#define WAVE_PIO pio0
#define WAVE_SM_DAC 0u
#define WAVE_SM_CAM 1u
#define DAC_GPIO_LDAC 4u
#define DAC_GPIO_CS 5u
#define DAC_GPIO_SCK 6u
#define DAC_GPIO_MOSI 7u
#define WAVE_CAM_GPIO 2u
#define WAVE_MARKER_GPIO 22u
#define WAVE_SAMPLES 8u
#define WAVE_MAX_WORDS (ENGINE_MAX_TEETH * 2u * WAVE_SAMPLES)
#define WAVE_CYCLES_PER_SAMPLE 103u
#define WAVE_CENTER 2048
#define WAVE_AMPLITUDE 1800

static uint32_t wave_dac[WAVE_MAX_WORDS];
static uint32_t wave_cam[WAVE_MAX_WORDS];
static uint32_t wave_words;
static int wave_dma_dac=-1, wave_dma_cam=-1;
static int wave_program_dac=-1, wave_program_cam=-1;
static volatile bool wave_running=false;
static volatile uint32_t wave_dma_underruns=0;
static const int16_t wave_sine_q10[8]={0,724,1024,724,0,-724,-1024,-724};

static inline uint16_t wave_clip(int n) { return (uint16_t)(n<0?0:(n>4095?4095:n)); }
static void wave_build(const engine_shape_t *s) {
    wave_words = (uint32_t)engine_cycle_slots(s)*WAVE_SAMPLES;
    for (uint32_t i=0;i<wave_words;++i) {
        uint32_t slot=i/WAVE_SAMPLES, sub=i%WAVE_SAMPLES;
        bool present=(slot%s->crank.teeth)<(s->crank.teeth-s->crank.missing);
        int v=present ? ((int)WAVE_AMPLITUDE*wave_sine_q10[sub])/1024 : 0;
        if(s->crank.invert) v=-v;
        uint16_t a=(uint16_t)(0x3000u|wave_clip(WAVE_CENTER+v));
        uint16_t b=(uint16_t)(0xb000u|wave_clip(WAVE_CENTER-v));
        wave_dac[i]=((uint32_t)a<<16)|b;
        wave_cam[i]=(uint32_t)engine_cam_bits(s,(uint16_t)slot) | ((slot==0u)?4u:0u);
    }
}

static void __isr wave_dma_irq(void) {
    uint32_t mask=(1u<<wave_dma_dac)|(1u<<wave_dma_cam);
    uint32_t fired=dma_hw->ints0 & mask;
    dma_hw->ints0=fired;
    if(!wave_running) return;
    if(fired & (1u<<wave_dma_dac)) {
        dma_channel_set_read_addr(wave_dma_dac,wave_dac,false);
        dma_channel_set_trans_count(wave_dma_dac,wave_words,true);
    }
    if(fired & (1u<<wave_dma_cam)) {
        dma_channel_set_read_addr(wave_dma_cam,wave_cam,false);
        dma_channel_set_trans_count(wave_dma_cam,wave_words,true);
    }
}

static void wave_init(void) {
    wave_program_dac=(int)pio_add_program(WAVE_PIO,&dac_stream_program);
    wave_program_cam=(int)pio_add_program(WAVE_PIO,&cam_stream_program);
    pio_gpio_init(WAVE_PIO,DAC_GPIO_MOSI);
    for(uint i=4;i<=6;++i)pio_gpio_init(WAVE_PIO,i);
    pio_gpio_init(WAVE_PIO,WAVE_CAM_GPIO);
    pio_gpio_init(WAVE_PIO,WAVE_CAM_GPIO+1);
    pio_gpio_init(WAVE_PIO,WAVE_MARKER_GPIO);
    pio_sm_config sd=dac_stream_program_get_default_config((uint)wave_program_dac);
    sm_config_set_out_pins(&sd,DAC_GPIO_MOSI,1);
    sm_config_set_sideset_pins(&sd,DAC_GPIO_LDAC);
    sm_config_set_out_shift(&sd,false,false,32); // left-shift; explicitly pull each 32-bit pair
    sm_config_set_fifo_join(&sd,PIO_FIFO_JOIN_TX);
    pio_sm_init(WAVE_PIO,WAVE_SM_DAC,(uint)wave_program_dac,&sd);
    pio_sm_set_consecutive_pindirs(WAVE_PIO,WAVE_SM_DAC,DAC_GPIO_MOSI,1,true);
    pio_sm_set_consecutive_pindirs(WAVE_PIO,WAVE_SM_DAC,DAC_GPIO_LDAC,3,true);
    pio_sm_config sc=cam_stream_program_get_default_config((uint)wave_program_cam);
    sm_config_set_out_pins(&sc,WAVE_CAM_GPIO,2);
    sm_config_set_set_pins(&sc,WAVE_MARKER_GPIO,1);
    sm_config_set_out_shift(&sc,true,false,3); // right-shift cams first; explicitly pull each control word
    sm_config_set_fifo_join(&sc,PIO_FIFO_JOIN_TX);
    pio_sm_init(WAVE_PIO,WAVE_SM_CAM,(uint)wave_program_cam,&sc);
    pio_sm_set_consecutive_pindirs(WAVE_PIO,WAVE_SM_CAM,WAVE_CAM_GPIO,2,true);
    pio_sm_set_consecutive_pindirs(WAVE_PIO,WAVE_SM_CAM,WAVE_MARKER_GPIO,1,true);
    wave_dma_dac=dma_claim_unused_channel(true);
    wave_dma_cam=dma_claim_unused_channel(true);
    dma_channel_config d=dma_channel_get_default_config(wave_dma_dac);
    channel_config_set_transfer_data_size(&d,DMA_SIZE_32);
    channel_config_set_read_increment(&d,true);
    channel_config_set_write_increment(&d,false);
    channel_config_set_dreq(&d,pio_get_dreq(WAVE_PIO,WAVE_SM_DAC,true));
    dma_channel_configure(wave_dma_dac,&d,&WAVE_PIO->txf[WAVE_SM_DAC],wave_dac,0,false);
    d=dma_channel_get_default_config(wave_dma_cam);
    channel_config_set_transfer_data_size(&d,DMA_SIZE_32);
    channel_config_set_read_increment(&d,true);
    channel_config_set_write_increment(&d,false);
    channel_config_set_dreq(&d,pio_get_dreq(WAVE_PIO,WAVE_SM_CAM,true));
    dma_channel_configure(wave_dma_cam,&d,&WAVE_PIO->txf[WAVE_SM_CAM],wave_cam,0,false);
    dma_channel_set_irq0_enabled(wave_dma_dac,true);
    dma_channel_set_irq0_enabled(wave_dma_cam,true);
    irq_set_exclusive_handler(DMA_IRQ_0,wave_dma_irq);
    irq_set_enabled(DMA_IRQ_0,true);
}

// Return both DAC legs to their 1.024 V midpoint when RPM is zero.
// This slow GPIO transaction is used ONLY at startup/stop, never per sample.
static void wave_idle_dac(void) {
    const uint pins[]={DAC_GPIO_LDAC,DAC_GPIO_CS,DAC_GPIO_SCK,DAC_GPIO_MOSI};
    for(uint i=0;i<4;++i) {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i],GPIO_OUT);
    }
    gpio_put(DAC_GPIO_LDAC,1);
    gpio_put(DAC_GPIO_CS,1);
    gpio_put(DAC_GPIO_SCK,0);
    uint16_t frames[2]={0x3000u|WAVE_CENTER,0xb000u|WAVE_CENTER};
    for(uint ch=0;ch<2;++ch) {
        gpio_put(DAC_GPIO_CS,0);
        for(int bit=15;bit>=0;--bit) {
            gpio_put(DAC_GPIO_MOSI,(frames[ch]>>bit)&1u);
            busy_wait_us_32(1);
            gpio_put(DAC_GPIO_SCK,1);
            busy_wait_us_32(1);
            gpio_put(DAC_GPIO_SCK,0);
        }
        gpio_put(DAC_GPIO_CS,1);
    }
    gpio_put(DAC_GPIO_LDAC,0);
    busy_wait_us_32(1);
    gpio_put(DAC_GPIO_LDAC,1);
    const uint pins_cam[]={WAVE_CAM_GPIO,WAVE_CAM_GPIO+1,WAVE_MARKER_GPIO};
    for(uint i=0;i<3;++i) {
        gpio_init(pins_cam[i]);gpio_set_dir(pins_cam[i],GPIO_OUT);gpio_put(pins_cam[i],0);
    }
}

static void wave_stop(void) {
    wave_running=false;
    pio_sm_set_enabled(WAVE_PIO,WAVE_SM_DAC,false);
    pio_sm_set_enabled(WAVE_PIO,WAVE_SM_CAM,false);
    dma_channel_abort(wave_dma_dac);
    dma_channel_abort(wave_dma_cam);
    dma_hw->ints0=(1u<<wave_dma_dac)|(1u<<wave_dma_cam);
    pio_sm_clear_fifos(WAVE_PIO,WAVE_SM_DAC);
    pio_sm_clear_fifos(WAVE_PIO,WAVE_SM_CAM);
    wave_idle_dac();
}

static void wave_start(uint32_t engine_rpm,const engine_shape_t *s) {
    wave_stop();
    // Restore the PIO mux after the safe-idle GPIO transaction.
    for(uint i=DAC_GPIO_LDAC;i<=DAC_GPIO_MOSI;++i)pio_gpio_init(WAVE_PIO,i);
    pio_gpio_init(WAVE_PIO,WAVE_CAM_GPIO);
    pio_gpio_init(WAVE_PIO,WAVE_CAM_GPIO+1);
    pio_gpio_init(WAVE_PIO,WAVE_MARKER_GPIO);
    wave_build(s);
    // 60 seconds / (RPM * nominal slots/rev * samples/slot).
    // Both state machines execute exactly 103 PIO cycles per sample.
    double sample_hz=((double)engine_rpm*s->crank.teeth*WAVE_SAMPLES)/60.;
    double div=(double)clock_get_hz(clk_sys)/(sample_hz*WAVE_CYCLES_PER_SAMPLE);
    if(div<1.)div=1.;
    if(div>65535.)div=65535.;
    pio_sm_set_clkdiv(WAVE_PIO,WAVE_SM_DAC,(float)div);
    pio_sm_set_clkdiv(WAVE_PIO,WAVE_SM_CAM,(float)div);
    pio_sm_restart(WAVE_PIO,WAVE_SM_DAC);
    pio_sm_restart(WAVE_PIO,WAVE_SM_CAM);
    pio_sm_clkdiv_restart(WAVE_PIO,WAVE_SM_DAC);
    pio_sm_clkdiv_restart(WAVE_PIO,WAVE_SM_CAM);
    dma_channel_set_read_addr(wave_dma_dac,wave_dac,false);
    dma_channel_set_trans_count(wave_dma_dac,wave_words,false);
    dma_channel_set_read_addr(wave_dma_cam,wave_cam,false);
    dma_channel_set_trans_count(wave_dma_cam,wave_words,false);
    wave_running=true;
    dma_start_channel_mask((1u<<wave_dma_dac)|(1u<<wave_dma_cam));
    pio_enable_sm_mask_in_sync(WAVE_PIO,(1u<<WAVE_SM_DAC)|(1u<<WAVE_SM_CAM));
}
