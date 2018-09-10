#include <mic.h>
#include <stm32l083xx.h>

typedef struct
{
    uint8_t rx_buffer[128];
    bool is_busy;
} mic_t;

mic_t _mic;

bool _adc_calibration(void);

static void _adc_init(void)
{
    // Enable ADC clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // Errata workaround
    RCC->APB2ENR;

    // Set auto-off mode, right align
    ADC1->CFGR1 |= ADC_CFGR1_AUTOFF;

    // Set 8bit resolution
    ADC1->CFGR1 |= ADC_CFGR1_RES_1;

    // Set PCLK as a clock source
    ADC1->CFGR2 = ADC_CFGR2_CKMODE_0 | ADC_CFGR2_CKMODE_1;

    // Sampling time selection (12.5 cycles)
    ADC1->SMPR |= ADC_SMPR_SMP_1 | ADC_SMPR_SMP_0;

    // Enable ADC voltage regulator
    ADC1->CR |= ADC_CR_ADVREGEN;

    // Configure trigger by timer TRGO signal
    // Rising edge, TRG0 (TIM6_TRGO)
    ADC1->CFGR1 |= ADC_CFGR1_EXTEN_0;
}

static void _tim6_init(void)
{
    // Enable TIM6 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

    // Errata workaround
    RCC->APB1ENR;

    // Disable counter if it is running
    TIM6->CR1 &= ~TIM_CR1_CEN;

    // Set prescaler to 5 * 32 (5 microseconds resolution)
    TIM6->PSC = 1000 * 32 - 1;
    TIM6->ARR = 50 - 1;

    // Configure update event TRGO to ADC
    TIM6->CR2 |= TIM_CR2_MMS_1;
}

static void _dma_init(void)
{
    // DMA1, Channel 1, Request number 0
    DMA_Channel_TypeDef *dma_channel = DMA1_Channel1;

    // Enable DMA1
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    // Errata workaround
    RCC->AHBENR;

    // Disable DMA
    dma_channel->CCR &= ~DMA_CCR_EN;

    // Peripheral to memory
    dma_channel->CCR &= ~DMA_CCR_DIR;

    // Memory data size 8 bits
    dma_channel->CCR &= ~DMA_CCR_MSIZE_Msk;
    // Peripheral data size 8 bits
    dma_channel->CCR &= ~DMA_CCR_PSIZE_Msk;

    // DMA Mode
    dma_channel->CCR &= ~DMA_CCR_CIRC_Msk;

    // Set memory incrementation
    dma_channel->CCR |= DMA_CCR_MINC;

    // DMA channel selection
    DMA1_CSELR->CSELR &= ~DMA_CSELR_C1S_Msk;

    // Configure DMA channel data length
    dma_channel->CNDTR = sizeof(_mic.rx_buffer);

    // Configure DMA channel source address
    dma_channel->CPAR = (uint32_t) &ADC1->DR;

    // Configure DMA channel destination address
    dma_channel->CMAR = (uint32_t) &_mic.rx_buffer;

    // Enable the transfer complete and error interrupts
    dma_channel->CCR |= DMA_CCR_TCIE | DMA_CCR_TEIE;

    // Enable DMA 1 channel 1 interrupt
    NVIC_SetPriority(DMA1_Channel1_IRQn, 0);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    // Enable DMA
    dma_channel->CCR |= DMA_CCR_EN;
}

void DMA1_Channel1_IRQHandler(void)
{
    if ((DMA1->ISR & DMA_ISR_GIF1) != 0)
    {
        if ((DMA1->ISR & DMA_ISR_TEIF1) != 0)
        {
            // DMA Error
            _mic.is_busy = false;
        }
        else if ((DMA1->ISR & DMA_ISR_TCIF1) != 0)
        {
            // DMA Transfer complete
            _mic.is_busy = false;
            // Stop sampling timer
            TIM6->CR1 &= ~TIM_CR1_CEN;
            // Clear DMA flag
            DMA1->IFCR |= DMA_IFCR_CTCIF1;
        }
    }
}

void mic_init(void)
{
    _adc_init();
    _adc_calibration();

    _dma_init();

    _tim6_init();
}

bool mic_start_measure(void)
{
    if (_mic.is_busy)
    {
        return false;
    }

    // Set flag
    _mic.is_busy = true;

    // Reinit the DMA
    _dma_init();

    // Set ADC channel 2
    ADC1->CHSELR = ADC_CHSELR_CHSEL2;

    // Enable DMA - this muset be set after calibration phase!
    ADC1->CFGR1 |= ADC_CFGR1_DMAEN;
    //ADC1->CFGR1 |= ADC_CFGR1_DMACFG; // debug circular mode

    // Start the AD measurement
    ADC1->CR |= ADC_CR_ADSTART;

    // Start sampling timer
    TIM6->CR1 |= TIM_CR1_CEN;

    return true;
}

bool mic_measure_is_done(void)
{
    return !_mic.is_busy;
}

bool _adc_calibration(void)
{
    if (!(ADC1->CR & ADC_CR_ADEN))
    {
        return false;
    }

    // Perform ADC calibration
    ADC1->CR |= ADC_CR_ADCAL;
    while ((ADC1->ISR & ADC_ISR_EOCAL) == 0)
    {
        continue;
    }

    return true;
}
