A platform for sound effects based on an STM32F407 Discovery board. The HMI to the platform is a resistive touch panel.
The analog I/O is for now the internal ADC and DAC of the STM32. To keep the input and output sample rates at the same speed, they are synchronized with a timer. The signal chain is operated with DMA. Support for an external audio codec will be added in the future.
