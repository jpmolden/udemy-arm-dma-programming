# udemy-arm-dma-programming
### Examples from the udemy DMA programming series
- [Udemy DMA programming][udemy dma course]

    Tools used:
- [Keil MDK-5][KeilMDK5_link]
- [OpenSTM32][OpenSTM32_link]
- [STM32CubeMX][STM32CubeMX_link]
- [Eclipse for C/C++][EclipsePackages_link]
- [ST-Link][ST-Link_link]

> Hardware Used: [STM32L476G-DISCO][STM32L476G-DISCO_link]
(32-Bit ARM® Cortex®-M4).
> Discovery Board Docs: [32L476GDISCOVERY][32L476GDISCOVERY_link].
> H/W Datasheet: [STM32L476xx][STM32L476xx_link].
> Reference Manual: [RM0351][RM0351_link]


Projects:

Project 1 - Simple UART transmission over a USB VCP(Virtual COM port) on press of button connected to PD0 GPIO (pull-up)

![screen shot](/img/project1_terraterm.png)

##Project 2 - DMA (M2P)

Using DMA, read from SRAM1 write to LED's (GPIO B/E), toggle the LED's

From the block diagram:

> ![Block Diagram STM32L476xx](/img/STM32L476xx_Block_Diagram.png)

- LED (LD4) connected on PB2 to GPIO Port B over the AHB2 bus
- LED (LD5) connected on PE8 to GPIO Port E over the AHB2 bus

From the overview of the ABH Bus matrix the AHB2 Peripherals can be controlled by both DMA1 and DMA2:
> ![Bus Matrix STM32L476](/img/STM32L476xx_BusMatrix.png)



Generic instrucations for using DMA
+ 1) Identify which DMAx controller to use
+ 2) Initialize the DMA
+ 3) Trigger the DMA (Automatic or manual trigger)
+ 4) Wait for TC(poll) or get the callback from the DMA driver(interrupt)



<!-- Reference Links -->
[udemy dma course]: https://www.udemy.com/microcontroller-dma-programming-fundamentals-to-advanced/learn/v4/overview
[KeilMDK5_link]: http://www2.keil.com/mdk5/
[OpenSTM32_link]: http://www.openstm32.org/HomePage
[STM32CubeMX_link]: https://www.st.com/en/development-tools/stm32cubemx.html
[EclipsePackages_link]: https://www.eclipse.org/downloads/packages/
[STM32L476G-DISCO_link]: https://www.digikey.com/product-detail/en/stmicroelectronics/STM32L476G-DISCO/497-15879-ND/5344355
[ST-Link_link]: https://www.st.com/content/st_com/en/products/development-tools/software-development-tools/stm32-software-development-tools/stm32-utilities/stsw-link009.html
[STM32L476xx_link]: https://www.st.com/content/ccc/resource/technical/document/datasheet/c5/ed/2f/60/aa/79/42/0b/DM00108832.pdf/files/DM00108832.pdf/jcr:content/translations/en.DM00108832.pdf
[RM0351_link]: https://www.st.com/content/ccc/resource/technical/document/reference_manual/02/35/09/0c/4f/f7/40/03/DM00083560.pdf/files/DM00083560.pdf/jcr:content/translations/en.DM00083560.pdf
[32L476GDISCOVERY_link]: https://www.st.com/en/evaluation-tools/32l476gdiscovery.html


