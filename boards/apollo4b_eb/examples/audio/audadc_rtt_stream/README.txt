Name:
=====
 audadc_rtt_stream


Description:
============
 This example uses AUDADC to capture and send audio data to PC via SEGGER RTT.


Purpose:
========
This example uses AUDADC INTTRIGTIMER to capture audio samples at 16 kHz
LPMODE1 is used for power efficiency
DMA is used to transfer samples from the AUDADC FIFO into an SRAM buffer

Printing takes place over the ITM at 1M Baud.



******************************************************************************


