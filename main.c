/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include "board.h"
#include "uart_imx.h"
#include "debug_console_imx.h"
#include "ecspi.h"
#include "clock_freq.h"

/* define ECSPI slave mode parameters configuration. */
#define ECSPI_SLAVE_BURSTLENGTH            (7)
#define FIRST_DATA                         0xFF

static void ECSPI_SlaveConfig(ecspi_init_config_t* initConfig);
static bool ECSPI_SlaveGetTransferStatus(void);

//static volatile uint8_t txData;
//static volatile uint8_t rxData;
static uint8_t txBuffer[32];
static uint8_t rxBuffer[32];
static volatile bool isBusy;
static uint8_t countData = 0;

int main(void)
{
    uint8_t i,j;

    ecspi_init_config_t ecspiSlaveInitConfig = {
        .clockRate = 0,
        .baudRate = 0,
        .mode = ecspiSlaveMode,
        .burstLength = ECSPI_SLAVE_BURSTLENGTH,
        .channelSelect = BOARD_ECSPI_CHANNEL,
        .clockPhase = ecspiClockPhaseSecondEdge,
        .clockPolarity = ecspiClockPolarityActiveHigh,
        .ecspiAutoStart = 0
    };

    /* Hardware initialize, include RDC, CLOCK, IOMUX, ENABLE MODULE */
    hardware_init();

    PRINTF("-------------- ECSPI slave driver example --------------\n\r");
    PRINTF("This example application demonstrates usage of SPI driver in slave mode.\n\r");
    PRINTF("It responses to remote processor in SPI master mode.\n\r");

    /* Ecspi slave initialize, include configure parameters */
    txBuffer[0] = FIRST_DATA;
    ECSPI_SlaveConfig(&ecspiSlaveInitConfig);

    PRINTF("SLAVE: Ready to transfer data with Master.\n\r");

    /* Send data to master and receive data from master */
    while(true)
    {
        /* Wait for transfer */        
        while(ECSPI_SlaveGetTransferStatus());
        
        /* print out rx/tx data */
        PRINTF("\n\rSLAVE: Data transfer result: \n\r");
        PRINTF("Rx data: ");
        for(i = 0; i < countData; i++)
        {
            PRINTF(" 0x%02X", rxBuffer[i]);
        }

        PRINTF("\n\rTx data: ");
        for(j = 0; j < countData; j++)
        {
            PRINTF(" 0x%02X", txBuffer[j]);
        }
        
        /* clear flags */
        countData = 0;
        isBusy = true;
    }
}


/******************************************************************************
*
* Function Name: ECSPI_SlaveGetTransferStatus
* Comments: Get transfer status.
*
******************************************************************************/
static bool ECSPI_SlaveGetTransferStatus(void)
{
    return isBusy;
}

/******************************************************************************
*
* Function Name: ECSPI_SlaveConfig
* Comments: ECSPI module initialize
*
******************************************************************************/
static void ECSPI_SlaveConfig(ecspi_init_config_t* initConfig)
{
    int i;

    /* Initialize ECSPI, parameter configure */
    ECSPI_Init(BOARD_ECSPI_BASEADDR, initConfig);

    /* Initial first 16 bytes data to tx data register to be ready for transmition. */
    for (i=0; i< 16 ;i++)
    {
        txBuffer[i+1] = txBuffer[i] -1;
        ECSPI_SendData(BOARD_ECSPI_BASEADDR, txBuffer[i]);
    }
    //ECSPI_SendData(BOARD_ECSPI_BASEADDR, txBuffer[0]);

    /* Call core API to enable the IRQ. */
    NVIC_EnableIRQ(BOARD_ECSPI_IRQ_NUM);

    /* Clear ECSPI status register */
    ECSPI_ClearStatusFlag(BOARD_ECSPI_BASEADDR, ecspiFlagTxfifoTc);
    ECSPI_ClearStatusFlag(BOARD_ECSPI_BASEADDR, ecspiFlagRxfifoOverflow);

    /* Enable RXFIFO Ready Interrupt.*/
    ECSPI_SetIntCmd(BOARD_ECSPI_BASEADDR, ecspiFlagRxfifoReady, true);

    /* Set ECSPI transfer state. */
    isBusy = true;
}

/******************************************************************************
*
* Function Name: BOARD_ECSPI_SLAVE_HANDLER
* Comments: The interrupt service routine triggered by ECSPI interrupt
*
******************************************************************************/
void BOARD_ECSPI_HANDLER(void)
{
    
    /* Are one word or more in RX FIFO */
    if(ECSPI_GetStatusFlag(BOARD_ECSPI_BASEADDR, ecspiFlagRxfifoReady) != 0)
    {
        /* Read byte from rx data register */
        rxBuffer[countData] = ECSPI_ReceiveData(BOARD_ECSPI_BASEADDR);
        
        /* Store tx data to tx data register for next transmition */
        ECSPI_SendData(BOARD_ECSPI_BASEADDR, txBuffer[countData]);

        /* Clear the status */
        ECSPI_ClearStatusFlag(BOARD_ECSPI_BASEADDR, ecspiFlagTxfifoTc);
        ECSPI_ClearStatusFlag(BOARD_ECSPI_BASEADDR, ecspiFlagRxfifoOverflow);

        /* Set transfer status */
        isBusy = false;

        countData++;
        if (countData > 31)
        {
            countData = 0;
        }
    }

}

/*******************************************************************************
 * EOF
 ******************************************************************************/
