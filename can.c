
#include "sam.h"
#include "can.h"
#include <stdio.h>

void can_printmsg(CanMsg m){
    printf("CanMsg(id:%d, length:%d, data:{", m.id, m.length);
    if(m.length){
        printf("%d", m.byte[0]);
    }
    for(uint8_t i = 1; i < m.length; i++){
        printf(", %d", m.byte[i]);
    }
    printf("})\r\n");
}

#define txMailbox 0
#define rxMailbox 1


void can_init(CanInit init, uint8_t rxInterrupt){
    // Disable CAN
    CAN0->CAN_MR &= ~CAN_MR_CANEN; 
    
    // Clear status register by reading it
    __attribute__((unused)) uint32_t ul_status = CAN0->CAN_SR;     
    
    // Disable interrupts on CANH and CANL pins
    PIOA->PIO_IDR = PIO_PA8A_URXD | PIO_PA9A_UTXD;
    
    // Select CAN0 RX and TX in PIOA
    PIOA->PIO_ABSR &= ~(PIO_PA1A_CANRX0 | PIO_PA0A_CANTX0);
    
    // Disable the Parallel IO (PIO) of the Rx and Tx pins so that the peripheral controller can use them
    PIOA->PIO_PDR = PIO_PA1A_CANRX0 | PIO_PA0A_CANTX0;
    
    // Enable pull up on CANH and CANL pin
    PIOA->PIO_PUER = (PIO_PA1A_CANRX0 | PIO_PA0A_CANTX0);    
    
    // Enable Clock for CAN0 in PMC
    // DIV = 1 (can clk = MCK/2), CMD = 1 (write), PID = 2B (CAN0)
    PMC->PMC_PCR = PMC_PCR_EN | (0/*??*/ << PMC_PCR_DIV_Pos) | PMC_PCR_CMD | (ID_CAN0 << PMC_PCR_PID_Pos); 
    PMC->PMC_PCER1 |= 1 << (ID_CAN0 - 32);
    
    //Set baudrate, Phase1, phase2 and propagation delay for can bus. Must match on all nodes!
    CAN0->CAN_BR = init.reg; 
    


    // Configure mailboxes
    // transmit
    CAN0->CAN_MB[txMailbox].CAN_MID = 0;
    CAN0->CAN_MB[txMailbox].CAN_MMR = CAN_MMR_MOT_MB_TX;
    
    // receive
    CAN0->CAN_MB[rxMailbox].CAN_MAM = CAN_MAM_MIDE; // Accept all messages
    CAN0->CAN_MB[rxMailbox].CAN_MID = 0;
    CAN0->CAN_MB[rxMailbox].CAN_MMR = CAN_MMR_MOT_MB_RX;
    CAN0->CAN_MB[rxMailbox].CAN_MCR |= CAN_MCR_MTCR;
    if(rxInterrupt){
        // Enable interrupt on receive
        CAN0->CAN_IER |= (1 << rxMailbox); 
        // Enable interrupt in NVIC. highest priority so it can interrupt the motor
        // controller (TC0) and empty the mailbox before the next frame arrives
        NVIC_SetPriority(CAN0_IRQn, 0);
        NVIC_EnableIRQ(CAN0_IRQn);
    }

    // Enable CAN
    CAN0->CAN_MR |= CAN_MR_CANEN;
}


// how long can_tx waits for the previous frame before giving up (busy-wait iterations, roughly 1 ms)
#define TX_TIMEOUT 20000

uint8_t can_tx(CanMsg m){
    // wait for the previous frame to be sent. if nobody acks it (node 1 off or unplugged)
    // the controller retries forever, so give up instead of freezing the whole program.
    uint32_t timeout = TX_TIMEOUT;
    while(!(CAN0->CAN_MB[txMailbox].CAN_MSR & CAN_MSR_MRDY)){
        if (--timeout == 0) {
            CAN0->CAN_MB[txMailbox].CAN_MCR = CAN_MCR_MACR; // abort the stuck frame
            return 0;
        }
    }
    
    // Set message ID and use CAN 2.0B protocol
    CAN0->CAN_MB[txMailbox].CAN_MID = CAN_MID_MIDvA(m.id) ;
        
    // Coerce maximum 8 byte length
    m.length = m.length > 8 ? 8 : m.length;
    
    //  Put message in can data registers
    CAN0->CAN_MB[txMailbox].CAN_MDL = m.dword[0];
    CAN0->CAN_MB[txMailbox].CAN_MDH = m.dword[1];
        
    // Set message length and mailbox ready to send
    CAN0->CAN_MB[txMailbox].CAN_MCR = (m.length << CAN_MCR_MDLC_Pos) | CAN_MCR_MTCR;
    return 1;
}

// received frames are copied out of the mailbox by the interrupt and queued here.
// the mailbox only holds one frame, so without this any frame that arrives while the
// main loop is busy (solenoid pulse, printf) would be lost.
#define RX_QUEUE_SIZE 32 // must be a power of two
static volatile CanMsg rxQueue[RX_QUEUE_SIZE];
static volatile uint8_t rxHead = 0; // written by the interrupt
static volatile uint8_t rxTail = 0; // written by can_rx

void CAN0_Handler(void){
    if(CAN0->CAN_MB[rxMailbox].CAN_MSR & CAN_MSR_MRDY){
        CanMsg m;
        m.id = (uint16_t)((CAN0->CAN_MB[rxMailbox].CAN_MID & CAN_MID_MIDvA_Msk) >> CAN_MID_MIDvA_Pos);
        m.length = (uint8_t)((CAN0->CAN_MB[rxMailbox].CAN_MSR & CAN_MSR_MDLC_Msk) >> CAN_MSR_MDLC_Pos);
        m.dword[0] = CAN0->CAN_MB[rxMailbox].CAN_MDL;
        m.dword[1] = CAN0->CAN_MB[rxMailbox].CAN_MDH;

        // re-arm the mailbox for the next frame, this also clears the interrupt
        CAN0->CAN_MB[rxMailbox].CAN_MCR = CAN_MCR_MTCR;

        uint8_t next = (rxHead + 1) & (RX_QUEUE_SIZE - 1);
        if(next != rxTail){ // drop the frame if the queue is full
            rxQueue[rxHead].id = m.id;
            rxQueue[rxHead].length = m.length;
            rxQueue[rxHead].dword[0] = m.dword[0];
            rxQueue[rxHead].dword[1] = m.dword[1];
            rxHead = next;
        }
    }
    NVIC_ClearPendingIRQ(CAN0_IRQn);
}

// Receive a CAN message from the queue. Does not block.
uint8_t can_rx(CanMsg* m){
    if(rxTail == rxHead){
        return 0;
    }
    m->id = rxQueue[rxTail].id;
    m->length = rxQueue[rxTail].length;
    m->dword[0] = rxQueue[rxTail].dword[0];
    m->dword[1] = rxQueue[rxTail].dword[1];
    rxTail = (rxTail + 1) & (RX_QUEUE_SIZE - 1);
    return 1;
}
