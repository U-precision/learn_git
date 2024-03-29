/****************************************************************************************
* Filename :            ast_Header.h                                                                
* Version:              1.0                                                                      
* Programmer(s):        AST
* Created :             2016-03-08                                                                     
* Description :         vxworks driver for M72                                
* Modification History:   
*****************************************************************************************/

#ifndef _AST_HEADER_H
#define _AST_HEADER_H

#include "vxWorks.h"

#define u_int8   UINT8
#define u_int16  UINT16
#define u_int32  UINT32
#define u_int64  UINT64

#define int32    int

/* PLD_IF_REG: PLD bit locations */
#define PSDAT			0	
#define PSCLK			1
#define PSCONF			3
#define PSSTAT			1
#define PSDONE			2

/* PLD_IF_REG: non-PLD bit state */
#define PLD_IF_MASK		0x00
#define PLD_FIRSTBLOCK          1
#define PLD_LASTBLOCK           2 

/* PLD error codes */
#define PLD_ERR_NOTFOUND	0x1		/* no response from PLD */
#define PLD_ERR_INIT		0x2		/* error when initializing PLD */
#define PLD_ERR_LOAD		0x3		/* error when loading PLD */
#define PLD_ERR_TERM		0x4		/* error when terminating PLD */
#define MACCESS_CLONE(ma_src,ma_dst,offs)	ma_dst=(MACCESS)((u_int8*)(ma_src)+(offs))

#define MREAD_D8(ma,offs)  \
        (*(volatile u_int8* )((u_int8*)(ma)+(offs)))
#define MREAD_D16(ma,offs) \
        (*(volatile u_int16*)((u_int8*)(ma)+(offs)))
#define MREAD_D32(ma,offs) \
        (*(volatile u_int32*)((u_int8*)(ma)+(offs)))
#define MREAD_D64(ma,offs) \
        (*(volatile u_int64* )((u_int8*)(ma)+(offs)))

#define MWRITE_D8(ma,offs,val) \
    *(volatile u_int8* )((u_int8*)(ma)+(offs)) = val
#define MWRITE_D16(ma,offs,val) \
   *(volatile u_int16*)((u_int8*)(ma)+(offs)) = (u_int16)(val)
#define MWRITE_D32(ma,offs,val) \
   *(volatile u_int32*)((u_int8*)(ma)+(offs)) = (val)
#define MWRITE_D64(ma,offs,val) \
   *(volatile u_int64* )((u_int8*)(ma)+(offs)) = (val)

#define MSETMASK_D8(ma,offs,mask) \
   *(volatile u_int8* )((u_int8*)(ma)+(offs)) |= (mask)
#define MSETMASK_D16(ma,offs,mask) \
   *(volatile u_int16*)((u_int8*)(ma)+(offs)) |= (mask)
#define MSETMASK_D32(ma,offs,mask) \
   *(volatile u_int32*)((u_int8*)(ma)+(offs)) |= (mask)
#define MSETMASK_D64(ma,offs,mask) \
   *(volatile u_int64* )((u_int8*)(ma)+(offs)) |= (mask)

#define MCLRMASK_D8(ma,offs,mask)  \
   *(volatile u_int8* )((u_int8*)(ma)+(offs)) &= ~(mask)
#define MCLRMASK_D16(ma,offs,mask) \
   *(volatile u_int16*)((u_int8*)(ma)+(offs)) &= ~(mask)
#define MCLRMASK_D32(ma,offs,mask) \
   *(volatile u_int32*)((u_int8*)(ma)+(offs)) &= ~(mask)
#define MCLRMASK_D64(ma,offs,mask) \
   *(volatile u_int64* )((u_int8*)(ma)+(offs)) &= ~(mask)

/*--------------------------------------+
|   DEFINES                             |
+--------------------------------------*/
/* bit state */
#define LOW		0
#define HIGH	1

typedef volatile void* MACCESS;         /* access pointer */


/* parameters for the callback functions */
typedef struct {
	MACCESS ma;										/* access handle */
	void *arg;										/* callback fkt. arg */
	void (*msec_delay)(void *arg, u_int32 msec);	/* callback fkt. */
	u_int16 ctrl_bits;								/* shadow register */
	u_int16 data; 									/* bit mask */
	u_int16 dclk;  									/* bit mask */
	u_int16 config;  								/* bit mask */
	u_int16 status;  								/* bit mask */
	u_int16 cfgdone; 								/* bit mask */
} ACCPARAMS;

#define M72_FPGA_CONTROL          0x00

#define M72_FPGA_COUNTER_L        0x10
#define M72_FPGA_COUNTER_H        0x12

#define M72_FPGA_REGISTER         0xFE

#define AST_MEN_M72_MAX_BOARD (4)


#define M72_CLEAR_MASK              0x0003
#define M72_CLEAR_NOW           	0x02            /* at now (immediately) (software)*/
///
#define BaseAdrs 0x901c0000 /*A203N BaseAdress */
//#define Module_Slot  0
//#define Channel 2 

#define Module_Slot  2
#define Channel 0 

//#define Module_baseadrs BaseAdrs+Module_Slot * 0x400


#define Module_baseadrs BaseAdrs+Module_Slot * 0x200

#define M72_Channel_baseadrs  Module_baseadrs +Channel * 0x20/*A201*/
#define M72_Channel_INTConfig *(short*) (M72_Channel_baseadrs + 0x08)

#define M72_Channel_INTConfig1 *(short*) (Module_baseadrs +0 * 0x20 + 0x08)
#define M72_Channel_INTConfig2 *(short*) (Module_baseadrs +1 * 0x20 + 0x08)
#define M72_Channel_INTConfig3 *(short*) (Module_baseadrs +2 * 0x20 + 0x08)
#define M72_Channel_INTConfig4 *(short*) (Module_baseadrs +3 * 0x20 + 0x08)






#define M72_Channel_INTStatusLow *(short*) (Module_baseadrs + 0x80)
#define M72_Channel_INTStatusHigh *(short*) (Module_baseadrs + 0x82)

#define INTConfig *(short*) (Module_baseadrs + 0x100)
#define INTVector *(short*) (Module_baseadrs + 0x102)

/*struct to save board information*/
typedef struct _MEN_M72_INFO_STRUCT
{
        UINT32 address;         /* local address */ 
        UINT8  mode;           	/* counter mode */ 
} AST_MEN_M72_INFO;


/*PLD status*/
#define PLD_STATUS_OK                0
#define PLD_STATUS_NRDY              1
#define PLD_STATUS_LOADING           2

#endif
