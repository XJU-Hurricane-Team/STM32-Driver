/* -----------------------------------------------------------------------------
 * Copyright (c) 2014 ARM Ltd.
 *
 * This software is provided 'as-is', without any express or implied warranty. 
 * In no event will the authors be held liable for any damages arising from 
 * the use of this software. Permission is granted to anyone to use this 
 * software for any purpose, including commercial applications, and to alter 
 * it and redistribute it freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not 
 *    claim that you wrote the original software. If you use this software in
 *    a product, an acknowledgment in the product documentation would be 
 *    appreciated but is not required. 
 * 
 * 2. Altered source versions must be plainly marked as such, and must not be 
 *    misrepresented as being the original software. 
 * 
 * 3. This notice may not be removed or altered from any source distribution.
 *   
 *
 * $Date:        15. April 2014
 * $Revision:    V1.00
 *  
 * Project:      Flash Programming Functions for ST Microelectronics STM32F3xx Flash
 * --------------------------------------------------------------------------- */

/* History:
 *  Version 1.00
 *    Initial release
 */ 

#include "FlashOS.H"        // FlashOS Structures

typedef volatile unsigned short   vu16;
typedef          unsigned short    u16;
typedef volatile unsigned long    vu32;
typedef          unsigned long     u32;

#define M16(adr) (*((vu16 *) (adr)))
#define M32(adr) (*((vu32 *) (adr)))

// Peripheral Memory Map
#define IWDG_BASE       0x40003000
#define FLASH_BASE      0x40022000

#define IWDG            ((IWDG_TypeDef *) IWDG_BASE)
#define FLASH           ((FLASH_TypeDef*) FLASH_BASE)

// Independent WATCHDOG
typedef struct {
  vu32 KR;
  vu32 PR;
  vu32 RLR;
  vu32 SR;
} IWDG_TypeDef;

// Flash Registers
typedef struct {
  vu32 ACR;                                     // offset  0x000
  vu32 KEYR;                                    // offset  0x004
  vu32 OPTKEYR;                                 // offset  0x008
  vu32 SR;                                      // offset  0x00C
  vu32 CR;                                      // offset  0x010
  vu32 AR;                                      // offset  0x014
  vu32 RESERVED0[1];
  vu32 OBR;                                     // offset  0x01C
  vu32 WRPR;                                    // offset  0x020
} FLASH_TypeDef;


// Flash Keys
#define RDPRT_KEY       0x55AA
#define FLASH_KEY1      0x45670123
#define FLASH_KEY2      0xCDEF89AB

// Flash Control Register definitions
#define FLASH_PG                ((unsigned int)(1ul << 0))
#define FLASH_PER               ((unsigned int)(1ul << 1))
#define FLASH_MER               ((unsigned int)(1ul << 2))
#define FLASH_OPTPG             ((unsigned int)(1ul << 4))
#define FLASH_OPTER             ((unsigned int)(1ul << 5))
#define FLASH_STRT              ((unsigned int)(1ul << 6))
#define FLASH_LOCK              ((unsigned int)(1ul << 7))
#define FLASH_OPTWRE            ((unsigned int)(1ul << 9))

// Flash Status Register definitions
#define FLASH_BSY               ((unsigned int)(1ul << 0))
#define FLASH_PGERR             ((unsigned int)(1ul << 2))
#define FLASH_WRPRTERR          ((unsigned int)(1ul << 4))
#define FLASH_EOP               ((unsigned int)(1ul << 5))

#define FLASH_ERR               (FLASH_PGERR | FLASH_WRPRTERR)



/*
 *  Initialize Flash Programming Functions
 *    Parameter:      adr:  Device Base Address
 *                    clk:  Clock Frequency (Hz)
 *                    fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

#if defined FLASH_MEM
int Init (unsigned long adr, unsigned long clk, unsigned long fnc) {

  FLASH->KEYR = FLASH_KEY1;                             // Unlock Flash
  FLASH->KEYR = FLASH_KEY2;

  FLASH->ACR  = 0x00000000;                             // Zero Wait State, no Cache, no Prefetch
  FLASH->SR  |= FLASH_ERR;                              // Reset Error Flags

  if ((FLASH->OBR & (1ul << 8)) == 0) {                 // Test if IWDG is running (IWDG in HW mode)
    // Set IWDG time out to ~32.768 second
    IWDG->KR  = 0x5555;                                 // Enable write access to IWDG_PR and IWDG_RLR     
    IWDG->PR  = 0x06;                                   // Set prescaler to 256  
    IWDG->RLR = 4095;                                   // Set reload value to 4095
  }

  return (0);
}
#endif

#ifdef FLASH_OPT
int Init (unsigned long adr, unsigned long clk, unsigned long fnc) {

  FLASH->KEYR = FLASH_KEY1;                             // Unlock Flash
  FLASH->KEYR = FLASH_KEY2;

  FLASH->OPTKEYR = FLASH_KEY1;                          // Unlock Option Bytes
  FLASH->OPTKEYR = FLASH_KEY2;

  FLASH->ACR  = 0x00000000;                             // Zero Wait State, no Cache, no Prefetch
  FLASH->SR  |= FLASH_ERR;                              // Reset Error Flags

  if ((FLASH->OBR & (1ul << 8)) == 0) {                 // Test if IWDG is running (IWDG in HW mode)
    // Set IWDG time out to ~32.768 second
    IWDG->KR  = 0x5555;                                 // Enable write access to IWDG_PR and IWDG_RLR     
    IWDG->PR  = 0x06;                                   // Set prescaler to 256  
    IWDG->RLR = 4095;                                   // Set reload value to 4095
  }

  return (0);
}
#endif


/*
 *  De-Initialize Flash Programming Functions
 *    Parameter:      fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

#if defined FLASH_MEM
int UnInit (unsigned long fnc) {

  FLASH->CR |=  FLASH_LOCK;                             // Lock Flash

  return (0);
}
#endif

#ifdef FLASH_OPT
int UnInit (unsigned long fnc) {

  FLASH->CR &= ~FLASH_OPTWRE;                           // Lock Option Bytes
  FLASH->CR |=  FLASH_LOCK;                             // Lock Flash

  return (0);
}
#endif


/*
 *  Erase complete Flash Memory
 *    Return Value:   0 - OK,  1 - Failed
 */

#ifdef FLASH_MEM
int EraseChip (void) {

  FLASH->CR |=  FLASH_MER;                              // Mass Erase Enabled
  FLASH->CR |=  FLASH_STRT;                             // Start Erase

  while (FLASH->SR & FLASH_BSY) {
    IWDG->KR = 0xAAAA;                                  // Reload IWDG
  }

  FLASH->CR &= ~FLASH_MER;                              // Mass Erase Disabled

  return (0);                                           // Done
}
#endif

#ifdef FLASH_OPT
int EraseChip (void) {

  FLASH->SR |=  FLASH_ERR;                              // Reset Error Flags
  FLASH->CR |=  FLASH_OPTER;                            // Option Byte Erase Enabled 
  FLASH->CR |=  FLASH_STRT;                             // Start Erase

  while (FLASH->SR & FLASH_BSY) {
    IWDG->KR = 0xAAAA;                                  // Reload IWDG
  }

  FLASH->CR &= ~FLASH_OPTER;                            // Option Byte Erase Disabled 

  // Unprotect Flash
  FLASH->CR |=  FLASH_OPTPG;                            // Option Byte Programming Enabled

  M16(0x1FFFF800) = RDPRT_KEY;                          // Program Half Word: RDPRT Key
  while (FLASH->SR & FLASH_BSY) {
    IWDG->KR = 0xAAAA;                                  // Reload IWDG
  }

  FLASH->CR &= ~FLASH_OPTPG;                            // Option Byte Programming Disabled

  if (FLASH->SR & FLASH_ERR) {                          // Check for Error
    FLASH->SR |= FLASH_ERR;                             // Reset Error Flags
    return (1);                                         // Failed
  }

  return (0);                                           // Done
}
#endif


/*
 *  Erase Sector in Flash Memory
 *    Parameter:      adr:  Sector Address
 *    Return Value:   0 - OK,  1 - Failed
 */

#ifdef FLASH_MEM
int EraseSector (unsigned long adr) {

    FLASH->CR  |=  FLASH_PER;                           // Page Erase Enabled 
    FLASH->AR   =  adr;                                 // Page Address
    FLASH->CR  |=  FLASH_STRT;                          // Start Erase

    while (FLASH->SR  & FLASH_BSY) {
      IWDG->KR = 0xAAAA;                                // Reload IWDG
    }

    FLASH->CR  &= ~FLASH_PER;                           // Page Erase Disabled 

  return (0);                                           // Done
}
#endif

#ifdef FLASH_OPT
int EraseSector (unsigned long adr) {

  FLASH->CR |=  FLASH_OPTER;                            // Option Byte Erase Enabled 
  FLASH->CR |=  FLASH_STRT;                             // Start Erase

  while (FLASH->SR & FLASH_BSY) {
    IWDG->KR = 0xAAAA;                                  // Reload IWDG
  }

  FLASH->CR &= ~FLASH_OPTER;                            // Option Byte Erase Disabled 

  return (0);                                           // Done
}
#endif


/*  
 *  Blank Check Checks if Memory is Blank
 *    Parameter:      adr:  Block Start Address
 *                    sz:   Block Size (in bytes)
 *                    pat:  Block Pattern
 *    Return Value:   0 - OK,  1 - Failed
 */

#ifdef FLASH_OPT
int BlankCheck (unsigned long adr, unsigned long sz, unsigned char pat) {
  return (1);                                           // Always Force Erase
}
#endif


/*
 *  Program Page in Flash Memory
 *    Parameter:      adr:  Page Start Address
 *                    sz:   Page Size
 *                    buf:  Page Data
 *    Return Value:   0 - OK,  1 - Failed
 */

#if defined FLASH_MEM
int ProgramPage (unsigned long adr, unsigned long sz, unsigned char *buf) {

  sz = (sz + 1) & ~1;                                   // Adjust size for Half Words
  
  while (sz) {

    FLASH->CR  |=  FLASH_PG;                            // Programming Enabled

    M16(adr) = *((u16 *)buf);                           // Program Half Word
    while (FLASH->SR  & FLASH_BSY);

    FLASH->CR  &= ~FLASH_PG;                            // Programming Disabled

    if (FLASH->SR & FLASH_ERR) {                        // Check for Error
      FLASH->SR |= FLASH_ERR;                           // Reset Error Flags
      return (1);                                       // Failed
    }

    adr += 2;                                           // Go to next Half Word
    buf += 2;
    sz  -= 2;
  }

  return (0);                                           // Done
}
#endif

#ifdef FLASH_OPT
int ProgramPage (unsigned long adr, unsigned long sz, unsigned char *buf) {

  sz = (sz + 1) & ~1;                                   // Adjust size for Half Words
  
  while (sz) {

    FLASH->CR |=  FLASH_OPTPG;                          // Option Byte Programming Enabled

    M16(adr) = *((u16 *)buf);                           // Program Half Word
    while (FLASH->SR & FLASH_BSY) {
      IWDG->KR = 0xAAAA;                                // Reload IWDG
    }

    FLASH->CR &= ~FLASH_OPTPG;                          // Option Byte Programming Disabled

    if (FLASH->SR & FLASH_ERR) {                        // Check for Error
      FLASH->SR |= FLASH_ERR;                           // Reset Error Flags
      return (1);                                       // Failed
    }

    // Go to next Half Word
    adr += 2;
    buf += 2;
    sz  -= 2;
   
  }

  return (0);                                           // Done
}
#endif
