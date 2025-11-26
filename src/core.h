// core.h

#ifndef CORE_H
#define CORE_H

#include <stdint.h>

#define REGFILE_SIZE 32

typedef struct core_state_t
{
    uint32_t pc_reg; // Program counter, points to next instruction
    uint32_t regfile[REGFILE_SIZE]; // Main regfile
    
    uint8_t (*read8)(uint32_t addr);
    uint16_t (*read16)(uint32_t addr);
    uint32_t (*read32)(uint32_t addr);
    
    void (*write8)(uint32_t addr, uint8_t value);
    void (*write16)(uint32_t addr, uint16_t value);
    void (*write32)(uint32_t addr, uint32_t value);
} core_state_t;

typedef enum execute_result_t
{
    EXECUTE_SUCCESS = 0,
    EXECUTE_ILLEGAL_INSTRUCTION = -1,
    EXECUTE_INVALID_LOAD_WIDTH = -2,
} execute_result_t;

int execute_rv32i(core_state_t* next);

#endif
