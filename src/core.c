// core.c

#include "core.h"
#include "decode.h"
#include "opcodes.h"

#include <stddef.h>
#include <string.h>

#define DO_BOUNDS_CHECK 1
#define NO_BOUNDS_CHECK 0

// Forward decls of local functions

// Mutates regfile according to reg-reg instruction parameters
static int execute_reg_reg(core_state_t* core, uint32_t instruction_bits, r_type_rv32i_t data);
static int execute_imm_arith(core_state_t* core, uint32_t instruction_bits, i_type_rv32i_t data);
static int execute_load(core_state_t* core, i_type_rv32i_t data);
static int execute_store(core_state_t* core, s_type_rv32i_t data);
static int execute_lui(core_state_t* core, u_type_rv32i_t data);
static int execute_auipc(core_state_t* core, u_type_rv32i_t data);
static int execute_branch(core_state_t* core, b_type_rv32i_t data);
static int execute_jal(core_state_t* core, j_type_rv32i_t data);
static int execute_jalr(core_state_t* core, i_type_rv32i_t data);

// Fetches from memory, performs bounds check depending on "check"
// Fetches "width" bytes, in little-endian order
// Performs no sign extension
static uint32_t fetch_width(core_state_t* core, uint32_t byte_addr, uint8_t width, uint8_t check) {
    (void)check;
    
    switch (width) {
        case 1:
            return core->read8(byte_addr);
        case 2:
            return core->read16(byte_addr);
        case 4:
            return core->read32(byte_addr);
        default:
            return 0xDEADC0DE;
    }
}

// Stores to memory
// Stores "width" bytes, in little-endian order
// Performs no sign extension
static uint32_t store_width(core_state_t* core, uint32_t word, uint32_t byte_addr, uint8_t width, uint8_t check) {
    (void)check;

    switch (width) {
        case 1:
            core->write8(byte_addr, (uint8_t)(word & 0xFF));
            break;
        case 2:
            core->write16(byte_addr, (uint16_t)(word & 0xFFFF));
            break;
        case 4:
            core->write32(byte_addr, word);
            break;
        default:
            return 0xDEADC0DE;
    }

    return word;
}


int execute_rv32i(core_state_t* next) {
    // Fetch instruction from memory
    // fetch_width will perform bounds checking and frustrate
    // anyone trying to perform a VM escape
    uint32_t instruction_bits = fetch_width(next, next->pc_reg, 4, DO_BOUNDS_CHECK);


    // We set r0 to zero before executing the next instruction.
    // In hardware, r0 is wired directly to 0x0, but performing
    // checks on every instruction to do special behavior depending
    // on whether it uses x0 would introduce a large number of
    // unnecessary branches/checks. By setting it to zero first,
    // we effectively discard the x0 result of the last execution.
    next->regfile[0] = 0;

    // Decode the instruction
    instruction_rv32i_t decoded_ins;
    decode_rv32i(instruction_bits, &decoded_ins);
    
    // Each exec function will write to this as the return value
    int exec_result = 0; 

    // Execute the instruction
    switch (decoded_ins.opcode) {
    case OP_REG:
        // Thankfully, r_type instructions are only reg-reg instructions
        exec_result = execute_reg_reg(next, instruction_bits, decoded_ins.r_data);
        break;
    case OP_IMM:
        exec_result = execute_imm_arith(next, instruction_bits, decoded_ins.i_data);
        break;
    case OP_LD:
        exec_result = execute_load(next, decoded_ins.i_data);
        break;
    case OP_ST:
        exec_result = execute_store(next, decoded_ins.s_data);
        break;
    case OP_AUIPC:
        exec_result = execute_auipc(next, decoded_ins.u_data);
        break;
    case OP_LUI:
        exec_result = execute_lui(next, decoded_ins.u_data);
        break;
    case OP_BR:
        exec_result = execute_branch(next, decoded_ins.b_data);
        break;
    case OP_JAL:
        exec_result = execute_jal(next, decoded_ins.j_data);
        break;
    case OP_JALR:
        exec_result = execute_jalr(next, decoded_ins.i_data);
        break;
    default:
        exec_result = EXECUTE_ILLEGAL_INSTRUCTION;
        break;
    }

    next->pc_reg += 4;
    return exec_result;
}

static int execute_reg_reg(core_state_t* core, uint32_t instruction_bits, r_type_rv32i_t data) {
    switch (data.funct3) {
        // ADD/SUB
        case 0x0:  {
            int sign_bit = GET_MATH_BIT(instruction_bits) == 0 ? 1 : -1;
            core->regfile[data.rd] = (int32_t)core->regfile[data.rs1] + sign_bit * core->regfile[data.rs2];
            break;
        }
        // SLL (Shift Left Logical)
        case 0x1:
            core->regfile[data.rd] = core->regfile[data.rs1] << core->regfile[data.rs2];
            break;
        // SLT (Set if Less Than)
        case 0x2:
            core->regfile[data.rd] = (int32_t)core->regfile[data.rs1] < (int32_t)core->regfile[data.rs2];
            break;
        // SLTU (Set if Less Than, Unsigned)
        case 0x3:
            core->regfile[data.rd] = (uint32_t)core->regfile[data.rs1] < (uint32_t)core->regfile[data.rs2];
            break;
        // XOR
        case 0x4:
            core->regfile[data.rd] = core->regfile[data.rs1] ^ core->regfile[data.rs2];
            break;
        // SRL, SRA (Shift right, Logical or Arithmetic)
        case 0x5:
            // If bit30 is zero, it is a logical (unsigned) shift
            if(GET_MATH_BIT(instruction_bits) == 0){
                core->regfile[data.rd] = (uint32_t)core->regfile[data.rs1] >> (uint32_t)core->regfile[data.rs2];
            }
            else { // Else, arithmetic/signed shift
                core->regfile[data.rd] = (int32_t)core->regfile[data.rs1] >> (int32_t)core->regfile[data.rs2];
            }
            
            break;
        // OR
        case 0x6:
            core->regfile[data.rd] = core->regfile[data.rs1] | core->regfile[data.rs2];
            break;
        // AND
        case 0x7:
            core->regfile[data.rd] = core->regfile[data.rs1] & core->regfile[data.rs2];
            break;
        default:
            return EXECUTE_ILLEGAL_INSTRUCTION;
    }
    
    return EXECUTE_SUCCESS;
}

static int execute_imm_arith(core_state_t* core, uint32_t instruction_bits, i_type_rv32i_t data){
    switch (data.funct3) {
        // Sign-extended addition, immediate
        case IMM_ADDI:
            core->regfile[data.rd] = (int32_t)core->regfile[data.rs1] + SIGN_EXTEND(data.imm12, 12);
            break;
        // Set if less than, immediate
        case IMM_SLTI:
            core->regfile[data.rd] = (int32_t)core->regfile[data.rs1] < SIGN_EXTEND(data.imm12, 12);
            break;
        // Set if less than, immediate, unsigned
        case IMM_SLTIU:
            core->regfile[data.rd] = (uint32_t)core->regfile[data.rs1] < (uint32_t)data.imm12;
            break;
        // Bitwise XOR, immediate, sign-extended
        case IMM_XORI:
            core->regfile[data.rd] = (int32_t)core->regfile[data.rs1] ^ SIGN_EXTEND(data.imm12, 12);
            break;
        // Bitwise OR, immediate, sign-extended
        case IMM_ORI:
            core->regfile[data.rd] = (int32_t)core->regfile[data.rs1] | SIGN_EXTEND(data.imm12, 12);
            break;
        // Bitwise AND, immediate, sign-extended
        case IMM_ANDI:
            core->regfile[data.rd] = (int32_t)core->regfile[data.rs1] & SIGN_EXTEND(data.imm12, 12);
        default:
            return EXECUTE_ILLEGAL_INSTRUCTION;
    }
    
    return EXECUTE_SUCCESS;
}

static int execute_load(core_state_t* core, i_type_rv32i_t data) {
    // Sign extend 12-bit immediate, add to rs1 base address
    uint32_t addr = (uint32_t)core->regfile[data.rs1] + SIGN_EXTEND(data.imm12, 12);
    
    uint32_t loaded_data;
    // Mask the lower two bits just to get the load width
    switch (data.funct3 & LD_WIDTH_MASK) {
        // Load byte
        case LD_B:
            // Fetch single byte from memory, request bounds check
            loaded_data = fetch_width(core, addr, 1, DO_BOUNDS_CHECK);

            if(data.funct3 & LD_UNSIGNED_MASK){
                core->regfile[data.rd] = (uint32_t)loaded_data;
            } else {
                core->regfile[data.rd] = SIGN_EXTEND(loaded_data, 8);
            }

            break;

        // Load half
        case LD_H:
        {
            // Fetch the two bytes from memory (little endian)
            uint32_t loaded_data = fetch_width(core, addr, 2, DO_BOUNDS_CHECK);
            
            if(data.funct3 & LD_UNSIGNED_MASK){
                core->regfile[data.rd] = (uint32_t)loaded_data;
            }
            else {
                core->regfile[data.rd] = SIGN_EXTEND(loaded_data, 16);
            }
            
            break;
        }
        // Load word
        case LD_W:
            // Fetch the four bytes from memory (little endian)
            core->regfile[data.rd] = fetch_width(core, addr, 4, DO_BOUNDS_CHECK);
            break;
        default:
            return EXECUTE_INVALID_LOAD_WIDTH;
    }

    return EXECUTE_SUCCESS;
}

static int execute_store(core_state_t* core, s_type_rv32i_t data) {
    // Sign extend 12-bit immediate, add to rs1 base address
    uint32_t addr = (uint32_t)core->regfile[data.rs1] + SIGN_EXTEND(data.imm12, 12);
    uint32_t width = 1 << (data.funct3 & LD_WIDTH_MASK);
    
    store_width(core, core->regfile[data.rs2], addr, width, 1);
    return EXECUTE_SUCCESS;
}

static int execute_lui(core_state_t* core, u_type_rv32i_t data) {
    core->regfile[data.rd] = data.imm32;
    return EXECUTE_SUCCESS;
}

static int execute_auipc(core_state_t* core, u_type_rv32i_t data) {
    core->regfile[data.rd] = data.imm32 + core->pc_reg;
    return EXECUTE_SUCCESS;
}

static int execute_branch(core_state_t* core, b_type_rv32i_t data) {
    int should_branch = 0;
    
    switch (data.funct3) {
    case BR_BEQ:
        should_branch = core->regfile[data.rs1] == core->regfile[data.rs2];
        break;
    case BR_BNE:
        should_branch = core->regfile[data.rs1] != core->regfile[data.rs2];
        break;
    case BR_BLT:
        should_branch = (int32_t)core->regfile[data.rs1] < (int32_t)core->regfile[data.rs2];
        break;
    case BR_BGE:
        should_branch = (int32_t)core->regfile[data.rs1] >= (int32_t)core->regfile[data.rs2];
        break;
    case BR_BLTU:
        should_branch = core->regfile[data.rs1] < core->regfile[data.rs2];
        break;
    case BR_BGEU:
        should_branch = core->regfile[data.rs1] >= core->regfile[data.rs2];
        break;
    }

    if (should_branch) {
        core->pc_reg += SIGN_EXTEND(data.imm13, 13) - 4;
    }
    
    return EXECUTE_SUCCESS;
}

static int execute_jal(core_state_t* core, j_type_rv32i_t data) {
    core->regfile[data.rd] = core->pc_reg + 4;
    core->pc_reg += SIGN_EXTEND(data.imm21, 21) - 4;

    return EXECUTE_SUCCESS;
}

static int execute_jalr(core_state_t* core, i_type_rv32i_t data) {
    core->regfile[data.rd] = core->pc_reg + 4;
    core->pc_reg = ((SIGN_EXTEND(data.imm12, 12) + core->regfile[data.rs1]) & ~0x1) - 4;

    return EXECUTE_SUCCESS;
}
