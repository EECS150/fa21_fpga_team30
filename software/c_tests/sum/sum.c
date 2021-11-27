#include "types.h"
#include "memory_map.h"
#include "ascii.h"
#include "uart.h"

// Source: one of the bmark tests from ASIC lab
// John C. Wright
// johnwright@eecs.berkeley.edu
// Do some random stuff to test EECS151/251A rv32ui processors

#define NUMELTS 64
#define CONST   2080
#define BUF_LEN 128

unsigned int assert_equals(unsigned int a, unsigned int b);
int x[NUMELTS];
int y[NUMELTS];

void main() {
  csr_tohost(0);
  int i;
  int j;
  int k;
  uint32_t result, time, instructions;
  int8_t buffer[BUF_LEN];


  for(k=0; k< 2 ;k++){
    COUNTER_RST = 0;
    for(i = 0; i < NUMELTS; i++) {
      x[i] = i + 1;
    }
  
    for(i = 0; i < NUMELTS; i++) {
      y[i] = 0;
      for(j=0; j < i+1; j++) {
        y[i] = y[i] + x[j];
      }
    }

    time = CYCLE_COUNTER;
    instructions = INSTRUCTION_COUNTER;
//    uwrite_int8s("\r\nCycle Count: ");
//    uwrite_int8s(uint32_to_ascii_hex(time, buffer, BUF_LEN));
//    uwrite_int8s("\r\nInstruction Count: ");
//   uwrite_int8s(uint32_to_ascii_hex(instructions, buffer, BUF_LEN));
//    uwrite_int8s("\r\n");
    COUNTER_RST = 0;
  }

  if (assert_equals(y[NUMELTS-1], CONST)) {
    csr_tohost(1);
  } else {
    csr_tohost(2);
  }

  // spin
  for( ; ; ) {
    asm volatile ("nop");
  }
}

unsigned int assert_equals(unsigned int a, unsigned int b) {
  return (a == b);
}
