/* Real S5F CPU -> real MMIO -> slidewin/fsrc/readback/D1 integration test.
 * D1 uses golden M10 activations installed by the testbench as an input fixture. */
#include <stdint.h>
#include "gate_regs.h"
#include "gate_map.h"
#define GPIO (*(volatile uint32_t *)0x20000000u)
static void fail(uint32_t n) { GPIO=0xe0u|n; for (;;) {} }
static void wait(uint32_t mask) {
    uint32_t i;
    for(i=0;i<2000000;i++) if(GATE_REG(R_STATUS)&mask) return;
    fail(1);
}
static uint32_t read_word(uint16_t pixel, unsigned index) {
    GATE_REG(R_CLR)=X_RD;
    GATE_REG(R_RDADDR)=pixel|(1u<<16);
    GATE_REG(R_CTRL)=C_RD_GO;
    wait(S_RD_READY);
    GATE_REG(R_RDIDX)=index;
    return GATE_REG(R_RDDATA);
}
int main(void) {
    volatile uint32_t *timer=(volatile uint32_t *)GATE_TIMER_BASE;
    uint32_t t, a;
    if(GATE_REG(R_ID)!=GATE_ID_EXPECT) fail(2);
    t=*timer; a=*timer; if(a==t) fail(3);
    GPIO=1;
    GATE_REG(R_CLR)=X_ALL;
    GATE_REG(R_WINSEL)=0;
    GATE_REG(R_WINSEL2)=0;
    GATE_REG(R_CTRL)=C_SEEK;
    wait(S_SW_WIN);
    if(GATE_REG(R_SWSTAT)!=0) fail(4);
    /* First pixel of frame0 is 131 -> q8 = 4 in all RGB channels. */
    GATE_REG(R_CLR)=X_FSRC;
    GATE_REG(R_CTRL)=C_FSRC_GO;
    wait(S_FSRC_DONE);
    if(read_word(6144,0)!=0x00040404u) fail(5);
    if(read_word(6144,1)!=0u) fail(6);
    GATE_REG(R_CLR)=X_ALL;
    if(GATE_REG(R_STATUS)&(S_FSRC_DONE|S_RD_READY)) fail(7);
    GPIO=2; /* TB now installs the M10 golden fixture. */
    GATE_REG(R_D1B)=GATE_D1_IN_BASE;
    GATE_REG(R_D1SH)=GATE_D1_SH_REQ;
    GATE_REG(R_CTRL)=C_D1_GO;
    wait(S_D1_DONE);
    /* TB compares all 64 embedding values; exercise CPU readback too. */
    for(a=0;a<2;a++) {
        uint32_t k;
        GATE_REG(R_CLR)=X_RD;
        GATE_REG(R_RDADDR)=a|(1u<<16);
        GATE_REG(R_CTRL)=C_RD_GO|C_HOLD;
        wait(S_RD_READY);
        for(k=0;k<8;k++) {
            GATE_REG(R_RDIDX)=k;
            ((volatile uint32_t *)0x7000u)[a*8+k]=GATE_REG(R_RDDATA);
        }
    }
    GPIO=0xa5;
    for (;;) {}
}
