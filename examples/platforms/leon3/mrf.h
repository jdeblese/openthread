#ifndef __MRF_H__
#define __MRF_H__

#define shortWr(addr,data) ((((addr) & 0x3F)<<1) | 1)<<24, (data)<<24
#define shortRd(addr)      ((((addr) & 0x3F)<<1)    )<<24, 0x00
#define longWr(addr,data)  (0x800 | (((addr) & 0x3FF)<<1) | 1)<<20, (data)<<20
#define longRd(addr)       (0x800 | (((addr) & 0x3FF)<<1))    <<20, 0x00

uint8_t transaction(uint32_t addr, uint32_t data);
PhyState getRadioState(void);
PhyState getAckWait(void);

#endif
