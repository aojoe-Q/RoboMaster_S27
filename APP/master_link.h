#ifndef MASTER_LINK_H
#define MASTER_LINK_H
#include <stdint.h>
#include <stdbool.h>
typedef struct {
    uint8_t payload[30], wire_seq;
    uint32_t seq, rx_tick;
    bool valid;
} MasterRawSnapshot;
typedef struct {
    uint8_t payload[12], wire_seq;
    uint32_t seq, rx_tick;
    bool valid;
} ImageKeyboardSnapshot;
/* 调用方拿到的是完整一致的副本；不做任何猜测性的通道解码或映射。 */
bool MasterLink_Read(MasterRawSnapshot *out);
bool ImageKeyboard_Read(ImageKeyboardSnapshot *out);
void MasterLink_Invalidate(void);
void MasterLink_Poll(void);
void Image_Transmission_StreamReset(void);
void Image_Transmission_DMAEvent(uint16_t position);
#endif
