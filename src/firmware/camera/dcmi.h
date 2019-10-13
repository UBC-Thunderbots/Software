#include <stdint.h>
#include <stdbool.h>


extern uint8_t img_buffer1[101376];
//extern uint8_t img_buffer2[50688];

bool dcmi_init(void);
bool dcmi_dma_init(void);
void dcmi_sync_with_new_frame(void);
void dma2_stream1_isr(void);
void dcmi_isr(void);
