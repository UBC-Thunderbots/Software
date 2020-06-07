// https://github.com/dhess/c-ringbuf/blob/master/ringbuf.c
// https://github.com/dhess/c-ringbuf/blob/master/ringbuf.h
// https://stackoverflow.com/questions/42903600/ring-buffer-on-c

struct circular_buffer
{
    float *bufferData;
    int head;
    int tail;
};

// TODO comment headers
void circular_buffer_init(struct circular_buffer *cbuff);

void circular_buffer_free(struct circular_buffer *cbuff);

void circular_buffer_push(struct circular_buffer *cbuff, float data);

float circular_buffer_get_index(struct circular_buffer *cbuff, int index);
