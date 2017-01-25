#ifndef ACTIONS_H_INCLUDED
#define ACTIONS_H_INCLUDED

#include <stdbool.h>

void searchBall(uint8_t *m,  uint8_t *s, int span, int gap, float *ret);
void leaveLittleStadium(uint8_t *m,  uint8_t *s);
void leaveBiggerStadium(uint8_t *m,  uint8_t *s, int begSide);

#endif // ACTIONS_H_INCLUDED

