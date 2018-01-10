#include "cassie_user_in_t.h"
#include <stddef.h>

void cassie_user_in_t_initialize(void)
{
}

void cassie_user_in_t_terminate(void)
{
}

void pack_cassie_user_in_t(const cassie_user_in_t *bus, unsigned char bytes[98])
{
  double x[10];
  unsigned char y[80];
  int i;
  unsigned char b_y[18];
  short b_x[9];
  memcpy(&x[0], &bus->torque[0], 10U * sizeof(double));
  memcpy((void *)&y[0], (void *)&x[0], (unsigned int)((size_t)80 * sizeof
          (unsigned char)));
  memcpy(&bytes[0], &y[0], 80U * sizeof(unsigned char));
  for (i = 0; i < 9; i++) {
    b_x[i] = bus->telemetry[i];
  }

  memcpy((void *)&b_y[0], (void *)&b_x[0], (unsigned int)((size_t)18 * sizeof
          (unsigned char)));
  for (i = 0; i < 18; i++) {
    bytes[i + 80] = b_y[i];
  }
}

void unpack_cassie_user_in_t(const unsigned char bytes[98], cassie_user_in_t
  *bus)
{
  unsigned char x[80];
  int i;
  unsigned char b_x[18];
  memcpy(&x[0], &bytes[0], 80U * sizeof(unsigned char));
  memcpy((void *)&bus->torque[0], (void *)&x[0], (unsigned int)((size_t)10 *
          sizeof(double)));
  for (i = 0; i < 18; i++) {
    b_x[i] = bytes[i + 80];
  }

  memcpy((void *)&bus->telemetry[0], (void *)&b_x[0], (unsigned int)((size_t)9 *
          sizeof(short)));
}
