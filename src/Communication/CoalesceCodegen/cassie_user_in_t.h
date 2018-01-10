#ifndef CASSIE_USER_IN_T_H
#define CASSIE_USER_IN_T_H
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "cassie_user_in_t_types.h"

#ifdef __cplusplus
extern "C" {
#endif
extern void cassie_user_in_t_initialize(void);
extern void cassie_user_in_t_terminate(void);
extern void pack_cassie_user_in_t(const cassie_user_in_t *bus, unsigned char
  bytes[98]);
extern void unpack_cassie_user_in_t(const unsigned char bytes[98],
  cassie_user_in_t *bus);
#ifdef __cplusplus
}
#endif


#endif
