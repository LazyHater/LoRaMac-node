#ifdef D_MOTE

#include "CommissioningDMote.h"

#elif defined K_MOTE

#include "CommissioningKMote.h"

#elif defined TEST_MOTE

#include "CommissioningTestMote.h"

#else

#error Pls add define for D_MOTE or TEST_MOTE

#endif