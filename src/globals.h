#ifndef GLOBAL_H
#define GLOBAL_H

#include "frisbeem.h"
#include "system_info.h"
#include <WString.h>
extern Frisbeem frisbeem;

using namespace std;

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

#endif //GLOBAL_H
