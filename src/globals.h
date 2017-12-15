#ifndef GLOBAL_H
#define GLOBAL_H

#include "frisbeem.h"
#include "system_info.h"
#include <WString.h>
extern Frisbeem frisbeem;

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

using namespace std;
#endif //GLOBAL_H
