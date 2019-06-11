#ifndef __HIROP_DEBUG_H__
#define __HIROP_DEBUG_H__

namespace hirop_vision{

#ifdef DEBUG
#define IDebug(format, ...)  printf("\033[32m[debug]: " #format "\033[0m\n", ##__VA_ARGS__);
#else
#define IDebug(format, ...)
#endif

#define ISuccessPrint(format, ...)  printf("\033[32m[success]: " #format "\033[0m \n", ##__VA_ARGS__);

#define IErrorPrint(format, ...)  printf("\033[32m[error]: " #format "\033[0m \n", ##__VA_ARGS__);
}

#endif
