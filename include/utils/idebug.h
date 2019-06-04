#ifndef __IDEBUG_H__
#define __IDEBUG_H__

#ifdef DEBUG
#define IDebug(format, ...)  printf("\033[32m[debug]: " #format "\033[0m\n", ##__VA_ARGS__);
#else
#define IDebug(format, ...)
#endif

#define ISuccessPrint(format, ...)  printf("\033[32m[output]: " #format "\033[0m \n", ##__VA_ARGS__);

#define IErrorPrint(format, ...)  printf("\033[32m[output]: " #format "\033[0m \n", ##__VA_ARGS__);

#endif
