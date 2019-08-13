#ifndef __HLIBLOAD_H__
#define __HLIBLOAD_H__

#ifdef __LINUX__

#include <dlfcn.h>

inline void *loadLib(const char *libName){
    return dlopen(libName, RTLD_LAZY);
}
inline void *loadFun(void *libHandle, const char *funName){
    return dlsym(libHandle, funName);
}

inline int unloadLib(void *libHandle){
    return dlclose(libHandle);
}

#else

#include <windows.h>

inline void *loadLib(const char *libName){
    WCHAR wszClassName[256];
    memset(wszClassName,0,sizeof(wszClassName));
    MultiByteToWideChar(CP_ACP,0,libName,strlen(libName)+1,wszClassName,
                        sizeof(wszClassName)/sizeof(wszClassName[0]));
    return LoadLibrary(wszClassName);
}

inline void *loadFun(void *libHandle, const char *funName){
    return (void *)GetProcAddress((HINSTANCE )libHandle, funName);
}

inline int unloadLib(void *libHandle){
    return FreeLibrary((HINSTANCE)libHandle)? 0 : -1;
}
#endif

#endif
