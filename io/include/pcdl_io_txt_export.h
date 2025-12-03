//
// Created by Mengfanyong on 2025/12/3.
//

#ifndef POINTCLOUDDOCKLIB_PCDL_IO_TXT_EXPORT_H
#define POINTCLOUDDOCKLIB_PCDL_IO_TXT_EXPORT_H
#ifdef _WIN32
    #ifdef PCDL_IO_TXT_EXPORTS
        #define PCDL_IO_TXT_API __declspec(dllexport)
    #else
        #define PCDL_IO_TXT_API __declspec(dllimport)
    #endif
#else
    #define PCDL_VISUALIZATION_API
#endif
#endif //POINTCLOUDDOCKLIB_PCDL_IO_TXT_EXPORT_H