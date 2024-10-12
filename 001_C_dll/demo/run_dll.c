#include <windows.h>
#include "stdio.h"
#include "stdint.h"
// #include "cJSON_Utils.h"
// #include "my_lib.h"
#include "my_dll.h"
#include "my_lib.h"
// #pragma comment(lib,"my_lib.lib")
// extern uint32_t multiply(uint16_t a,uint16_t b);
// __declspec(dllimport) uint32_t add(uint16_t a,uint16_t b);
// __declspec(dllimport) uint32_t multiply(uint16_t a,uint16_t b);

int main()
{
    // define functional pointer
    // uint32_t(*p_add)(uint16_t, uint16_t) = NULL;
    // uint32_t(*p_multiply)(uint16_t, uint16_t) = NULL;
    // char *post_str = NULL;
    // cJSON *root = cJSON_CreateObject();

    // HINSTANCE handle = LoadLibrary("./lib_out/my_dll.a");
    // HINSTANCE my_dll_handle = LoadLibrary("./dll/my_dll.dll");
    // HINSTANCE my_lib_handle = LoadLibrary("./lib_out/my_lib.dll");

    // p_add = (uint32_t(*)(uint16_t, uint16_t))GetProcAddress(my_dll_handle, "add");
    // p_multiply = (uint32_t(*)(uint16_t, uint16_t))GetProcAddress(my_lib_handle, "add");

    printf("fout:%d\n",add(1, 32));
    printf("fout:%d\n",multiply(1, 32));
    // printf("fout:%d\n",multiply(1, 32));
    // FreeLibrary(my_dll_handle);
    // printf("fout:%d\n",multiply(1, 32));

    // cJSON_AddStringToObject(root, "user", "jasonkay");
    // cJSON_AddStringToObject(root, "pwd", "hkcw3cjbc");
    // post_str = cJSON_Print(root);
    // cJSON_Delete(root);
    // root = NULL;
    // printf("post_str is %s \n",post_str);
    return 0;

}
