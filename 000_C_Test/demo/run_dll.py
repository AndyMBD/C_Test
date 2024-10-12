# from ctypes import *
import ctypes

class stru_RC_t(ctypes.Structure):
    _fields_ = [ ("yk_1", ctypes.c_int32), 
                ("coef", ctypes.c_uint16)]
stru_RC_Curr = stru_RC_t()
# class POINT(Structure):
#     _fields_ = [("x", c_int),
#                 ("y", c_int)]
dll = ctypes.CDLL("./build/004_C_lpf/004_C_lpf/Debug/my_dll.dll")
# a = dll.add(1, 2)
# i=ctypes.c_uint16(100)
# dll.lowPass_filter()
# a=dll.
i=dll.lowPass_filter(&stru_RC_Curr,ctypes.c_uint16(100))
print(i)


