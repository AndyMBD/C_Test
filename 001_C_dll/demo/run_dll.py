import ctypes

dll = ctypes.CDLL("./build/my_dll.dll")
a = dll.add(1, 2)
print(a)
