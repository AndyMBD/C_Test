package main

import (
	"fmt"
	"syscall"
)

var (
	dll     = syscall.NewLazyDLL("../my_dll.dll")
	addFunc = dll.NewProc("add")
)

func main() {
	ret1, ret2, err := addFunc.Call(123, 22)
	fmt.Println(ret1, ret2, err)
}
