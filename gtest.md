# Cmake Learn
## 1. 使用Cmake

    1.1 项目文件目录结构
    
项目目录结构如下图，facedetect和facefeature都要用到opencl中的include文件,1.2,2.0,2.1是不同的opencl版本的头文件，  
希望在cmake创建Makefile时，opencl文件夹下的CMakeLists.txt能定义类似INCLUDE_OPENCL_1_2,INCLUDE_OPENCL_2_0,INCLUDE_OPENCL_2_1,这样的变量，以保存不同版本的opencl 头文件位置，    
最后关键是能让项目中其他子目录的CMakeList.txt能使用这些变量。
```c
├─facedetect
├─facefeature
└─opencl
    └─include
       ├─1.2
       │  └─CL
       ├─2.0
       │  └─CL
       └─2.1
           └─CL
```

2. 方法

   2.1. set:

一般用[set]1命令定义的变量能从父目录传递到子目录，  
但opencl与facedetect和facefeature在同级目录，所以用set定义的变量无法共享，  
要用set(variable value CACHE INTERNAL docstring )这种方式定义的变量会把变量加入到CMakeCache.txt然后各级目录共享会访问到这个变量  
比如:   
在opencl下的CMakeLists.txt中定义一个变量
```cmake
set(ICD_LIBRARY "${PROJECT_BINARY_DIR}/lib" CACHE INTERNAL "ICD Library location" )
//"ICD Library location"这个字符串相当于对变量的描述说明，不能省略，但可以自己随便定义
```  
在facedetect下的CMakeLists.txt中读取这个一个变量
```cmake
MESSAGE(STATUS "ICD_LIBRARY :${ICD_LIBRARY}")
```  
每次运行cmake都会更新这个变量，你会在CMakeCache.txt中找到这个变量
```cmake
//ICD Library location
ICD_LIBRARY:INTERNAL=J:/workspace/facecl.prj/lib
```  
2. set_property / get_property  

使用[set_property]2实现共享变量的方法，不会将变量写入CMakeCache.txt，应该是内存中实现的。   
当用set_property定义的property时，第一个指定作用域(scope)的参数设为GLOBAL，这个property在cmake运行期间作用域就是全局的。然后其他目录下的CMakeLists.txt可以用[get_property]3来读取这个property  


在opencl下的CMakeLists.txt中定义一个名为INCLUDE_OPENCL_1_2 的global property  
```cmake
set_property(GLOBAL PROPERTY INCLUDE_OPENCL_1_2 "${CMAKE_CURRENT_LIST_DIR}/include/1.2" )
```  
在facedetect下的CMakeLists.txt中读取这个一个property
```cmake
//先调用get_property将这个property读取到一个变量中(variable)INCLUDE_OPENCL 
get_property(INCLUDE_OPENCL GLOBAL PROPERTY "INCLUDE_OPENCL_1_2" ) 
//显示INCLUDE_OPENCL 
MESSAGE(STATUS "INCLUDE_OPENCL :${INCLUDE_OPENCL}")
```

上面的例子可以看出这种方式相比方法一在使用变量时多了一步，先要将先调用get_property将这个property读取到一个变量中(variable)才能使用。

Summary:  
两种方法相比，从使用便利性角度，方法一好一些，但方法一将变量保存在CMakeCache.txt，需要读写CMakeCache.txt文件，目前没有发现别的副作用，但记住这个区别是有好处的。   

3. CMake Library output directory Set  

   1. CMAKE_ARCHIVE_OUTPUT_DIRECTORY：默认存放静态库的文件夹位置
   2. CMAKE_LIBRARY_OUTPUT_DIRECTORY：默认存放动态库的文件夹位置
   3. LIBRARY_OUTPUT_PATH：默认存放库文件的位置，如果产生的是静态库并且没有指定 
   4. CMAKE_ARCHIVE_OUTPUT_DIRECTORY 则存放在该目录下，动态库也类似
   5. CMAKE_RUNTIME_OUTPUT_DIRECTORY：存放可执行软件的目录

## 2. dllexport and dllimport
动态链接库的使用可分为：  
显式调用：使用LoadLibrary载入动态链接库-GetProcAddress获取某函数地址。  
隐式调用：使用```c#pragma comment(lib, "XX.lib")```的方式，也可以直接将XX.lib加入到工程中。　　
  
    2.1 定义及基本用法  

　　按C++标准，class 与className 中间不可以存在任何实质性的东西的。但dllimport / dllexport只是修饰符，Windows平台下为了dll的兼容性的特有关键字，他们都是DLL内的关键字，即导出与导入。他们是将DLL内部的类与函数以及数据导出与导入时使用的，看它的具体定义是什么。一般类的修饰符有导入或导出即：

__declspec(dllexport) 
extern __declspec(dllimport) 
       dllexport是在这些类、函数以及数据声明的时候使用。用他表明这些东西可以被外部函数使用，即（dllexport）是把 DLL中的相关代码（类，函数，数据）暴露出来为其他应用程序使用。使用了（dllexport）关键字，相当于声明了紧接在（dllexport）关键字后面的相关内容是可以为其他程序使用的。

       dllimport是在外部程序需要使用DLL内相关内容时使用的关键字。当一个外部程序要使用DLL 内部代码（类，函数，全局变量）时，只需要在程序内部使用（dllimport）关键字声明需要使用的代码就可以了，即（dllimport）关键字是在外部程序需要使用DLL内部相关内容的时候才使用。（dllimport）作用是把DLL中的相关代码插入到应用程序中。

      　_declspec(dllexport)与_declspec(dllimport)是相互呼应，只有在DLL内部用dllexport作了声明，才能在外部函数中用dllimport导入相关代码。但MSDN文档里面，对于 __declspec(dllimport)的说明让人感觉有点奇怪，先来看看MSDN里面是怎么说的：

         不使用 __declspec(dllimport)也能正确编译代码，但使用 __declspec(dllimport) 使编译器可以生成更好的代码。编译器之所以能够生成更好的代码，是因为它可以确定函数是否存在于 DLL 中，这使得编译器可以生成跳过间接寻址级别的代码，而这些代码通常会出现在跨DLL 边界的函数调用中。但是，必须使用 __declspec(dllimport) 才能导入 DLL 中使用的变量。

      使用__declspec(dllimport)可以生成更好的代码，这点好理解，但必须使用它才能导出dll中的变量，对于动态库本身必须使用关键字__declspec(dllexport)，对于应用程序，如果不使用动态库导出的变量，不使用关键字__declspec(dllimport)也可以保证动态库的正常使用，但实际使用中，还是建议应用程序使用关键字__declspec(dllimport)，具体原因，还是上面MSDN的那段话。

> 注意：动态库与静态库并存

 　　另外，有时我们的程序需要同时提供动态库和静态库库，且都使用一个头文件，为了解决关键字的使用冲突，建议使用如下的宏定义：

```c
#ifdefined DLL_EXPORTS
   #ifdefined INSIDE_DLL
      #define SIMPLE_CLASS_EXPORT__declspec(dllexport)
   #else
     #define SIMPLE_CLASS_EXPORT__declspec(dllimport)
   #endif
   #else
      #define SIMPLE_CLASS_EXPORT
#endif 
```  

　　  对于动态库本身，需要定义宏DLL_EXPORTS和INSIDE_DLL 使用动态库的应用程序定义宏DLL_EXPORTS，对于静态库，不需要定义DLL_EXPORTS，当然静态库的应用程序也不需要定义。如此定义，就可以让动态库和静态库的导出都使用同一份头文件。

 二、实现及相关问题（导出类的简单方式）
　　加载一个dll时，其实你的程序是运行在两个独立空间的（dll的空间和你自己的程序空间），dll的对象模型其实相当严格，要访问dll空间的变量和函数，必须导出他们，否则这些对象是不可见的。这可以通过加入一个def文件，或者在声明中使用__declspec(dllimport)前缀，告诉编译器以下这些变量和函数是从dll导出的。同时定义这些变量的dll源文件必须加上__declspec(dllexport)前缀，告诉编译器这些函数需要被导出。 

　　对类对象来说，静态成员和函数必须加上这个前缀，因为这些对象都是在dll空间内的。在类的前面加上这些前缀就对整个类的成员进行了声明。这样在你的dll工程中定义__DLLEXPORT_IMP，__DLLEXPORT就会根据不同的工程转换成相应的前缀声明了。如果不加入这些前缀，链接会出现找不到符号的错误，因为这些符号在lib文件中被隐藏了。

复制代码
//一般这样写一个宏： 
#if defined __DLLEXPORT_IMP 
#define __DLLEXPORT __declspec(dllexport) 
#else 
#define __DLLEXPORT __descspec(dllimport) 
#endif 
复制代码
　　分析如下代码：

class VTK_PARALLEL_EXPORT vtkCompositer: public vtkObject
{
　　//...
};
　　关键字class和类名之间包含其他内容，这里的VTK_PARALLEL_EXPORT应该就是之前定义的可修饰class导入/到处的宏了。这样主要还是为了使用方便，在编写库时，只要定义了VTK_PARALLEL_EXPORT 宏，所有动态库中的类都会自动导出。如果内部使用的话将该宏定义将被展开为空串，在多文件或多个dll的情况下使用非常方便。

　　这种方式是比较简单的，同时也是不建议采用的不合适方式。只需要在导出类加上__declspec(dllexport)，就可以实现导出类。对象空间还是在使用者的模块里，dll只提供类中的函数代码。不足的地方是：使用者需要知道整个类的实现，包括基类、类中成员对象，也就是说所有跟导出类相关的东西，使用者都要知道。通过Dependency Walker可以看到，这时候的dll导出的是跟类相关的函数：如构造函数、赋值操作符、析构函数、其它函数，这些都是使用者可能会用到的函数。

　　这种导出类的方式，除了导出的东西太多、使用者对类的实现依赖太多之外，还有其它问题：必须保证使用同一种编译器。导出类的本质是导出类里的函数，因为语法上直接导出了类，没有对函数的调用方式、重命名进行设置，导致了产生的dll并不通用。 

三、使用虚函数导出（不使用_declspec(dllexport) / _declspec(dllimport)）
　　跟com类似，导出类是一个派生类，派生自一个抽象类——都是纯虚函数。使用者需要知道这个抽象类的结构。DLL最少只需要提供一个用于获取类对象指针的接口。使用者跟DLL提供者共用一个抽象类的头文件，使用者依赖于DLL的东西很少，只需要知道抽象类的接口，以及获取对象指针的导出函数，对象内存空间的申请是在DLL模块中做的，释放也在DLL模块中完成,最后记得要调用释放对象的函数。

这种方式通用，产生的DLL没有特定环境限制。借助了C++类的虚函数。一般都是采用这种方式。除了对DLL导出类有好处外，采用接口跟实现分离，可以使得工程的结构更清晰，使用者只需要知道接口，而不需要知道实现。