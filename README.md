# PointCloudDockLibrary
PointCloudDockLibrary(pcdl)是一个用于处理码头场景的C++库。


# 贡献


# 构建及依赖
- 构建：
  - 使用CMake进行构建和管理。
- 依赖：
  - 使用PCL（Point Cloud Library）进行点云处理。
# 代码风格：
参考 Google C++ Style Guide 以及PCL的代码风格。
https://zh-google-styleguide.readthedocs.io/en/latest/google-cpp-styleguide/contents.html
## 命名规范 
- 文件名：
  - 使用小写字母和下划线分隔单词（例如：point_cloud_processor.cpp）。
- 类名：
  - 使用大写字母开头的驼峰命名法（例如：PointCloudProcessor）,不要有下划线。
- 普通变量名：
  - 变量 (包括函数参数) 和数据成员名一律小写, 单词之间用下划线连接. 类的成员变量以下划线结尾, 但结构体的就不用, 如: a_local_variable, a_struct_data_member, a_class_data_member_.
- 常量命名：
  - 声明为 constexpr 或 const 的变量, 或在程序运行期间其值始终保持不变的, 命名时以 “k” 开头, 大小写混合. 例如: kMaxSize, kBufferSize.
- 函数名：
  - 常规函数使用大小写混合, 取值和设值函数则要求与变量名匹配: MyExcitingFunction(), MyExcitingMethod(), my_exciting_member_variable(), set_my_exciting_member_variable().
- 命名空间命名：
  - 命名空间以小写字母命名. 最高级命名空间的名字取决于项目名称. 要注意避免嵌套命名空间的名字之间和常见的顶级命名空间的名字之间发生冲突.
  顶级命名空间的名称应当是项目名或者是该命名空间中的代码所属的团队的名字. 命名空间中的代码, 应当存放于和命名空间的名字匹配的文件夹或其子文件夹中.
  注意 不使用缩写作为名称 的规则同样适用于命名空间. 命名空间中的代码极少需要涉及命名空间的名称, 因此没有必要在命名空间中使用缩写.
  要避免嵌套的命名空间与常见的顶级命名空间发生名称冲突. 由于名称查找规则的存在, 命名空间之间的冲突完全有可能导致编译失败. 尤其是, 不要创建嵌套的 std 命名空间. 建议使用更独特的项目标识符 (websearch::index, websearch::index_util) 而非常见的极易发生冲突的名称 (比如 websearch::util).
  对于 internal 命名空间, 要当心加入到同一 internal 命名空间的代码之间发生冲突 (由于内部维护人员通常来自同一团队, 因此常有可能导致冲突). 在这种情况下, 请使用文件名以使得内部名称独一无二 (例如对于 frobber.h, 使用 websearch::index::frobber_internal).
- 枚举
  - 枚举的命名应当和 常量 或 宏 一致: kEnumName 或是 ENUM_NAME.

