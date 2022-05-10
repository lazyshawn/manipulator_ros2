#ifndef USER_INTERFACE_HPP
#define USER_INTERFACE_HPP

/*************************************************************************
 * @funcs : colorful_printf
 * @brief : 特殊状态的彩色输出
 * @refes :
 *    Colorful printf:
 *      https://www.cnblogs.com/lewki/p/14343894.html
 *    template<typename... Args>:
 *      https://stackoverflow.com/questions/45891152
*************************************************************************/
#include <stdio.h>

template<typename... Args>
void printf_error(const char* f, Args... args) {
  printf("\033[0m\033[1;31mERROR:\033[0m ");
  printf(f, args...);
  printf("\n");
}

template<typename... Args>
void printf_warning(const char* f, Args... args) {
  printf("\033[0m\033[1;33mWarning:\033[0m ");
  printf(f, args...);
  printf("\n");
}

template<typename... Args>
void printf_info(const char* f, Args... args) {
  printf("\033[0m\033[1;32mInfo:\033[0m ");
  printf(f, args...);
  printf("\n");
}

/*************************************************************************
 * @funcs : scanKeyboard
 * @brief : 监听键盘事件
 * @param : void
 * @return: 键盘按键的 ASCII 码
 * @refes : https://www.cnblogs.com/SchrodingerDoggy/p/14072739.html
*************************************************************************/
// 监听键盘事件
#include <stdio.h>
#include <termio.h>

int scanKeyboard() {
  // 通过tcsetattr函数设置terminal的属性来控制需不需要回车来结束输入
  struct termios new_settings;
  struct termios stored_settings;
  // 备份 termios 设置
  tcgetattr(0, &stored_settings);
  new_settings = stored_settings;
  new_settings.c_lflag &= (~ICANON);
  new_settings.c_cc[VTIME] = 0;
  tcgetattr(0, &stored_settings);
  new_settings.c_cc[VMIN] = 1;
  tcsetattr(0, TCSANOW, &new_settings);
  // 监听键盘事件
  int input = getchar();
  // 还原设置
  tcsetattr(0, TCSANOW, &stored_settings);
  return input;
}

#endif

