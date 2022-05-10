#ifndef USER_INTERFACE_H
#define USER_INTERFACE_H

// 监听键盘事件
#include <stdio.h>
#include <termio.h>

/*************************************************************************
 * @func  : scanKeyboard
 * @brief : 监听键盘事件
 * @param : void
 * @return: 键盘按键的 ASCII 码
 * @refes : https://www.cnblogs.com/SchrodingerDoggy/p/14072739.html
*************************************************************************/
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

