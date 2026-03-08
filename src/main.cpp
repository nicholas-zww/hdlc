#include <stdio.h>

extern void app_entry();

extern "C" void app_main()
{
    app_entry();
}