#include <iostream>
using namespace std;

int main(int argc, char const *argv[])
{
    system("pkill avahi");
    system("avahi-autoipd -D --force-bind --no-chroot eth0");
    return 0;
}
