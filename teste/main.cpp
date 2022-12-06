#include <iostream>
#include <chrono>
#include <iomanip>

using namespace std;

int main(int argc, char const *argv[])
{
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y:%H-%M-%S");
    auto displayVideoName = oss.str().append("_display_video.avi");
    auto disparityVideoName = oss.str().append("_disparity_video.avi");

    cout << displayVideoName << " " << disparityVideoName << endl;

    return 0;
}
