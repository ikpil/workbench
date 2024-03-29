#include <fmt/core.h>
#include <zlib.h>
#include <iostream>
#include <fmt/color.h>

int main() {
    gzFile file;
    file = gzopen("test.gz", "w");
    gzwrite(file, "hello\0", 6);
    gzclose(file);

    fmt::print(fg(fmt::color::crimson) |
               fmt::emphasis::bold,
               "Hello, {}!\n", "world");
    fmt::print(fg(fmt::color::floral_white) |
               bg(fmt::color::slate_gray) |
               fmt::emphasis::underline,
               "Hello, {}!\n", "мир");

    fmt::print(fg(fmt::color::steel_blue) | fmt::emphasis::italic,
               "Hello, {}!\n", "世界");
}