#include <iostream>
#include <string>

#include "emulator.hpp"

int main(int argc, char *argv[])
{
    std::string inFilename;

    int res = EE_OK;

    if (argc > 1)
        inFilename = argv[1];
    else {
        std::cout << "No input file provided\n";
        res = EE_FILE;
    }

    if (res == EE_OK) {
        Emulator emulator;
        res = emulator.run(inFilename);
    }

    return res;
}
