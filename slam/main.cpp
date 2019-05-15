#include <iostream>

int main() {
    std::cout << "Hello, World!" << std::endl;

    std::string a = "6";
    std::cout << "a: " << a << std::endl;

    std::string& b = a;
    std::cout << "b: " << b << std::endl;

    std::string c = "9";
    b = c;
    std::cout << "b: " << b << std::endl;

    return 0;
}