#include <iostream>
int main(){
    int num = 42;           // 4 bytes
    int *pointer = &num;    // 8 bytes
    std::cout << "Pointer: " << pointer << std::endl;
    std::cout << "Int: " << *pointer << std::endl;
}
