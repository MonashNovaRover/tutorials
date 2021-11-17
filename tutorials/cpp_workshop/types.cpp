#include <iostream>

int main(){
    
    /*
     * 1. Creating variables of different types
    */

    // declare and assign
    short s = 32767;           // 2 bytes
    int i = 2147483647;        // 4 bytes
    float f = 123123.12312;    // 4 bytes
    double d = 32423.23423;    // 8 bytes
    
    // declare then assign
    bool b; b = false;         // 1 byte
    char c; c = 'a';           // 1 byte

    // array of fixed size of ints (stack memory)
    int arr[5] = {5,4,3,2,1};
    
    // array of variable size (heap memory) -- this will work in modern C++ 
    std::cout << "Enter array size: "; std::cin >> i; // gets user input 
    int var_arr[s];

    /*
     * 2. Other variable modifiers
    */

    const int x = 1; // we now can't re-assign x to a new value;

	
    /*
     * If you declare and assign a static variable like so, it will be 
     * declared and assigned once and only once, the first time the function
     * is called. Whatever value it has after you leave the function will
     * still be there when you next call the function. Use with caution!
     * */
    static int y = 1; 
    
    return 0;

}

