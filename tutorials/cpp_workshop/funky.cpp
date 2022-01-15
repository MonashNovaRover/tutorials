#include <iostream>

/* We can declare the "prototype" of the function 
 * without defining its implementation
 */
int fact(int);

int main(){
	int num;
	std::cout << "Enter Number: "; std::cin >> num; std::cout;
	std::cout << num << "! = " << fact(num) << std::endl;
}

// here's the function definition
int fact(int num){
    if(num <= 1){
    	return 1;
    }
    else{
    	return num * fact(num - 1);
    }
}

