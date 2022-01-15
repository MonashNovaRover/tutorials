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
   int result = 1;

   // for loop implementation of factorial
   for(int i = 1; i <= num; i++){
   	result = result * i;
   }
   return result;
}


